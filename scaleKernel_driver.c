

///
/// Usage
///

//////////////////////////////
/*
 * make																// Build kernel module
 * sudo dtoverlay scale_overlay.dtbo
 * sudo insmod scaleKernel_driver.ko  								// Load kernel module
 * sudo rmmod scaleKernel_driver										// Un load kernel module
 * /sys/module/scaleKernel_driver/parameters/scale_value				// Userspace interface for reading data from scale device
 * dmesg -xw														// While loop read kernel log messages
 * sudo dtoverlay -r scale_overlay
 */
//////////////////////////////

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/uaccess.h> //copy_to/from_user()
#include <linux/gpio.h>	   //GPIO
#include <linux/interrupt.h>
#include <linux/workqueue.h> // Required for workqueues
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>

#define DRIVER_NAME "ivan_scale"

struct ivan_scale
{
	struct spi_device *spi_device;
	struct gpio_desc *gpio_out;
	struct gpio_desc *gpio_irq;
	int irq;
	struct mutex lock;
};

struct ivan_scale *scale;

// // RESET is connected to this GPIO 2
// #define GPIO_RESET (2)
// // INTERRUPT line is connected to this GPIO 3
// #define GPIO_INT (3)

// This used for storing the IRQ number for the GPIO
unsigned int GPIO_irqNumber;

// SPI Transmit buffer
unsigned char transmitData[4] = {0xA0, 0xFF, 0xFF, 0xAA};
// SPI Receive buffer
unsigned char receiveData[4] = {0x00, 0x00, 0x00, 0x00};
unsigned int receiveDataValidRaw = 0;

// creating the dev with our custom major and minor number
dev_t dev = MKDEV(266, 0);

// Read from SPI Scale sensor (Background thread worker)
void workqueue_fn(struct work_struct *work);
// Creating work by Static Method for Scale measurement
DECLARE_WORK(workqueue, workqueue_fn);

// Kernel module initialization function
static int init(void);
// Kernel module deinitialization function (Cleanup on unload)
static void exit(void);

// Read raw scale data
static int spi_read_scale_raw(void);

// SPI Data ready to read interrupt
static irqreturn_t gpio_irq_handler(int irq, void *dev_id);

// Create parameter endpoint on
// /sys/module/scaleKernelDriver/parameters/scale_value
// Use for READing `cat /sys/module/scaleKernelDriver/parameters/scale_value`
// Use for WRITing `echo 1 > /sys/module/scaleKernelDriver/parameters/scale_value`
/*----------------------Module_param_cb()--------------------------------*/
int cb_scale_value = 0;

int notify_param(const char *val, const struct kernel_param *kp)
{
	int res = param_set_int(val, kp); // Use helper for write variable
	if (res == 0)
	{

		pr_info("Written to param value...\n");
		// pr_info("New value of tare = %d\n", cb_scale_value);
		return 0;
	}
	return -1;
}

const struct kernel_param_ops my_param_ops =
	{
		.set = &notify_param,  // Use custom setter ...
		.get = &param_get_int, // .. and standard getter
};

module_param_cb(scale_value, &my_param_ops, &cb_scale_value, S_IRUGO | S_IWUSR);
/*-------------------------------------------------------------------------*/

/*
// Cleanup GPIOs on unload
static void gpio_exit(void)
{
	// gpio_unexport(GPIO_RESET);
	gpio_free(GPIO_RESET);

	free_irq(GPIO_irqNumber, NULL);
	gpio_free(GPIO_INT);
}

*/

// Interrupt handler for GPIO_INT. This will be called whenever there is a falling edge detected.
static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct ivan_scale *data = dev_id;
	static unsigned long flags = 0;

	// printk("Interrupt triggered!\n");

	// Disable hard interrupts
	local_irq_save(flags);

	// Schedule read from scale device for later (So we would not block IRQ Handler for long time)
	schedule_work(&workqueue);

	// Re enable hard interrupts
	local_irq_restore(flags);

	return IRQ_HANDLED;
}

/* SPI transfer function */
static int ivan_scale_spi_transfer(struct ivan_scale *data, u8 *tx_buf, u8 *rx_buf, size_t len)
{
	struct spi_message msg;
	struct spi_transfer xfer = {
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len = len,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	return spi_sync(data->spi_device, &msg);
}

// Reads raw value from scale device and returns it
static int spi_read_scale_raw(void)
{
	pr_info("spi_read_scale_raw\n");
	if (scale->spi_device)
	{
		pr_info("spi_read_scale_raw - if (scale->spi_device)\n");
		ivan_scale_spi_transfer(scale, transmitData, receiveData, 4);
		return 0;
	}
	return -ENODEV;
}

// SPI cleanup on unload
static void spi_exit(void)
{
	if (scale->spi_device)
	{
		spi_unregister_device(scale->spi_device);
	}

	pr_info("Fully UNloaded SPI.\n");
}

// SPI workqueue task (Called from Scale device int handler to read data later in background)
void workqueue_fn(struct work_struct *work)
{
	spi_read_scale_raw();

	// unsigned int val = receiveData[0] << 16 | receiveData[1] << 8 | receiveData[2];
	// pr_info("NON VERIFIED VAL = 0x%X \n", val);
	// printk("About to check validity of the data\n");

	// Check if data is valid
	if (!((receiveData[0] == 0xFF) && (receiveData[1] == 0xFF) && (receiveData[2] == 0xFF) && (receiveData[3] == 0xFF)))
	{
		receiveDataValidRaw = receiveData[0] << 16 | receiveData[1] << 8 | receiveData[2];
		pr_info("ReceivedIntHex = 0x%X \n", receiveDataValidRaw);
		cb_scale_value = receiveDataValidRaw;
	}
}

// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Declate the probe and remove functions */
static int ivan_scale_probe(struct spi_device *client);
static void ivan_scale_remove(struct spi_device *spi);

static struct of_device_id my_driver_ids[] = {
	{
		.compatible = "ivan,ivanscale",
	},
	{/* sentinel */}};
MODULE_DEVICE_TABLE(of, my_driver_ids);

static struct spi_device_id ivan_scale[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(spi, ivan_scale);

static struct spi_driver my_driver = {
	.probe = ivan_scale_probe,
	.remove = ivan_scale_remove,
	.id_table = ivan_scale,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = my_driver_ids,
	},
};

/**
 * @brief This function is called on loading the driver
 */

static int ivan_scale_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
	// struct my_adc *adc = iio_priv(indio_dev);
	// int ret;

	// if (mask == IIO_CHAN_INFO_RAW)
	// {
	// 	ret = spi_w8r8(adc->client, CMD_GET_ADC_VAL);
	// 	if (ret < 0)
	// 	{
	// 		printk("dt_iio - Error reading ADC value!\n");
	// 		return ret;
	// 	}
	// 	*val = ret;
	// }
	// else
	// 	return -EINVAL;
	return IIO_VAL_INT;
}

static const struct iio_chan_spec ivan_scale_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}};

static const struct iio_info ivan_scale_info = {
	.read_raw = ivan_scale_read_raw,
};

// static int ivan_scale_probe(struct spi_device *client)
// {
// 	struct iio_dev *indio_dev;

// 	int ret;
// 	u8 buffer[2];

// 	printk("dt_iio - Now I am in the Probe function!\n");

// 	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(struct iio_dev));
// 	if (!indio_dev)
// 	{
// 		printk("dt_iio - Error! Out of memory\n");
// 		return -ENOMEM;
// 	}

// 	scale = iio_priv(indio_dev);
// 	client->bits_per_word = 8;
// 	scale->spi_device = client;

// 	indio_dev->name = "ivanscale";
// 	indio_dev->info = &ivan_scale_info;
// 	indio_dev->modes = INDIO_DIRECT_MODE;
// 	indio_dev->channels = ivan_scale_channels;
// 	indio_dev->num_channels = ARRAY_SIZE(ivan_scale_channels);

// 	ret = spi_setup(client);
// 	if (ret < 0)
// 	{
// 		printk("dt_iio - Error! Failed to set up the SPI Bus\n");
// 		return ret;
// 	}

// 	// /* Set the scale to power up mode */
// 	// buffer[0] = CMD_SET_STATE;
// 	// buffer[1] = 0x1;

// 	// ret = spi_write(scale->spi_device, buffer, 2);
// 	// if(ret < 0) {
// 	// 	printk("dt_iio - Error! Could not power scale up\n");
// 	// 	return -1;
// 	// }

// 	spi_set_drvdata(client, indio_dev);

// 	return devm_iio_device_register(&client->dev, indio_dev);
// }

static int ivan_scale_probe(struct spi_device *spi)
{
	// struct mydevice_data *data;
	int ret;

	dev_info(&spi->dev, "Probing %s\n", DRIVER_NAME);

	/* Allocate driver data */
	scale = devm_kzalloc(&spi->dev, sizeof(*scale), GFP_KERNEL);
	if (!scale)
		return -ENOMEM;

	scale->spi_device = spi;
	mutex_init(&scale->lock);
	spi_set_drvdata(spi, scale);

	/* Setup SPI */
	spi->mode = SPI_MODE_3;
	// spi->bits_per_word = 8;
	spi->max_speed_hz = 100000; /* 1 MHz */
	ret = spi_setup(spi);
	if (ret)
	{
		dev_err(&spi->dev, "SPI setup failed\n");
		return ret;
	}

	/* Get GPIO output */
	scale->gpio_out = devm_gpiod_get(&spi->dev, "out", GPIOD_OUT_LOW);
	if (IS_ERR(scale->gpio_out))
	{
		dev_err(&spi->dev, "Failed to get output GPIO\n");
		return PTR_ERR(scale->gpio_out);
	}

	/* Get GPIO interrupt */
	scale->gpio_irq = devm_gpiod_get(&spi->dev, "irq", GPIOD_IN);
	if (IS_ERR(scale->gpio_irq))
	{
		dev_err(&spi->dev, "Failed to get IRQ GPIO\n");
		return PTR_ERR(scale->gpio_irq);
	}

	/* Setup interrupt */
	scale->irq = gpiod_to_irq(scale->gpio_irq);
	if (scale->irq < 0)
	{
		dev_err(&spi->dev, "Failed to get IRQ number\n");
		return scale->irq;
	}

	ret = devm_request_threaded_irq(&spi->dev, scale->irq,
									NULL, gpio_irq_handler,
									IRQF_TRIGGER_RISING | IRQF_ONESHOT,
									DRIVER_NAME, scale);
	if (ret)
	{
		dev_err(&spi->dev, "Failed to request IRQ\n");
		return ret;
	}

	/* Set GPIO to HIGH */
	printk("Set GPIO to HIGH\n");

	gpiod_set_value(scale->gpio_out, 0); // Power up scale (Power Cycle)
	msleep(500);
	gpiod_set_value(scale->gpio_out, 1); // Power up scale (Power Cycle)

	dev_info(&spi->dev, "%s probe successful\n", DRIVER_NAME);
	return 0;
}

static void ivan_scale_remove(struct spi_device *spi)
{
	struct mydevice_data *data = spi_get_drvdata(spi);

	/* Set GPIO to low before removing */
	gpiod_set_value(scale->gpio_out, 0);

	dev_info(&spi->dev, "%s removed\n", DRIVER_NAME);
	exit();
}

// /**
//  * @brief This function is called on unloading the driver
//  */
// static void ivan_scale_remove(struct spi_device *spi)
// {
// 	struct iio_dev *indio_dev;
// 	struct ivan_scale *scale;
// 	u8 buffer[2];

// 	printk("dt_iio - Now I am in the Remove function!\n");
// 	/* Set device to power down mode */
// 	// buffer[0] = CMD_SET_STATE;
// 	// buffer[1] = 0x0;
// 	indio_dev = spi_get_drvdata(spi);
// 	scale = iio_priv(indio_dev);

// 	exit();
// 	// spi_write(scale->spi_device, buffer, 2);
// }

/* This will create the init and exit function automatically */
module_spi_driver(my_driver);

// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Module Init function
static int init(void)
{

	// Allocating Major number
	if ((alloc_chrdev_region(&dev, 0, 1, "Ivan&Co")) < 0)
	{
		pr_err("Cannot allocate major number for device\n");
		goto r_class;
	}

	// spi_init();
	pr_info("gpio_init about to be called\n");
	// gpio_init();

	return 0;

r_class:
	unregister_chrdev_region(dev, 1);

	return -1;
}

// Module Exit function
static void exit(void)
{
	// spi_exit();
	// gpio_exit();

	unregister_chrdev_region(dev, 1);

	// r_gpio_in:
	// 	gpio_free(GPIO_INT);
	// r_gpio:
	// 	gpio_free(GPIO_RESET);
	// r_device:

	pr_info("Kernel Module Removed Successfully...\n");
}
/*
// Entry function (onLoad)
module_init(main_init);
// Exit function (onUnLoad)
module_exit(main_exit);
*/
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Ivan Palijan ivan95.603@hotmail.com");
MODULE_DESCRIPTION("SPI Scale driver");
MODULE_VERSION("1.0");
