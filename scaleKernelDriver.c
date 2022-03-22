

///
/// Usage
///

//////////////////////////////
/*
 * make																														// Build kernel module
* sudo insmod scaleKernelDriver.ko  															// Load kernel module
 * sudo rmmod scaleKernelDriver																// Un load kernel module
 * /sys/module/scaleKernelDriver/parameters/scale_value				// Userspace interface for reading data from scale device
 * dmesg -xw																											// While loop read kernel log messages
 */
//////////////////////////////


#include<linux/kernel.h>
#include<linux/init.h>
#include<linux/module.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/uaccess.h>					//copy_to/from_user()
#include <linux/gpio.h>							//GPIO
#include <linux/interrupt.h>
#include <linux/workqueue.h>			// Required for workqueues
#include <linux/delay.h>


// RESET is connected to this GPIO 2
#define GPIO_RESET (2)
// INTERRUPT line is connected to this GPIO 3
#define GPIO_INT  (3)

// This used for storing the IRQ number for the GPIO
unsigned int GPIO_irqNumber;

// SPI Bus number
#define MY_BUS_NUM 6
// Handle for SPI sensor
static struct spi_device *spi_device;
// SPI Transmit buffer
unsigned char transmitData[4] = {0xA0, 0xFF, 0xFF, 0xAA};
// SPI Receive buffer
unsigned char receiveData[4] = {0x00, 0x00, 0x00, 0x00};
unsigned int receiveDataValidRaw = 0;


//creating the dev with our custom major and minor number
dev_t dev = MKDEV(266, 0);

// Read from SPI Scale sensor (Background thread worker)
void workqueue_fn(struct work_struct *work);
// Creating work by Static Method for Scale measurement
DECLARE_WORK(workqueue,workqueue_fn);

// Kernel module initialization function
static int __init main_init(void);
// Kernel module deinitialization function (Cleanup on unload)
static void __exit main_exit(void);

// Read raw scale data
static int spi_read_scale_raw(void);

// SPI Data ready to read interrupt
static irqreturn_t gpio_irq_handler(int irq,void *dev_id);

// Create parameter endpoint on
// /sys/module/scaleKernelDriver/parameters/scale_value
// Use for READing `cat /sys/module/scaleKernelDriver/parameters/scale_value`
// Use for WRITing `echo 1 > /sys/module/scaleKernelDriver/parameters/scale_value`
/*----------------------Module_param_cb()--------------------------------*/
int cb_scale_value = 0;

int notify_param(const char *val, const struct kernel_param *kp)
{
	int res = param_set_int(val, kp); // Use helper for write variable
	if(res==0) {

		pr_info("Written to param value...\n");
		//pr_info("New value of tare = %d\n", cb_scale_value);
		return 0;
	}
	return -1;
}

const struct kernel_param_ops my_param_ops =
{
		.set = &notify_param, 		// Use custom setter ...
		.get = &param_get_int, 	// .. and standard getter
};

module_param_cb(scale_value, &my_param_ops, &cb_scale_value, S_IRUGO|S_IWUSR );
/*-------------------------------------------------------------------------*/


static int gpio_init(void)
{
	//Checking the GPIO is valid or not
	if(gpio_is_valid(GPIO_RESET) == false)
	{
		pr_err("GPIO %d is not valid\n", GPIO_RESET);
		goto r_device;
	}

	//Requesting the GPIO
	if(gpio_request(GPIO_RESET, "GPIO_RESET") < 0)
	{
		pr_err("ERROR: GPIO %d request\n", GPIO_RESET);
		goto r_gpio;
	}

	//configure the GPIO as output
	gpio_direction_output(GPIO_RESET, 0);

	gpio_set_value(GPIO_RESET, 0);                      // Power down scale (Power Cycle)

	//Input GPIO configuration
	//Checking the GPIO is valid or not
	if(gpio_is_valid(GPIO_INT) == false){
		pr_err("GPIO %d is not valid\n", GPIO_INT);
		goto r_gpio_in;
	}

	//Requesting the GPIO
	if(gpio_request(GPIO_INT,"GPIO_25_IN") < 0){
		pr_err("ERROR: GPIO %d request\n", GPIO_INT);
		goto r_gpio_in;
	}

	//configure the GPIO as input
	gpio_direction_input(GPIO_INT);

	//Get the IRQ number for scale GPIO interrupt pin
	GPIO_irqNumber = gpio_to_irq(GPIO_INT);
	pr_info("GPIO_irqNumber = %d\n", GPIO_irqNumber);

	msleep(500);
	gpio_set_value(GPIO_RESET, 1);					 // Power up scale (Power Cycle)

	// Read raw value from scale
	spi_read_scale_raw();
	pr_info("SPI ReadWriteHARD!!!\n");

	//  Setup sensor data ready interrupt handler
	// Trigger on falling edge
	if (request_irq(GPIO_irqNumber,					//IRQ number
			(void *)gpio_irq_handler,						 	//IRQ handler
			IRQF_TRIGGER_FALLING,        			 	 	//Handler will be called on the falling edge
			"scale_device",               								 	//used to identify the device name using this IRQ
			NULL))																	//device id for shared IRQ
	{
		pr_err("my_device: cannot register IRQ ");
		goto r_gpio_in;
	}

	return 0;

	r_gpio_in:
	gpio_free(GPIO_INT);
	r_gpio:
	gpio_free(GPIO_RESET);
	r_device:

	return -1;

}

// Cleanup GPIOs on unload
static void gpio_exit(void)
{
	gpio_unexport(GPIO_RESET);
	gpio_free(GPIO_RESET);

	free_irq(GPIO_irqNumber, NULL);
	gpio_free(GPIO_INT);
}

//Interrupt handler for GPIO_INT. This will be called whenever there is a falling edge detected.
static irqreturn_t gpio_irq_handler(int irq,void *dev_id)
{
	static unsigned long flags = 0;

	// Disable hard interrupts
	local_irq_save(flags);

	// Schedule read from scale device for later (So we would not block IRQ Handler for long time)
	schedule_work(&workqueue);

	// Re enable hard interrupts
	local_irq_restore(flags);

	return IRQ_HANDLED;
}

// Setting up SPI bus for Scale device
struct spi_board_info spi_device_info = {
		.modalias = "ivan-scale",				 // SPI bus alias name
		.max_speed_hz = 1000000,		 // Set bus speed to (1MHz)
		.bus_num = MY_BUS_NUM,		// Board specific identifier for SPI controller
		.chip_select = 1,								// GPIO number of the chip select line
		.mode = SPI_MODE_3,					// Polarity of bus for r/w operations
};

// Initialize SPI bus
static int spi_init(void)
{
	int ret;
	struct spi_master *master;

	// Returns a ref-counted pointer to the relevant spi_master (which the caller must release)
	master = spi_busnum_to_master( spi_device_info.bus_num );
	if( !master ){
		pr_err("MASTER not found.\n");
		return -ENODEV;
	}

	// create a new slave device, given the master and device info
	spi_device = spi_new_device( master, &spi_device_info );

	if( !spi_device ) {
		pr_err("FAILED to create slave.\n");
		return -ENODEV;
	}

	// Setup device to use 8 bit messages
	spi_device->bits_per_word = 8;

	// Setup SPI mode and clock rate
	ret = spi_setup( spi_device );

	if( ret ){
		pr_err("FAILED to setup slave.\n");
		spi_unregister_device( spi_device );
		return -ENODEV;
	}

	return 0;
}

// Reads raw value from scale device and returns it
static int spi_read_scale_raw(void)
{
	if( spi_device )
	{
		// Setup SPI buffers
		struct spi_transfer tr =
		{
				.tx_buf	= transmitData,
				.rx_buf = receiveData,
				.len		= 4,
		};

		// Issue synchronous SPI data transfer
		spi_sync_transfer( spi_device, &tr, 1 );

		return 0;
	}
	return -ENODEV;
}


// SPI cleanup on unload
static void  spi_exit(void)
{

	if( spi_device ){
		spi_unregister_device( spi_device );
	}

	pr_info("Fully UNloaded SPI.\n");

}

// SPI workqueue task (Called from Scale device int handler to read data later in background)
void workqueue_fn(struct work_struct *work)
{
	spi_read_scale_raw();

	// Check if data is valid
	if  (!( (receiveData[0] ==0xFF)  && (receiveData[1]==0xFF) && (receiveData[2]==0xFF) && (receiveData[3]==0xFF) ))
	{
		receiveDataValidRaw = receiveData[0] << 16 | receiveData[1] << 8 | receiveData[2];
		pr_info("ReceivedIntHex = 0x%X \n", receiveDataValidRaw);
		cb_scale_value = receiveDataValidRaw;
	}
}

// Module Init function
static int __init main_init(void)
{

	// Allocating Major number
	if((alloc_chrdev_region(&dev, 0, 1, "Ivan&Co")) <0){
		pr_err("Cannot allocate major number for device\n");
		goto r_class;
	}

	spi_init();
	gpio_init();

	return 0;

	r_class:
	unregister_chrdev_region(dev,1);

	return -1;

}


// Module Exit function
static void __exit main_exit(void)
{
	spi_exit();
	gpio_exit();

	unregister_chrdev_region(dev, 1);

	pr_info( "Kernel Module Removed Successfully...\n");
}

// Entry function (onLoad)
module_init(main_init);
// Exit function (onUnLoad)
module_exit(main_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Ivan Palijan ivan95.603@hotmail.com");
MODULE_DESCRIPTION("SPI Scale driver");
MODULE_VERSION("1.0");
