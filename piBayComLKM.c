#include <linux/init.h>           // Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>         // Core header for loading LKMs into the kernel
#include <linux/device.h>         // Header to support the kernel Driver Model
#include <linux/kernel.h>         // Contains types, macros, functions for the kernel
#include <linux/fs.h>             // Header for the Linux file system support
#include <linux/uaccess.h>        // Required for the copy to user function
#include <linux/mutex.h>          // Required for the mutex functionality
#include <linux/spinlock.h>
#include <linux/gpio.h>           // Required for the GPIO functions
#include <linux/interrupt.h>      // Required for the IRQ code
#include <linux/time.h>           // Using the clock to measure time between button presses
#include <linux/delay.h>
#define  DEVICE_NAME "piBayCom"
#define  CLASS_NAME  "PiB"


MODULE_LICENSE("GPL");                  //< The license type -- this affects available functionality
MODULE_AUTHOR("G7TAJ");                 //< The author -- visible when you use modinfo
MODULE_DESCRIPTION("Linux BayCom Kernel Module for RasPi");  //< The description -- see modinfo
MODULE_VERSION("0.1.7a");



static DEFINE_MUTEX(ebbchar_mutex);  /// A macro that is used to declare a new mutex that is visible in this file
                                     /// results in a semaphore variable ebbchar_mutex with value 1 (unlocked)
                                     /// DEFINE_MUTEX_LOCKED() results in a variable with value 0 (locked)

#define DEFINE_SPINLOCK(x)    spinlock_t x = __SPIN_LOCK_UNLOCKED(x)

typedef int BOOL;
#define TRUE 1
#define FALSE 0
#define HIGH 1
#define LOW 0


static DEFINE_SPINLOCK(flop_lock);
unsigned long flags;
static short bittime;
static int TXDELAY;
static int BAUD;
static short bittime_arr[1025];

static int    majorNumber;                  ///< Stores the device number -- determined automatically
static short  arr_pos;                          // how many elements in arr
static int    numberOpens = 0;              ///< Counts the number of times the device is opened
static struct class*  ebbcharClass  = NULL; ///< The device-driver class struct pointer
static struct device* ebbcharDevice = NULL; ///< The device-driver device struct pointer


//static unsigned int RX_PIN = 17;
//static unsigned int TX_PIN = 23;
//static unsigned int PTT_PIN = 27;
static int RX_PIN = 0;
static int TX_PIN = 0;
static int PTT_PIN = 0;

 module_param(RX_PIN, int, S_IRUGO);
 MODULE_PARM_DESC(RX_PIN, "piBayCom RX PIN Pi assignment");
 module_param(TX_PIN, int, S_IRUGO);
 MODULE_PARM_DESC(TX_PIN, "piBayCom TX PIN Pi assignment");
 module_param(PTT_PIN, int, S_IRUGO);
 MODULE_PARM_DESC(PTT_PIN, "piBayCom PTT PIN Pi assignment");
 module_param(BAUD, int, S_IRUGO);
 MODULE_PARM_DESC(BAUD, "piBayCom BAUD Rate (1200 / 300)");

static unsigned int irqNumber;          ///< Used to share the IRQ number within this file
static struct timespec ts_last, ts_current, ts_diff;

/// Function prototype for the custom IRQ handler function -- see below for the implementation
static irq_handler_t  ebbgpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);


#define IOCTL_CAL_DROP _IO( 'q' , 0 )
#define IOCTL_CAL_HIGH _IO( 'q' , 1 )
#define IOCTL_CAL_LOW _IO( 'q' , 2 )
#define IOCTL_CAL_DIDDLE _IO( 'q' , 3 )


// The prototype functions for the character driver -- must come before the struct definition
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);
static void __exit ebbchar_exit(void);
static long   baycom_ioctl(struct file *filp, unsigned int cmd, unsigned long argp);


/** @brief Devices are represented as file structure in the kernel. The file_operations structure from
 *  /linux/fs.h lists the callback functions that you wish to associated with your file operations
 *  using a C99 syntax structure. char devices usually implement open, read, write and release calls
 */
static struct file_operations fops =
{
   .owner = THIS_MODULE,
   .open = dev_open,
   .read = dev_read,
   .write = dev_write,
   .release = dev_release,
   .unlocked_ioctl   = baycom_ioctl,
};


/* ioctl - CAL control routeine */
static long baycom_ioctl(struct file *filp, unsigned int cmd, unsigned long argp) {
int x;

        switch (cmd) {
                default:
                        printk(KERN_INFO "piBayCom LKM: CAL-MODE : default (%d)\n",cmd);
                        break;

                case IOCTL_CAL_DROP :   //PTT off
                        gpio_set_value(PTT_PIN, LOW);          // drop PTT
                        printk(KERN_INFO "piBayCom LKM: CAL-MODE : PTT Dropped\n");
                        break;

                case IOCTL_CAL_HIGH  : // PTT a 1
                        gpio_set_value(PTT_PIN, HIGH);         // raise PTT
                        gpio_set_value(TX_PIN, HIGH);
                        printk(KERN_INFO "piBayCom LKM: CAL-MODE : PTT HIGH\n");
                        break;

                case IOCTL_CAL_LOW :    // PTT a 0
                        gpio_set_value(PTT_PIN, HIGH);         // raise PTT
                        gpio_set_value(TX_PIN, LOW);
                        printk(KERN_INFO "piBayCom LKM: CAL-MODE : PTT LOW\n");
                        break;

                case IOCTL_CAL_DIDDLE : // PTT  a diddle
                        printk(KERN_INFO "piBayCom LKM: CAL-MODE : PTT DIDDLE\n");
                        gpio_set_value(PTT_PIN, HIGH);         // raise PTT

                        for (x=1 ; x < 10000; x++) {
                                gpio_set_value(TX_PIN, LOW);
                                udelay(bittime);
                                gpio_set_value(TX_PIN, HIGH);
                                udelay(bittime);
                        }
                        gpio_set_value(PTT_PIN, LOW);         // drop PTT
                        printk(KERN_INFO "piBayCom LKM: CAL-MODE : PTT Dropped\n");

                        break;
        }
    return 0;
}



/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point.
 *  @return returns 0 if successful
 */
static int __init ebbchar_init(void){
   long seconds;
   long micros;
   static struct timespec init_ts_current;

   printk(KERN_INFO "piBayCom LKM: Initializing the RX GPIO LKM\n");

   if (BAUD == 0) BAUD=1200; // if its not specified, assume 1200 BAUD

   if ( RX_PIN == 0 || TX_PIN ==0 || PTT_PIN == 0 || RX_PIN==TX_PIN || RX_PIN==PTT_PIN || TX_PIN==PTT_PIN) {
      printk(KERN_WARNING "piBayCom LKM: Error - PIN assignments wrong\r\npiBayCom LKM: Moduie NOT loaded\r\n");
      return -EINVAL;
   } else {
      printk(KERN_INFO "piBayCom LKM: RX=%d  TX=%d  PTT=%d\r\n", RX_PIN,TX_PIN,PTT_PIN);
   }
   getnstimeofday(&ts_last);                          // set the last time to be the current time
   ts_diff = timespec_sub(ts_last, ts_last);          // set the initial time difference to be 0

// find bittime  - should be about 833 for 1200 BAUD;

   udelay(1000);                          //sleep 1 sec - seems off!
   getnstimeofday(&init_ts_current);      // Get the time again
   seconds = (init_ts_current.tv_sec - ts_last.tv_sec);
   micros = ((seconds * 100000000) + init_ts_current.tv_nsec) - (ts_last.tv_nsec);
   bittime = micros / BAUD;

   printk(KERN_INFO "piBayCom LKM: BITtime =%d (%d BAUD)\n", bittime, BAUD);

   gpio_request(RX_PIN, "sysfs");       // Set up the gpio
   gpio_direction_input(RX_PIN);        // Set the RX GPIO to be an input
   gpio_export(RX_PIN, false);          // Causes gpio to appear in /sys/class/gpio
                     // the bool argument prevents the direction from being changed

   gpio_request(TX_PIN, "sysfs");
   gpio_direction_output(TX_PIN, LOW);  // Set the gpio to be in output
   gpio_export(TX_PIN, false);          // Causes gpio to appear in /sys/class/gpio

   gpio_request(PTT_PIN, "sysfs");
   gpio_direction_output(PTT_PIN, LOW); // Set the gpio to be in output mode
   gpio_export(PTT_PIN, false);         // Causes gpio to appear in /sys/class/gpio


   // GPIO numbers and IRQ numbers are not the same! This function performs the mapping
   irqNumber = gpio_to_irq(RX_PIN);
   printk(KERN_INFO "piBayCom LKM: The RX pin is mapped to IRQ: %d\n", irqNumber);


   // Try to dynamically allocate a major number for the device -- more difficult but worth it
   majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
   if (majorNumber<0){
      printk(KERN_ALERT "piBayCom LKM: failed to register a major number\n");
      return majorNumber;
   }
   printk(KERN_INFO "piBayCom LKM: registered correctly with major number %d\n", majorNumber);

   // Register the device class
   ebbcharClass = class_create(THIS_MODULE, CLASS_NAME);
   if (IS_ERR(ebbcharClass)){                // Check for error and clean up if there is
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_ALERT "Failed to register device class\n");
      return PTR_ERR(ebbcharClass);          // Correct way to return an error on a pointer
   }
   printk(KERN_INFO "piBayCom LKM: device class registered correctly\n");

   // Register the device driver
   ebbcharDevice = device_create(ebbcharClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
   if (IS_ERR(ebbcharDevice)){               // Clean up if there is an error
      class_destroy(ebbcharClass);           // Repeated code but the alternative is goto statements
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_ALERT "Failed to create the device\n");
      return PTR_ERR(ebbcharDevice);
   }


   printk(KERN_INFO "piBayCom LKM: device class created correctly\n"); // Made it! device was initialized
   return 0;
}



/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit ebbchar_exit(void){


   if(!mutex_trylock(&ebbchar_mutex)){  // check if device is open (has mutex lock) -check if we need to free IRQ
      printk(KERN_ALERT "piBayCom LKM: Device was open, so cleaning IRQ");
      free_irq(irqNumber, NULL);
   }


   mutex_destroy(&ebbchar_mutex);        /// destroy the dynamically-allocated mutex

   gpio_unexport(RX_PIN);               // Unexport the RX GPIO
   gpio_unexport(TX_PIN);               // Unexport the TX GPIO
   gpio_unexport(PTT_PIN);               // Unexport the PTT GPIO

   gpio_free(RX_PIN);                   // Free GPIO's
   gpio_free(TX_PIN);
   gpio_free(PTT_PIN);

   device_destroy(ebbcharClass, MKDEV(majorNumber, 0));     // remove the device
   class_unregister(ebbcharClass);                          // unregister the device class
   class_destroy(ebbcharClass);                             // remove the device class
   unregister_chrdev(majorNumber, DEVICE_NAME);             // unregister the major number
   printk(KERN_INFO "piBayCom LKM: Unloaded!\n");
}



/** @brief The GPIO IRQ Handler function
 *  This function is a custom interrupt handler that is attached to the GPIO above. The same interrupt
 *  handler cannot be invoked concurrently as the interrupt line is masked out until the function is complete.
 *  This function is static as it should not be invoked directly from outside of this file.
 *  @param irq    the IRQ number that is associated with the GPIO -- useful for logging.
 *  @param dev_id the *dev_id that is provided -- can be used to identify which device caused the interrupt
 *  Not used in this example as NULL is passed.
 *  @param regs   h/w specific register values -- only really ever used for debugging.
 *  return returns IRQ_HANDLED if successful -- should return IRQ_NONE otherwise.
 */
static irq_handler_t ebbgpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){


   getnstimeofday(&ts_current);         // Get the current time as ts_current
   ts_diff = timespec_sub(ts_current, ts_last);   // Determine the time difference between last 2 presses
   ts_last = ts_current;                // Store the current time as the last time ts_last

   spin_lock_irqsave(&flop_lock, flags);

   bittime_arr[arr_pos++]= ts_diff.tv_nsec/1000;

   if (arr_pos > 1024) {
        printk(KERN_INFO "piBayCom LKM: RX Overrun\n");
        arr_pos=0;
   }

   spin_unlock_irqrestore(&flop_lock, flags);
   return (irq_handler_t) IRQ_HANDLED;      // Announce that the IRQ has been handled correctly
}




/** @brief The device open function that is called each time the device is opened
 *  This will only increment the numberOpens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_open(struct inode *inodep, struct file *filep){

  static int result;

   if(!mutex_trylock(&ebbchar_mutex)){    /// Try to acquire the mutex (i.e., put the lock on/down)
                                          /// returns 1 if successful and 0 if there is contention
      printk(KERN_ALERT "piBayCom LKM: Device in use by another process");
      return -EBUSY;
   }


   // This next call requests an interrupt line
   result = request_irq(irqNumber,                                      // The interrupt number requested
                        (irq_handler_t) ebbgpio_irq_handler,            // The pointer to the handler function below
                        IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,     // Interrupt on rising edge & falling
                        "piBayCom_RXgpio_handler",    // Used in /proc/interrupts to identify the owner
                        NULL);                 // The *dev_id for shared interrupt lines, NULL is okay

   printk(KERN_INFO "piBayCom LKM: The interrupt request result is: %d\n", result);
   if ( result < 0) {
    printk(KERN_INFO "Error setting IRQ. Quitting...\n");
    return  -EINVAL; // does it not load module?
   }


   numberOpens++;
   printk(KERN_INFO "piBayCom LKM: Device has been opened %d time(s)\n", numberOpens);
   return 0;
}


/** @brief This function is called whenever device is being read from user space i.e. data is
 *  being sent from the device to the user. In this case is uses the copy_to_user() function to
 *  send the buffer string to the user and captures any errors.
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 *  @param buffer The pointer to the buffer to which this function writes the data
 *  @param len The length of the b
 *  @param offset The offset if required
 */
static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset){

   int error_count = 0;
   int total = 0;


   spin_lock_irqsave(&flop_lock, flags);

  // copy_to_user has the format ( * to, *from, size) and returns 0 on success
   error_count = copy_to_user(buffer, (char *)bittime_arr, arr_pos*2);

   if (error_count==0){            // if true then have success
//      printk(KERN_INFO "EBBChar: Sent %d elements to the user\n", arr_pos);
        total=arr_pos*2;
        arr_pos=0;
        spin_unlock_irqrestore(&flop_lock, flags);
      return (total);  // clear the position to the start and return 0
   }
   else {
      printk(KERN_INFO "piBayCom LKM: Failed to send %d elements to piBayComKISS\n", error_count);
      spin_unlock_irqrestore(&flop_lock, flags);
      return -EFAULT;              // Failed -- return a bad address message (i.e. -14)
   }
}


static int decode_user_msg( char *buffer, size_t len) {

   int BUFF_LEN;
   int x;
   int a = 0;
   int TXDelayFlags;
   int TxBlockPtr;
   BOOL Level;

   if (len>4096) {
       BUFF_LEN=4096;
   } else {
       BUFF_LEN=len;
   }


   TXDelayFlags = ( TXDELAY * 1000 ) / ( bittime * 8);
   Level=LOW;

   gpio_set_value(PTT_PIN, HIGH);          // raise PTT

   // PTT delay...send number of flags
   for (x=0; x < TXDelayFlags; x++) {
      gpio_set_value(TX_PIN, LOW);
      udelay(bittime);
      gpio_set_value(TX_PIN, HIGH);
      udelay(bittime * 6);
      gpio_set_value(TX_PIN, LOW);
      udelay(bittime);
   }

   TxBlockPtr=BUFF_LEN;
   a=0;
   while(TxBlockPtr!=0) {
      if ( buffer[a++] == 0 ) {
         // toggle GPIO_TX_PIN

         if(Level==LOW) {
            Level =HIGH;
         } else {
            Level=LOW;
         }

         gpio_set_value(TX_PIN, Level);

      }
      udelay(bittime);
      TxBlockPtr--;                // continue through
   }


   // Lower PTT
   gpio_set_value(PTT_PIN, LOW);
   return 0;
}




/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user. The data is copied to the message[] array in this
 *  LKM using the sprintf() function along with the length of the string.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){

//   printk(KERN_INFO "piBayCom LKM: Received %zu bytes from piBayComKISS\n", len);


  // If [0] ==FF then KISS params. Currently only sending/setting TXDelay - probably better to do
 //  this via the IOCTL routines now.
	
  if ( buffer[0]==0xff && len==2) {
//      printk(KERN_INFO "TXDelay Parameter = %dmSec\n", buffer[1]*10);
        TXDELAY = buffer[1]*10;
        return len;
  }

  decode_user_msg( (char *)buffer,len);
  return len;
}




/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_release(struct inode *inodep, struct file *filep){
   mutex_unlock(&ebbchar_mutex);          /// Releases the mutex (i.e., the lock goes up)
   free_irq(irqNumber, NULL);               // Free the IRQ number, no *dev_id required in this case
   printk(KERN_INFO "piBayCom LKM: RX Interrupt Stopped\n");
   printk(KERN_INFO "piBayCom LKM: Device successfully closed\n");
   return 0;
}

/** @brief A module must use the module_init() module_exit() macros from linux/init.h, which
 *  identify the initialization function at insertion time and the cleanup function (as
 *  listed above)
 */
module_init(ebbchar_init);
module_exit(ebbchar_exit);
