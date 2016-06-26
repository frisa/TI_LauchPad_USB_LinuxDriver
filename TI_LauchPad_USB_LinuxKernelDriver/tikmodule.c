/*
 * tikmodule.c
 *
 *  Created on: Jun 25, 2016
 *      Author: root
 */

#include <linux/init.h>           // Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>         // Core header for loading LKMs into the kernel
#include <linux/device.h>         // Header to support the kernel Driver Model
#include <linux/kernel.h>         // Contains types, macros, functions for the kernel
#include <linux/fs.h>             // Header for the Linux file system support
#include <linux/usb.h>

#include <asm/uaccess.h>          // Required for the copy to user function
#include <linux/mutex.h>	      // Required for the mutex functionality
#include </root/_git/linux/include/linux/usb.h>
#include <linux/slab.h>			  /* kmalloc() */
#include <linux/usb/hcd.h>
#include <linux/spinlock.h>
#include <linux/spinlock_types.h>

/* ===================================================================================================================*/
MODULE_LICENSE("GPL");                 //< The license type -- this affects available functionality
MODULE_AUTHOR("Jan Fridrichovsky");    //< The author -- visible when you use modinfo
MODULE_DESCRIPTION("A simple Linux USB char driver");  //< The description -- see modinfo
MODULE_VERSION("0.1");                             //< A version number to inform users

/*
#define USB_VID_TI_1CBE         0x1cbe
#define USB_PID_MOUSE           0x0000
#define USB_PID_KEYBOARD        0x0001
#define USB_PID_SERIAL          0x0002
#define USB_PID_BULK            0x0003
#define USB_PID_SCOPE           0x0004
#define USB_PID_MSC             0x0005
#define USB_PID_AUDIO           0x0006
#define USB_PID_COMP_SERIAL     0x0007
#define USB_PID_COMP_AUDIO_HID  0x0008
#define USB_PID_COMP_HID_SER    0x0009
#define USB_PID_COMP_HID_DFU    0x000A
#define USB_PID_DATA_LOGGER     0x000B
#define USB_PID_COMP_HID_HID    0x000D
#define USB_PID_GAMEPAD         0x000F
#define USB_PID_LP_CGAMEPAD     0x0010
#define USB_PID_DFU             0x00FF
*/

#define VENDOR_ID	0x1234 /* Luminary Micro Inc. */
#define PRODUCT_ID	0x0001 /* Generic Bulk Device */
#define XBDEV_MINOR_BASE	0
//#define XB_DEBUG_ENABLE

#define XB_CTRL_BUFFER_SIZE     8
#define XB_CTRL_REQUEST_TYPE	0x21    //(bmRequestType) Host to device, class specific, interface
#define XB_CTRL_REQUEST		    0x00    //the value 0 is ADSC request other valueas are reserved
#define XB_CTRL_VALUE		    0x0
#define XB_CTRL_INDEX		    0x0

#ifdef XB_DEBUG_ENABLE
#define PDEBUG(fmt, args...) printk( KERN_DEBUG "xb[%s <%d>]:"fmt"\n", __FUNCTION__, __LINE__, ## args);
#define SPDEBUG(fmt, args...) printk( KERN_DEBUG "xb:"fmt"\n", ## args);
#else
#define PDEBUG(fmt, args...) /* empty define */
#define SPDEBUG(fmt, args...) /* empty define */
#endif

static struct usb_device_id xbdrv_id_table[] =
{
	{ USB_DEVICE(VENDOR_ID, PRODUCT_ID) },
	{ },
};
MODULE_DEVICE_TABLE (usb, xbdrv_id_table);
/* ===================================================================================================================*/
/* FUNCTIONS PROTOTYPES */
static void vPrintURB(struct urb *iurb);
static void vPrintUSB_DRIVER(struct usb_driver *udriver);
static void vPrintUSB_DEVICE(struct usb_device *udev);
static void vPrintUSB_INTERFACE(struct usb_interface *uinterface);
static void vPrintUSB_HOST_INTERFACE(struct usb_host_interface *uhinterface);

static int     	xbdev_open(struct inode *, struct file *);
static int     	xbdev_release(struct inode *, struct file *);
static ssize_t 	xbdev_read(struct file *, char *, size_t, loff_t *);
static ssize_t 	xbdev_write(struct file *, const char *, size_t, loff_t *);

static int 		xbdrv_probe(struct usb_interface *interface, const struct usb_device_id *id);
static void 	xbdrv_disconnect(struct usb_interface *interface);
/* ===================================================================================================================*/
/* DRIVER DEVICE OPERATIONS */
static struct file_operations xbdev_fops =
{
   .open    = xbdev_open,
   .read    = xbdev_read,
   .write   = xbdev_write,
   .release = xbdev_release,
};

/* DRIVER CORE INFO */
static struct usb_driver usb_xbdrvInfo =
{
    .name       = "xbdrv",
    .id_table   = xbdrv_id_table,
    .probe      = xbdrv_probe,
    .disconnect = xbdrv_disconnect,
};

/* DEVICE INFO STRUCTURE */
struct usb_xbdevInfo
{
	struct usb_device 		*udev;                      // kernels representation of the usb device
	struct usb_interface 	*interface;                 // interface what usb device drivers talk to
	unsigned char			minor;                      // minor version number
	char					serial_number[8];
	int						open_count;                 // open files counter
	struct 					semaphore sem;	            // semaphore to lock this structure
	spinlock_t				cmd_spinlock;	            // spinlock to lock the command value

	struct usb_endpoint_descriptor  *int_in_endpoint;   // EPIN used by drivers to retrieve a USB-defined endpoint descriptor.
	char					*int_in_buffer;             // URB INTERRUPT IN buffer
	struct urb 				*int_in_urb;                // URB INTERRUPT IN urb structure
	int						int_in_running;             // URB INTERRUPT IN running flag

	struct usb_endpoint_descriptor  *int_out_endpoint;   // EROUT
	char					*int_out_buffer;            // URB INTERRUPT OUT buffer
	struct urb 				*int_out_urb;               // URB INTERRUPT OUT urb structure
	int						int_out_running;            // URB INTERRUPT OUT running flag

	char					*ctrl_buffer;               // URB CMD EP0 8 byte buffer for ctrl msg
	struct urb				*ctrl_urb;                  // URB CMD EP0 urb structure
	struct usb_ctrlrequest  *ctrl_dr;                   // URB CMD EP0 Setup packet information
	int						correction_required;
	__u8			        command;                    // Last issued command
};

/* DRIVER CLASS */
static struct usb_class_driver xbdev_class =
{
	.name       = "xbdev%d",
	.fops       = &xbdev_fops,
	.minor_base = XBDEV_MINOR_BASE,
};
/* ===================================================================================================================*/
/* GLOBALS */
static DEFINE_MUTEX(disconnect_mutex);

/* ===================================================================================================================*/
static void vPrintUSB_DRIVER(struct usb_driver *udriver)
{
    if (udriver)
    {
        SPDEBUG("-usb_driver--------------");
        SPDEBUG("---name: %s", udriver->name);
        SPDEBUG("---no_dynamic_id: %d", udriver->no_dynamic_id);
        SPDEBUG("---supports_autosuspend: %d", udriver->supports_autosuspend);
        SPDEBUG("---disable_hub_initiated_lpm: %d", udriver->disable_hub_initiated_lpm);
        SPDEBUG("---soft_unbind: %d", udriver->soft_unbind);
    }
}

/* ===================================================================================================================*/
static void vPrintURB(struct urb *iurb)
{
    if (iurb)
    {
        SPDEBUG("-urb--------------");
        SPDEBUG("---pipe: 0x%x", iurb->pipe);
        SPDEBUG("---stream_id: %d", iurb->stream_id);
        SPDEBUG("---status: %d", iurb->status);
        SPDEBUG("---transfer_flags: %d", iurb->transfer_flags);
        SPDEBUG("---num_sgs: %d", iurb->num_sgs);
        SPDEBUG("---transfer_buffer_length: %d", iurb->transfer_buffer_length);
        SPDEBUG("---actual_length: %d", iurb->actual_length);
        SPDEBUG("---start_frame: %d", iurb->start_frame);
        SPDEBUG("---number_of_packets: %d", iurb->number_of_packets);
        SPDEBUG("---interval: %d", iurb->interval);
    }
}
/* ===================================================================================================================*/
static void vPrintUSB_HOST_INTERFACE(struct usb_host_interface *uhinterface)
{
    int i;
    struct usb_endpoint_descriptor endpoint;
    if(uhinterface)
    {
        SPDEBUG("-usb_host_interface--------------");
        SPDEBUG("---extralen: %d", uhinterface->extralen);
        SPDEBUG("---extra: %s", uhinterface->extra);
        SPDEBUG("---string: %s", uhinterface->string);
        for (i = 0; i < uhinterface->desc.bNumEndpoints; ++i)
        {
            endpoint = uhinterface->endpoint[i].desc;
            SPDEBUG("------endpoint%d.bLength: %d", i, endpoint.bLength);
            SPDEBUG("------endpoint%d.bEndpointAddress: 0x%x", i, endpoint.bEndpointAddress);
            SPDEBUG("------endpoint%d.bInterval: %d", i, endpoint.bInterval);
            SPDEBUG("------endpoint%d.bmAttributes: %d", i, endpoint.bmAttributes);
            SPDEBUG("------endpoint%d.bRefresh: %d", i, endpoint.bRefresh);
            SPDEBUG("------endpoint%d.bSynchAddress: 0x%x", i, endpoint.bSynchAddress);
            SPDEBUG("------endpoint%d.wMaxPacketSize: %d", i, endpoint.wMaxPacketSize);
        }
	}
}

/* ===================================================================================================================*/
static void vPrintUSB_INTERFACE(struct usb_interface *uinterface)
{
    if (uinterface)
    {
        SPDEBUG("-usb_interface--------------");
        SPDEBUG("---minor: %d", uinterface->minor);
        SPDEBUG("---sysfs_files_created: %s", uinterface->sysfs_files_created);
        SPDEBUG("---ep_devs_created: %d", uinterface->ep_devs_created);
        SPDEBUG("---unregistering: %s", uinterface->unregistering);
        SPDEBUG("---needs_remote_wakeup: %d", uinterface->needs_remote_wakeup);
        SPDEBUG("---resetting_device: %s", uinterface->resetting_device);
    }
}
/* ===================================================================================================================*/
static void vPrintUSB_DEVICE(struct usb_device *udev)
{
    if(udev)
    {
        SPDEBUG("-usb_device--------------");
        SPDEBUG("---devnum: %d", udev->devnum);
        SPDEBUG("---devpath: %s", udev->devpath);
        SPDEBUG("---route: %d", udev->route);
        SPDEBUG("---state: %d (usb_device_state)", udev->state);
        SPDEBUG("---speed: %d (usb_device_speed)", udev->speed);
        if (udev->tt)
        {
            SPDEBUG("---usb_tt->multi: %d", udev->tt->multi);
            SPDEBUG("---usb_tt->think_time: %d", udev->tt->think_time);
        }
        SPDEBUG("---descriptor.bLength: %d", udev->descriptor.bLength);
        SPDEBUG("---descriptor.bDescriptorType: %d", udev->descriptor.bDescriptorType);
        SPDEBUG("---descriptor.bcdUSB: 0x%x", udev->descriptor.bcdUSB);
        SPDEBUG("---descriptor.bDeviceClass: %d", udev->descriptor.bDeviceClass);
        SPDEBUG("---descriptor.bDeviceSubClass: %d", udev->descriptor.bDeviceSubClass);
        SPDEBUG("---descriptor.bDeviceProtocol: %d", udev->descriptor.bDeviceProtocol);
        SPDEBUG("---descriptor.bMaxPacketSize0: %d", udev->descriptor.bMaxPacketSize0);
        SPDEBUG("---descriptor.idVendor: 0x%x", udev->descriptor.idVendor);
        SPDEBUG("---descriptor.idProduct: 0x%x", udev->descriptor.idProduct);
        SPDEBUG("---descriptor.bcdDevice: %d", udev->descriptor.bcdDevice);
        SPDEBUG("---descriptor.iManufacturer: %d", udev->descriptor.iManufacturer);
        SPDEBUG("---descriptor.iProduct: %d", udev->descriptor.iProduct);
        SPDEBUG("---descriptor.iSerialNumber: %d", udev->descriptor.iSerialNumber);
        SPDEBUG("---descriptor.bNumConfigurations: %d", udev->descriptor.bNumConfigurations);
    }

}
/* ===================================================================================================================*/
static int __init xbdrv_init(void)
{
	int result;
	SPDEBUG("=============== LOAD KERNEL MODULE ==============");
	result = usb_register(&usb_xbdrvInfo);
	if (result)
	{
		PDEBUG("Registering driver failed");
	}
	else
	{
		PDEBUG("Registering of driver successfully");
		PDEBUG("VendorID: %x, ProductID: %x", VENDOR_ID, PRODUCT_ID);
		vPrintUSB_DRIVER(&usb_xbdrvInfo);
	}
	return result;
}

/* ===================================================================================================================*/
static void __exit xbdrv_exit(void)
{
	usb_deregister(&usb_xbdrvInfo);
    PDEBUG("Deregistering of driver successfully");
}

/* ===================================================================================================================*/
static void xb_int_in_callback(struct urb *urb)
{
    PDEBUG("IN INTERRUPT callback");
    vPrintURB(urb);
}

static void xb_ctrl_callback(struct urb *urb)
{
    PDEBUG("OUT CONTROL callback");
    vPrintURB(urb);
}
/** @brief The device open function that is called each time the device is opened
 *  This will only increment the u32NumbrOpens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int xbdev_open(struct inode *inodep, struct file *filep)
{
	struct usb_xbdevInfo *dev = NULL;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

    mutex_lock(&disconnect_mutex);
    subminor = iminor(inodep);
	interface = usb_find_interface(&usb_xbdrvInfo, subminor);
    dev = usb_get_intfdata(interface);
	down_interruptible (&dev->sem);
	++dev->open_count;
	usb_fill_int_urb(
                        dev->int_in_urb,
                        dev->udev,
                        usb_rcvintpipe(
                                        dev->udev,
                                        dev->int_in_endpoint->bEndpointAddress
                                        ),
                        dev->int_in_buffer,
                        le16_to_cpu(
                                    dev->int_in_endpoint->wMaxPacketSize
                                    ),
                        xb_int_in_callback,
                        dev,
                        dev->int_in_endpoint->bInterval
                    );
	dev->int_in_running = 1;
	mb();
	retval = usb_submit_urb(dev->int_in_urb, GFP_KERNEL);
	if (retval)
	{
		PDEBUG("submitting of IN_INT_UBR failed (%d)", retval);
	}
	else
	{
        PDEBUG("submitting of IN_INT_UBR sucessful");
	}
	filep->private_data = dev;
unlock_exit:
	up(&dev->sem);
    // ------ LOCKED SECTION END-------
exit:
	mutex_unlock(&disconnect_mutex);
	return retval;
}

/* ===================================================================================================================*/
/** @brief This function is called whenever device is being read from user space i.e. data is
 *  being sent from the device to the user. In this case is uses the copy_to_user() function to
 *  send the buffer string to the user and captures any errors.
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 *  @param buffer The pointer to the buffer to which this function writes the data
 *  @param len The length of the b
 *  @param offset The offset if required
 */
static ssize_t xbdev_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	PDEBUG("Read");
}

/* ===================================================================================================================*/
/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user. The data is copied to the message[] array in this
 *  LKM using the sprintf() function along with the length of the string.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */
static ssize_t xbdev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	struct usb_xbdevInfo *dev;
	int retval = 0;
    unsigned char buf[8];
    unsigned char u8Value;
	unsigned char cmd = 0x8;
	unsigned int u32sndpipe;

	dev = filep->private_data;
	u32sndpipe = usb_sndctrlpipe(dev->udev, 0);
	PDEBUG("u32sndpipe: 0x%x", u32sndpipe);
	down_interruptible(&dev->sem);
    memset(&buf, 0, sizeof(buf));
    buf[0] = 2;
    buf[1] = cmd;

    u8Value = *buffer;
	spin_lock(&dev->cmd_spinlock);
	dev->command = cmd;
	spin_unlock(&dev->cmd_spinlock);
	PDEBUG("CONTROL_MSG data=0x%x", *buffer);
	// Build a control urb, sends it off and waits for completion
	retval = usb_control_msg(
                                dev->udev,
                                u32sndpipe,
                                0x01,
                                0x21,
                                u8Value,
                                0x04,
                                &buf,
                                sizeof(buf),
                                HZ*5
                            );
    if (retval < 0)
    {
		PDEBUG("usb_control_msg failed (%d)", retval);
	}
	else
    {
		PDEBUG("usb_control_msg was sent the count of bytes: ", retval);
	}
unlock_exit:
	up(&dev->sem);
exit:
	return retval;
}
/* ===================================================================================================================*/
static void xb_abort_transfers(struct usb_xbdevInfo *dev)
{
	if (! dev) {
		PDEBUG("dev is NULL");
		return;
	}

	if (! dev->udev) {
		PDEBUG("udev is NULL");
		return;
	}

	if (dev->udev->state == USB_STATE_NOTATTACHED) {
		PDEBUG("udev not attached");
		return;
	}

	/* Shutdown transfer */
	if (dev->int_in_running) {
		dev->int_in_running = 0;
		mb();
		if (dev->int_in_urb)
			usb_kill_urb(dev->int_in_urb);
	}

	if (dev->ctrl_urb)
		usb_kill_urb(dev->ctrl_urb);
}
/* ===================================================================================================================*/
static inline void xb_delete(struct usb_xbdevInfo *dev)
{
	if (dev->int_in_urb)
		usb_free_urb(dev->int_in_urb);
	if (dev->ctrl_urb)
		usb_free_urb(dev->ctrl_urb);
	kfree(dev->int_in_buffer);
	kfree(dev->ctrl_buffer);
	kfree(dev->ctrl_dr);
	kfree(dev);
}
/* ===================================================================================================================*/
/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int xbdev_release(struct inode *inodep, struct file *filep)
{
	struct usb_xbdevInfo *dev = NULL;
	int retval = 0;

	dev = filep->private_data;
	if (!dev)
	{
		PDEBUG("dev is NULL");
		retval =  -ENODEV;
		goto exit;
	}
	if (down_interruptible(&dev->sem))
	{
		retval = -ERESTARTSYS;
		goto exit;
	}
	if (dev->open_count <= 0)
	{
		retval = -ENODEV;
		goto unlock_exit;
	}
	if (!dev->udev)
	{
		up (&dev->sem);
		xb_delete(dev);
		goto exit;
	}
	if (dev->open_count > 1)
		PDEBUG("open_count = %d", dev->open_count);
	xb_abort_transfers(dev);
	--dev->open_count;
unlock_exit:
	up(&dev->sem);

exit:
   PDEBUG("Device successfully released from user-space\n");
   return retval;
}



/* ===================================================================================================================*/
/* Called when a USB device is connected to the computer. */
static int xbdrv_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);  //get the kernel's representation of a USB device
	struct usb_xbdevInfo *dev = NULL;                          //repere my presentation of device
	struct usb_host_interface *iface_desc;                     //interface structures containing all of the alternate settings that may be selected for this interface
	struct usb_endpoint_descriptor *endpoint;                  //endpoint descriptors
	int i, int_end_size;
	int retval = -ENODEV;

	SPDEBUG("============= PROBE DEV ==============");
	if (!udev)
	{
		PDEBUG("udev is NULL");
		goto error;
	}
	dev = kzalloc(sizeof(struct usb_xbdevInfo), GFP_KERNEL);
	if (!dev)
	{
		PDEBUG("cannot allocate memory for struct usb_xbdevInfo");
		retval = -ENOMEM;
		goto error;
	}
	dev->command = 0x0;                     //init with 0 command
	sema_init(&dev->sem, 1);                //init the semaphore unlocked
	spin_lock_init(&dev->cmd_spinlock);     //init spinlock for command
	dev->udev = udev;                       //store usb device from kernel
	dev->interface = interface;             //store interface from kernel
	iface_desc = interface->cur_altsetting; //store alternate settings
	//find the interrupt endpoint
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i)
	{
		endpoint = &iface_desc->endpoint[i].desc;
		PDEBUG("[endpoint] number:%d, address: %x, attributes: %x", i, endpoint->bEndpointAddress, endpoint->bmAttributes);
		if ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_INT)
		{
            if ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT)
            {
                dev->int_out_endpoint = endpoint;    //store interrupt enpoint
                PDEBUG("[endpoint] endpoint out stored: number:%d, address: %x, attributes: %x, interval: %d", i, endpoint->bEndpointAddress, endpoint->bmAttributes, endpoint->bInterval);
            }
            if ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)
            {
                dev->int_in_endpoint = endpoint;    //store interrupt enpoint
                PDEBUG("[endpoint] endpoint in stored: number:%d, address: %x, attributes: %x, interval: %d", i, endpoint->bEndpointAddress, endpoint->bmAttributes, endpoint->bInterval);
            }
		}
	}
	// allocate the interrupt URB
	if (!dev->int_in_endpoint)
	{
		PDEBUG("could not find interrupt endpoint");
		retval = -ENOMEM;
	}
	else
	{
        int_end_size = le16_to_cpu(dev->int_in_endpoint->wMaxPacketSize);
        dev->int_in_buffer = kmalloc(int_end_size, GFP_KERNEL);
        if (! dev->int_in_buffer)
        {
            PDEBUG("could not allocate int_in_buffer");
            retval = -ENOMEM;
            goto error;
        }
        dev->int_in_urb = usb_alloc_urb(0, GFP_KERNEL);
        if (! dev->int_in_urb)
        {
            PDEBUG("could not allocate int_in_urb");
            retval = -ENOMEM;
            goto error;
        }
	}
	// allocate the control URB
    dev->ctrl_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->ctrl_urb)
	{
		PDEBUG("could not allocate ctrl_urb");
		retval = -ENOMEM;
		goto error;
	}
	else
	{
	    dev->ctrl_buffer = kzalloc(XB_CTRL_BUFFER_SIZE, GFP_KERNEL);
        if (!dev->ctrl_buffer)
        {
            PDEBUG("could not allocate ctrl_buffer");
            retval = -ENOMEM;
        }
        else
        {
            dev->ctrl_dr = kmalloc(sizeof(struct usb_ctrlrequest), GFP_KERNEL);
            if (!dev->ctrl_dr)
            {
                PDEBUG("could not allocate usb_ctrlrequest");
                retval = -ENOMEM;
            }
            else
            {
                dev->ctrl_dr->bRequestType = XB_CTRL_REQUEST_TYPE;
                dev->ctrl_dr->bRequest = XB_CTRL_REQUEST;
                dev->ctrl_dr->wValue = cpu_to_le16(XB_CTRL_VALUE);
                dev->ctrl_dr->wIndex = cpu_to_le16(XB_CTRL_INDEX);
                dev->ctrl_dr->wLength = cpu_to_le16(XB_CTRL_BUFFER_SIZE);
                usb_fill_control_urb(dev->ctrl_urb, dev->udev, usb_sndctrlpipe(dev->udev, 0),
                                        (unsigned char *)dev->ctrl_dr, dev->ctrl_buffer,
                                        XB_CTRL_BUFFER_SIZE, xb_ctrl_callback, dev);
            }
        }
	}
	if (!usb_string(udev, udev->descriptor.iSerialNumber, dev->serial_number, sizeof(dev->serial_number)))
    {
		PDEBUG("could not retrieve serial number");
    }
	usb_set_intfdata(interface, dev);                   //store the data to the file interface
	retval = usb_register_dev(interface, &xbdev_class); //register device class
	if (retval)
	{
		PDEBUG("Device class registration failed");
		usb_set_intfdata(interface, NULL);
	}
	else
	{
        dev->minor = interface->minor;              //store the minr number for the device
        PDEBUG("Device connected to /dev/xbdev%d\n", interface->minor - XBDEV_MINOR_BASE);
        //print debug informations
        vPrintUSB_HOST_INTERFACE(interface->cur_altsetting);
        vPrintUSB_INTERFACE(interface);
        vPrintUSB_DEVICE(udev);
        PDEBUG("URB (CONTROL)");
        vPrintURB(dev->ctrl_urb);
        PDEBUG("URB (IN,INTERRUPT)");
        vPrintURB(dev->int_in_urb);
	}
	return retval;
error:
	xb_delete(dev);
    PDEBUG("Device connection error\n");
	return retval;
}

/* called when unplugging a USB device. */
static void xbdrv_disconnect(struct usb_interface *interface)
{
	struct usb_xbdevInfo *dev = usb_get_intfdata(interface); //get the my device info from file
	int minor;

    mutex_lock(&disconnect_mutex);
	usb_set_intfdata(interface, NULL);                      //free the device in interface
	down(&dev->sem);
	minor = dev->minor;
	usb_deregister_dev(interface, &xbdev_class);            //free the device class so free minor

	if (! dev->open_count)                                  //free opened device
	{
		up(&dev->sem);
		xb_delete(dev);
	}
	else
	{
		dev->udev = NULL;
		up(&dev->sem);
	}
    mutex_unlock(&disconnect_mutex);
	PDEBUG("Device /dev/xbdev%d is disconnected", minor - XBDEV_MINOR_BASE);
}



/* ===================================================================================================================*/
/** @brief A module must use the module_init() module_exit() macros from linux/init.h, which
 *  identify the initialization function at insertion time and the cleanup function (as
 *  listed above)
 */
module_init(xbdrv_init);
module_exit(xbdrv_exit);

