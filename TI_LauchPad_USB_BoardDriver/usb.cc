/******************************************************************
 * Usb.cc
 *
 *  Created on: Jan 8, 2016
 *      Author: root
 ******************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "usb.hh"
#include "uart.hh"

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/usb.h"
#include "inc/hw_memmap.h"
#include "usblib/usblib.h"
#include "usblib/device/usbdevice.h"

namespace tiboard
{

//*****************************************************************************
// The languages supported by this device.
//*****************************************************************************
const uint8_t g_pu8LangDescriptor[] =
{
    4,
    USB_DTYPE_STRING,
    USBShort(USB_LANG_EN_US)
};

//*****************************************************************************
// The manufacturer string.
//*****************************************************************************
const uint8_t g_pu8ManufacturerString[] =
{
    (14 + 1) * 2,
    USB_DTYPE_STRING,
    'X', 0, 'b', 0, 'M', 0, 'a', 0, 'n', 0, 'u', 0, 'f', 0, 'a', 0, 'c', 0,
    't', 0, 'u', 0, 'r', 0, 'e', 0, 'r', 0
};

//*****************************************************************************
// The product string.
//*****************************************************************************
const uint8_t g_pu8ProductString[] =
{
    (6 + 1) * 2,
    USB_DTYPE_STRING,
    'X', 0, 'b', 0, 'o', 0, 'a', 0, 'r', 0, 'd', 0
};

//*****************************************************************************
// The serial number string.
//*****************************************************************************
const uint8_t g_pu8SerialNumberString[] =
{
    (8 + 1) * 2,
    USB_DTYPE_STRING,
    '2', 0, '7', 0, '1', 0, '0', 0, '1', 0, '9', 0, '8', 0, '3', 0
};

//*****************************************************************************
// The data interface description string.
//*****************************************************************************
const uint8_t g_pu8DataInterfaceString[] =
{
    (11 + 1) * 2,
    USB_DTYPE_STRING,
    'X', 0, 'b', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0,
    'a', 0, 'c', 0, 'e', 0
};

//*****************************************************************************
// The configuration description string.
//*****************************************************************************
const uint8_t g_pu8ConfigString[] =
{
    (15 + 1) * 2,
    USB_DTYPE_STRING,
    'X', 0, 'b', 0, 'C', 0, 'o', 0, 'n', 0, 'f', 0, 'i', 0, 'g', 0,
    'u', 0, 'r', 0, 'a', 0, 't', 0, 'i', 0, 'o', 0, 'n', 0
};

#define NUM_STRING_DESCRIPTORS (sizeof(g_ppu8StringDescriptors) / sizeof(uint8_t *))

//*****************************************************************************
// String descriptors table
//*****************************************************************************
const uint8_t *const g_ppu8StringDescriptors[] =
{
    g_pu8LangDescriptor,
    g_pu8ManufacturerString,
    g_pu8ProductString,
    g_pu8SerialNumberString,
    g_pu8DataInterfaceString,
    g_pu8ConfigString
};

//*****************************************************************************
// Configuration descriptor header.
//*****************************************************************************
const uint8_t g_pu8HeaderDescriptor[9] =
{
    9,                          	//1 	UCHAR(bLength)				Size of the configuration descriptor.
    USB_DTYPE_CONFIGURATION,    	//2 	UCHAR(bDescriptorType)		Type of this descriptor.
    USBShort(32),               	//3,4 	USHORT(wTotalLength)		The total size of this full structure.
    1,                          	//5 	UCHAR(bNumInterfaces)		The number of interfaces in this configuration.
    1,                          	//6 	UCHAR(bConfigurationValue)	The unique value for this configuration.
    5,                          	//7 	UCHAR(iConfiguration)		The string identifier that describes this configuration.
    USB_CONF_ATTR_SELF_PWR,     	//8 	UCHAR(bmAttributes)			Bus Powered, Self Powered, remote wake up.
    100,                        	//9 	UCHAR(MaxPower)				The maximum power in 2mA increments.
};

//*****************************************************************************
// Vendor-specific Interface Descriptor.
//*****************************************************************************
const uint8_t g_pu8Interface[9] =
{
    9,                              //1 	UCHAR(bLength) 				Size of the interface descriptor.
    USB_DTYPE_INTERFACE,            //2 	UCHAR(bDescriptorType)		Type of this descriptor.
    0,                              //3 	UCHAR(bInterfaceNumber)		The index for this interface.
    0,                              //4 	UCHAR(bAlternateSetting)	The alternate setting for this interface.
    2,                              //5 	UCHAR(bNumEndpoints)		The number of endpoints used by this interface.
    USB_CLASS_VEND_SPECIFIC,        //6 	UCHAR(bInterfaceClass)		The interface class
    0,                              //7 	UCHAR(bInterfaceSubClass)	The interface sub-class.
    0,                              //8 	UCHAR(bInterfaceProtocol)	The interface protocol for the sub-class specified above (0 = None).
    4,                              //9 	UCHAR(iInterface)			The string index for this interface.
};

//*****************************************************************************
// Endpoint In Descriptor
//******************************Uart::stdio_interrupt_hdl***********************************************
const uint8_t g_pu8EndpointIn[7] =
{
    7,                              			//1 	UCHAR(bLength)				The size of the endpoint descriptor (in bytes).
    USB_DTYPE_ENDPOINT,             			//2 	UCHAR(bDescriptorType)		Descriptor type is an endpoint.
    USB_EP_DESC_IN | USBEPToIndex(USB_EP_1),	//3 	UCHAR(bEndpointAddress)		Endpoint address
    USB_EP_ATTR_INT,                			//4 	UCHAR(bmAttributes)			Endpoint is a interrupt endpoint.
	USBShort(32),  								//5,6 	USHORT(wMaxPacketSizeThe) 	maximum packet size.
    1,                              			//7 	UCHAR(bInterval)			The polling interval for this endpoint.
};

//*****************************************************************************
// Endpoint Out Descriptor
//*****************************************************************************
const uint8_t g_pu8EndpointOut[7] =
{
    7,                              			//1 	UCHAR(bLength)				The size of the endpoint descriptor (in bytes).
    USB_DTYPE_ENDPOINT,             			//2 	UCHAR(bDescriptorType)		Descriptor type is an endpoint.
    USB_EP_DESC_OUT | USBEPToIndex(USB_EP_1),  	//3 	UCHAR(bEndpointAddress)		Endpoint address
    USB_EP_ATTR_INT,                			//4 	UCHAR(bmAttributes)			Endpoint is a interrupt endpoint.
	USBShort(32),  								//5,6 	USHORT(wMaxPacketSizeThe) 	maximum packet size.
    1,                              			//7 	UCHAR(bInterval)			The polling interval for this endpoint.
};

//*****************************************************************************
// The bulk configuration descriptor is defined as two sections, one
// containing just the 9 byte USB configuration descriptor and the other
// containing everything else that is sent to the host along with it.
//*****************************************************************************
const tConfigSection g_sHeaderSection =
{
    sizeof(g_pu8HeaderDescriptor),
	g_pu8HeaderDescriptor
};

const tConfigSection g_sInterfaceSection =
{
    sizeof(g_pu8Interface),
    g_pu8Interface
};

const tConfigSection g_sEndpointInSection =
{
    sizeof(g_pu8EndpointIn),
	g_pu8EndpointIn
};

const tConfigSection g_sEndpointOutSection =
{
    sizeof(g_pu8EndpointOut),
	g_pu8EndpointOut
};

//*****************************************************************************
// This array lists all the sections that must be concatenated to make a
// single, complete bulk device configuration descriptor.
//*****************************************************************************
const tConfigSection *g_psSections[] =
{
    &g_sHeaderSection,
    &g_sInterfaceSection,
	&g_sEndpointInSection,
	&g_sEndpointOutSection
};

//*****************************************************************************
// The header for the single configuration we support.  This is the root of
// the data structure that defines all the bits and pieces that are pulled
// together to generate the configuration descriptor.
//*****************************************************************************
const tConfigHeader g_sConfigHeader =
{
	(sizeof(g_psSections) / sizeof(g_psSections[0])),
	g_psSections
};

//*****************************************************************************
// Configuration Descriptor.
//*****************************************************************************
const tConfigHeader * const g_ppConfigDescriptors[] =
{
    &g_sConfigHeader
};

const uint8_t g_pu8DeviceDescriptor[] =
{
	18,                    					//1 		UCHAR(bLength)				Size of this structure.
	USB_DTYPE_DEVICE,                    	//2 		UCHAR(bDescriptorType)		Type of this structure.
	USBShort(0x110),                    	//3,4 		USHORT(bcdUSB)				USB version 1.1 (if we say 2.0, hosts assume high-speed - see USB 2.0 spec 9.2.6.6)
	USB_CLASS_DEVICE,                    	//5 		UCHAR(bDeviceClass)			USB Device Class
	0,                    					//6 		UCHAR(bDeviceSubClass)		USB Device Sub-class
	0,                    					//7 		UCHAR(bDeviceProtocol)		USB Device protocol
	64,                    					//8 		UCHAR(bMaxPacketSize0)		Maximum packet size for default pipe.
	USBShort(0x1234),                    	//9,10 		USHORT(idVendor)			Vendor ID (VID).
	USBShort(0x0001),                    	//11,12		USHORT(idProduct)			Product ID (PID).
	USBShort(0x100),                    	//13,14 	USHORT(bcdDevice)			Device Version BCD.
	1,                    					//15	 	UCHAR(iManufacturer)		Manufacturer string identifier.
	2,                    					//16	 	UCHAR(iProduct)				Product string identifier.
	3,                    					//17	 	UCHAR(iSerialNumber)		Product serial number.
	1, };                 					//18	 	UCHAR(bNumConfigurations)	Number of configurations.


//*****************************************************************************
// Device event handler callbacks.
//*****************************************************************************
uint8_t Usb::u8Value = 0x02;

void Usb::vRequestIntHandler(void * pvInstance, tUSBRequest *pUSBRequest)
{
	Uart::printf("bmRequestType=0x%x, bRequest=0x%x, wValue=0x%x\n", pUSBRequest->bmRequestType, pUSBRequest->bRequest, pUSBRequest->wValue);
	if (pUSBRequest->bmRequestType == 0x21)
	{
		u8Value = pUSBRequest->wValue;
	}
}

uint8_t Usb::get_value()
{
	return(u8Value);
}

const tCustomHandlers g_sXbHandlers =
{
    0,												// GetDescriptor
	(tStdRequest)Usb::vRequestIntHandler,			// RequestHandler
    0,												// InterfaceChange
    0,												// ConfigChange
	0,												// DataReceived
    0,												// DataSentCallback
	0,												// ResetHandler
	0,												// SuspendHandler
    0,												// ResumeHandler
    0,												// DisconnectHandler
	0,												// EndpointHandler
	0												// Device handler
};


Usb::Usb()
{
	// TODO Auto-generated constructor stub
	
}

Usb::~Usb()
{
	// TODO Auto-generated destructor stub
}

tDeviceInfo g_XbDeviceInfo =
{
	(const tCustomHandlers *)		&g_sXbHandlers,				//1 psCallbacks
	(const uint8_t *)				g_pu8DeviceDescriptor,		//2 psDeviceDescriptors
	(const tConfigHeader * const *)	g_ppConfigDescriptors, 	//3 ppui8ConfigDescriptor
	(const uint8_t * const *)		g_ppu8StringDescriptors,	//4 ppui8StringDescriptor
	(uint32_t) 						NUM_STRING_DESCRIPTORS		//5 ui32NumStringDescriptors
};

void Usb::init()
{
	// Set the clocking to run from the PLL at 50MHz
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	// Enable the GPIO peripheral used for USB, and configure the USB pins
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeUSBAnalog(GPIO_PORTD_AHB_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    USBStackModeSet(0, eUSBModeForceDevice, 0);
    USBDCDInit(0, &g_XbDeviceInfo, 0x00);
}

extern "C" void USBInterruptHdl(void)
{
	USB0DeviceIntHandler();
}

} /* namespace lphw */
