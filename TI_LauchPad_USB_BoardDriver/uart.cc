/******************************************************************
 * uart.cpp
 *
 *  Created on: Jan 8, 2016
 *      Author: root
 ******************************************************************/

#include <stdarg.h>

#include "uart.hh"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"


namespace tiboard
{
uint32_t  Uart::m_u8RxWriteIdx;
uint32_t  Uart::m_u8RxReadIdx;
uint32_t  Uart::m_u8TxWriteIdx;
uint32_t  Uart::m_u8TxReadIdx;
uint8_t   Uart::m_pu8TxBuffer[kTxBufferMaxSize];

Uart::~Uart()
{
	// TODO Auto-generated destructor stub
}
Uart::Uart()
{
	// TODO Auto-generated constructor stub
}
void Uart::init()
{
	// Enable the GPIO and UART Peripheral used by the UART.
	SysCtlPeripheralEnable(kSysCtlPeriphGPIOA);
    SysCtlPeripheralEnable(kSysCtlPeriphUART0);

    // Enable the GPIO, UART Peripherals (set the value of bit).
    HWREGBITW(kSysCtlRCGCBASE + ((kSysCtlPeriphGPIOA & 0xff00) >> 8), kSysCtlPeriphGPIOA & 0xff) = 1;
    HWREGBITW(kSysCtlRCGCBASE + ((kSysCtlPeriphUART0 & 0xff00) >> 8), kSysCtlPeriphUART0 & 0xff) = 1;

    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(kUart0Base, kUartClockPIOSC);

    // Initialize the UART for console I/O.
    init_stdio();
}
void Uart::init_stdio()
{
    // Check to make sure the UART peripheral is present.
	SysCtlPeripheralPresent(SYSCTL_PERIPH_UART0);
    // Enable the UART peripheral for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Configure the UART for 115200, n, 8, 1
    UARTConfigSetExpClk(UART0_BASE, 16000000, 115200,
                            (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_WLEN_8));
    // Set the UART to interrupt whenever the TX FIFO is almost empty or
    // when any character is received.
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    // Flush both the buffers.
    flush_rx();
    flush_tx();
    // We are configured for buffered output so enable the master interrupt
    // for this UART and the receive interrupts.  We don't actually enable the
    // transmit interrupt in the UART itself until some data has been placed
    // in the transmit buffer.
    UARTIntDisable(UART0_BASE, 0xFFFFFFFF);
    UARTIntEnable(kUart0Base, UART_INT_RX | UART_INT_RT);
    IntEnable(kUart0InterruptIdx);
    // Enable the UART operation.
    UARTEnable(UART0_BASE);
}
void Uart::flush_tx()
{
    uint32_t ui32Int;
    // Should the remaining data be discarded or transmitted?
    // The remaining data should be discarded, so temporarily turn off
    // interrupts.
    ui32Int = IntMasterDisable();
    // Flush the transmit buffer.
    m_u8TxWriteIdx = 0;
    m_u8TxReadIdx = 0;
    // If interrupts were enabled when we turned them off, turn them
    // back on again.
    if(!ui32Int)
    {
       IntMasterEnable();
    }
}
void Uart::flush_rx()
{
    uint32_t ui32Int;
    // Temporarily turn off interrupts.
    ui32Int = IntMasterDisable();
    // Flush the receive buffer.
    m_u8RxWriteIdx = 0;
    m_u8RxReadIdx = 0;
    // If interrupts were enabled when we turned them off, turn them
    // back on again.
    if(!ui32Int)
    {
        IntMasterEnable();
    }
}
bool Uart::is_buffer_full(volatile uint32_t *pui32Read, volatile uint32_t *pui32Write, uint32_t ui32Size)
{
    uint32_t ui32Write = *pui32Write;
    uint32_t ui32Read = *pui32Read;
    return((((ui32Write + 1) % ui32Size) == ui32Read) ? true : false);
}
bool Uart::is_buffer_empty(volatile uint32_t *pui32Read, volatile uint32_t *pui32Write)
{
    uint32_t ui32Write = *pui32Write;
    uint32_t ui32Read = *pui32Read;
    return((ui32Write == ui32Read) ? true : false);
}
void Uart::prime_transmit(uint32_t ui32Base)
{
    // Do we have any data to transmit?
    if(!(is_buffer_empty(&m_u8TxReadIdx, &m_u8TxWriteIdx)))
    {
        // Disable the UART interrupt.  If we don't do this there is a race
        // condition which can cause the read index to be corrupted.
        IntDisable(kUart0InterruptIdx);
        // Yes - take some characters out of the transmit buffer and feed
        // them to the UART transmit FIFO.
        while(UARTSpaceAvail(kUart0Base) && !(is_buffer_empty(&m_u8TxReadIdx, &m_u8TxWriteIdx)))
        {
            UARTCharPutNonBlocking(kUart0Base, m_pu8TxBuffer[m_u8TxReadIdx]);
            m_u8TxReadIdx = (m_u8TxReadIdx + 1) % kTxBufferMaxSize;
        }
        // Reenable the UART interrupt.
        IntEnable(kUart0InterruptIdx);
    }
}
uint32_t Uart::write(const char *pcBuf, uint32_t ui32Len)
{
    unsigned int uIdx;
    // Send the characters
    for(uIdx = 0; uIdx < ui32Len; uIdx++)
    {
        // If the character to the UART is \n, then add a \r before it so that
        // \n is translated to \n\r in the output.
        if(pcBuf[uIdx] == '\n')
        {
            if(!(is_buffer_full(&m_u8TxReadIdx, &m_u8TxWriteIdx, kTxBufferMaxSize)))
            {
            	m_pu8TxBuffer[m_u8TxWriteIdx] = '\r';
            	m_u8TxWriteIdx = (m_u8TxWriteIdx + 1) % kTxBufferMaxSize;
            }
            else
            {
                // Buffer is full - discard remaining characters and return.
                break;
            }
        }
        // Send the character to the UART output.
        if(!(is_buffer_full(&m_u8TxReadIdx, &m_u8TxWriteIdx, kTxBufferMaxSize)))
        {
        	m_pu8TxBuffer[m_u8TxWriteIdx] = pcBuf[uIdx];
        	m_u8TxWriteIdx = (m_u8TxWriteIdx + 1) % kTxBufferMaxSize;
        }
        else
        {
            // Buffer is full - discard remaining characters and return.
            break;
        }
    }
    // If we have anything in the buffer, make sure that the UART is set
    // up to transmit it.
    if(!(is_buffer_empty(&m_u8TxReadIdx, &m_u8TxWriteIdx)))
    {
    	prime_transmit(kUart0Base);
        UARTIntEnable(kUart0Base, UART_INT_TX);
    }
    // Return the number of characters written.
    return(uIdx);
}
void Uart::vprintf(const char *pcString, va_list vaArgP)
{
    uint32_t ui32Idx, ui32Value, ui32Pos, ui32Count, ui32Base, ui32Neg;
    char *pcStr, pcBuf[16], cFill;
    // Check the arguments.
    // Loop while there are more characters in the string.
    while(*pcString)
    {
        for(ui32Idx = 0; (pcString[ui32Idx] != '%') && (pcString[ui32Idx] != '\0');  ui32Idx++)
        {
        }
        write(pcString, ui32Idx);
        // Skip the portion of the string that was written.
        pcString += ui32Idx;
        // Skip the %.
        if(*pcString == '%')
        {
            pcString++;
            // Set the digit count to zero, and the fill character to space
            // (in other words, to the defaults).
            ui32Count = 0;
            cFill = ' ';
            // It may be necessary to get back here to process more characters.
            // Goto's aren't pretty, but effective.  I feel extremely dirty for
            // using not one but two of the beasts.
again:
            // Determine how to handle the next character.
            switch(*pcString++)
            {
                // Handle the digit characters.
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                {
                    // If this is a zero, and it is the first digit, then the
                    // fill character is a zero instead of a space.
                    if((pcString[-1] == '0') && (ui32Count == 0))
                    {
                        cFill = '0';
                    }
                    // Update the digit count.
                    ui32Count *= 10;
                    ui32Count += pcString[-1] - '0';
                    // Get the next character.
                    goto again;
                }
                // Handle the %c command.
                case 'c':
                {
                    // Get the value from the varargs.
                    ui32Value = va_arg(vaArgP, uint32_t);
                    // Print out the character.
                    write((char *)&ui32Value, 1);
                    // This command has been handled.
                    break;
                }
                // Handle the %d and %i commands.
                case 'd':
                case 'i':
                {
                    // Get the value from the varargs.
                    ui32Value = va_arg(vaArgP, uint32_t);
                    // Reset the buffer position.
                    ui32Pos = 0;
                    // If the value is negative, make it positive and indicate
                    // that a minus sign is needed.
                    if((int32_t)ui32Value < 0)
                    {
                        // Make the value positive.
                        ui32Value = -(int32_t)ui32Value;
                        // Indicate that the value is negative.
                        ui32Neg = 1;
                    }
                    else
                    {
                        // Indicate that the value is positive so that a minus
                        // sign isn't inserted.
                        ui32Neg = 0;
                    }
                    // Set the base to 10.
                    ui32Base = 10;
                    // Convert the value to ASCII.
                    goto convert;
                }
                // Handle the %s command.
                case 's':
                {
                    // Get the string pointer from the varargs.
                    pcStr = va_arg(vaArgP, char *);
                    // Determine the length of the string.
                    for(ui32Idx = 0; pcStr[ui32Idx] != '\0'; ui32Idx++)
                    {
                    }
                    // Write the string.
                    write(pcStr, ui32Idx);
                    // Write any required padding spaces
                    if(ui32Count > ui32Idx)
                    {
                        ui32Count -= ui32Idx;
                        while(ui32Count--)
                        {
                        	write(" ", 1);
                        }
                    }
                    // This command has been handled.
                    break;
                }
                // Handle the %u command.
                case 'u':
                {
                    // Get the value from the varargs.
                    ui32Value = va_arg(vaArgP, uint32_t);
                    // Reset the buffer position.
                    ui32Pos = 0;
                    // Set the base to 10.
                    ui32Base = 10;
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    ui32Neg = 0;
                    // Convert the value to ASCII.
                    goto convert;
                }
                // Handle the %x and %X commands.  Note that they are treated
                // identically; in other words, %X will use lower case letters
                // for a-f instead of the upper case letters it should use.  We
                // also alias %p to %x.
                case 'x':
                case 'X':
                case 'p':
                {
                    // Get the value from the varargs.
                    ui32Value = va_arg(vaArgP, uint32_t);
                    // Reset the buffer position.
                    ui32Pos = 0;
                    // Set the base to 16.
                    ui32Base = 16;
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    ui32Neg = 0;
                    // Determine the number of digits in the string version of
                    // the value.
convert:
                    for(ui32Idx = 1; (((ui32Idx * ui32Base) <= ui32Value) && (((ui32Idx * ui32Base) / ui32Base) == ui32Idx));
                        ui32Idx *= ui32Base, ui32Count--)
                    {
                    }
                    // If the value is negative, reduce the count of padding
                    // characters needed.
                    if(ui32Neg)
                    {
                        ui32Count--;
                    }
                    // If the value is negative and the value is padded with
                    // zeros, then place the minus sign before the padding.
                    if(ui32Neg && (cFill == '0'))
                    {
                        // Place the minus sign in the output buffer.
                        pcBuf[ui32Pos++] = '-';
                        // The minus sign has been placed, so turn off the
                        // negative flag.
                        ui32Neg = 0;
                    }
                    // Provide additional padding at the beginning of the
                    // string conversion if needed.
                    if((ui32Count > 1) && (ui32Count < 16))
                    {
                        for(ui32Count--; ui32Count; ui32Count--)
                        {
                            pcBuf[ui32Pos++] = cFill;
                        }
                    }
                    // If the value is negative, then place the minus sign
                    // before the number.
                    if(ui32Neg)
                    {
                        // Place the minus sign in the output buffer.
                        pcBuf[ui32Pos++] = '-';
                    }
                    // Convert the value into a string.
                    for(; ui32Idx; ui32Idx /= ui32Base)
                    {
                        pcBuf[ui32Pos++] = kHex[(ui32Value / ui32Idx) % ui32Base];
                    }
                    // Write the string.
                    write(pcBuf, ui32Pos);
                    // This command has been handled.
                    break;
                }
                // Handle the %% command.
                case '%':
                {
                    // Simply write a single %.
                	write(pcString - 1, 1);
                    // This command has been handled.
                    break;
                }
                // Handle all other commands.
                default:
                {
                    // Indicate an error.
                	write("ERROR", 5);
                    // This command has been handled.
                    break;
                }
            }
        }
    }
}
void Uart::stdio_interrupt_hdl(void)
{
    uint32_t ui32Ints;
    // Get and clear the current interrupt source(s)
    ui32Ints = UARTIntStatus(kUart0Base, true);
    UARTIntClear(kUart0Base, ui32Ints);
    // Are we being interrupted because the TX FIFO has space available?
    if(ui32Ints & UART_INT_TX)
    {
        // Move as many bytes as we can into the transmit FIFO.
    	prime_transmit(kUart0Base);
        // If the output buffer is empty, turn off the transmit interrupt.
        if(is_buffer_empty(&m_u8TxReadIdx, &m_u8TxWriteIdx))
        {
            UARTIntDisable(kUart0Base, UART_INT_TX);
        }
    }
}
void Uart::printf(const char *pcString, ...)
{
    va_list vaArgP;
    // Start the varargs processing.
    va_start(vaArgP, pcString);
    vprintf(pcString, vaArgP);
    // We're finished with the varargs now.
    va_end(vaArgP);
}

extern "C" void UARTInterruptHdl(void)
{
	Uart::stdio_interrupt_hdl();
}

} /* namespace lphw */
