/******************************************************************
 * uart.hh
 *
 *  Created on: Jan 8, 2016
 *      Author: root
 ******************************************************************/

#ifndef UART_HH_
#define UART_HH_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef __cplusplus

void UARTInterruptHdl(void);

#else
namespace tiboard
{

extern "C" void UARTInterruptHdl(void);
static const char * const kHex = "0123456789abcdef";

/*******************************************************************
 *	This cals represents the UART0 port of the M4 Microcontroler
 *******************************************************************/
class Uart
{
		static const uint32_t kSysCtlRCGCBASE = 0x400FE600;
		static const uint32_t kSysCtlPeriphGPIOA = 0xF0000800;
		static const uint32_t kSysCtlPeriphUART0 = 0xF0001800;

		static const uint32_t kUart0Base = 0x4000C000;
		static const uint32_t kUart0InterruptIdx = 21;
		static const uint32_t kUARTCCOffset = 0x00000FC8;
		static const uint32_t kUartClockPIOSC = 0x00000005;
		static const uint32_t kRxBufferMaxSize = 128;
		static const uint32_t kTxBufferMaxSize = 1024;


	public:
		Uart();
		virtual ~Uart();
		static void init(void);
		static void printf(const char *pcString, ...);
		static void stdio_interrupt_hdl(void);
	private:
		static uint32_t  m_u8RxWriteIdx;
		static uint32_t  m_u8RxReadIdx;
		static uint32_t  m_u8TxWriteIdx;
		static uint32_t  m_u8TxReadIdx;
		static uint8_t   m_pu8TxBuffer[kTxBufferMaxSize];

		static void init_stdio(void);
		static void flush_tx(void);
		static void flush_rx(void);
		static void prime_transmit(uint32_t ui32Base);
		static bool is_buffer_full(volatile uint32_t *pui32Read, volatile uint32_t *pui32Write, uint32_t ui32Size);
		static bool is_buffer_empty(volatile uint32_t *pui32Read, volatile uint32_t *pui32Write);
		static uint32_t write(const char *pcBuf, uint32_t ui32Len);
		static void vprintf(const char *pcString, va_list vaArgP);
};

} /* namespace lphw */

#endif /* __cplusplus */

#endif /* UART_HH_ */
