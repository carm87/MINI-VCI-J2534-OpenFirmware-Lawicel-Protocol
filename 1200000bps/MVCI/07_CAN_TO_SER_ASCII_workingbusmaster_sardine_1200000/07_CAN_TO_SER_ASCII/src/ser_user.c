

#include "datatypes.h"
#include "serial.h"
#include "ser_user.h"
#include "hardware_user.h"

// serial baudrate
#define	SER_BAUD		1500000


// TX Fifo (soft-fifo read by TX complete interrupt)
static u8_t  TxFifo[SER_TX_FIFO_SIZE];

// RX Fifo (soft-fifo write by RX complete interrupt)
static u8_t  RxFifo[SER_RX_FIFO_SIZE];



// Init the serial interface
void  SER_UserInit ( void)
{

	SERInit_t  setup;
	
	
	setup.prescaler = ( HW_CPU_CLOCK_HZ + 8 * SER_BAUD) / ( 16 * SER_BAUD);
	setup.databits = 8;
	setup.stopbits = 1;
	setup.parity = SER_PARITY_NONE;
	
	setup.pTxFifo = &TxFifo;
	setup.pRxFifo = &RxFifo;
	
	setup.TxFifoSize = SER_TX_FIFO_SIZE;
	setup.RxFifoSize = SER_RX_FIFO_SIZE;
	
	setup.ISRnum = 5;		// VIC channels 0 to 4 used for CAN
	
	SER_Initialize ( SER_PORT1, &setup);
	
}

