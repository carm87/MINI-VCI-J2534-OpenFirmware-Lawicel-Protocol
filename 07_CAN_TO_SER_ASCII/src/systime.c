
#include "datatypes.h"
#include "systime.h"
#include "lpc21xx.h"



// wait for some micros
void	SYSTIME_wait_us ( u32_t  TimeMicroSec)
{

	if ( TimeMicroSec)
	{
		u32_t  TimeStart, TimeNow;


		TimeStart = SYSTIME_NOW;	

		do {
			TimeNow = SYSTIME_NOW;
		} while ( SYSTIME_DIFF ( TimeStart, TimeNow) < TimeMicroSec);
	}
}


// init timer 1 as systemtimer
void  SYSTIME_Init ( void)
{

	// Timer config, 1 µs Resolution

	// Timer halt
	T1TCR = 1 << 1;

	// Set Prescaler
	T1PR = 59;

	// no Matches
	T1MCR = 0;

	//no Capture
	T1CCR = 0;

	// no external Toggles
	T1EMR = 0;

	// Clear all interrupts
	// we do not use interrupts
	T1IR = 0xFF;

	// Timer start
	T1TCR = 1;

}

