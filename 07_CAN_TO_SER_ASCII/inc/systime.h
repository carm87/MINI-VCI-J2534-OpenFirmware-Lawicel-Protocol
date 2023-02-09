
#ifndef  _SYSTIME_H_
#define  _SYSTIME_H_


// get the current time
#define	SYSTIME_NOW		T1TC

// max for this MCU
#define	SYSTIME_MAX		0xFFFFFFFF

// calc a timediff
#define	SYSTIME_DIFF( _First, _Second)		(( _First <= _Second ? ( _Second - _First) : (( SYSTIME_MAX - _First) + _Second)))

// this is our basetype for time calculations
#define	SYSTIME_t	u32_t


// function protos

void	SYSTIME_wait_us ( u32_t  TimeMicroSec);


void  SYSTIME_Init ( void);


#endif

