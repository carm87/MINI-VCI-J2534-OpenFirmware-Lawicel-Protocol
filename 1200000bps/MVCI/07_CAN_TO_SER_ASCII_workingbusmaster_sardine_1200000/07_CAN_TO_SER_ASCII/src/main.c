
#include "datatypes.h"
#include "can.h"
#include "can_user.h"
#include "hardware.h"
#include "crc_data.h"
#include "systime.h"
#include "serial.h"
#include "ser_user.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lpc21xx.h"

// size for user text buffer received from partner like terminal program or other device
#define	USER_TEXT_SIZE		30


#define ret_OK    0xD // [CR]   "\r"
#define ret_ERROR 0x7 // [Bell] "\a"

// max. serial buffer length: 29bit-frame plus timestamp plus \r plus trailing zero
#define buflen (sizeof("T1FFFFFFF81122334455667788EA5F\r")+1)
u8_t	SerXmtBuf[buflen]; // will assemble and send data to the Host-PC / RS232
u8_t	SerXmtBufPtr = 0;
u8_t	SerRcvBuf[buflen]; // holds commands from the Host-PC / RS232
u8_t	SerRcvBufPtr = 0;
CANMsg_t  CanRxMsg, CanTxMsg;
u32_t CanSpeed=6;
void ProcessMsgFromSerial(void);
static void TransmitStandardFrame(void) ;
static void TransmitExtendedFrame(void) ;
u8_t ascii2hex(u8_t chr);
static void SetCanBTR0BTR1(void) ;
static void TransmitStandardRtr(void) ;
static void TransmitExtendedRtr(void) ;
static void GetStatusFlags(void) ;
static void SetAutoPollAutoSend(void) ;
static void SetFilterMode(void) ;
static void GetVersionInfo(void) ;
static void GetSerialNumber(void) ;
static void SetRcvTimestampMode(void) ;
void ProcessMsgFromCan(void) ;
u8_t hex2ascii(u8_t chr);
extern CANStatus_t  CAN_InitChannelMio (	CANHandle_t  hBus, u32_t Timing);


u16_t Millitimer = 0;
u32_t TimeDiff1000Hz = 0;
u32_t LedTimeDiff = 0;



// variables for LED toggle
static u8_t LED_toggleCAN1;

// variabili di  stato
u8_t CanChnOpen=0;
u8_t CanInitialized=0;
u8_t CanBusMode = BUS_OFF;
u8_t AutostartMode=0;
u8_t AutoPollAutoSend=1;
u8_t FilterMode=0;
u8_t CanRcvTimestampOn=0;



u8_t SerRxOverrunOccurred=0;
u8_t SerTxCongestionOccurred=0;
// Write to UART was either ok or TX queue was full (repeat in latter case)
SERStatus_t UARTWriteResult;

u8_t hex2ascii(u8_t chr)
{
	chr = chr & 0xF;
	if(chr > 9)
	{
		chr += 0x37;
	}
	else
	{
		chr += 0x30;
	}
	return chr;
}

u8_t ascii2hex(u8_t chr)
{
	if((chr <= 0x5A) && (chr >= 0x41)) // Großbuchstaben
	{
		chr -= 0x37;
		return chr &0xF;
	}
	if((chr <= 0x7A) && (chr >= 0x61)) // Kleinbuchstaben
	{
		chr -=0x57;
		return chr &0xF;
	}
	
	if((chr <= 0x39) && (chr >= 0x30)) // Zahlen
	{
		chr -= 0x30;
	}
	return chr & 0xF;
}





// main()
// entry point from crt0.S
int  main ( void)
{
	// init hardware
	HW_Init();
	IODIR1 |= 0x00100000;                     /* P1.16..23 defined as Outputs  */
	// init timer
	SYSTIME_Init();
	IOSET1 = 0x00100000;	//led on
	// init serial
	SER_UserInit();
	// wait 180 millis for CAN DC/DC converters
	SYSTIME_wait_us ( 180000);
	// init CAN
	CAN_UserInit();
	CAN_SetBusMode ( CAN_BUS1, BUS_ON);
	CanChnOpen = 1;
	CanBusMode = BUS_ON;
	SYSTIME_wait_us ( 180000);
	IOCLR1 = 0x00100000;	//led off
	// Set green LEDs for CAN1 and CAN2
//	HW_SetLED ( HW_LED_CAN1, HW_LED_GREEN);
//	HW_SetLED ( HW_LED_CAN2, HW_LED_GREEN);

	// main loop
	while ( 1)
	{

		
		// millisec timer for CAN receive timestamp
		if (SYSTIME_DIFF (TimeDiff1000Hz, SYSTIME_NOW) > 1000){
//			#if testtimer
//			if(SYSTIME_DIFF (TimeDiff1000Hz, SYSTIME_NOW) > 2000)
//			{
//				system_too_slow++;
//			}
//			#endif
			Millitimer++;
//			Millitimer+=(SYSTIME_DIFF (TimeDiff1000Hz, SYSTIME_NOW)/1000);
			if(Millitimer>60000) Millitimer=0;
			//TimeDiff1000Hz+=1000;
			TimeDiff1000Hz=SYSTIME_NOW;
		}
		//-------------------------------------------------

		// 1Hz LED Blinker
		if (SYSTIME_DIFF (LedTimeDiff, SYSTIME_NOW) > 500000){
			if (CanChnOpen) {
				// toggle LED
				LED_toggleCAN1 ^= 1;
			if ( LED_toggleCAN1)
			{	IOSET1 = 0x00100000;		}
			else
			{	IOCLR1 = 0x00100000;}
			}//IF: CAN chan. open
			else { // CAN chn. closed, set green static
				IOSET1 = 0x00100000;
			}
			LedTimeDiff = SYSTIME_NOW;
		}//IF: SYSTIME_DIFF
		
		
		ProcessMsgFromCan();

		ProcessMsgFromSerial();
		
		
		
	}
}




void ProcessMsgFromSerial(void)
//-------------------------------------------------
{
	SERStatus_t ReadResult;
	u8_t CurRcvChar; 	// currently received character from UART
	u8_t BytesRead;		// dummy parameter, not used here
	u8_t CmdLength;		// length of currently received Command
	u8_t RetValue;		// 

	do {
		// Read one byte from serial
		ReadResult = SER_Read (SER_PORT1, &CurRcvChar, 1, &BytesRead);
		
		// No bytes received,
		if (ReadResult == SER_ERR_RX_EMPTY){
			return;	 // ... quit, don't waste time
		}	

		//-------------------------------------------------
	    else if (ReadResult == SER_ERR_OK){
			SerRcvBuf[SerRcvBufPtr] = CurRcvChar;
		
			// increase pointer up to max.
			if (SerRcvBufPtr <= buflen){
				SerRcvBufPtr++;
			}
		} // else if
		//-------------------------------------------------
		else if (ReadResult == SER_ERR_RX_OVERRUN){
			SerRxOverrunOccurred = 1; // to be inserted in status register (command F)
		} // else if
		//-------------------------------------------------
	} // do
	while (CurRcvChar != '\r');	
	CmdLength = SerRcvBufPtr - 1; // without carriage return
		//-------------------------------------------------
	
	RetValue = ret_OK;
	switch (SerRcvBuf[0]) { // Char 0 always contains cmd
		//-------------------------------------------------
	case ('S'): // Set CAN speed 0..8
		if ((CmdLength == 2) 
		&&  (ascii2hex(SerRcvBuf[1]) <= 8)&&(CanChnOpen==0)) {
			CanSpeed = ascii2hex(SerRcvBuf[1]);
			CAN_InitChannelMio (CAN_BUS1, CanSpeed);
			CanInitialized = 1;
			// Send ASCII OK
			SER_Write (SER_PORT1, &RetValue, 1);
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('s'): // Setup CAN by BTR0/BTR1
		if (CmdLength == 5) {
			SetCanBTR0BTR1();
		//	EepromData.CanSpeed = 0xFF; 	// not a standard speed
		//	WriteToEeprom(0); 				// 0 = write current settings to EEPROM
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('O'): // Open CAN channel in normal mode
		if ((CmdLength == 1)&& (CanChnOpen == 0) && (CanInitialized == 1)) {
			CAN_ReInitChannel ( CAN_BUS1);
			CAN_SetBusMode ( CAN_BUS1, BUS_ON);
			CanChnOpen = 1;
			CanBusMode = BUS_ON;
			// Send ASCII OK
			SER_Write (SER_PORT1, &RetValue, 1);
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('Y'): // Open CAN channel in normal mode with autopolling
		if ((CmdLength == 1)&& (CanChnOpen == 0) && (CanInitialized == 1)) {
			AutoPollAutoSend=1;
			CAN_ReInitChannel ( CAN_BUS1);
			CAN_SetBusMode ( CAN_BUS1, BUS_ON);
			CanChnOpen = 1;
			CanBusMode = BUS_ON;
			// Send ASCII OK
			SER_Write (SER_PORT1, &RetValue, 1);
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('L'): // Open CAN channel in listen-only mode
		if ((CmdLength == 1)&& (CanChnOpen == 0) && (CanInitialized == 1)) {
			CAN_ReInitChannel ( CAN_BUS1);
			CAN_SetBusMode ( CAN_BUS1, BUS_LOM);
			CanChnOpen = 1;
			CanBusMode =BUS_LOM;
						// Send ASCII OK
			SER_Write (SER_PORT1, &RetValue, 1);
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('C'): // Close CAN channel
		if ((CmdLength == 1)) {
			CAN_SetBusMode ( CAN_BUS1, BUS_OFF);
			CanChnOpen = 0;
			CanBusMode = BUS_OFF;
			// Send ASCII OK
			SER_Write (SER_PORT1, &RetValue, 1);
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('t'): // Send 11bit CAN message
		if ((CmdLength == 5 + (ascii2hex(SerRcvBuf[4])* 2)&&  (CanChnOpen == 1) 
		&&  (CanBusMode == BUS_ON))  ) {
			TransmitStandardFrame();
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('T'): // Send 29bit CAN message
				if ((CmdLength == 10 + (ascii2hex(SerRcvBuf[9])* 2)) 
		&&  (CanChnOpen == 1) 
		&&  (CanBusMode == BUS_ON)) {
			TransmitExtendedFrame();
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('r'):// Send 11bit RTR
		if ((CmdLength == 5) 
		&&  (CanChnOpen == 1) 
		&&  (CanBusMode == BUS_ON)) {
			TransmitStandardRtr();
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('R'): // Send 29bit RTR
		if ((CmdLength == 10) 
		&&  (CanChnOpen == 1) 
		&&  (CanBusMode == BUS_ON)) {
			TransmitExtendedRtr();
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('P'):
		//	PollIncomingFifoSingle(); // not supported, use AutoPollAutoSend instead !
		RetValue = ret_ERROR;
		
		//-------------------------------------------------
		break;
		//-------------------------------------------------
	case ('A'):
		//	PollIncomingFifoAll(); // not supported, use AutoPollAutoSend instead !
		RetValue = ret_ERROR;
		//-------------------------------------------------
		break;
		//-------------------------------------------------
	case ('F'):
		if (CmdLength == 1) {
			GetStatusFlags(); // returns 1 byte with 7 flags
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('X'): // activate AutoPollAutoSend feature
		if ((CmdLength == 2) 
		&&  (ascii2hex(SerRcvBuf[1]) <= 1)) {
			SetAutoPollAutoSend();
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('D'): // single or dual filter
		if ((CmdLength == 2) )
		 {
			// Send ASCII OK
			SER_Write (SER_PORT1, &RetValue, 1);
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('W'): // Dual or Single Filter
		if ((CmdLength == 2) 
		&&  (ascii2hex(SerRcvBuf[1]) <= 1)) {
			SetFilterMode();
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('m'): // Register AM0, AM1, AM2 & AM3
		if (CmdLength == 9) {
		//	SetAcceptanceMask();
			RetValue=ret_OK;
			SER_Write (SER_PORT1, &RetValue, 1);
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('M'): // Register AC0, AC1, AC2 & AC3
		if (CmdLength == 9) {
		//	SetAcceptanceCode();
		RetValue=ret_OK;
		SER_Write (SER_PORT1, &RetValue, 1);
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('U'): // Set UART speed
		if ((CmdLength == 2) 
		&&  (ascii2hex(SerRcvBuf[1])) <= 6) {
		//	EepromData.SerialSpeed = ascii2hex(SerRcvBuf[1]);
		//	SetUartSpeed(EepromData.SerialSpeed); // u8_t SerSpeed
		//	WriteToEeprom(0); 				// 0 = write current settings to EEPROM
			RetValue=ret_ERROR;
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('V'): // request version HW+SW
		if (CmdLength == 1) {
			GetVersionInfo();
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
			case ('v'): // request version HW+SW
		if (CmdLength == 1) {
			GetVersionInfo();
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('N'): // request serial number
		if (CmdLength == 1) {
			GetSerialNumber();
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('Z'): // activate timestamp feature
		if ((CmdLength == 2) 
		&&  (ascii2hex(SerRcvBuf[1]) <= 1)) {
			SetRcvTimestampMode(); // u8_t TimestampMode
		//	WriteToEeprom(0); 				// 0 = write current settings to EEPROM
		}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('Q'): // Set Auto Startup feature
		if ((CmdLength == 2) 
		&&  (ascii2hex(SerRcvBuf[1]) <= 2)) {
		//	SetAutostartMode();
		//	WriteToEeprom(0); 				// 0 = write current settings to EEPROM
		
		RetValue=ret_ERROR;
			}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	case ('e'): // Write operating parameter set to EEPROM
		if ((CmdLength == 2) 
		&&  (ascii2hex(SerRcvBuf[1]) <= 2)) {
		//	WriteToEeprom(ascii2hex(SerRcvBuf[1])); // for debug only
		RetValue=ret_ERROR;
			}
		else {
			RetValue=ret_ERROR; // command wrong length or parameter out of range 
		}
		break;
		//-------------------------------------------------
	default:
			// received command not specified, do nothing, return BELL 
			RetValue = ret_ERROR;
		break;
	}
	//-------------------------------------------------

	if (RetValue == ret_ERROR) {
		// then blink red once
	//	HW_SetLED ( HW_LED_CAN1, HW_LED_RED);
	//	LedTimeDiff = SYSTIME_NOW;

		// Send ASCII string to RS232 UART (Host-PC)
		SerXmtBuf[0] = RetValue; // BEL
		SerXmtBufPtr = 1;
		SER_Write (SER_PORT1, &SerXmtBuf, SerXmtBufPtr);
	}
	
	SerRcvBufPtr = 0;
}
//-------------------------------------------------




static void TransmitStandardFrame(void) 
//-------------------------------------------------
{
	u8_t Result;
	u8_t i;			// all purpose loop counter

	Result = ret_OK;

	// if CAN channel is open and -not- listen only
		if ((CanChnOpen == 1) 
	&&  (CanBusMode == BUS_ON)
	&&  (AutostartMode == 0)) {

		// Erase former CAN data bytes
		CanTxMsg.Data32[0] = 0;
		CanTxMsg.Data32[1] = 0;
	
		// prepare 11 bit ID
		CanTxMsg.Id =                 (ascii2hex(SerRcvBuf[1])<<8 )& 0xF00;
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[2])<<4 )& 0x0F0));
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[3])    )& 0x00F));
		
		// Set message type 11 bit
		CanTxMsg.Type = CAN_MSG_STANDARD;
	
		// set DLC
		CanTxMsg.Len = (ascii2hex(SerRcvBuf[4]))& 0x00F;
		
		// copy message data
		for (i=0;i<(CanTxMsg.Len);i++) {
			CanTxMsg.Data8[i] = (ascii2hex(SerRcvBuf[5+2*i])<<4 )& 0x0F0;
			CanTxMsg.Data8[i] = (CanTxMsg.Data8[i] | ((ascii2hex(SerRcvBuf[i*2+6]))& 0x00F));
		}
		
		// send the message
		CAN_UserWrite (CAN_BUS1, &CanTxMsg);
	}
	else {
		Result = ret_ERROR;
	}
	SerXmtBufPtr = 0;
	// return string to RS232 UART (Host-PC)
	if (AutoPollAutoSend == 1) {
		if (Result == ret_OK) {
			SerXmtBuf[SerXmtBufPtr] = 'Z';
			SerXmtBufPtr++;
		}
	}
	SerXmtBuf[SerXmtBufPtr] = Result;
	SerXmtBufPtr++;
	
	// Send ASCII string to RS232 UART (Host-PC)
	SER_Write (SER_PORT1, &SerXmtBuf, SerXmtBufPtr);

}
/**************************************************/







//*************************************************
//! @brief	Tiiiiiiiildd...[CR]
//! Transmit an extended (29bit) CAN frame.
//!
//! @pre This command is accepted only if the CAN channel is open in normal mode. 
//!
//! @param	iiiiiiii Identifier in hex (00000000-1FFFFFFF)
//! @param	l Data length (0-8)
//! @param	dd Byte value in hex (00-FF). 
//! Numbers of dd pairs must match data length, otherwise an error occurs.
//!
//! @note example: T0000010021133[CR]
//! @note Sends a 29bit CAN frame with ID=0x100, 2 bytes
//! with the value 0x11 and 0x33.
//!
//! @return If Auto Poll is disabled:
//! @return	BELL (Ascii 7) for ERROR when CAN channel is not open or Listen-Only
//! @return	CR (Ascii 13) for OK
//!
//! @return If Auto Poll is enabled:
//! @return	BELL (Ascii 7) for ERROR when CAN channel is not open or Listen-Only
//! @return	z[CR] for OK
//!
static void TransmitExtendedFrame(void) 
//-------------------------------------------------
{
	u8_t Result;
	u8_t i;			// all purpose loop counter

	Result = ret_OK;

	// if CAN channel is open and -not- listen only
		if ((CanChnOpen == 1) 
	&&  (CanBusMode == BUS_ON) 
	&&  (AutostartMode == 0)) {

		// Erase former CAN data bytes
		CanTxMsg.Data32[0] = 0;
		CanTxMsg.Data32[1] = 0;
	
		// prepare 29 bit ID
		CanTxMsg.Id =                 (ascii2hex(SerRcvBuf[1])<<28 )& 0xF0000000;
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[2])<<24 )& 0x0F000000));
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[3])<<20 )& 0x00F00000));
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[4])<<16 )& 0x000F0000));
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[5])<<12 )& 0x0000F000));
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[6])<< 8 )& 0x00000F00));
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[7])<< 4 )& 0x000000F0));
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[8])     )& 0x0000000F));
		
		// Set message type 29 bit
		CanTxMsg.Type = CAN_MSG_EXTENDED;
	
		// set DLC
		CanTxMsg.Len = (ascii2hex(SerRcvBuf[9]))& 0xF;
		
		// copy message data
		for (i=0;i<(CanTxMsg.Len);i++) {
			CanTxMsg.Data8[i] = (ascii2hex(SerRcvBuf[10+2*i])<<4 )& 0x0F0;
			CanTxMsg.Data8[i] = (CanTxMsg.Data8[i] | ((ascii2hex(SerRcvBuf[11+2*i]))& 0x00F));
		}
		
		// send the message
		CAN_UserWrite (CAN_BUS1, &CanTxMsg);
	}
	else {
		Result = ret_ERROR;
	}
	
	//-------------------------------------------------

	SerXmtBufPtr = 0;
	// return string to RS232 UART (Host-PC)
	if (AutoPollAutoSend == 1) {
		if (Result == ret_OK) {
			SerXmtBuf[SerXmtBufPtr] = 'Z';
			SerXmtBufPtr++;
		}
	}
	SerXmtBuf[SerXmtBufPtr] = Result;
	SerXmtBufPtr++;
	
	// Send ASCII string to RS232 UART (Host-PC)
	SER_Write (SER_PORT1, &SerXmtBuf, SerXmtBufPtr);
}
/**************************************************/




//*************************************************
//! @brief	sxxyy[CR]
//! Setup with BTR0/BTR1 CAN bit-rates where xx and yy is a hex value.
//!
//! @pre This command is accepted only if the CAN channel is closed. 
//!
//! @param	xx BTR0 value in hex
//! @param	yy BTR1 value in hex
//!
//! @note example: s031C[CR]
//! @note Setup CAN with BTR0=0x03 & BTR1=0x1C which equals to 125 kbit/s.
//!
//! @return	BELL (Ascii 7) for ERROR when CAN channel is open
//! @return	CR (Ascii 13) for OK
//!
static void SetCanBTR0BTR1(void) 
//-------------------------------------------------
{
	u8_t  Result;
	u32_t LpcBtr; // This is the LPC2194's "CANBTR" register
	u8_t BYTE0;
	u8_t BYTE1;

	Result = ret_OK;
	
	// get BTR0 from the received UART command line
	BYTE0  = ((ascii2hex(SerRcvBuf[1])<<4)& 0xF0);
	BYTE0 |= ((ascii2hex(SerRcvBuf[2])   )& 0x0F);
	// get BTR1 from the received UART command line
	BYTE1  = ((ascii2hex(SerRcvBuf[3])<<4)& 0xF0);
	BYTE1 |= ((ascii2hex(SerRcvBuf[4])   )& 0x0F);

	// only if can channel = closed
	if (CanChnOpen == 0) { // precondition
		// Shuffle together LPC2194's "CANBTR" register
		LpcBtr  = ((((u32_t)BYTE0 &0x3F) +1) *6) -1; 	// BRP, bit 0..9
		LpcBtr |=   ((u32_t)BYTE0 &0xC0) << 8;			// SJW, bit 14..15
		LpcBtr |=   ((u32_t)BYTE1       << 16);			// TSEG1, TSEG2, SAM
		// Write values to CAN controller
		PINSEL1 |= (1<<22) | (1<<24) |(1<<26) |(1<<28) |(1<<18) |(1<<16) |(1<<14) ;
		C1MOD=1<<0;			//RESET MODE 
		CAN_SetBusMode ( CAN_BUS1, BUS_OFF);
		C1IER=0x00; // Disabling all interrupts
		C1BTR = LpcBtr;
		//CAN_InitChannel (CAN_BUS1, LpcBtr);
		C1MOD=0;
		CAN_SetBusMode ( CAN_BUS1, BUS_OFF);					// CAN Bus On
		CAN_ReInitChannel ( CAN_BUS1);

		CanInitialized = 1; 	// Now CAN channel can be opened
	}
	else {
		Result = ret_ERROR;				// CAN channel not closed, command ignored
	}	

	//-------------------------------------------------

	SerXmtBufPtr = 0;

	SerXmtBuf[SerXmtBufPtr] = Result;
	SerXmtBufPtr++;
	
	// Send ASCII string to RS232 UART (Host-PC)
	SER_Write (SER_PORT1, &SerXmtBuf, SerXmtBufPtr);
}
/**************************************************/


//*************************************************
//! @brief	riiil[CR]
//! Transmit a standard RTR (11bit) CAN frame.
//!
//! @pre This command is accepted only if the CAN channel is open in normal mode. 
//!
//! @param iii Identifier in hex (000-7FF)
//! @param l Data length (0-8)
//!
//! @note example: r1002[CR]
//! @note Sends an 11bit RTR CAN frame with ID=0x100
//! and DLC set to two (2 bytes).
//!
//! @return If Auto Poll is disabled:
//! @return	BELL (Ascii 7) for ERROR when CAN channel is not open or Listen-Only
//! @return	CR (Ascii 13) for OK
//!
//! @return If Auto Poll is enabled:
//! @return	BELL (Ascii 7) for ERROR when CAN channel is not open or Listen-Only
//! @return	z[CR] for OK
//!
static void TransmitStandardRtr(void) 
//-------------------------------------------------
{
	u8_t Result;

	Result = ret_OK;

	// if CAN channel is open and -not- listen only
	if ((CanChnOpen == 1) && (CanBusMode == BUS_ON)) {

		// Erase all former CAN data bytes
		CanTxMsg.Data32[0] = 0;
		CanTxMsg.Data32[1] = 0;
	
		// prepare 11 bit ID
		CanTxMsg.Id =                 (ascii2hex(SerRcvBuf[1])<<8 )& 0xF00;
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[2])<<4 )& 0x0F0));
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[3])    )& 0x00F));
		
		// Set message type 11 bit
		CanTxMsg.Type = CAN_MSG_STANDARD | CAN_MSG_RTR;
	
		// set DLC
		CanTxMsg.Len = (ascii2hex(SerRcvBuf[4]))& 0x00F;
				
		// send the message
		CAN_UserWrite (CAN_BUS1, &CanTxMsg);
	}
	else {
		Result = ret_ERROR;
	}

	//-------------------------------------------------

	SerXmtBufPtr = 0;

	if (AutoPollAutoSend == 1) {
		if (Result == ret_OK) {
			SerXmtBuf[SerXmtBufPtr] = 'z';
			SerXmtBufPtr++;
		}
	}
	SerXmtBuf[SerXmtBufPtr] = Result;
	SerXmtBufPtr++;
	
	// Send ASCII string to RS232 UART (Host-PC)
	SER_Write (SER_PORT1, &SerXmtBuf, SerXmtBufPtr);
}
/**************************************************/


//*************************************************
//! @brief	Riiiiiiiil[CR]
//! Transmit an extended RTR (29bit) CAN frame.
//!
//! @pre This command is accepted only if the CAN channel is open in normal mode. 
//!
//! @param iiiiiiii Identifier in hex (00000000-1FFFFFFF)
//! @param l Data length (0-8)
//!
//! @note example: R000001002[CR]
//! @note Sends a 29bit RTR CAN frame with ID=0x00000100 
//! and DLC set to two (2 bytes).
//!
//! @return If Auto Poll is disabled:
//! @return	BELL (Ascii 7) for ERROR when CAN channel is not open or Listen-Only.
//! @return	CR (Ascii 13) for OK
//!
//! @return If Auto Poll is enabled:
//! @return	BELL (Ascii 7) for ERROR when CAN channel is not open or Listen-Only
//! @return	z[CR] for OK
//!
static void TransmitExtendedRtr(void) 
//-------------------------------------------------
{
	u8_t Result;

	Result = ret_OK;

	// if CAN channel is open and -not- listen only
	if ((CanChnOpen == 1) && (CanBusMode == BUS_ON)) {

		// Erase former CAN data bytes
		CanTxMsg.Data32[0] = 0;
		CanTxMsg.Data32[1] = 0;
	
		// prepare 29 bit ID
		CanTxMsg.Id =                 (ascii2hex(SerRcvBuf[1])<<28 )& 0xF0000000;
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[2])<<24 )& 0x0F000000));
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[3])<<20 )& 0x00F00000));
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[4])<<16 )& 0x000F0000));
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[5])<<12 )& 0x0000F000));
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[6])<< 8 )& 0x00000F00));
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[7])<< 4 )& 0x000000F0));
		CanTxMsg.Id = (CanTxMsg.Id | ((ascii2hex(SerRcvBuf[8])     )& 0x0000000F));
		
		// Set message type 29 bit RTR
		CanTxMsg.Type = CAN_MSG_EXTENDED | CAN_MSG_RTR;
	
		// set DLC
		CanTxMsg.Len = (ascii2hex(SerRcvBuf[9]))& 0xF;
				
		// send the message
		CAN_UserWrite (CAN_BUS1, &CanTxMsg);
	}
	else {
		Result = ret_ERROR;
	}
	//-------------------------------------------------

	SerXmtBufPtr = 0;

	if (AutoPollAutoSend == 1) {
		if (Result == ret_OK) {
			SerXmtBuf[SerXmtBufPtr] = 'z';
			SerXmtBufPtr++;
		}
	}
	SerXmtBuf[SerXmtBufPtr] = Result;
	SerXmtBufPtr++;
	
	// Send ASCII string to RS232 UART (Host-PC)
	SER_Write (SER_PORT1, &SerXmtBuf, SerXmtBufPtr);
}
/**************************************************/

//*************************************************
//! @brief	F[CR]
//! Read Status Flags.
//!
//! @pre This command is accepted only if the CAN channel is open. 
//!
//! @param none
//!
//! @note example: F[CR]
//! @note Read Status Flags.
//!
//! @return	BELL (Ascii 7) for ERROR when CAN channel is not open
//! @return	An F with 2 bytes BCD hex value plus CR (Ascii 13) for OK, e.g. F01[CR]
//! @li Bit 0 : CAN receive FIFO queue full
//! @li Bit 1 : CAN transmit FIFO queue full
//! @li Bit 2 : Error warning (EI), see SJA1000 datasheet
//! @li Bit 3 : Data Overrun (DOI), see SJA1000 datasheet
//! @li Bit 4 : Not used.
//! @li Bit 5 : Error Passive (EPI), see SJA1000 datasheet
//! @li Bit 6 : Arbitration Lost (ALI), see SJA1000 datasheet
//! @li Bit 7 : Bus Error (BEI), see SJA1000 datasheet
//!
static void GetStatusFlags(void) 
//-------------------------------------------------
{
//	u8_t 	Result;
	
//	u8_t	shCANICR; // controller's interrupt flag register
	u8_t 	statusbyte=0;

//	Result = ret_OK;
	SerXmtBufPtr = 0;

	if (CanChnOpen == 1){ // precondition	

		// read status register from controller
		//CAN_GetLastIR (CAN_BUS1, &shCANICR);
		// mask out unspecified bits
		statusbyte |= (C1ICR & 0xEC);
		statusbyte |= (SerRxOverrunOccurred & 0x01);
		statusbyte |= (SerTxCongestionOccurred & 0x01)*2;
		
		SerRxOverrunOccurred = 0; 		// reset flag after reading
		//statusbits.bit1 = (SerTxCongestionOccurred & 0x01); // transmit FIFO full
		SerTxCongestionOccurred = 0; 	// reset flag after reading

		// return result string
		SerXmtBuf[SerXmtBufPtr] = 'F';
		SerXmtBufPtr++;
		SerXmtBuf[SerXmtBufPtr] = hex2ascii((statusbyte & 0xF0) >> 4);
		SerXmtBufPtr++;
		SerXmtBuf[SerXmtBufPtr] = hex2ascii (statusbyte & 0x0F);
		SerXmtBufPtr++;
		SerXmtBuf[SerXmtBufPtr] = ret_OK; // [<CR>]
		SerXmtBufPtr++;

	} // if CAN chn open
	//-------------------------------------------------
	else { // CAN channel was not open
		SerXmtBuf[SerXmtBufPtr] = ret_ERROR;
		SerXmtBufPtr++;
	}
	//-------------------------------------------------
	
	// Send ASCII string to RS232 UART (Host-PC)
	SER_Write (SER_PORT1, &SerXmtBuf, SerXmtBufPtr);
}
/**************************************************/


//*************************************************
//! @brief	Xn[CR]
//! Sets Auto Poll/Send ON/OFF for received frames. 
//!
//! @pre This command is accepted only if the CAN channel is closed. 
//!
//! The value will be saved in EEPROM and remembered next time the PCAN-RS-232 is powered up.
//! It is set to OFF by default, to be compatible with old programs written for PCAN-RS-232. 
//! Setting it to ON will disable the P and A commands and
//! change the reply back from using the t and T command 
//! (see these commands for more information on the reply). 
//! It is strongly recommended to set this feature and upgrade from the old polling mechanism. 
//! Doing so will save bandwith and increases number of CAN frames that can be sent to the PCAN-RS-232. 
//! With this feature set, CAN frames will be sent out on the RS232 as soon as the CAN channel is opened.
//!
//! @param n=0 turn off Auto Poll/Send feature
//! @param n=1 turn on Auto Poll/Send feature
//!
//! @note example 1: X0[CR]
//! @note Turn OFF the Auto Poll/Send feature
//! @note example 2: X1[CR]
//! @note Turn ON the Auto Poll/Send feature.
//!
//! @return	BELL (Ascii 7) for ERROR when CAN channel is open
//! @return	CR (Ascii 13) for OK
//!
static void SetAutoPollAutoSend(void) 
//-------------------------------------------------
{
	u8_t Result;
	
	Result = ret_OK;
	
	if (CanChnOpen == 0){ // precondition	
		AutoPollAutoSend = ascii2hex(SerRcvBuf[1]);
	}
	else {
		Result = ret_ERROR;
	}
	//-------------------------------------------------

	SerXmtBufPtr = 0;

	SerXmtBuf[SerXmtBufPtr] = Result;
	SerXmtBufPtr++;
	
	// Send ASCII string to RS232 UART (Host-PC)
	SER_Write (SER_PORT1, &SerXmtBuf, SerXmtBufPtr);
}
/**************************************************/



//*************************************************
//! @brief	Wn[CR]
//! Filter mode setting. 
//!
//! @pre This command is accepted only if CAN channel is initialized but closed. 
//!
//! The setting is is saved in EEPROM and since remembered on next startup.
//!
//! @param n=0 set dual filter mode
//! @param n=1 set single filter mode
//!
//! @note example 1: W0[CR]
//! @note Set dual filter mode
//! @note example 2: W1[CR]
//! @note Set single filter mode
//!
//! @return	BELL (Ascii 7) for ERROR when CAN channel is open
//! @return	CR (Ascii 13) for OK
//!
static void SetFilterMode(void) 
//-------------------------------------------------
{
	u8_t Result;
	
	Result = ret_OK;
	
	if (CanChnOpen == 0){ // precondition	
		FilterMode = ascii2hex(SerRcvBuf[1]);
	}
	else {
		Result = ret_ERROR;
	}
	//-------------------------------------------------

	SerXmtBufPtr = 0;

	SerXmtBuf[SerXmtBufPtr] = Result;
	SerXmtBufPtr++;
	
	// Send ASCII string to RS232 UART (Host-PC)
	SER_Write (SER_PORT1, &SerXmtBuf, SerXmtBufPtr);
}
/**************************************************/


//*************************************************
//! @brief	V[CR]
//! Get version number of both PCAN-RS-232 hardware and software
//!
//! @pre This command is always accepted.
//!
//! @param	none
//!
//! @note example: V[CR]
//! @note Get version numbers.
//!
//! @return	V and a 2 bytes BCD value for hardware version and a 2 byte BCD value for software version plus CR (Ascii 13) for OK, e.g. V1013[CR]
//!
static void GetVersionInfo(void) 
//-------------------------------------------------
{
	SerXmtBufPtr = 0;

	SerXmtBuf[SerXmtBufPtr] = 'V';
	SerXmtBufPtr++;
	SerXmtBuf[SerXmtBufPtr] = '0';
	SerXmtBufPtr++;
	SerXmtBuf[SerXmtBufPtr] = '0';
	SerXmtBufPtr++;
	SerXmtBuf[SerXmtBufPtr] = '0';
	SerXmtBufPtr++;
	SerXmtBuf[SerXmtBufPtr] = '1';
	SerXmtBufPtr++;
	SerXmtBuf[SerXmtBufPtr] = '\r'; // Append <CR>
	SerXmtBufPtr++;

	// Send ASCII string to RS232 UART (Host-PC)
	SER_Write (SER_PORT1, &SerXmtBuf, SerXmtBufPtr);
}
/**************************************************/



//! Get serial number of the PCAN-RS-232.
//!
//! @pre This command is always accepted.
//!
//! @param	none
//!
//! @note example: N[CR]
//! @note Get serial number.
//!
//! @note The serial number may be composed of both numerical and alphanumerical values. 
//! The serial number is also printed on the PCAN-RS-232 enclosure.
//!
//! @return	N and a 4 bytes value for serial number plus CR (Ascii 13) for OK, e.g. NA123[CR]
//!
static void GetSerialNumber(void) 
//-------------------------------------------------
{
	SerXmtBufPtr = 0;

 	SerXmtBuf[SerXmtBufPtr] = 'N';
	SerXmtBufPtr++;

	memcpy (&SerXmtBuf[SerXmtBufPtr],"0001",4);
//	memcpy (&SerXmtBuf[SerXmtBufPtr],EepromData.DeviceSN,4);
	SerXmtBufPtr = SerXmtBufPtr + 4;

	SerXmtBuf[SerXmtBufPtr] = '\r'; // Append <CR>
	SerXmtBufPtr++;
	
	// Send ASCII string to RS232 UART (Host-PC)
	SER_Write (SER_PORT1, &SerXmtBuf, SerXmtBufPtr);
}
/**************************************************/


//*************************************************
//! @brief	Zn[CR]
//! Sets Time Stamp ON/OFF for received frames only.
//!
//! @pre This command is accepted only if the CAN channel is closed. 
//!
//! This command is set to OFF by default (to be compatible with older programs written for PCAN-RS-232).
//! Setting it to ON will add 4 bytes sent out from PCAN-RS-232 when the Auto Poll/Send feature is enabled.
//! With Timestamp ON, each message gets a time in milliseconds when it was received at the PCAN-RS-232, 
//! this can be used for realtime applications for e.g. knowing time inbetween messages etc.
//! Using this feature will decrease bandwith on the PCAN-RS-232, since it adds 4 bytes to each message being sent.
//! If the timestamp is OFF, the incoming frames looks like this:
//! t10021133[CR] (a standard frame with ID=0x100 & 2 bytes)
//! If the timestamp is ON, the incoming frames looks like this:
//! t100211334D67[CR] (a standard frame with ID=0x100 & 2 bytes)
//! Note the last 4 bytes 0x4D67, which is a timestamp for this specific message in milliseconds (and of course in hex). 
//! The timer in the PCAN-RS-232 starts at zero 0x0000 and goes up to 0xEA5F before it loops around to 0x0000. 
//! This corresponds to 60,000mS (i.e. 1 minute, which will be more than enough in most systems).
//!
//! @param	n=0 Turn OFF the timestamp feature 
//! @param	n=1 Turn ON the timestamp feature
//!
//! @note The value is saved in EEPROM and is set each time the PCAN-RS-232 is powered up.
//! @note example 1: Z0[CR]
//! @note Turn OFF the Time Stamp feature.
//! @note example 2: Z1[CR]
//! @note Turn ON the Time Stamp feature.
//!
//! @return	BELL (Ascii 7) for ERROR  when CAN channel is open
//! @return	CR (Ascii 13) for OK
//!
static void SetRcvTimestampMode(void) 
//-------------------------------------------------
{
	u8_t Result;
	
	Result = ret_OK;
		
	if (CanChnOpen == 0){ // precondition
		CanRcvTimestampOn = ascii2hex(SerRcvBuf[1]);
	}
	else  {
		Result = ret_ERROR;
	}
	//-------------------------------------------------

	SerXmtBufPtr = 0;

	SerXmtBuf[SerXmtBufPtr] = Result;
	SerXmtBufPtr++;
	
	// Send ASCII string to RS232 UART (Host-PC)
	SER_Write (SER_PORT1, &SerXmtBuf, SerXmtBufPtr);
}
/**************************************************/






//*************************************************
//! @brief	
//! Reads one MSG or RTR from CAN and passes this to RS232 UART (Host-PC).
//! To be called from MAIN() cyclically, as fast as possible.
//!
//! Reads messages from CAN and forwards them to the host PC (RS232), with or without timestamp.
//!
//! @param	void
//!
//! @return	void
//!
void ProcessMsgFromCan(void) 
//-------------------------------------------------
{
	static u8_t i; //all purpose loop counter
	static u16_t Timestamp;

	if (AutoPollAutoSend == 1) {

		// repeat previous message if not successfully sent in last cycle
		if (UARTWriteResult != SER_ERR_OK) {
			SerTxCongestionOccurred = 1; // set flag indicationg Tx buffer was full before
			// Repeat sending last ASCII string to RS232 UART (Host-PC)
			UARTWriteResult = SER_Write ( SER_PORT1, &SerXmtBuf[0], SerXmtBufPtr);
			return;
		}
		//-------------------------------------------------

		if (CAN_UserRead (CAN_BUS1, &CanRxMsg) != 0) {
			
			Timestamp = Millitimer; // get current timestamp 

			// if 11 bit Id
			if ((CanRxMsg.Type & CAN_MSG_EXTENDED) == 0) { // is -not- 29 bit

				if ((CanRxMsg.Type & CAN_MSG_RTR) == 0) { // is -not- RTR
					SerXmtBuf[0] = 't';
				}
				else { // is 11 bit RTR	
					SerXmtBuf[0] = 'r';
				}
				// convert 11 bit Id
				SerXmtBuf[1] = hex2ascii((CanRxMsg.Id & 0x700)>>8);
				SerXmtBuf[2] = hex2ascii((CanRxMsg.Id & 0x0F0)>>4);
				SerXmtBuf[3] = hex2ascii (CanRxMsg.Id & 0x00F);
				// convert DLC
				SerXmtBuf[4] = hex2ascii(CanRxMsg.Len);

				if ((CanRxMsg.Type & CAN_MSG_RTR) == 0) { // is -not- RTR
					// convert data bytes to ASCII
					for (i=0;i<CanRxMsg.Len;++i) {
						SerXmtBuf[5+i*2] = hex2ascii((CanRxMsg.Data8[i] & 0xF0)>>4);
						SerXmtBuf[6+i*2] = hex2ascii (CanRxMsg.Data8[i] & 0x0F);
					}
					SerXmtBufPtr = 5 + (CanRxMsg.Len *2);
				}
				else { // is RTR: no data
					SerXmtBufPtr = 5;
				}

				// append timestamp, if enabled
				if (CanRcvTimestampOn == 1) {
					SerXmtBuf[SerXmtBufPtr]   = hex2ascii((Timestamp & 0xF000)>>12);
					SerXmtBuf[SerXmtBufPtr+1] = hex2ascii((Timestamp & 0x0F00)>>8);
					SerXmtBuf[SerXmtBufPtr+2] = hex2ascii((Timestamp & 0x00F0)>>4);
					SerXmtBuf[SerXmtBufPtr+3] = hex2ascii (Timestamp & 0x000F);
					SerXmtBufPtr = SerXmtBufPtr +4;
				}   
	
				SerXmtBuf[SerXmtBufPtr] = '\r'; // Append <CR>
				SerXmtBufPtr ++;

				// Send ASCII string to RS232 UART (Host-PC)
				UARTWriteResult = SER_Write ( SER_PORT1, &SerXmtBuf[0], SerXmtBufPtr);
			} // IF Id=not Extended
			//-------------------------------------------------

			else { // Id = EXTENDED 29 bit
				if ((CanRxMsg.Type & CAN_MSG_RTR) == 0) { // is -not- RTR
					SerXmtBuf[0] = 'T';
				}
				else { // is 29 bit RTR	
					SerXmtBuf[0] = 'R';
				}
				SerXmtBufPtr++;
				// 29 bit Id
				SerXmtBuf[1] = hex2ascii((CanRxMsg.Id & 0x10000000)>>28);
				SerXmtBuf[2] = hex2ascii((CanRxMsg.Id & 0x0F000000)>>24);
				SerXmtBuf[3] = hex2ascii((CanRxMsg.Id & 0x00F00000)>>20);
				SerXmtBuf[4] = hex2ascii((CanRxMsg.Id & 0x000F0000)>>16);
				SerXmtBuf[5] = hex2ascii((CanRxMsg.Id & 0x0000F000)>>12);
				SerXmtBuf[6] = hex2ascii((CanRxMsg.Id & 0x00000F00)>>8);
				SerXmtBuf[7] = hex2ascii((CanRxMsg.Id & 0x000000F0)>>4);
				SerXmtBuf[8] = hex2ascii (CanRxMsg.Id & 0x0000000F);
				// DLC
				SerXmtBuf[9] = hex2ascii(CanRxMsg.Len);
				// data bytes
				if ((CanRxMsg.Type & CAN_MSG_RTR) == 0) { // is -not- RTR
					// convert data bytes to ASCII
					for (i=0;i<CanRxMsg.Len;++i) {
						SerXmtBuf[10+i*2] = hex2ascii((CanRxMsg.Data8[i] & 0xF0)>>4);
						SerXmtBuf[11+i*2] = hex2ascii (CanRxMsg.Data8[i] & 0x0F);
					}
					SerXmtBufPtr = 10 + (CanRxMsg.Len *2);
				}
				else { // is RTR: no data
					SerXmtBufPtr = 10;
				}
				// append timestamp, if enabled
				if (CanRcvTimestampOn == 1) {
					SerXmtBuf[SerXmtBufPtr]   = hex2ascii((Timestamp & 0xF000)>>12);
					SerXmtBuf[SerXmtBufPtr+1] = hex2ascii((Timestamp & 0x0F00)>>8);
					SerXmtBuf[SerXmtBufPtr+2] = hex2ascii((Timestamp & 0x00F0)>>4);
					SerXmtBuf[SerXmtBufPtr+3] = hex2ascii (Timestamp & 0x000F);
					SerXmtBufPtr = SerXmtBufPtr +4;
				}   

				SerXmtBuf[SerXmtBufPtr] = '\r'; // Append <CR>
				SerXmtBufPtr ++;

				// Send ASCII string to RS232 UART (Host-PC)
				UARTWriteResult = SER_Write ( SER_PORT1, &SerXmtBuf[0], SerXmtBufPtr);
			} // IF Id=Extended
			//-------------------------------------------------
		} // IF Can read
	} // IF auto poll auto send
}
/**************************************************/
