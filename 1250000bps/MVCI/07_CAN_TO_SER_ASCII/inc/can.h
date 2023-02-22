
#ifndef  _CAN_H_
#define  _CAN_H_

//
//	can.h
//
//	-------------------------------------------------------------------
//! @addtogroup CAN
//! <h3> API Functions for CAN access </h3>
//!
//! The functions can be used to access the CAN buses.
//! The functions are interrupt protected internally and will target to the
//! main() level. If the user want to send from interrupts like timers
//! he has to build own wrapper functions for interrupt safety.
//!
//! <h3> Targets: </h3>
//! - PCAN-Router @n
//! - PCAN-Router Pro
//! - PCAN-RS-232
//!
//	-------------------------------------------------------------------
//
//	Copyright (C) 1999-2011  PEAK-System Technik GmbH, Darmstadt
//	more Info at http://www.peak-system.com 
//
//! @{


////////////////////////////////////////////////////////////
//! @name buses
//! Use one of these handles to access a CAN bus. Typically this
//! is done by a 'hBus' parameter.
//! @{
#define	CAN_BUS1		0			//!< CAN BUS 1
#define	CAN_BUS2		1			//!< CAN BUS 2
#define	CAN_BUS3		2			//!< CAN BUS 3 (PCAN-Router Pro only)
#define	CAN_BUS4		3			//!< CAN BUS 4 (PCAN-Router Pro only)
/*! @}*/


////////////////////////////////////////////////////////////
//! @name errors
//! A function returns one of these error codes:
//! @{
#define	CAN_ERR_FAIL		0		//!< fail
#define	CAN_ERR_OK			1		//!< OK, no error
/*! @}*/


////////////////////////////////////////////////////////////
//! @name busmodes
//! Used with CAN_SetBusMode(). Get involved into bus activity.
//! @{
#define	BUS_ON				0		//!< Bus on
#define	BUS_OFF				1		//!< Reset mode
#define	BUS_LOM				2		//!< Listen only
/*! @}*/


////////////////////////////////////////////////////////////
//! @name filtertypes
//! Used with CAN_FilterAddId() as the type for filter entry
//! @{
#define	FILTER_11BIT_ID					0	//!< 11 bit single ID
#define	FILTER_29BIT_ID					1	//!< 29 bit single ID
#define	FILTER_11BIT_ID_RANGE			2	//!< 11 bit ID range
#define	FILTER_29BIT_ID_RANGE			3	//!< 29 bit ID range
/*! @}*/


////////////////////////////////////////////////////////////
//! @name transceivertypes
//! Used with CAN_GetTransceiverType()
//! @{
#define	CAN_TRANSCEIVER_TYPE_NONE		0	//!< nothing detected
#define	CAN_TRANSCEIVER_TYPE_HS8		1	//!< highspeed 82C251
#define	CAN_TRANSCEIVER_TYPE_HS8_GE	2	//!< highspeed 82C251 (opto isolated)
#define	CAN_TRANSCEIVER_TYPE_LS_DW		3	//!< low speed double wire
#define	CAN_TRANSCEIVER_TYPE_LS_SW		4	//!< low speed single wire
#define	CAN_TRANSCEIVER_TYPE_HS14		5	//!< highspeed 1041
/*! @}*/


////////////////////////////////////////////////////////////
//! @name transceivermodes
//! Used with CAN_SetTransceiverMode()
//! @{
#define	CAN_TRANSCEIVER_MODE_NORMAL		0	//!< normal mode
#define	CAN_TRANSCEIVER_MODE_WAKEUP		1	//!< wakeup (high voltage SW only)
#define	CAN_TRANSCEIVER_MODE_POWERDOWN	2	//!< power down for system shut down
#define	CAN_TRANSCEIVER_MODE_HIGHSPEED	4	//!< highspeed (no slope for SW only)
#define	CAN_TRANSCEIVER_MODE_STANDBY		5	//!< standby, low power
/*! @}*/


////////////////////////////////////////////////////////////
//! @name filtermodes
//! Used with CAN_SetFilterMode().
//! @{
#define	AF_ON							0		//!< Filter ON, use entries
#define	AF_OFF						1		//!< Filter OFF
#define	AF_ON_BYPASS_ON			2		//!< Filter ON, bypass
/*! @}*/


////////////////////////////////////////////////////////////
//! @name options
//! Options for RX Callback functions and some misc. defines
//! @{
#define	LEAVE_MESSAGE				0		//!< put message to Queue
#define	SKIP_MESSAGE				1		//!< skip message from Queue

#define	STD_TX_ERRORLIMIT			96		//!< std limit
/*! @}*/


////////////////////////////////////////////////////////////
//! @name interruptnumbers
//! Used to configure the VIC channels for interrupts
//! @{
#define	GLOBAL_CAN_INTSOURCE				19		//!< global CAN INT

#define	CAN1_TX_INTSOURCE					20		//!< CAN1 TX INT
#define	CAN2_TX_INTSOURCE					21		//!< CAN2 TX INT
#define	CAN3_TX_INTSOURCE					22		//!< CAN3 TX INT
#define	CAN4_TX_INTSOURCE					23		//!< CAN4 TX INT

#define	CAN1_RX_INTSOURCE					26		//!< CAN1 RX INT
#define	CAN2_RX_INTSOURCE					27		//!< CAN2 RX INT
#define	CAN3_RX_INTSOURCE					28		//!< CAN3 RX INT
#define	CAN4_RX_INTSOURCE					29		//!< CAN4 RX INT
/*! @}*/


////////////////////////////////////////////////////////////
//! @name messagetypes
//! Types for CAN messages. Can be combined for e.g. EXTENDED | RTR
//! @{
#define	CAN_MSG_STANDARD		( 0 << 0)	//!< 11 bit ID flag
#define	CAN_MSG_RTR				( 1 << 0)	//!< RTR flag
#define	CAN_MSG_EXTENDED		( 1 << 1)	//!< 29 bit ID flag
#define	CAN_MSG_SINGLESHOT	( 1 << 2)	//!< single shot
/*! @}*/


////////////////////////////////////////////////////////////
//! @name types
//! Some basetypes for CAN functions
//! @{
#define	CANStatus_t			u32_t		//!< return type for functions
#define	CANHandle_t			u8_t		//!< CAN handle type
/*! @}*/


////////////////////////////////////////////////////////////
//! @name structures
//! structures for CAN messages
//! @{

//! @brief
//! CAN message struct used for CAN messages on transmitt
//! and receive. Does not include a timestamp.
typedef struct {

	u8_t			N_A;							//!< not used
	u8_t			NetNr;						//!< busnumber see CAN_BUSx
	u8_t			Type;							//!< Type of message see CAN_MSG_...
	u8_t			Len;							//!< Length of message

	u32_t			Id;						 	//!< Id of message

	union {
		u8_t			Data8[8];				//!< message data as byte array
		u16_t			Data16[4];				//!< message data as word array
		u32_t			Data32[2];				//!< message data as dword array
	};
} CANMsg_t;



//! @brief
//! CAN message struct with timestamp used for CAN messages
//! on receive. Includes a timestamp.
typedef struct {

	u8_t			N_A;							//!< not used
	u8_t			NetNr;						//!< busnumber see CAN_BUSx
	u8_t			Type;							//!< Type of message see CAN_MSG_...
	u8_t			Len;							//!< Length of message

	u32_t			Id;							//!< Id of message

	union {
		u8_t			Data8[8];				//!< message data as byte array
		u16_t			Data16[4];				//!< message data as word array
		u32_t			Data32[2];				//!< message data as dword array
	};

	u32_t			TimeStamp32;				//!< Timestamp
} CANRxMsg_t;

/*! @}*/


#ifdef __cplusplus
extern "C" {
#endif


////////////////////////////////////////////////////////////
//! @name functions
//! This section will describe the API functions. The functions are target
//! to the main() level. Interrupt protection is now done internally by the
//! VIC registers.
//! @{


//! @brief
//! Reference a CANMsg_t array as TxQueue to a CAN bus. The user can vary queue
//! size depending on the application.
//!
//! @param		hBus				handle for CAN_BUSx
//! @param		pQueueStart		start of the queue
//! @param		QueueSize		size of the queue as message count
//!
//! @return		CAN_ERR_OK		OK, queue is referenced
CANStatus_t  CAN_ReferenceTxQueue (	CANHandle_t hBus, CANMsg_t  *pQueueStart, u16_t  QueueSize);



//! @brief
//! Reference a CANMsg_t array or a CANRxMsg_t array as RxQueue to a CAN bus.
//! The user can vary queue size depending on the application.
//!
//! @param		hBus				handle for CAN_BUSx
//! @param		pQueueStart		start of the queue
//! @param		QueueSize		size of the queue as message count
//!
//! @return		CAN_ERR_OK		OK, queue is referenced
CANStatus_t  CAN_ReferenceRxQueue (	CANHandle_t  hBus, void  *pQueueStart, u16_t  QueueSize);



//! @brief
//! Returns the address of a CAN interrupt vector. Used to configure the
//! VIC of the controller.
//!
//! @param		Number		number of the vector, see ..._INTSOURCE
//!
//! @return		address of the ISR function or NULL
void*  CAN_GetIsrVector ( u8_t Number);



//! @brief
//! Set a timestamp handler callback function. If NULL the RxQueue passed by
//! CAN_ReferenceRxQueue is handled as a CANMsg_t array. If != NULL the RxQueue
//! passed by CAN_ReferenceRxQueue is handled as a CANRxMsg_t array. Additionally
//! the callback is called to fill in the timestamp. The message's data is
//! invalid at this time. Use a RX Callback to handle RX data on interrupt level.
//!
//! @param		hBus				handle for CAN_BUSx
//! @param		Handler			pointer to the callback function
//!
//! @return		CAN_ERR_OK		OK, callback was set
CANStatus_t  CAN_SetTimestampHandler (	CANHandle_t  hBus, void (*Handler)( CANRxMsg_t *msg));



//! @brief
//! Sets a callback for interrupt protection. <b> This function is obsolete! </b>
//!
//! @param		hBus				handle for CAN_BUSx
//! @param		Handler			pointer to the callback function
//!
//! @return		CAN_ERR_OK		OK, callback was set
CANStatus_t  CAN_SetAtomicHandler (	CANHandle_t  hBus, void (*Handler)(void));



//! @brief
//! Sets a callback for interrupt protection. <b> This function is obsolete! </b>
//!
//! @param		hBus				handle for CAN_BUSx
//! @param		Handler			pointer to the callback function
//!
//! @return		CAN_ERR_OK		OK, callback was set
CANStatus_t  CAN_SetUnatomicHandler (	CANHandle_t  hBus, void (*Handler)(void));



//! @brief
//! Sets the error warning limit used on CAN_InitChannel().
//!
//! @param		hBus				handle for CAN_BUSx
//! @param		NewLimit			limit value
//!
//! @return		CAN_ERR_OK		OK, callback was set
CANStatus_t  CAN_SetErrorLimit (	CANHandle_t  hBus, u8_t  NewLimit);



//! @brief
//! During transmission on Tx Errors a Error Interrupt can occour. If the
//! Handler is set to NULL a autobuson is done and the Bus is set to Bus on.
//! If a handler is assigned the Handler is called and user have to setup
//! the hardware and handle the Error.
//!
//! @param		hBus				handle for CAN_BUSx
//! @param		Handler			pointer to the callback function
//!
//! @return		CAN_ERR_OK		OK, callback was set
CANStatus_t  CAN_SetTxErrorCallback (	CANHandle_t  hBus, u32_t (*Handler)( void));



//! @brief
//! Sets a Handler for Rx Callback. The Rx Callback is called on every
//! incoming msg. The Msg the void pointer points to must be casted to
//! CANRxMsg_t or CANMsg_t. The Msg is filled in and valid.
//! The callback have to return either SKIP_MESSAGE to skip the msg or
//! LEAVE_MESSAGE to pass it to Application Layer.
//!
//! @param		hBus				handle for CAN_BUSx
//! @param		Handler			pointer to the callback function
//!
//! @return		CAN_ERR_OK		OK, callback was set
CANStatus_t  CAN_SetRxCallback (	CANHandle_t  hBus, u32_t (*Handler)(void*));



//! @brief
//! Sets a Textinfo to a CAN bus
//!
//! @param		hBus				handle for CAN_BUSx
//! @param		NewInfo			pointer to the textinfo
//!
//! @return		CAN_ERR_OK		OK, callback was set
CANStatus_t  CAN_SetChannelInfo (	CANHandle_t  hBus, u8_t* NewInfo);



//! @brief
//! Returns the pointer to the InfoText
//!
//! @param		hBus				handle for CAN_BUSx
//!
//! @return		pointer to the text
u8_t*  CAN_GetChannelInfo ( CANHandle_t  hBus);



//! @brief
//! Sets a busmode on CAN_BUSx. After this the application gets involved into
//! bus activity. When using BUS_LOM for listen only the user can not send
//! messages.
//!
//! @param		hBus				handle for CAN_BUSx
//! @param		NewMode			mode for CAN_BUSx, see BUS_...
//!
//! @return		CAN_ERR_OK		OK
CANStatus_t  CAN_SetBusMode (	CANHandle_t  hBus, u8_t NewMode);



//! @brief
//! Retrieve the type of the installed transceiver. On PCAN-Router this is always
//! 'CAN_TRANSCEIVER_TYPE_HS8'. On PCAN-Router Pro you can check the installed
//! module.
//!
//! @param		hBus			handle for CAN_BUSx
//! @param		buff			user buffer to write to
//!
//! @return		CAN_ERR_OK		OK
CANStatus_t  CAN_GetTransceiverType ( CANHandle_t  hBus, u8_t  *buff);



//! @brief
//! Set a transceiver mode to CAN_BUSx.
//!
//! @param		hBus			handle for CAN_BUSx
//! @param		mode			new mode for the transceiver
//!
//! @return		CAN_ERR_OK		OK
CANStatus_t  CAN_SetTransceiverMode ( CANHandle_t  hBus, u8_t  mode);



//! @brief
//! Get the last value of the CAN interrupt register. This function is only
//! available on PCAN-RS-232.
//!
//! @param		hBus			handle for CAN_BUSx
//! @param		pBuff			buffer for the value
//!
//! @return		CAN_ERR_OK		OK
CANStatus_t  CAN_GetLastIR ( CANHandle_t  hBus, u8_t  *pBuff);



//! @brief
//! Returns a pointer to a CAN message from the TX queue.
//!
//! @param		hBus				handle for CAN_BUSx
//!
//! @return		pointer to message or NULL when queue full
CANMsg_t*  CAN_TxQueueGetNext (	CANHandle_t  hBus);



//! @brief
//! Returns a pointer to a CAN message from the RX queue.
//!
//! @param		hBus				handle for CAN_BUSx
//!
//! @return		pointer to message or NULL when queue is empty
void*  CAN_RxQueueGetNext (	CANHandle_t  hBus);



//! @brief
//! Send the message on CAN_BUSx. The message written to the queue is
//! copied to hardware or queued for transmission. This is done after
//! CAN_TxQueueGetNext().
//!
//! @param		hBus				handle for CAN_BUSx
//!
//! @return		CAN_ERR_OK		OK
CANStatus_t  CAN_TxQueueWriteNext (	CANHandle_t  hBus);



//! @brief
//! Frees a message on CAN_BUSx. This is done after CAN_RxQueueGetNext().
//!
//! @param		hBus				handle for CAN_BUSx
//!
//! @return		CAN_ERR_OK		OK
CANStatus_t  CAN_RxQueueReadNext (	CANHandle_t  hBus);



//! @brief
//! Initialize a CAN bus and set up internal resources. This is done before
//! CAN_SetBusMode(). Queues must be referenced before.
//!
//! @param		hBus				handle for CAN_BUSx
//! @param		Timing			baudrate for CAN, see can_user.h
//!
//! @return		CAN_ERR_OK		OK
CANStatus_t  CAN_InitChannel (	CANHandle_t  hBus, u32_t Timing);



//! @brief
//! Reinit a CAN bus and frees Queues
//!
//! @param		hBus				handle for CAN_BUSx
//!
//! @return		CAN_ERR_OK		OK
CANStatus_t  CAN_ReInitChannel ( CANHandle_t  hBus);



//! @brief
//! Clears and sets up the hardware filter table. The function returns with
//! filters OFF! This is done before CAN_FilterAddId() and CAN_SetFilterMode().
//!
//! @return		CAN_ERR_OK		OK
CANStatus_t  CAN_InitFilters ( void);



//! @brief
//! Sets a new Filter Mode. Use this after CAN_InitFilters() and optionally
//! CAN_FilterAddId() to enable the hardware filter.
//!
//! @param		NewMode				filter mode, see AF_...
//!
//! @return		CAN_ERR_OK		OK
CANStatus_t  CAN_SetFilterMode (	u8_t  NewMode);



//! @brief
//! Adds an ID or ID range to the hardware filter table. Sorting is done
//! internally. Returns with filters OFF!
//!
//! @param		hBus				handle for CAN_BUSx
//! @param		id_type			type of ID, see FILTER_....
//! @param		IdStart			single ID or start of range
//! @param		...				used only as end of range
//!
//! @return		CAN_ERR_OK		OK
CANStatus_t  CAN_FilterAddId (	CANHandle_t  hBus, u8_t id_type, u32_t IdStart, ...);

/*! @}*/

#ifdef __cplusplus
}
#endif

/*! @}*/

#endif

