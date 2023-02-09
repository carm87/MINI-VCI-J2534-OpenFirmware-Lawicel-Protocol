
#include "datatypes.h"
#include "lpc21xx.h"
#include "can.h"
#include "can_user.h"

#define	TSEG1  (10 << 16) //The delay from the nominal Sync point to the sample point is (this value plus one) CAN clocks.
#define	TSEG2  (2 << 20) //  The delay from the sample point to the next nominal sync point is (this value plus one) CAN clocks. The nominal CAN bit time is (this value plus the value in TSEG1 plus 3) CANclocks.
#define BRP  (7 << 0)  //Baud Rate Prescaler. The VPB clock is divided by (this value plus one) to produce the CAN clock.
#define	SJW  (0 << 14)  // The Synchronization Jump Width is (this value plus one) CAN clocks
#define	SAM (0 << 23) // 0=The bus is sampled once (recommended for high speedbuses) 1=The bus is sampled 3 times (recommended for low tomedium speed buses)


#define CAN_BAUD_10K  0
#define CAN_BAUD_20K  1
#define CAN_BAUD_50K  2
#define CAN_BAUD_100K  3
#define CAN_BAUD_125K  4
#define CAN_BAUD_250K  5
#define CAN_BAUD_500K  6
#define CAN_BAUD_800K  7
#define CAN_BAUD_1M  8






// Queues for CAN1
CANMsg_t  TxQueueCAN1[CAN1_TX_QUEUE_SIZE];
CANMsg_t  RxQueueCAN1[CAN1_RX_QUEUE_SIZE];


// Queues for CAN2
CANMsg_t  TxQueueCAN2[CAN2_TX_QUEUE_SIZE];
CANMsg_t  RxQueueCAN2[CAN2_RX_QUEUE_SIZE];

CANStatus_t  CAN_InitChannelMio (	CANHandle_t  hBus, u32_t Timing);

CANStatus_t  CAN_InitChannelMio (	CANHandle_t  hBus, u32_t Timing)
{

	PINSEL1 |= (1<<22) | (1<<24) |(1<<26) |(1<<28) |(1<<18) |(1<<16) |(1<<14) ;
	C1MOD=1<<0;			//RESET MODE 
	CAN_SetBusMode ( CAN_BUS1, BUS_OFF);
	C1IER=0x00; // Disabling all interrupts
	
			switch (Timing){
			case 0: 
			C1BTR = TSEG1 |TSEG2|399|SAM;
			break;
			case 1: 
			C1BTR = TSEG1 |TSEG2|199|SAM;
			break;
			case 2:
			C1BTR = TSEG1 |TSEG2|79|SAM;
			break;
			case 3:
			C1BTR = TSEG1 |TSEG2|39|SAM;
			break;
			case 4:
			C1BTR = TSEG1 |TSEG2|31|SAM;
			break;
			case 5:
			C1BTR = TSEG1 |TSEG2|15|SAM;
			break;
			case 6:
			C1BTR = TSEG1 |TSEG2|7|SAM;
			break;
			case 7:
			C1BTR = TSEG1 |TSEG2|4|SAM;
			break;
			case 8:
			C1BTR = TSEG1 |TSEG2|3|SAM;
			break;

			}
	
	C1MOD=0;
	CAN_SetBusMode ( CAN_BUS1, BUS_OFF);					// CAN Bus Off
	CAN_ReInitChannel ( CAN_BUS1);
	





}
// CAN_UserWrite()
// Send a message on CAN_BUSx
CANStatus_t  CAN_UserWrite ( CANHandle_t  hBus, CANMsg_t  *pBuff)
{
	CANStatus_t  ret;
	CANMsg_t  *pMsg;
	
	
	ret = CAN_ERR_OK;
	
	pMsg = CAN_TxQueueGetNext ( hBus);

	if ( pMsg != NULL)
	{
		pMsg->Id   = pBuff->Id;
		pMsg->Len  = pBuff->Len;
		pMsg->Type = pBuff->Type;
		
		pMsg->Data32[0] = pBuff->Data32[0];
		pMsg->Data32[1] = pBuff->Data32[1];
		
		// Send Msg
		ret = CAN_TxQueueWriteNext ( hBus);
	}
	
	else
	{
		// Tx Queue FULL
		ret = CAN_ERR_FAIL;
	}
	
	return ret;
}




// CAN_UserRead()
// read message from CAN_BUSx
u32_t  CAN_UserRead ( CANHandle_t  hBus, CANMsg_t  *pBuff)
{
	u32_t  ret;
	CANMsg_t  *pMsg;
	
	
	ret = 0;
	
	pMsg = CAN_RxQueueGetNext ( hBus);

	if ( pMsg != NULL)
	{
		pBuff->Id   = pMsg->Id;
		pBuff->Len  = pMsg->Len;
		pBuff->Type = pMsg->Type;
		
		pBuff->Data32[0] = pMsg->Data32[0];
		pBuff->Data32[1] = pMsg->Data32[1];
		
		CAN_RxQueueReadNext ( hBus);
		ret = 1;
	}
	
	return ret;
}




// CAN_UserInit()
// initialize CAN1 and CAN2
void  CAN_UserInit ( void)
{

	// init CAN1

	CAN_ReferenceTxQueue ( CAN_BUS1, &TxQueueCAN1[0], CAN1_TX_QUEUE_SIZE);				// Reference above Arrays as Queues
	CAN_ReferenceRxQueue ( CAN_BUS1, &RxQueueCAN1[0], CAN1_RX_QUEUE_SIZE);

	CAN_SetTimestampHandler ( CAN_BUS1, NULL);

	VICVectAddr1 = (u32_t) CAN_GetIsrVector ( CAN1_TX_INTSOURCE);
	VICVectAddr3 = (u32_t) CAN_GetIsrVector ( CAN1_RX_INTSOURCE);

	VICVectCntl1 = 1 << 5 | CAN1_TX_INTSOURCE;											// Setup VIC
	VICVectCntl3 = 1 << 5 | CAN1_RX_INTSOURCE;

	VICIntEnable = 1 << CAN1_TX_INTSOURCE | 1 << CAN1_RX_INTSOURCE;

	CAN_SetErrorLimit ( CAN_BUS1, STD_TX_ERRORLIMIT);

	CAN_SetTxErrorCallback ( CAN_BUS1, NULL);											// Set ErrorLimit & Callbacks
	CAN_SetRxCallback ( CAN_BUS1, NULL);

	CAN_SetChannelInfo ( CAN_BUS1, NULL);													// Textinfo is NULL

	

	// init CAN2

	CAN_ReferenceTxQueue ( CAN_BUS2, &TxQueueCAN2[0], CAN2_TX_QUEUE_SIZE);
	CAN_ReferenceRxQueue ( CAN_BUS2, &RxQueueCAN2[0], CAN2_RX_QUEUE_SIZE);				// See above

	CAN_SetTimestampHandler ( CAN_BUS2, NULL);

	VICVectAddr2 = (u32_t) CAN_GetIsrVector ( CAN2_TX_INTSOURCE);
	VICVectAddr4 = (u32_t) CAN_GetIsrVector ( CAN2_RX_INTSOURCE);

	VICVectCntl2 = 1 << 5 | CAN2_TX_INTSOURCE;
	VICVectCntl4 = 1 << 5 | CAN2_RX_INTSOURCE;

	VICIntEnable = 1 << CAN2_TX_INTSOURCE | 1 << CAN2_RX_INTSOURCE;

	CAN_SetErrorLimit ( CAN_BUS2, STD_TX_ERRORLIMIT);

	CAN_SetTxErrorCallback ( CAN_BUS2, NULL);
	CAN_SetRxCallback ( CAN_BUS2, NULL);

	CAN_SetChannelInfo ( CAN_BUS2, NULL);


	// Set Error Handler

	VICVectAddr0 = (u32_t) CAN_GetIsrVector ( GLOBAL_CAN_INTSOURCE);
	VICVectCntl0 = 1 << 5 | GLOBAL_CAN_INTSOURCE;
	VICIntEnable = 1 << GLOBAL_CAN_INTSOURCE;

	
	// Setup Filters

	CAN_InitFilters();										// Clear Filter LUT
	CAN_SetFilterMode ( AF_ON_BYPASS_ON);				// No Filters ( Bypassed)


	// init CAN1 and CAN2 with Values above
	IOSET1 = 0x00100000;
	//CAN_InitChannel ( CAN_BUS1, CAN_BAUD_500K);
	//CAN_InitChannel ( CAN_BUS2, CAN_BAUD_500K);
	
	
	CAN_InitChannelMio (CAN_BUS1, CAN_BAUD_500K);
	//
	CAN_SetTransceiverMode ( CAN_BUS1, CAN_TRANSCEIVER_MODE_NORMAL);
//	CAN_SetTransceiverMode ( CAN_BUS2, CAN_TRANSCEIVER_MODE_NORMAL);


	// Busses on

	CAN_SetBusMode ( CAN_BUS1, BUS_ON);					// CAN Bus On
//	CAN_SetBusMode ( CAN_BUS2, BUS_ON);

}


