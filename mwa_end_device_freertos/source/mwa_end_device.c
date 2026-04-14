/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
* All rights reserved.
*
* \file
*
* MyWirelessApp Demo End Device application.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "mwa_end_device.h"

/* Fwk */
#include "LED.h"
#include "Keyboard.h"
#include "SecLib.h"
#include "SerialManager.h"
#include "RNG_Interface.h"
#include "Panic.h"
#include "MemManager.h"
#include "TimersManager.h"
#include "FunctionLib.h"

//#if mEnterLowPowerWhenIdle_c
//  #include "PWR_Interface.h"
//#endif

#if gNvmTestActive_d
  #include "NVM_Interface.h"
#endif

/* 802.15.4 */
#include "PhyInterface.h"
#include "MacInterface.h"

/* KSDK */
#include "board.h"
#include "fsl_os_abstraction.h"

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define MY_PAN_ID   0x5555  // PAN ID 5555
#define MY_CHANNEL  0x15   // Canal 15
#define COORD_SHORT_ADDR  0x0000  //0000
static uint8_t mAppCounter = 0;
#define gAppEvtSW3Pressed_c (1 << 4)
#define gAppEvtSW4Pressed_c (1 << 5)
/* If there are too many pending packets to be send over the air, */
/* receive mMaxKeysToReceive_c chars. */
/* The chars will be send over the air when there are no pending packets*/
#define mMaxKeysToReceive_c 32

#define mAppStackSize_c 700
#define mAppTaskPrio_c  4

#if gNvmTestActive_d

  /* Current IDs for each data set. */
  #define mCoordInfo_ID_c          0x0010
  #define maMyAddress_ID_c         0x0011
  #define mAddrMode_ID_c           0x0012
#endif

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
/* Forward declarations of helper functions */
static void    UartRxCallBack(void*);
static uint8_t App_WaitMsg(nwkMessage_t *pMsg, uint8_t msgType);
static uint8_t App_SendAssociateRequest(void);
static uint8_t App_HandleAssociateConfirm(nwkMessage_t *pMsg);
static uint8_t App_HandleMlmeInput(nwkMessage_t *pMsg);
static void    App_HandleMcpsInput(mcpsToNwkMessage_t *pMsgIn);
static void    App_TransmitUartData(void);
static void    AppPollWaitTimeout(void *);
static void    App_HandleKeys( key_event_t events );

void App_init( void );
void AppThread (osaTaskParam_t argument);
void App_Idle_Task(uint32_t argument);
resultType_t MLME_NWK_SapHandler (nwkMessage_t* pMsg, instanceId_t instanceId);
resultType_t MCPS_NWK_SapHandler (mcpsToNwkMessage_t* pMsg, instanceId_t instanceId);
extern void Mac_SetExtendedAddress(uint8_t *pAddr, instanceId_t instanceId);

OSA_TASK_DEFINE( AppThread, mAppTaskPrio_c, 1, mAppStackSize_c, FALSE );

/* Information about the PAN we are part of */
static panDescriptor_t mCoordInfo;

/* This is either the short address assigned by the PAN coordinator
   during association, or our own extended MAC address. */
static uint8_t maMyAddress[8];
/* The devices address mode. If 2, then maMyAddress contains the short
   address assigned by the PAN coordinator. If 3, then maMyAddress is
   equal to the extended address. */
static addrModeType_t mAddrMode;

/* Data request packet for sending UART input to the coordinator */
static nwkToMcpsMessage_t *mpPacket;

/* The MSDU handle is a unique data packet identifier */
static uint8_t mMsduHandle;

/* Number of pending data packets */
static uint8_t mcPendingPackets;

/* Signals that an MLME-Poll request is pending, and that we must wait for
   the MLME-Poll confirm message before sending the next poll request. */
static bool_t mWaitPollConfirm;

/* Time between MLME-Poll requests */
static uint16_t mPollInterval;

/* Application input queues */
static anchor_t mMlmeNwkInputQueue;
static anchor_t mMcpsNwkInputQueue;

static tmrTimerID_t mTimer_c = gTmrInvalidTimerID_c;

static const uint64_t mExtendedAddress  = mMacExtendedAddress_c;
static instanceId_t   macInstance;
static uint8_t        interfaceId;
osaEventId_t          mAppEvent;
osaTaskId_t           mAppTaskHandler;

#if gNvmTestActive_d

static uint16_t timeoutCounter = 0;

/*
 * Datasets used by the NVM test application
 */

/* This data set contains application variables to be preserved across resets */
NVM_RegisterDataSet(&mCoordInfo,          1,   sizeof(panDescriptor_t), mCoordInfo_ID_c, gNVM_MirroredInRam_c);
NVM_RegisterDataSet(&maMyAddress,         1,   8, maMyAddress_ID_c, gNVM_MirroredInRam_c);
NVM_RegisterDataSet(&mAddrMode,           1,   sizeof(addrModeType_t), mAddrMode_ID_c, gNVM_MirroredInRam_c);
#endif

uint8_t gState;

void main_task(uint32_t param){
    static uint8_t initialized = FALSE;

    if( !initialized ){
        initialized = TRUE;  
        hardware_init();
        MEM_Init();
        TMR_Init();
        LED_Init();
        SecLib_Init();
        SerialManager_Init();
        Phy_Init();
        RNG_Init(); /* RNG must be initialized after the PHY is Initialized */
        MAC_Init();
//#if mEnterLowPowerWhenIdle_c
//        PWR_Init();
//        PWR_DisallowDeviceToSleep();
//#endif
#if gNvmTestActive_d
        NvModuleInit();
#endif

        /* Bind to MAC layer */
        macInstance = BindToMAC( (instanceId_t)0 );
        Mac_RegisterSapHandlers( MCPS_NWK_SapHandler, MLME_NWK_SapHandler, macInstance );

        App_init();

        /* Create application task */
        mAppTaskHandler = OSA_TaskCreate(OSA_TASK(AppThread), NULL);
        if( NULL == mAppTaskHandler )
        {
            panic(0,0,0,0);
            return;
        }
    }

    /* Call application Idle task */
    App_Idle_Task( param );
}

void App_Idle_Task(uint32_t argument)
{
//#if mEnterLowPowerWhenIdle_c
//    PWRLib_WakeupReason_t wakeupReason;
//#endif

    while(1)
    {
#if gNvmTestActive_d
        /* Process NV Storage save-on-idle and save-on-count requests. */
        NvIdle();
#endif

        if( !gUseRtos_c )
        {
            break;
        }
    }
}

void App_init( void )
{
    mAppEvent = OSA_EventCreate(TRUE);
    /* The initial application state */
    gState = stateInit;
    /* Reset number of pending packets */
    mcPendingPackets = 0;

    /* Allow sending a poll request */
    mWaitPollConfirm = FALSE;    

    /* Initialize the poll interval */
    mPollInterval = mDefaultValueOfPollIntervalSlow_c;
    
    /* Initialize the MAC 802.15.4 extended address */
    Mac_SetExtendedAddress( (uint8_t*)&mExtendedAddress, macInstance );
    
    mTimer_c = TMR_AllocateTimer();
    /* register keyboard callback function */
    KBD_Init(App_HandleKeys);

    /* Initialize the UART so that we can print out status messages */
    Serial_InitInterface(&interfaceId, APP_SERIAL_INTERFACE_TYPE, APP_SERIAL_INTERFACE_INSTANCE);
    Serial_SetBaudRate(interfaceId, gUARTBaudRate115200_c);
    Serial_SetRxCallBack(interfaceId, UartRxCallBack, NULL);

    /* Prepare input queues.*/
    MSG_InitQueue(&mMlmeNwkInputQueue); 
    MSG_InitQueue(&mMcpsNwkInputQueue);

    /*signal app ready*/
    LED_StartSerialFlash(LED1);

    Serial_Print(interfaceId, "\n\rPress any switch on board to start running the application.\n\r", gAllowToBlock_d);
#if gNvmTestActive_d
        Serial_Print(interfaceId, "Long press switch_1 on board to use MAC data restore from NVM.\n\r", gAllowToBlock_d);
#endif
}

static void App_UpdateLeds(uint8_t counter){
    Led1Off(); Led2Off(); Led3Off(); Led4Off();
    switch(counter)
    {
        case 0: // Magenta (Rojo + Azul)
            Led1On();
            Led3On();
            break;
        case 1: // Azul
            Led3On();
            break;
        case 2: // Rojo
            Led1On();
            break;
        case 3: // Verde
            Led2On();
            break;
    }
}
static void App_SendCounterPacket(uint8_t counter){
    nwkToMcpsMessage_t *pMsg = MSG_Alloc(sizeof(nwkToMcpsMessage_t) + 1);

    if(pMsg != NULL)
    {
        pMsg->msgType = gMcpsDataReq_c;

        pMsg->msgData.dataReq.pMsdu = (uint8_t*)pMsg + sizeof(nwkToMcpsMessage_t);

        *(pMsg->msgData.dataReq.pMsdu) = counter;

        FLib_MemCpy(&pMsg->msgData.dataReq.dstAddr, &mCoordInfo.coordAddress, 8);
        FLib_MemCpy(&pMsg->msgData.dataReq.srcAddr, maMyAddress, 8);

        FLib_MemCpy(&pMsg->msgData.dataReq.dstPanId, &mCoordInfo.coordPanId, 2);
        FLib_MemCpy(&pMsg->msgData.dataReq.srcPanId, &mCoordInfo.coordPanId, 2);

        pMsg->msgData.dataReq.dstAddrMode = mCoordInfo.coordAddrMode;
        pMsg->msgData.dataReq.srcAddrMode = mAddrMode;

        pMsg->msgData.dataReq.msduLength = 1;

        pMsg->msgData.dataReq.txOptions = gMacTxOptionsAck_c;
        pMsg->msgData.dataReq.msduHandle = mMsduHandle++;
        pMsg->msgData.dataReq.securityLevel = gMacSecurityNone_c;

        (void)NWK_MCPS_SapHandler(pMsg, macInstance);
    }
}

void AppThread(osaTaskParam_t argument)
{ 
    osaEventFlags_t ev;
    /* Pointer for storing the messages from MLME, MCPS, and ASP. */
    void *pMsgIn = NULL;
    /* Stores the status code returned by some functions. */
    uint8_t rc;
    
    while(1)
    {
        OSA_EventWait(mAppEvent, osaEventFlagsAll_c, FALSE, osaWaitForever_c, &ev);

        if( !gUseRtos_c && !ev)
        {
            break;
        }

        pMsgIn = NULL;

        /* Dequeue the MLME message */
        if( ev & gAppEvtMessageFromMLME_c )
        {
            /* Get the message from MLME */
            pMsgIn = MSG_DeQueue(&mMlmeNwkInputQueue);

            /* Any time a beacon might arrive. Always handle the beacon frame first */
            if (pMsgIn)
            {
                rc = App_WaitMsg(pMsgIn, gMlmeBeaconNotifyInd_c);
                if(rc == errorNoError)
                {
                    /* ALWAYS free the beacon frame contained in the beacon notify indication.*/
                    /* ALSO the application can use the beacon payload.*/
                    MSG_Free(((nwkMessage_t *)pMsgIn)->msgData.beaconNotifyInd.pBufferRoot);
                    Serial_Print(interfaceId, "Received an MLME-Beacon Notify Indication\n\r", gAllowToBlock_d);
                }
            }
        }
        
        /* The application state machine */
        switch(gState){
        	case stateInit:
        		Serial_Print(interfaceId, "Manual Configuration - Skipping Scan...\n\r", gAllowToBlock_d);

        		/* Forzamos la configuración del PAN */
        		mCoordInfo.coordPanId = MY_PAN_ID;
        		mCoordInfo.logicalChannel = MY_CHANNEL;
        		mCoordInfo.coordAddrMode = gAddrModeShortAddress_c;
                mCoordInfo.coordAddress = COORD_SHORT_ADDR;

                /* Configuramos parámetros del Beacon para que el stack no lo rechace */
                mCoordInfo.superframeSpec.associationPermit = 1;
                mCoordInfo.superframeSpec.beaconOrder = 0x0F; // Non-beacon network
                mCoordInfo.superframeSpec.superframeOrder = 0x0F;

                gState = stateAssociate;


                OSA_EventSet(mAppEvent, gAppEvtDummyEvent_c);
                break;

			case stateAssociate:
				/* Associate to the PAN coordinator */
				Serial_Print(interfaceId, "Associating to PAN coordinator on channel 0x", gAllowToBlock_d);
				Serial_PrintHex(interfaceId, &(mCoordInfo.logicalChannel), 1, gPrtHexNewLine_c);

				rc = App_SendAssociateRequest();
				if(rc == errorNoError)
					gState = stateAssociateWaitConfirm;
				break;
            
			case stateAssociateWaitConfirm:
			    if (ev & gAppEvtMessageFromMLME_c){
			        if (pMsgIn){
			            rc = App_WaitMsg(pMsgIn, gMlmeAssociateCnf_c);
			            if(rc == errorNoError){
			                rc = App_HandleAssociateConfirm(pMsgIn);
			                if (rc == errorNoError){
			                    /* --- INICIO DE LÓGICA REQUERIDA --- */
			                    mAppCounter = 0;
			                    App_UpdateLeds(mAppCounter);
			                    App_SendCounterPacket(mAppCounter);

			                    // Iniciamos el timer de 5 segundos estrictos
			                    TMR_StartLowPowerTimer(mTimer_c, gTmrSingleShotTimer_c, 5000, AppPollWaitTimeout, NULL);
			                    Serial_Print(interfaceId, " [EXITO] Conectado al Coordinador!\n\r", gAllowToBlock_d);
			                    Serial_Print(interfaceId, "\n\r Conectado a PAN ID 5555 CHANNEL 15\n\r", gAllowToBlock_d);


			                    gState = stateListen;
			                    OSA_EventSet(mAppEvent, gAppEvtDummyEvent_c);
			                }
        				else{
        					Serial_Print(interfaceId, "\n\r=================================================\n\r", gAllowToBlock_d);
        					Serial_Print(interfaceId, " [ERROR] No se pudo conectar al Coordinador\n\r", gAllowToBlock_d);
        					Serial_Print(interfaceId, " -> Verifique que el Coordinador PAN 0x5555 Canal 15 este encendido.\n\r", gAllowToBlock_d);
        					Serial_Print(interfaceId, " -> Reintentando conexion...\n\r", gAllowToBlock_d);
        					Serial_Print(interfaceId, "=================================================\n\r\n\r", gAllowToBlock_d);

        	#if gNvmTestActive_d
        					FLib_MemSet(&mCoordInfo, 0, sizeof(mCoordInfo));
        					FLib_MemSet(maMyAddress, 0, 8);
        					mAddrMode = gAddrModeNoAddress_c;
        					NvSaveOnIdle(&mCoordInfo, TRUE);
        					NvSaveOnIdle(&maMyAddress, TRUE);
        					NvSaveOnIdle(&mAddrMode, TRUE);
        	#endif
        					// si falla, volvemos al estado inicial para que asigne las variables y reintente
        					gState = stateInit;
        					OSA_EventSet(mAppEvent, gAppEvtDummyEvent_c);
        				}
        			}
        		}
        	}
        	break;

			case stateListen:
            if (ev & gAppEvtMessageFromMLME_c){
                if (pMsgIn){
                    rc = App_HandleMlmeInput(pMsgIn);
                }
            }

            if (ev & gAppEvtRxFromUart_c){
                /* get byte from UART */
                App_TransmitUartData();
            }
			#if gNvmTestActive_d
            if (timeoutCounter >= mDefaultValueOfTimeoutError_c)
            {
                  Serial_Print(interfaceId, "\n\rTimeout - No data received.\n\r\n\r", gAllowToBlock_d);
                  FLib_MemSet(&mCoordInfo, 0, sizeof(mCoordInfo));
                  FLib_MemSet(maMyAddress, 0, 8);
                  mAddrMode = gAddrModeNoAddress_c;
                  NvSaveOnIdle(&mCoordInfo, TRUE);
                  NvSaveOnIdle(&maMyAddress, TRUE);
                  NvSaveOnIdle(&mAddrMode, TRUE);
                  timeoutCounter = 0;
                  OSA_EventSet(mAppEvent, gAppEvtDummyEvent_c);
                  gState = stateInit;
            }
			#endif
            // Lógica para SW3 (Contador = 0)
            if (ev & gAppEvtSW3Pressed_c) {
            	mAppCounter = 0;
            	App_UpdateLeds(mAppCounter);
            	App_SendCounterPacket(mAppCounter);

                TMR_StopTimer(mTimer_c);
                TMR_StartLowPowerTimer(mTimer_c, gTmrSingleShotTimer_c, 5000, AppPollWaitTimeout, NULL);
                Serial_Print(interfaceId, "SW3 Presionado: Counter = 0\n\r", gAllowToBlock_d);
             }

             // Lógica para SW4 (Contador = 2)
             if (ev & gAppEvtSW4Pressed_c){
                mAppCounter = 2; // Especificación: SW4 pone el contador en 2
                App_UpdateLeds(mAppCounter);
                App_SendCounterPacket(mAppCounter);

                /* Reiniciamos el timer para que el próximo paquete sea en 5 segundos */
                TMR_StopTimer(mTimer_c);
                TMR_StartLowPowerTimer(mTimer_c, gTmrSingleShotTimer_c, 5000, AppPollWaitTimeout, NULL);
                Serial_Print(interfaceId, "SW4 Presionado: Counter = 2\n\r", gAllowToBlock_d);
              }
            break;
        }

        if (pMsgIn)
        {
            /* Messages must always be freed. */
            MSG_Free(pMsgIn);
            pMsgIn = NULL;
        }

        /* Handle MCPS confirms and transmit data from UART */
        if (ev & gAppEvtMessageFromMCPS_c)
        {      
            /* Get the message from MCPS */
            pMsgIn = MSG_DeQueue(&mMcpsNwkInputQueue);
            if (pMsgIn)
            {
                /* Process it */
                App_HandleMcpsInput(pMsgIn);
                /* Messages from the MCPS must always be freed. */
                MSG_Free(pMsgIn);
                pMsgIn = NULL;
            }
        }
        
        /* Check for pending messages in the Queue */
        if( MSG_Pending(&mMcpsNwkInputQueue) )
            OSA_EventSet(mAppEvent, gAppEvtMessageFromMCPS_c);
        if( MSG_Pending(&mMlmeNwkInputQueue) )
            OSA_EventSet(mAppEvent, gAppEvtMessageFromMLME_c);

        if( !gUseRtos_c )
        {
            break;
        }
    }
}

static void UartRxCallBack(void *pData)
{
    uint8_t pressedKey;
    uint16_t count;

    if( stateListen == gState )
    {
        OSA_EventSet(mAppEvent, gAppEvtRxFromUart_c);
        return;
    }

    if( gState == stateInit )
    {
        LED_StopFlashingAllLeds();
        OSA_EventSet(mAppEvent, gAppEvtDummyEvent_c);
    }

    do
    {
        Serial_GetByteFromRxBuffer(interfaceId, &pressedKey, &count);
    }while(count);

}

static uint8_t App_SendAssociateRequest(void)
{
  mlmeMessage_t *pMsg;
  mlmeAssociateReq_t *pAssocReq;

  Serial_Print(interfaceId, "Sending the MLME-Associate Request message to the MAC...", gAllowToBlock_d);

  /* Allocate a message for the MLME message. */
  pMsg = MSG_AllocType(mlmeMessage_t);
  if (pMsg != NULL)
  {
    /* This is a MLME-ASSOCIATE.req command. */
    pMsg->msgType = gMlmeAssociateReq_c;

    /* Create the Associate request message data. */
    pAssocReq = &pMsg->msgData.associateReq;

    /* Use the coordinator info we got from the Active Scan. */
    FLib_MemCpy(&pAssocReq->coordAddress, &mCoordInfo.coordAddress, 8);
    FLib_MemCpy(&pAssocReq->coordPanId,   &mCoordInfo.coordPanId, 2);
    pAssocReq->coordAddrMode      = mCoordInfo.coordAddrMode;
    pAssocReq->logicalChannel     = mCoordInfo.logicalChannel;
    pAssocReq->securityLevel      = gMacSecurityNone_c;
#ifdef gPHY_802_15_4g_d
    pAssocReq->channelPage = gChannelPageId9_c;
#else
    pAssocReq->channelPage = gDefaultChannelPageId_c;
#endif

    /* We want the coordinator to assign a short address to us. */
    pAssocReq->capabilityInfo     = gCapInfoAllocAddr_c;

    /* Send the Associate Request to the MLME. */
    if(NWK_MLME_SapHandler( pMsg, macInstance ) == gSuccess_c)
    {
      Serial_Print(interfaceId, "Done\n\r", gAllowToBlock_d);
      return errorNoError;
    }
    else
    {
      /* One or more parameters in the message were invalid. */
      Serial_Print(interfaceId, "Invalid parameter!\n\r", gAllowToBlock_d);
      return errorInvalidParameter;
    }
  }
  else
  {
    /* Allocation of a message buffer failed. */
    Serial_Print(interfaceId, "Message allocation failed!\n\r", gAllowToBlock_d);
    return errorAllocFailed;
  }
}

static uint8_t App_HandleAssociateConfirm(nwkMessage_t *pMsg)
{
  /* If the coordinator assigns a short address of 0xfffe then,
     that means we must use our own extended address in all
     communications with the coordinator. Otherwise, we use
     the short address assigned to us. */
  if ( pMsg->msgData.associateCnf.status == gSuccess_c)
  {

	  if( pMsg->msgData.associateCnf.assocShortAddress >= 0xFFFE)
	  {
	    mAddrMode = gAddrModeExtendedAddress_c;
	    FLib_MemCpy(maMyAddress, (void*)&mExtendedAddress, 8);
	  }
	  else
	  {
	    mAddrMode = gAddrModeShortAddress_c;
	    FLib_MemCpy(maMyAddress, &pMsg->msgData.associateCnf.assocShortAddress, 2);
	  }

	  return gSuccess_c;
  }

  else
  {
	return pMsg->msgData.associateCnf.status;
  }
}
/******************************************************************************
* The App_HandleMlmeInput(nwkMessage_t *pMsg) function will handle various
* messages from the MLME, e.g. poll confirm.
*
* The function may return either of the following values:
*   errorNoError:   The message was processed.
*   errorNoMessage: The message pointer is NULL.
******************************************************************************/
static uint8_t App_HandleMlmeInput(nwkMessage_t *pMsg)
{
//#if mEnterLowPowerWhenIdle_c
//  static uint8_t lowPowerCounter = 0;
//#endif
  if(pMsg == NULL)
    return errorNoMessage;

  /* Handle the incoming message. The type determines the sort of processing.*/
  switch(pMsg->msgType) {
  case gMlmePollCnf_c:
    if(pMsg->msgData.pollCnf.status != gSuccess_c)
    {
      /* The Poll Confirm status was not successful. Usually this happens if
         no data was available at the coordinator. In this case we start
         polling at a lower rate to conserve power. */
      mPollInterval = mDefaultValueOfPollIntervalSlow_c;

      /* If we get to this point, then no data was available, and we
         allow a new poll request. Otherwise, we wait for the data
         indication before allowing the next poll request. */
      mWaitPollConfirm = FALSE;
#if gNvmTestActive_d
      if(pMsg->msgData.pollCnf.status == gNoAck_c)
      {
          timeoutCounter++;
      }
      else
      {
          timeoutCounter = 0;
      }
#endif
    }
    break;

  default:
    break;
  }
  return errorNoError;
}

/******************************************************************************
* The App_HandleMcpsInput(mcpsToNwkMessage_t *pMsgIn) function will handle
* messages from the MCPS, e.g. Data Confirm, and Data Indication.
*
******************************************************************************/
static void App_HandleMcpsInput(mcpsToNwkMessage_t *pMsgIn)
{
  switch(pMsgIn->msgType)
  {
    /* The MCPS-Data confirm is sent by the MAC to the network
       or application layer when data has been sent. */
  case gMcpsDataCnf_c:
    if(mcPendingPackets)
      mcPendingPackets--;
    break;

  case gMcpsDataInd_c:
    /* Copy the received data to the UART. */
    Serial_SyncWrite(interfaceId, pMsgIn->msgData.dataInd.pMsdu, pMsgIn->msgData.dataInd.msduLength);
    /* Since we received data, the coordinator might have more to send. We
       reduce the polling interval to raise the throughput while data is
       available. */
    mPollInterval = mDefaultValueOfPollIntervalFast_c;
    /* Allow another MLME-Poll request. */
    mWaitPollConfirm = FALSE;
    break;

  default:
    break;
  }
}

/******************************************************************************
* The App_WaitMsg(nwkMessage_t *pMsg, uint8_t msgType) function does not, as
* the name implies, wait for a message, thus blocking the execution of the
* state machine. Instead the function analyzes the supplied message to
* determine whether or not the message is of the expected type.
* The function may return either of the following values:
*   errorNoError: The message was of the expected type.
*   errorNoMessage: The message pointer is NULL.
*   errorWrongConfirm: The message is not of the expected type.
*
******************************************************************************/
static uint8_t App_WaitMsg(nwkMessage_t *pMsg, uint8_t msgType)
{
  /* Do we have a message? If not, the exit with error code */
  if(pMsg == NULL)
    return errorNoMessage;

  /* Is it the expected message type? If not then exit with error code */
  if(pMsg->msgType != msgType)
    return errorWrongConfirm;

  /* Found the expected message. Return with success code */
  return errorNoError;
}

/******************************************************************************
* The App_TransmitUartData() function will perform (single/multi buffered)
* data transmissions of data received by the UART. Data could also come from
* other sources such as sensors etc. This is completely determined by the
* application. The constant mDefaultValueOfMaxPendingDataPackets_c determine the maximum
* number of packets pending for transmission in the MAC. A global variable
* is incremented each time a data packet is sent to the MCPS, and decremented
* when the corresponding MCPS-Data Confirm message is received. If the counter
* reaches the defined maximum no more data buffers are allocated until the
* counter is decreased below the maximum number of pending packets.
*
* The function uses the coordinator information gained during the Active Scan,
* and the short address assigned to us by coordinator, for building an MCPS-
* Data Request message. The message is sent to the MCPS service access point
* in the MAC.
******************************************************************************/
static void App_TransmitUartData(void)
{
    uint16_t count;

    /* Count bytes receive over the serial interface */
    Serial_RxBufferByteCount(interfaceId, &count);

    if( 0 == count )
    {
        return;
    }

    /* Limit data transfer size */
    if( count > mMaxKeysToReceive_c )
    {
        count = mMaxKeysToReceive_c;
    }

    /* Use multi buffering for increased TX performance. It does not really
    have any effect at low UART baud rates, but serves as an
    example of how the throughput may be improved in a real-world
    application where the data rate is of concern. */
    if( (mcPendingPackets < mDefaultValueOfMaxPendingDataPackets_c) && (mpPacket == NULL) )
    {
        /* If the maximum number of pending data buffes is below maximum limit
        and we do not have a data buffer already then allocate one. */
        mpPacket = MSG_Alloc(sizeof(nwkToMcpsMessage_t) + gMaxPHYPacketSize_c);
    }

    if(mpPacket != NULL)
    {
        /* Data is available in the SerialManager's receive buffer. Now create an
        MCPS-Data Request message containing the data. */
        mpPacket->msgType = gMcpsDataReq_c;
        mpPacket->msgData.dataReq.pMsdu = (uint8_t*)(&mpPacket->msgData.dataReq.pMsdu) +
                                          sizeof(mpPacket->msgData.dataReq.pMsdu);
        Serial_Read(interfaceId, mpPacket->msgData.dataReq.pMsdu, count, &count);
        /* Create the header using coordinator information gained during
        the scan procedure. Also use the short address we were assigned
        by the coordinator during association. */
        FLib_MemCpy(&mpPacket->msgData.dataReq.dstAddr, &mCoordInfo.coordAddress, 8);
        FLib_MemCpy(&mpPacket->msgData.dataReq.srcAddr, &maMyAddress, 8);
        FLib_MemCpy(&mpPacket->msgData.dataReq.dstPanId, &mCoordInfo.coordPanId, 2);
        FLib_MemCpy(&mpPacket->msgData.dataReq.srcPanId, &mCoordInfo.coordPanId, 2);
        mpPacket->msgData.dataReq.dstAddrMode = mCoordInfo.coordAddrMode;
        mpPacket->msgData.dataReq.srcAddrMode = mAddrMode;
        mpPacket->msgData.dataReq.msduLength = count;
        /* Request MAC level acknowledgement of the data packet */
        mpPacket->msgData.dataReq.txOptions = gMacTxOptionsAck_c;
        /* Give the data packet a handle. The handle is
        returned in the MCPS-Data Confirm message. */
        mpPacket->msgData.dataReq.msduHandle = mMsduHandle++;
        /* Don't use security */
        mpPacket->msgData.dataReq.securityLevel = gMacSecurityNone_c;

        /* Send the Data Request to the MCPS */
        (void)NWK_MCPS_SapHandler(mpPacket, macInstance);

        /* Prepare for another data buffer */
        mpPacket = NULL;
        mcPendingPackets++;
    }

    /* If the data wasn't send over the air because there are too many pending packets,
    or new data has beed received, try to send it later   */
    Serial_RxBufferByteCount(interfaceId, &count);

    if( count )
    {
        OSA_EventSet(mAppEvent, gAppEvtRxFromUart_c);
    }
}

/******************************************************************************
* The App_ReceiveUartData() function will check if it is time to send out an
* MLME-Poll request in order to receive data from the coordinator. If its time,
* and we are permitted then a poll request is created and sent.
*
* The function uses the coordinator information gained during the Active Scan
* for building the MLME-Poll Request message. The message is sent to the MLME
* service access point in the MAC.
******************************************************************************/
static void AppPollWaitTimeout(void *pData)
{
    /* 1. Incrementar contador rango 0-3 */
    mAppCounter = (mAppCounter + 1) % 4;

    /* 2. Actualizar LEDs */
    App_UpdateLeds(mAppCounter);

    /* 3. Enviar paquete */
    App_SendCounterPacket(mAppCounter);

    /* 4. Reiniciar Timer (5 segundos) */
    TMR_StartLowPowerTimer(mTimer_c, gTmrSingleShotTimer_c, 5000, AppPollWaitTimeout, NULL);
}

/*****************************************************************************
* Handles all key events for this device.
* Interface assumptions: None
* Return value: None
*****************************************************************************/
static void App_HandleKeys(key_event_t events  ){
#if gKBD_KeysCount_c > 0
    switch ( events ){
    case gKBD_EventLongSW1_c:
        OSA_EventSet(mAppEvent, gAppEvtPressedRestoreNvmBut_c);
        break;

    case gKBD_EventSW3_c:
    	if(gState == stateListen) {
    		OSA_EventSet(mAppEvent, gAppEvtSW3Pressed_c);
    	}
    	break;
    case gKBD_EventSW4_c:
    	if(gState == stateListen) {
    		OSA_EventSet(mAppEvent, gAppEvtSW4Pressed_c);
    	}
    	break;
    default:
    	break;
#if gTsiSupported_d
    case gKBD_EventSW5_c:
    case gKBD_EventSW6_c:
#endif
#if gTsiSupported_d
    case gKBD_EventLongSW5_c:
    case gKBD_EventLongSW6_c:
#endif
        if(gState == stateInit)
        {
            LED_StopFlashingAllLeds();
            OSA_EventSet(mAppEvent, gAppEvtDummyEvent_c);
        }
    }
#endif
}

/******************************************************************************
* The following functions are called by the MAC to put messages into the
* Application's queue. They need to be defined even if they are not used
* in order to avoid linker errors.
******************************************************************************/

resultType_t MLME_NWK_SapHandler (nwkMessage_t* pMsg, instanceId_t instanceId)
{
  /* Put the incoming MLME message in the applications input queue. */
  MSG_Queue(&mMlmeNwkInputQueue, pMsg);
  OSA_EventSet(mAppEvent, gAppEvtMessageFromMLME_c);
  return gSuccess_c;
}

resultType_t MCPS_NWK_SapHandler (mcpsToNwkMessage_t* pMsg, instanceId_t instanceId)
{
  /* Put the incoming MCPS message in the applications input queue. */
  MSG_Queue(&mMcpsNwkInputQueue, pMsg);
  OSA_EventSet(mAppEvent, gAppEvtMessageFromMCPS_c);
  return gSuccess_c;
}
