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

/* ---- Configuracion de red: Equipo 5 ---- */
#define MY_PAN_ID          0x5555
#define MY_CHANNEL         0x0F   /* Canal 15 decimal = 0x0F */
#define COORD_SHORT_ADDR   0x0000

/* Contador de la aplicacion [0-3] */
static uint8_t mAppCounter = 0;

/* If there are too many pending packets to be send over the air, */
/* receive mMaxKeysToReceive_c chars. */
/* The chars will be send over the air when there are no pending packets */
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
static void    App_UpdateLeds(uint8_t counter);
static void    App_SendCounterPacket(uint8_t counter);

void App_init( void );
void AppThread (osaTaskParam_t argument);
void App_Idle_Task(uint32_t argument);
resultType_t MLME_NWK_SapHandler (nwkMessage_t* pMsg, instanceId_t instanceId);
resultType_t MCPS_NWK_SapHandler (mcpsToNwkMessage_t* pMsg, instanceId_t instanceId);
extern void Mac_SetExtendedAddress(uint8_t *pAddr, instanceId_t instanceId);

OSA_TASK_DEFINE( AppThread, mAppTaskPrio_c, 1, mAppStackSize_c, FALSE );

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/

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

/* Timer ID para el timer de 5 segundos */
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

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

void main_task(uint32_t param)
{
    static uint8_t initialized = FALSE;

    if( !initialized )
    {
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

/*****************************************************************************
* App_init
*
* Interface assumptions: None
* Return value: None
*****************************************************************************/
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

    /* Allocate timer for counter */
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

/*****************************************************************************
* App_UpdateLeds
*
* Actualiza el LED RGB segun el valor del contador usando LED_SetRgbLed().
* En la FRDM-KW41Z el LED RGB se controla via PWM con (R, G, B).
* LED_MAX_RGB_VALUE_c = 0xFF
*
*   0 = Magenta (Rojo + Azul)
*   1 = Azul
*   2 = Rojo
*   3 = Verde
*****************************************************************************/
static void App_UpdateLeds(uint8_t counter)
{
    /* Primero apagar: poner todo a 0 */
    LED_SetRgbLed(LED_RGB, 0, 0, 0);

    switch(counter)
    {
        case 0: /* Magenta = Rojo + Azul */
            LED_SetRgbLed(LED_RGB, LED_MAX_RGB_VALUE_c, 0, LED_MAX_RGB_VALUE_c);
            break;
        case 1: /* Azul */
            LED_SetRgbLed(LED_RGB, 0, 0, LED_MAX_RGB_VALUE_c);
            break;
        case 2: /* Rojo */
            LED_SetRgbLed(LED_RGB, LED_MAX_RGB_VALUE_c, 0, 0);
            break;
        case 3: /* Verde */
            LED_SetRgbLed(LED_RGB, 0, LED_MAX_RGB_VALUE_c, 0);
            break;
        default:
            break;
    }
}

/*****************************************************************************
* App_SendCounterPacket
*
* Envia un paquete MAC al coordinador con el contenido "Counter: x"
* donde x es el valor actual del contador [0-3].
*****************************************************************************/
static void App_SendCounterPacket(uint8_t counter)
{
    /* Buffer para el string "Counter: x" = 10 chars + null */
    uint8_t txBuffer[12];
    uint8_t txLength;

    /* Construir el string del payload */
    txBuffer[0] = 'C';
    txBuffer[1] = 'o';
    txBuffer[2] = 'u';
    txBuffer[3] = 'n';
    txBuffer[4] = 't';
    txBuffer[5] = 'e';
    txBuffer[6] = 'r';
    txBuffer[7] = ':';
    txBuffer[8] = ' ';
    txBuffer[9] = '0' + (counter % 4);  /* ASCII '0','1','2','3' */
    txLength = 10;

    /* Verificar que no hay demasiados paquetes pendientes */
    if( mcPendingPackets >= mDefaultValueOfMaxPendingDataPackets_c )
    {
        return;
    }

    /* Allocate message with enough space for the payload */
    nwkToMcpsMessage_t *pMsg = MSG_Alloc(sizeof(nwkToMcpsMessage_t) + txLength);

    if(pMsg != NULL)
    {
        pMsg->msgType = gMcpsDataReq_c;

        /* Set the MSDU pointer to the space after the message struct */
        pMsg->msgData.dataReq.pMsdu = (uint8_t*)(&pMsg->msgData.dataReq.pMsdu) +
                                       sizeof(pMsg->msgData.dataReq.pMsdu);

        /* Copy the payload into the MSDU */
        FLib_MemCpy(pMsg->msgData.dataReq.pMsdu, txBuffer, txLength);

        /* Fill in the destination (coordinator) */
        FLib_MemCpy(&pMsg->msgData.dataReq.dstAddr, &mCoordInfo.coordAddress, 8);
        FLib_MemCpy(&pMsg->msgData.dataReq.srcAddr, maMyAddress, 8);
        FLib_MemCpy(&pMsg->msgData.dataReq.dstPanId, &mCoordInfo.coordPanId, 2);
        FLib_MemCpy(&pMsg->msgData.dataReq.srcPanId, &mCoordInfo.coordPanId, 2);

        pMsg->msgData.dataReq.dstAddrMode = mCoordInfo.coordAddrMode;
        pMsg->msgData.dataReq.srcAddrMode = mAddrMode;
        pMsg->msgData.dataReq.msduLength = txLength;

        /* Request MAC level acknowledgement */
        pMsg->msgData.dataReq.txOptions = gMacTxOptionsAck_c;
        pMsg->msgData.dataReq.msduHandle = mMsduHandle++;
        /* Don't use security */
        pMsg->msgData.dataReq.securityLevel = gMacSecurityNone_c;

        /* Send the Data Request to the MCPS */
        (void)NWK_MCPS_SapHandler(pMsg, macInstance);
        mcPendingPackets++;

        /* Print to UART what we sent */
        Serial_Print(interfaceId, "TX -> Counter: ", gAllowToBlock_d);
        Serial_PrintDec(interfaceId, (uint32_t)counter);
        Serial_Print(interfaceId, "\n\r", gAllowToBlock_d);
    }
}

/*****************************************************************************
* AppThread - Main application task
*****************************************************************************/
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
                    MSG_Free(((nwkMessage_t *)pMsgIn)->msgData.beaconNotifyInd.pBufferRoot);
                    Serial_Print(interfaceId, "Received an MLME-Beacon Notify Indication\n\r", gAllowToBlock_d);
                }
            }
        }

        /* ============================================================ */
        /*                Application State Machine                     */
        /* ============================================================ */
        switch(gState)
        {
            case stateInit:
            {
                Serial_Print(interfaceId, "\n\r=================================================\n\r", gAllowToBlock_d);
                Serial_Print(interfaceId, " End Device: Equipo 5\n\r", gAllowToBlock_d);
                Serial_Print(interfaceId, " PAN ID: 0x5555 | Canal: 15 (0x0F)\n\r", gAllowToBlock_d);
                Serial_Print(interfaceId, "=================================================\n\r", gAllowToBlock_d);

                /* Configuracion manual - saltamos el Active Scan */
                mCoordInfo.coordPanId = MY_PAN_ID;
                mCoordInfo.logicalChannel = MY_CHANNEL;
                mCoordInfo.coordAddrMode = gAddrModeShortAddress_c;
                mCoordInfo.coordAddress = COORD_SHORT_ADDR;

                /* Configuramos parametros del superframe para non-beacon */
                mCoordInfo.superframeSpec.associationPermit = 1;
                mCoordInfo.superframeSpec.beaconOrder = 0x0F;
                mCoordInfo.superframeSpec.superframeOrder = 0x0F;

                gState = stateAssociate;
                OSA_EventSet(mAppEvent, gAppEvtDummyEvent_c);
                break;
            }

            case stateAssociate:
            {
                /* Associate to the PAN coordinator */
                Serial_Print(interfaceId, "Sending Association Request on channel 0x", gAllowToBlock_d);
                Serial_PrintHex(interfaceId, &(mCoordInfo.logicalChannel), 1, gPrtHexNewLine_c);

                rc = App_SendAssociateRequest();
                if(rc == errorNoError)
                    gState = stateAssociateWaitConfirm;
                break;
            }

            case stateAssociateWaitConfirm:
            {
                if (ev & gAppEvtMessageFromMLME_c)
                {
                    if (pMsgIn)
                    {
                        rc = App_WaitMsg(pMsgIn, gMlmeAssociateCnf_c);
                        if(rc == errorNoError)
                        {
                            rc = App_HandleAssociateConfirm(pMsgIn);
                            if (rc == errorNoError)
                            {
                                Serial_Print(interfaceId, "\n\r [OK] Asociado al Coordinador!\n\r", gAllowToBlock_d);
                                Serial_Print(interfaceId, " PAN ID: 0x5555 | Canal: 15\n\r", gAllowToBlock_d);
                                Serial_Print(interfaceId, " Short Address asignada: 0x", gAllowToBlock_d);
                                Serial_PrintHex(interfaceId, maMyAddress, 2, gPrtHexNewLine_c);

                                /* --- INICIO DE LOGICA DE FASE 1 --- */

                                /* Inicializar counter en 0 */
                                mAppCounter = 0;

                                /* Reflejar en LEDs */
                                App_UpdateLeds(mAppCounter);

                                /* Enviar primer paquete "Counter: 0" */
                                App_SendCounterPacket(mAppCounter);

                                /* Iniciar timer de 5 segundos */
                                TMR_StartLowPowerTimer(mTimer_c,
                                                       gTmrSingleShotTimer_c,
                                                       5000,
                                                       AppPollWaitTimeout,
                                                       NULL);

                                gState = stateListen;
                                OSA_EventSet(mAppEvent, gAppEvtDummyEvent_c);
                            }
                            else
                            {
                                Serial_Print(interfaceId, "\n\r [ERROR] Asociacion fallida.\n\r", gAllowToBlock_d);
                                Serial_Print(interfaceId, " Verifique que el Coordinador este activo.\n\r", gAllowToBlock_d);
                                Serial_Print(interfaceId, " Reintentando...\n\r", gAllowToBlock_d);

#if gNvmTestActive_d
                                FLib_MemSet(&mCoordInfo, 0, sizeof(mCoordInfo));
                                FLib_MemSet(maMyAddress, 0, 8);
                                mAddrMode = gAddrModeNoAddress_c;
                                NvSaveOnIdle(&mCoordInfo, TRUE);
                                NvSaveOnIdle(&maMyAddress, TRUE);
                                NvSaveOnIdle(&mAddrMode, TRUE);
#endif
                                gState = stateInit;
                                OSA_EventSet(mAppEvent, gAppEvtDummyEvent_c);
                            }
                        }
                    }
                }
                break;
            }

            case stateListen:
            {
                /* Handle MLME messages (e.g. poll confirms) */
                if (ev & gAppEvtMessageFromMLME_c)
                {
                    if (pMsgIn)
                    {
                        rc = App_HandleMlmeInput(pMsgIn);
                    }
                }

                /* Handle UART data */
                if (ev & gAppEvtRxFromUart_c)
                {
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

                /* ---- SW3 Pressed: Counter = 0 ---- */
                if (ev & gAppEvtSW3Pressed_c)
                {
                    mAppCounter = 0;
                    App_UpdateLeds(mAppCounter);
                    App_SendCounterPacket(mAppCounter);

                    /* Reiniciar timer de 5 segundos */
                    TMR_StopTimer(mTimer_c);
                    TMR_StartLowPowerTimer(mTimer_c, gTmrSingleShotTimer_c,
                                           5000, AppPollWaitTimeout, NULL);

                    Serial_Print(interfaceId, "SW3 -> Counter = 0\n\r", gAllowToBlock_d);
                }

                /* ---- SW4 Pressed: Counter = 2 ---- */
                if (ev & gAppEvtSW4Pressed_c)
                {
                    mAppCounter = 2;
                    App_UpdateLeds(mAppCounter);
                    App_SendCounterPacket(mAppCounter);

                    /* Reiniciar timer de 5 segundos */
                    TMR_StopTimer(mTimer_c);
                    TMR_StartLowPowerTimer(mTimer_c, gTmrSingleShotTimer_c,
                                           5000, AppPollWaitTimeout, NULL);

                    Serial_Print(interfaceId, "SW4 -> Counter = 2\n\r", gAllowToBlock_d);
                }
                break;
            }
        } /* end switch(gState) */

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

/*****************************************************************************
* AppPollWaitTimeout
*
* Callback del timer de 5 segundos.
* Incrementa el counter, actualiza LEDs, envia paquete, y reinicia el timer.
*****************************************************************************/
static void AppPollWaitTimeout(void *pData)
{
    /* 1. Incrementar contador en rango [0-3] */
    mAppCounter = (mAppCounter + 1) % 4;

    /* 2. Actualizar LEDs */
    App_UpdateLeds(mAppCounter);

    /* 3. Enviar paquete "Counter: x" */
    App_SendCounterPacket(mAppCounter);

    /* 4. Reiniciar Timer (single shot, 5 segundos) */
    TMR_StartLowPowerTimer(mTimer_c, gTmrSingleShotTimer_c,
                           5000, AppPollWaitTimeout, NULL);
}

/*****************************************************************************
* UartRxCallBack
*****************************************************************************/
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
    } while(count);
}

/*****************************************************************************
* Handles all key events for this device.
*
* En la FRDM-KW41Z con gKBD_KeysCount_c = 2:
*   Boton fisico SW3 del board -> genera gKBD_EventSW1_c
*   Boton fisico SW4 del board -> genera gKBD_EventSW2_c
*****************************************************************************/
static void App_HandleKeys(key_event_t events)
{
#if gKBD_KeysCount_c > 0
    switch ( events )
    {
    case gKBD_EventLongSW1_c:
        OSA_EventSet(mAppEvent, gAppEvtPressedRestoreNvmBut_c);
        break;

    case gKBD_EventSW1_c:  /* Boton fisico SW3 */
        if(gState == stateListen)
        {
            OSA_EventSet(mAppEvent, gAppEvtSW3Pressed_c);
        }
        break;

    case gKBD_EventSW2_c:  /* Boton fisico SW4 */
        if(gState == stateListen)
        {
            OSA_EventSet(mAppEvent, gAppEvtSW4Pressed_c);
        }
        break;

    default:
        if(gState == stateInit)
        {
            LED_StopFlashingAllLeds();
            OSA_EventSet(mAppEvent, gAppEvtDummyEvent_c);
        }
        break;
    }
#endif
}

/*****************************************************************************
* App_SendAssociateRequest
*****************************************************************************/
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

    /* Use the coordinator info we configured manually. */
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
      Serial_Print(interfaceId, "Invalid parameter!\n\r", gAllowToBlock_d);
      return errorInvalidParameter;
    }
  }
  else
  {
    Serial_Print(interfaceId, "Message allocation failed!\n\r", gAllowToBlock_d);
    return errorAllocFailed;
  }
}

/*****************************************************************************
* App_HandleAssociateConfirm
*****************************************************************************/
static uint8_t App_HandleAssociateConfirm(nwkMessage_t *pMsg)
{
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

/*****************************************************************************
* App_HandleMlmeInput
*****************************************************************************/
static uint8_t App_HandleMlmeInput(nwkMessage_t *pMsg)
{
  if(pMsg == NULL)
    return errorNoMessage;

  switch(pMsg->msgType)
  {
  case gMlmePollCnf_c:
    if(pMsg->msgData.pollCnf.status != gSuccess_c)
    {
      mPollInterval = mDefaultValueOfPollIntervalSlow_c;
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

/*****************************************************************************
* App_HandleMcpsInput
*****************************************************************************/
static void App_HandleMcpsInput(mcpsToNwkMessage_t *pMsgIn)
{
  switch(pMsgIn->msgType)
  {
  case gMcpsDataCnf_c:
    if(mcPendingPackets)
      mcPendingPackets--;
    break;

  case gMcpsDataInd_c:
    /* Copy the received data to the UART. */
    Serial_SyncWrite(interfaceId, pMsgIn->msgData.dataInd.pMsdu, pMsgIn->msgData.dataInd.msduLength);
    mPollInterval = mDefaultValueOfPollIntervalFast_c;
    mWaitPollConfirm = FALSE;
    break;

  default:
    break;
  }
}

/*****************************************************************************
* App_WaitMsg
*****************************************************************************/
static uint8_t App_WaitMsg(nwkMessage_t *pMsg, uint8_t msgType)
{
  if(pMsg == NULL)
    return errorNoMessage;

  if(pMsg->msgType != msgType)
    return errorWrongConfirm;

  return errorNoError;
}

/*****************************************************************************
* App_TransmitUartData
*****************************************************************************/
static void App_TransmitUartData(void)
{
    uint16_t count;

    Serial_RxBufferByteCount(interfaceId, &count);

    if( 0 == count )
    {
        return;
    }

    if( count > mMaxKeysToReceive_c )
    {
        count = mMaxKeysToReceive_c;
    }

    if( (mcPendingPackets < mDefaultValueOfMaxPendingDataPackets_c) && (mpPacket == NULL) )
    {
        mpPacket = MSG_Alloc(sizeof(nwkToMcpsMessage_t) + gMaxPHYPacketSize_c);
    }

    if(mpPacket != NULL)
    {
        mpPacket->msgType = gMcpsDataReq_c;
        mpPacket->msgData.dataReq.pMsdu = (uint8_t*)(&mpPacket->msgData.dataReq.pMsdu) +
                                          sizeof(mpPacket->msgData.dataReq.pMsdu);
        Serial_Read(interfaceId, mpPacket->msgData.dataReq.pMsdu, count, &count);

        FLib_MemCpy(&mpPacket->msgData.dataReq.dstAddr, &mCoordInfo.coordAddress, 8);
        FLib_MemCpy(&mpPacket->msgData.dataReq.srcAddr, &maMyAddress, 8);
        FLib_MemCpy(&mpPacket->msgData.dataReq.dstPanId, &mCoordInfo.coordPanId, 2);
        FLib_MemCpy(&mpPacket->msgData.dataReq.srcPanId, &mCoordInfo.coordPanId, 2);
        mpPacket->msgData.dataReq.dstAddrMode = mCoordInfo.coordAddrMode;
        mpPacket->msgData.dataReq.srcAddrMode = mAddrMode;
        mpPacket->msgData.dataReq.msduLength = count;
        mpPacket->msgData.dataReq.txOptions = gMacTxOptionsAck_c;
        mpPacket->msgData.dataReq.msduHandle = mMsduHandle++;
        mpPacket->msgData.dataReq.securityLevel = gMacSecurityNone_c;

        (void)NWK_MCPS_SapHandler(mpPacket, macInstance);

        mpPacket = NULL;
        mcPendingPackets++;
    }

    Serial_RxBufferByteCount(interfaceId, &count);

    if( count )
    {
        OSA_EventSet(mAppEvent, gAppEvtRxFromUart_c);
    }
}

/******************************************************************************
* MAC SAP Handlers
******************************************************************************/

resultType_t MLME_NWK_SapHandler (nwkMessage_t* pMsg, instanceId_t instanceId)
{
  MSG_Queue(&mMlmeNwkInputQueue, pMsg);
  OSA_EventSet(mAppEvent, gAppEvtMessageFromMLME_c);
  return gSuccess_c;
}

resultType_t MCPS_NWK_SapHandler (mcpsToNwkMessage_t* pMsg, instanceId_t instanceId)
{
  MSG_Queue(&mMcpsNwkInputQueue, pMsg);
  OSA_EventSet(mAppEvent, gAppEvtMessageFromMCPS_c);
  return gSuccess_c;
}
