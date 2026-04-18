/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
* All rights reserved.
*
* \file
*
* MyWirelessApp Demo Coordinator application.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "mwa_coordinator.h"

/* Drv */
#include "LED.h"
#include "Keyboard.h"

/* Fwk */
#include "SecLib.h"
#include "SerialManager.h"
#include "RNG_Interface.h"
#include "MemManager.h"
#include "TimersManager.h"
#include "FunctionLib.h"

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
#define mMaxKeysToReceive_c 32

/* Canal 15 decimal = 0x0F */
#define MY_CHANNEL  0x0F

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
static void    UartRxCallBack(void*);
static uint8_t App_StartCoordinator( uint8_t appInstance );
static uint8_t App_HandleMlmeInput(nwkMessage_t *pMsg, uint8_t appInstance);
static uint8_t App_SendAssociateResponse(nwkMessage_t *pMsgIn, uint8_t appInstance);
static void    App_HandleMcpsInput(mcpsToNwkMessage_t *pMsgIn, uint8_t appInstance);
static void    App_TransmitUartData(void);
static uint8_t App_WaitMsg(nwkMessage_t *pMsg, uint8_t msgType);
static void    App_HandleKeys(uint8_t events);
static void    App_UpdateLeds(uint8_t counter);
static void    App_PrintNodeInfo(uint8_t index);

void App_init( void );
void AppThread (uint32_t argument);
resultType_t MLME_NWK_SapHandler (nwkMessage_t* pMsg, instanceId_t instanceId);
resultType_t MCPS_NWK_SapHandler (mcpsToNwkMessage_t* pMsg, instanceId_t instanceId);
extern void Mac_SetExtendedAddress(uint8_t *pAddr, instanceId_t instanceId);

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/

/* The short address and PAN ID of the coordinator */
static const uint16_t mShortAddress = mDefaultValueOfShortAddress_c;
static const uint16_t mPanId = mDefaultValueOfPanId_c;

/* The current logical channel (frequency band) */
static uint8_t mLogicalChannel;

/* ----- Fase 2: Estructura de nodos asociados ----- */
static nodeInfo_t mAssociatedNodes[MAX_ASSOCIATED_NODES];

/* Siguiente short address a asignar (empieza en 0x0001) */
static uint16_t mNextShortAddress = 0x0001;

/* Data request packet for sending UART input to the end device */
static nwkToMcpsMessage_t *mpPacket;

/* The MSDU handle is a unique data packet identifier */
static uint8_t mMsduHandle;

/* Number of pending data packets */
static uint8_t mcPendingPackets;

/* Application input queues */
static anchor_t mMlmeNwkInputQueue;
static anchor_t mMcpsNwkInputQueue;

static const uint64_t mExtendedAddress = mMacExtendedAddress_c;
static instanceId_t   macInstance;
static uint8_t        interfaceId;
osaEventId_t          mAppEvent;

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

        /* Bind to MAC layer */
        macInstance = BindToMAC( (instanceId_t)0 );
        Mac_RegisterSapHandlers( MCPS_NWK_SapHandler, MLME_NWK_SapHandler, macInstance );

        App_init();
    }

    /* Call application task */
    AppThread( param );
}

/*****************************************************************************
* App_init
*****************************************************************************/
void App_init( void )
{
    uint8_t i;

    mAppEvent = OSA_EventCreate(TRUE);
    /* The initial application state */
    gState = stateInit;
    /* Reset number of pending packets */
    mcPendingPackets = 0;

    /* Prepare input queues.*/
    MSG_InitQueue(&mMlmeNwkInputQueue);
    MSG_InitQueue(&mMcpsNwkInputQueue);

    /* Initialize the MAC 802.15.4 extended address */
    Mac_SetExtendedAddress( (uint8_t*)&mExtendedAddress, macInstance );

    /* register keyboard callback function */
    KBD_Init(App_HandleKeys);

    /* Initialize the serial terminal interface */
    Serial_InitInterface(&interfaceId, APP_SERIAL_INTERFACE_TYPE, APP_SERIAL_INTERFACE_INSTANCE);
    Serial_SetBaudRate(interfaceId, gUARTBaudRate115200_c);
    Serial_SetRxCallBack(interfaceId, UartRxCallBack, NULL);

    /* Fase 2: Inicializar la estructura de nodos a vacio */
    for(i = 0; i < MAX_ASSOCIATED_NODES; i++)
    {
        mAssociatedNodes[i].shortAddress    = 0xFFFF;
        mAssociatedNodes[i].extendedAddress = 0;
        mAssociatedNodes[i].rxOnWhenIdle    = FALSE;
        mAssociatedNodes[i].isFFD           = FALSE;
        mAssociatedNodes[i].isUsed          = FALSE;
    }

    /*signal app ready*/
    LED_StartSerialFlash(LED1);

    Serial_Print(interfaceId, "\n\rPress any switch on board to start running the application.\n\r", gAllowToBlock_d);
}

/*****************************************************************************
* App_UpdateLeds
*
* Actualiza el LED RGB segun el valor del contador recibido usando LED_SetRgbLed().
*   0 = Magenta (Rojo + Azul)
*   1 = Azul
*   2 = Rojo
*   3 = Verde
*****************************************************************************/
static void App_UpdateLeds(uint8_t counter)
{
    /* Apagar LED RGB */
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
* App_PrintNodeInfo
*
* Imprime la informacion almacenada de un nodo (punto extra).
*****************************************************************************/
static void App_PrintNodeInfo(uint8_t index)
{
    if(index >= MAX_ASSOCIATED_NODES || !mAssociatedNodes[index].isUsed)
        return;

    Serial_Print(interfaceId, "\n\r--- Nodo Asociado ---\n\r", gAllowToBlock_d);

    Serial_Print(interfaceId, "  Short Address  : 0x", gAllowToBlock_d);
    Serial_PrintHex(interfaceId, (uint8_t*)&mAssociatedNodes[index].shortAddress, 2, gPrtHexNoFormat_c);

    Serial_Print(interfaceId, "\n\r  Extended Addr  : 0x", gAllowToBlock_d);
    Serial_PrintHex(interfaceId, (uint8_t*)&mAssociatedNodes[index].extendedAddress, 8, gPrtHexNoFormat_c);

    Serial_Print(interfaceId, "\n\r  RxOnWhenIdle   : ", gAllowToBlock_d);
    if(mAssociatedNodes[index].rxOnWhenIdle)
        Serial_Print(interfaceId, "true", gAllowToBlock_d);
    else
        Serial_Print(interfaceId, "false", gAllowToBlock_d);

    Serial_Print(interfaceId, "\n\r  Device Type    : ", gAllowToBlock_d);
    if(mAssociatedNodes[index].isFFD)
        Serial_Print(interfaceId, "FFD", gAllowToBlock_d);
    else
        Serial_Print(interfaceId, "RFD", gAllowToBlock_d);

    Serial_Print(interfaceId, "\n\r---------------------\n\r", gAllowToBlock_d);
}

/*****************************************************************************
* AppThread - Main application task
*****************************************************************************/
void AppThread(uint32_t argument)
{
    osaEventFlags_t ev;
    /* Pointer for storing the messages from MLME, MCPS, and ASP. */
    void *pMsgIn;
    /* Stores the status code returned by some functions. */
    uint8_t ret;

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
                ret = App_WaitMsg(pMsgIn, gMlmeBeaconNotifyInd_c);
                if(ret == errorNoError)
                {
                    /* ALWAYS free the beacon frame contained in the beacon notify indication.*/
                    MSG_Free(((nwkMessage_t *)pMsgIn)->msgData.beaconNotifyInd.pBufferRoot);
                    Serial_Print(interfaceId, "Received an MLME-Beacon Notify Indication\n\r", gAllowToBlock_d);
                }
            }
        }

        /* The application state machine */
        switch(gState)
        {
        case stateInit:
        {
            Serial_Print(interfaceId, "\n\r=================================================\n\r", gAllowToBlock_d);
            Serial_Print(interfaceId, " Coordinador: Equipo 5\n\r", gAllowToBlock_d);
            Serial_Print(interfaceId, " PAN ID: 0x5555 | Canal: 15 (0x0F)\n\r", gAllowToBlock_d);
            Serial_Print(interfaceId, "=================================================\n\r", gAllowToBlock_d);

            /* Canal 15 decimal = 0x0F */
            mLogicalChannel = MY_CHANNEL;

            gState = stateStartCoordinator;

            /* Trigger the next state */
            OSA_EventSet(mAppEvent, gAppEvtStartCoordinator_c);
            break;
        }

        case stateStartCoordinator:
        {
            if (ev & gAppEvtStartCoordinator_c)
            {
                /* Start up as a PAN Coordinator on the selected channel. */
                Serial_Print(interfaceId, "\n\rStarting as PAN coordinator on channel 0x", gAllowToBlock_d);
                Serial_PrintHex(interfaceId, &mLogicalChannel, 1, gPrtHexNoFormat_c);
                Serial_Print(interfaceId, ".\n\r", gAllowToBlock_d);

                ret = App_StartCoordinator(0);
                if(ret == errorNoError)
                {
                    gState = stateStartCoordinatorWaitConfirm;
                }
            }
            break;
        }

        case stateStartCoordinatorWaitConfirm:
        {
            /* Stay in this state until the Start confirm message
               arrives, and then goto the Listen state. */
            if (ev & gAppEvtMessageFromMLME_c)
            {
                if (pMsgIn)
                {
                    ret = App_WaitMsg(pMsgIn, gMlmeStartCnf_c);
                    if(ret == errorNoError)
                    {
                        Serial_Print(interfaceId, "Started the coordinator with PAN ID 0x", gAllowToBlock_d);
                        Serial_PrintHex(interfaceId, (uint8_t *)&mPanId, 2, gPrtHexNoFormat_c);
                        Serial_Print(interfaceId, ", and short address 0x", gAllowToBlock_d);
                        Serial_PrintHex(interfaceId, (uint8_t *)&mShortAddress, 2, gPrtHexNoFormat_c);
                        Serial_Print(interfaceId, ".\n\rReady to send and receive data over the UART.\n\r\n\r", gAllowToBlock_d);

                        gState = stateListen;
                        OSA_EventSet(mAppEvent, gAppEvtDummyEvent_c);
                    }
                }
            }
            break;
        }

        case stateListen:
        {
            /* Stay in this state forever.
               Transmit the data received on UART */
            if (ev & gAppEvtMessageFromMLME_c)
            {
                /* Get the message from MLME */
                if (pMsgIn)
                {
                    /* Process it */
                    ret = App_HandleMlmeInput(pMsgIn, 0);
                }
            }

            if (ev & gAppEvtRxFromUart_c)
            {
                /* get byte from UART */
                App_TransmitUartData();
            }
            break;
        }
        } /* switch(gState) */

        if (pMsgIn)
        {
            /* Messages must always be freed. */
            MSG_Free(pMsgIn);
            pMsgIn = NULL;
        }

        if (ev & gAppEvtMessageFromMCPS_c)
        {
            /* Get the message from MCPS */
            pMsgIn = MSG_DeQueue(&mMcpsNwkInputQueue);
            if (pMsgIn)
            {
                /* Process it */
                App_HandleMcpsInput(pMsgIn, 0);
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
    } /* while(1) */
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
* App_StartCoordinator
*****************************************************************************/
static uint8_t App_StartCoordinator( uint8_t appInstance )
{
  mlmeMessage_t *pMsg;
  uint8_t value;
  mlmeStartReq_t *pStartReq;

  Serial_Print(interfaceId, "Sending the MLME-Start Request message to the MAC...", gAllowToBlock_d);

  /* Allocate a message for the MLME (We should check for NULL). */
  pMsg = MSG_AllocType(mlmeMessage_t);
  if(pMsg != NULL)
  {
    /* Initialize the MAC 802.15.4 extended address */
    pMsg->msgType = gMlmeSetReq_c;
    pMsg->msgData.setReq.pibAttribute = gMPibExtendedAddress_c;
    pMsg->msgData.setReq.pibAttributeValue = (uint8_t *)&mExtendedAddress;
    (void)NWK_MLME_SapHandler( pMsg, macInstance );

    /* We must always set the short address to something
       else than 0xFFFF before starting a PAN. */
    pMsg->msgType = gMlmeSetReq_c;
    pMsg->msgData.setReq.pibAttribute = gMPibShortAddress_c;
    pMsg->msgData.setReq.pibAttributeValue = (uint8_t *)&mShortAddress;
    (void)NWK_MLME_SapHandler( pMsg, macInstance );

    /* We must set the Association Permit flag to TRUE
       in order to allow devices to associate to us. */
    pMsg->msgType = gMlmeSetReq_c;
    pMsg->msgData.setReq.pibAttribute = gMPibAssociationPermit_c;
    value = TRUE;
    pMsg->msgData.setReq.pibAttributeValue = &value;
    (void)NWK_MLME_SapHandler( pMsg, macInstance );

    /* This is a MLME-START.req command */
    pMsg->msgType = gMlmeStartReq_c;

    /* Create the Start request message data. */
    pStartReq = &pMsg->msgData.startReq;
    FLib_MemCpy(&pStartReq->panId, (void *)&mPanId, 2);
    pStartReq->logicalChannel = mLogicalChannel;
#ifdef gPHY_802_15_4g_d
    pStartReq->channelPage = gChannelPageId9_c;
#endif
    /* Beacon Order - 0xF = turn off beacons (non-beacon network) */
    pStartReq->beaconOrder = 0x0F;
    /* Superframe Order - 0xF = turn off beacons */
    pStartReq->superframeOrder = 0x0F;
    /* Be a PAN coordinator */
    pStartReq->panCoordinator = TRUE;
    pStartReq->batteryLifeExtension = FALSE;
    pStartReq->coordRealignment = FALSE;
    pStartReq->startTime = 0;

    /* Don't use security */
    pStartReq->coordRealignSecurityLevel = gMacSecurityNone_c;
    pStartReq->beaconSecurityLevel       = gMacSecurityNone_c;

    /* Send the Start request to the MLME. */
    if(NWK_MLME_SapHandler( pMsg, macInstance ) != gSuccess_c)
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

  Serial_Print(interfaceId, "Done\n\r", gAllowToBlock_d);
  return errorNoError;
}

/*****************************************************************************
* App_SendAssociateResponse
*
* FASE 2: Verifica si el nodo ya estuvo asociado (por Extended Address).
*   - Si ya estaba: le reasigna la misma Short Address.
*   - Si es nuevo: le asigna una nueva Short Address y lo guarda.
*   - Si la tabla esta llena: rechaza la asociacion.
*****************************************************************************/
static uint8_t App_SendAssociateResponse(nwkMessage_t *pMsgIn, uint8_t appInstance)
{
  mlmeMessage_t *pMsg;
  mlmeAssociateRes_t *pAssocRes;
  uint64_t incomingExtAddr;
  uint16_t assignedShortAddr = 0xFFFF;
  uint8_t  nodeIndex = 0xFF;
  uint8_t  firstFreeSlot = 0xFF;
  uint8_t  i;

  /* Extraer la Extended Address del dispositivo que solicita asociacion */
  FLib_MemCpy(&incomingExtAddr, &pMsgIn->msgData.associateInd.deviceAddress, 8);

  /* Extraer capability info para determinar tipo de dispositivo */
  uint8_t capInfo = pMsgIn->msgData.associateInd.capabilityInfo;
  bool_t isFFD = (capInfo & gCapInfoDeviceFfd_c) ? TRUE : FALSE;
  bool_t rxOnIdle = (capInfo & gCapInfoRxWhenIdle_c) ? TRUE : FALSE;

  Serial_Print(interfaceId, "\n\r[ASSOC] Extended Address solicitante: 0x", gAllowToBlock_d);
  Serial_PrintHex(interfaceId, (uint8_t*)&incomingExtAddr, 8, gPrtHexNoFormat_c);
  Serial_Print(interfaceId, "\n\r", gAllowToBlock_d);

  /* ---- Fase 2: Buscar en la tabla de nodos ---- */
  for(i = 0; i < MAX_ASSOCIATED_NODES; i++)
  {
      if(mAssociatedNodes[i].isUsed)
      {
          /* Verificar si la Extended Address ya existe */
          if(mAssociatedNodes[i].extendedAddress == incomingExtAddr)
          {
              /* Ya estuvo asociado: reusar la misma Short Address */
              nodeIndex = i;
              assignedShortAddr = mAssociatedNodes[i].shortAddress;
              Serial_Print(interfaceId, "[ASSOC] Nodo previamente asociado. Reasignando Short Address 0x", gAllowToBlock_d);
              Serial_PrintHex(interfaceId, (uint8_t*)&assignedShortAddr, 2, gPrtHexNoFormat_c);
              Serial_Print(interfaceId, "\n\r", gAllowToBlock_d);
              break;
          }
      }
      else
      {
          /* Guardar el primer slot libre */
          if(firstFreeSlot == 0xFF)
          {
              firstFreeSlot = i;
          }
      }
  }

  /* Si no se encontro en la tabla, es un nodo nuevo */
  if(nodeIndex == 0xFF)
  {
      if(firstFreeSlot != 0xFF)
      {
          /* Asignar nueva Short Address */
          nodeIndex = firstFreeSlot;
          assignedShortAddr = mNextShortAddress;
          mNextShortAddress++;

          /* Guardar en la estructura */
          mAssociatedNodes[nodeIndex].shortAddress    = assignedShortAddr;
          mAssociatedNodes[nodeIndex].extendedAddress = incomingExtAddr;
          mAssociatedNodes[nodeIndex].rxOnWhenIdle    = rxOnIdle;
          mAssociatedNodes[nodeIndex].isFFD           = isFFD;
          mAssociatedNodes[nodeIndex].isUsed          = TRUE;

          Serial_Print(interfaceId, "[ASSOC] Nuevo nodo. Asignando Short Address 0x", gAllowToBlock_d);
          Serial_PrintHex(interfaceId, (uint8_t*)&assignedShortAddr, 2, gPrtHexNoFormat_c);
          Serial_Print(interfaceId, "\n\r", gAllowToBlock_d);
      }
      else
      {
          /* Tabla llena, no se puede asociar */
          Serial_Print(interfaceId, "[ASSOC] ERROR: Tabla de nodos llena. Rechazando asociacion.\n\r", gAllowToBlock_d);
      }
  }

  /* Actualizar campos del nodo (puede haber cambiado capability) */
  if(nodeIndex != 0xFF)
  {
      mAssociatedNodes[nodeIndex].rxOnWhenIdle = rxOnIdle;
      mAssociatedNodes[nodeIndex].isFFD        = isFFD;
  }

  Serial_Print(interfaceId, "Sending the MLME-Associate Response message to the MAC...", gAllowToBlock_d);

  /* Allocate a message for the MLME */
  pMsg = MSG_AllocType(mlmeMessage_t);
  if(pMsg != NULL)
  {
    /* This is a MLME-ASSOCIATE.res command */
    pMsg->msgType = gMlmeAssociateRes_c;

    /* Create the Associate response message data. */
    pAssocRes = &pMsg->msgData.associateRes;

    if(nodeIndex != 0xFF)
    {
        /* Asignar la short address determinada */
        if(capInfo & gCapInfoAllocAddr_c)
        {
            pAssocRes->assocShortAddress = assignedShortAddr;
        }
        else
        {
            /* El dispositivo no quiere short address */
            pAssocRes->assocShortAddress = 0xFFFE;
        }
        pAssocRes->status = gSuccess_c;
    }
    else
    {
        /* Tabla llena: rechazar */
        pAssocRes->assocShortAddress = 0xFFFF;
        pAssocRes->status = gPanAtCapacity_c;
    }

    /* Get the 64 bit address of the device requesting association. */
    FLib_MemCpy(&pAssocRes->deviceAddress, &pMsgIn->msgData.associateInd.deviceAddress, 8);
    /* Do not use security */
    pAssocRes->securityLevel = gMacSecurityNone_c;

    /* Send the Associate Response to the MLME. */
    if( gSuccess_c == NWK_MLME_SapHandler( pMsg, macInstance ) )
    {
      Serial_Print( interfaceId, "Done\n\r", gAllowToBlock_d );

      /* Punto extra: Imprimir info del nodo al unirse */
      if(nodeIndex != 0xFF)
      {
          App_PrintNodeInfo(nodeIndex);
      }

      return errorNoError;
    }
    else
    {
      Serial_Print( interfaceId, "Invalid parameter!\n\r", gAllowToBlock_d );
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
* App_HandleMlmeInput
*****************************************************************************/
static uint8_t App_HandleMlmeInput(nwkMessage_t *pMsg, uint8_t appInstance)
{
  if(pMsg == NULL)
    return errorNoMessage;

  switch(pMsg->msgType)
  {
  case gMlmeAssociateInd_c:
    Serial_Print(interfaceId, "Received an MLME-Associate Indication from the MAC\n\r", gAllowToBlock_d);
    /* A device sent us an Associate Request. We must send back a response. */
    return App_SendAssociateResponse(pMsg, appInstance);

  case gMlmeCommStatusInd_c:
    /* Sent by the MLME after the Association Response has been transmitted. */
    Serial_Print(interfaceId, "Received an MLME-Comm-Status Indication from the MAC\n\r", gAllowToBlock_d);
    break;

  default:
    break;
  }
  return errorNoError;
}

/*****************************************************************************
* App_HandleMcpsInput
*
* Fase 1: Al recibir un paquete de datos, extrae la info del paquete,
* imprime direccion de origen, LQI, tamano de payload, y si es un
* paquete "Counter: x" actualiza los LEDs.
*****************************************************************************/
static void App_HandleMcpsInput(mcpsToNwkMessage_t *pMsgIn, uint8_t appInstance)
{
  switch(pMsgIn->msgType)
  {
  case gMcpsDataCnf_c:
    if(mcPendingPackets)
      mcPendingPackets--;
    break;

  case gMcpsDataInd_c:
  {
    /* Extraer informacion del paquete recibido */
    uint8_t  payloadSize = pMsgIn->msgData.dataInd.msduLength;
    uint8_t  lqi         = pMsgIn->msgData.dataInd.mpduLinkQuality;
    uint8_t *payload     = pMsgIn->msgData.dataInd.pMsdu;

    /* CORRECCION: Extraer la direccion de origen correctamente */
    uint16_t srcAddress = 0;
    FLib_MemCpy(&srcAddress, &pMsgIn->msgData.dataInd.srcAddr, 2);

    /* Imprimir informacion del paquete (Fase 1 - 1 punto) */
    Serial_Print(interfaceId, "\n\r--- Paquete Recibido ---\n\r", gAllowToBlock_d);

    Serial_Print(interfaceId, "Direccion Origen : 0x", gAllowToBlock_d);
    Serial_PrintHex(interfaceId, (uint8_t *)&srcAddress, 2, gPrtHexBigEndian_c);

    Serial_Print(interfaceId, "\n\rLQI              : ", gAllowToBlock_d);
    Serial_PrintDec(interfaceId, (uint32_t)lqi);

    Serial_Print(interfaceId, "\n\rPayload Size     : ", gAllowToBlock_d);
    Serial_PrintDec(interfaceId, (uint32_t)payloadSize);
    Serial_Print(interfaceId, " bytes", gAllowToBlock_d);

    Serial_Print(interfaceId, "\n\rPayload          : ", gAllowToBlock_d);
    Serial_SyncWrite(interfaceId, payload, payloadSize);
    Serial_Print(interfaceId, "\n\r", gAllowToBlock_d);

    /* Verificar si es un paquete tipo "Counter: x" */
    /* El formato es: "Counter: " (9 chars) + digito ASCII */
    if(payloadSize >= 10)
    {
        /* Verificamos que empiece con "Counter: " */
        if(payload[0] == 'C' && payload[7] == ':' && payload[8] == ' ')
        {
            /* Extraer el digito del counter (ASCII -> numero) */
            uint8_t contador = payload[9] - '0';

            if(contador <= 3)
            {
                /* Actualizar LEDs del coordinador (Fase 1 - 1 punto) */
                App_UpdateLeds(contador);

                Serial_Print(interfaceId, "-> LED actualizado: Counter = ", gAllowToBlock_d);
                Serial_PrintDec(interfaceId, (uint32_t)contador);
                Serial_Print(interfaceId, "\n\r", gAllowToBlock_d);
            }
            else
            {
                Serial_Print(interfaceId, "-> Counter fuera de rango\n\r", gAllowToBlock_d);
            }
        }
    }

    Serial_Print(interfaceId, "------------------------\n\r", gAllowToBlock_d);
    break;
  }

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
        /* Solo transmitimos si hay al menos un nodo asociado en la tabla */
        uint8_t i;
        uint16_t destAddr = 0xFFFF;
        for(i = 0; i < MAX_ASSOCIATED_NODES; i++)
        {
            if(mAssociatedNodes[i].isUsed)
            {
                destAddr = mAssociatedNodes[i].shortAddress;
                break; /* Usar el primer nodo asociado */
            }
        }

        if(destAddr != 0xFFFF)
        {
            mpPacket = MSG_Alloc(sizeof(nwkToMcpsMessage_t) + count);
        }
    }

    if(mpPacket != NULL)
    {
        uint16_t destAddr = 0xFFFF;
        uint8_t i;
        for(i = 0; i < MAX_ASSOCIATED_NODES; i++)
        {
            if(mAssociatedNodes[i].isUsed)
            {
                destAddr = mAssociatedNodes[i].shortAddress;
                break;
            }
        }

        mpPacket->msgType = gMcpsDataReq_c;
        mpPacket->msgData.dataReq.pMsdu = (uint8_t*)(&mpPacket->msgData.dataReq.pMsdu) +
            sizeof(mpPacket->msgData.dataReq.pMsdu);
        Serial_Read(interfaceId, mpPacket->msgData.dataReq.pMsdu, count, &count);

        FLib_MemCpy(&mpPacket->msgData.dataReq.dstAddr, (void*)&destAddr, 2);
        FLib_MemCpy(&mpPacket->msgData.dataReq.srcAddr, (void*)&mShortAddress, 2);
        FLib_MemCpy(&mpPacket->msgData.dataReq.dstPanId, (void*)&mPanId, 2);
        FLib_MemCpy(&mpPacket->msgData.dataReq.srcPanId, (void*)&mPanId, 2);
        mpPacket->msgData.dataReq.dstAddrMode = gAddrModeShortAddress_c;
        mpPacket->msgData.dataReq.srcAddrMode = gAddrModeShortAddress_c;
        mpPacket->msgData.dataReq.msduLength = count;
        mpPacket->msgData.dataReq.txOptions = gMacTxOptionsAck_c;
        mpPacket->msgData.dataReq.txOptions |= gMacTxOptionIndirect_c;
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

/*****************************************************************************
* Handles all key events for this device.
*****************************************************************************/
static void App_HandleKeys(uint8_t events)
{
#if gKBD_KeysCount_c > 0
    switch ( events )
    {
    case gKBD_EventSW1_c:
    case gKBD_EventSW2_c:
    case gKBD_EventSW3_c:
    case gKBD_EventSW4_c:
    case gKBD_EventLongSW1_c:
    case gKBD_EventLongSW2_c:
    case gKBD_EventLongSW3_c:
    case gKBD_EventLongSW4_c:
        if(gState == stateInit)
        {
          LED_StopFlashingAllLeds();
          OSA_EventSet(mAppEvent, gAppEvtDummyEvent_c);
        }
    break;
    }
#endif
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
