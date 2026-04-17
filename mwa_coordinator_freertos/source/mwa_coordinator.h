/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
* All rights reserved.
*
* \file
*
* This header file is for MyWirelessApp Demo Coordinator application.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef _APP_H_
#define _APP_H_

/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/
#ifdef gPHY_802_15_4g_d
  #define mDefaultValueOfChannel_c (0x0001FFFF)
  #define mDefaultMaxChannel_c     (0x11)
#else
  #define mDefaultValueOfChannel_c (0x07FFF800)
#endif

/* Extended MAC address del Coordinador - unica */
#define mMacExtendedAddress_c    (0x1111111111110005)

/* Set the Coordinator short address */
#define mDefaultValueOfShortAddress_c     0x0000

/* Set the Coordinator PanID - Equipo 5 */
#define mDefaultValueOfPanId_c            0x5555

/* Maximum number of outstanding packets */
#define mDefaultValueOfMaxPendingDataPackets_c 2

/* Maximo de nodos que puede almacenar la estructura (Fase 2) */
#define MAX_ASSOCIATED_NODES  5

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/* The various states of the applications state machines. */
enum {
  stateInit,
  stateScanEdStart,
  stateScanEdWaitConfirm,
  stateStartCoordinator,
  stateStartCoordinatorWaitConfirm,
  stateListen
};

/* Events that are passed to the application task.
   Are defined as byte masks to make possible
   send multiple events to the task */

#define gAppEvtDummyEvent_c            (1 << 0)
#define gAppEvtRxFromUart_c            (1 << 1)
#define gAppEvtMessageFromMLME_c       (1 << 2)
#define gAppEvtMessageFromMCPS_c       (1 << 3)
#define gAppEvtStartCoordinator_c      (1 << 4)

/* Error codes */
enum {
  errorNoError,
  errorWrongConfirm,
  errorNotSuccessful,
  errorNoMessage,
  errorAllocFailed,
  errorInvalidParameter,
  errorNoScanResults
};

/************************************************************************************
* Estructura para almacenar informacion de nodos asociados (Fase 2)
************************************************************************************/
typedef struct {
    uint16_t shortAddress;      /* Short Address asignada */
    uint64_t extendedAddress;   /* Extended (MAC) Address del nodo */
    bool_t   rxOnWhenIdle;      /* TRUE si el nodo mantiene RX encendido */
    bool_t   isFFD;             /* TRUE = FFD, FALSE = RFD */
    bool_t   isUsed;            /* TRUE si esta entrada esta ocupada */
} nodeInfo_t;

/******************************************************************************
*******************************************************************************
* Public Prototypes
*******************************************************************************
******************************************************************************/
#ifdef __cplusplus
    extern "C" {
#endif


#ifdef __cplusplus
}
#endif

/**********************************************************************************/
#endif /* _APP_H_ */
