//*****************************************************************************
//
// usb_serial_structs.h - Data structures defining this USB CDC device.
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 9453 of the EK-LM4F120XL Firmware Package.
//
//*****************************************************************************

#ifndef _USB_SERIAL_STRUCTS_H_
#define _USB_SERIAL_STRUCTS_H_

#include "inttypes.h"

//*****************************************************************************
//
// The size of the transmit and receive buffers used for the redirected UART.
// This number should be a power of 2 for best performance.  256 is chosen
// pretty much at random though the buffer should be at least twice the size of
// a maxmum-sized USB packet.
//
//*****************************************************************************
#define UART_BUFFER_SIZE 256

extern unsigned long RxHandler(void *pvCBData, unsigned long ulEvent,
                               unsigned long ulMsgValue, void *pvMsgData);
extern unsigned long TxHandler(void *pvlCBData, unsigned long ulEvent,
                               unsigned long ulMsgValue, void *pvMsgData);

extern const tUSBBuffer g_sTxBuffer;
extern const tUSBBuffer g_sRxBuffer;
extern const tUSBDCDCDevice g_sCDCDevice;
extern unsigned char g_pucUSBTxBuffer[];
extern unsigned char g_pucUSBRxBuffer[];

// code quality tradeoffs must be applied here

typedef const struct __attribute__((__packed__)) {
	const uint32_t dwLength;
	const uint16_t bcdVersion;
	const uint16_t wIndex;
	const uint16_t wCount;

	const uint32_t dwSize;
	const uint32_t dwPropertyDataType;
	const uint16_t wPropertyNameLength;
	const uint16_t bPropertyName[20];
	const uint32_t dwPropertyDataLength;
	const uint16_t bPropertyData[39];

	const uint32_t dwSize2;
	const uint32_t dwPropertyDataType2;
	const uint16_t wPropertyNameLength2;
	const uint16_t bPropertyName2[6];
	const uint32_t dwPropertyDataLength2;
	const uint16_t bPropertyData2[15];

	const uint32_t dwSize3;
	const uint32_t dwPropertyDataType3;
	const uint16_t wPropertyNameLength3;
	const uint16_t bPropertyName3[6];
	const uint32_t dwPropertyDataLength3;
	const uint16_t bPropertyData3[37];
} g_sOSProperties;

extern const g_sOSProperties g_pOSProperties;

typedef const struct __attribute__((__packed__)) {
	const uint32_t dwLength;
	const uint16_t bcdVersion;
	const uint16_t wIndex;
	const uint8_t  wCount;
	const uint8_t  RESERVED[7];

	const uint8_t  bFirstInterfaceNumber;
	const uint8_t  RESERVED1;
	const uint8_t  compatibleID[8];
	const uint8_t  subCompatibleID[8];
	const uint8_t  RESERVED2[6];
} g_sOSFeature;

extern const g_sOSFeature g_pOSFeature;


#endif
