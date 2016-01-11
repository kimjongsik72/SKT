/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup cc26xx-examples
 * @{
 *
 * \defgroup cc26xx-web-demo CC26xx Web Demo
 * @{
 *
 *   An example demonstrating:
 *   * how to use a CC26XX-powered node in a deployment driven by a 6LBR
 *   * how to expose on-device sensors as CoAP resources
 *   * how to build a small web page which reports networking and sensory data
 *   * how to configure functionality through the aforementioned web page using
 *     HTTP POST requests
 *   * a network-based UART
 *
 * \file
 *   Main header file for the CC26XX web demo.
 */
/*---------------------------------------------------------------------------*/
#ifndef COMMAND_H_
#define COMMAND_H_
/*---------------------------------------------------------------------------*/

#include "sys/process.h"
#include <stdint.h>
/*---------------------------------------------------------------------------*/


void SendPacket(unsigned char func, unsigned short param, const unsigned char *pData, unsigned int len);




#define BUFF_SIZE 500
#define STX '@'
#define ETX 0x0D



typedef struct UPOPacketHeader {
    unsigned char stx;
    unsigned char len_h;
    unsigned char len_l;
    unsigned char gid;
    unsigned char nid_h;
    unsigned char nid_l;
    unsigned char func;
    unsigned char param_h;
    unsigned char param_l;
    unsigned char crc16_h;
    unsigned char crc16_l;
    unsigned char temp;
    unsigned char etx;
} UPOPacketHeader_t;



#pragma pack(1)
typedef struct UsnSendPacketHeader {
    unsigned char  stx;
    unsigned short len;
    unsigned char  gid;
    unsigned short nid;
    unsigned char  func;
    unsigned short param;
} UsnSendPacketHeader_t;
#pragma pack()




#pragma pack(1)
typedef union EnvironmentData {
  struct {
    unsigned char  GroupID;
    unsigned char  Gain;
    unsigned char  Bluetooth;
    unsigned char  CycleTime;
    unsigned char  OperationMode;
    unsigned short NodeID;
    unsigned short SamplingRate;
    unsigned short Filter;
    long Calibration_A;
    long Calibration_B;
    long Calibration_C;
    long Calibration_D;
  } Segment;
  unsigned char buffer[27];
} EnvironmentData_t;
#pragma pack()


#pragma pack(1)
typedef union ContollerData {
  struct {
    
    unsigned long SendData;
  } Segment;
  unsigned char buffer[4];
}ContollerData_t;
#pragma pack()


#pragma pack(1)
typedef union SendfloatData {
  struct {   
    
    unsigned char Hour;
    unsigned char Min;
    unsigned char Sec;
    long Data;
  }Segment;
  unsigned char buffer[7];
}SendfloatData_t;
#pragma pack()


/*---------------------------------------------------------------------------*/
#endif /* CC26XX_WEB_DEMO_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
