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
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup cc26xx-web-demo
 * @{
 *
 * \file
 *  CoAP resource handler for CC26XX software and hardware version
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "sys/clock.h"
#include "ti-lib.h"
#incldue "command.h"

#include <string.h>
/*---------------------------------------------------------------------------*/


unsigned int MakePacket(unsigned char func, unsigned short param, unsigned char gid, unsigned short nid, const unsigned char *pData, unsigned int len, unsigned char *pPacket)
{
    UsnSendPacketHeader_t send;
    send.stx = STX;
    send.len = len + sizeof(UsnSendPacketHeader_t) - 3/*STX+Len*/ + 4/*CRC16+ETX*/;
    send.func = func;
    send.param = param;
    send.gid = gid;
    send.nid = nid;

    unsigned char *pByte = pPacket;

    //========================================================================================
    // 2009.5.8 - ���� ����
    //========================================================================================
    memcpy(pByte, &send, sizeof(send));
    pByte += sizeof(send);

    //========================================================================================
    // 2009.5.8 - ������ ����
    //========================================================================================
    if (NULL != pData)
    {
        memcpy(pByte, pData, len);
        pByte += len;
    }

    //========================================================================================
    // 2009.5.8 - CRC16 �� ETX ����
    //========================================================================================
    unsigned short wCRC16;
    unsigned char temp;
    wCRC16 = CRC16(pPacket, send.len -4/*CRC16+ETX*/ +3/*STX+Len*/);
    
    memcpy(pByte, &wCRC16, sizeof(wCRC16));
    pByte += sizeof(wCRC16);
    temp = 0x0a;
    memcpy(pByte, &temp,1);
    pByte += 1;
    
    
   
    *pByte = (CPU_INT08U)ETX;

    return send.len + 4;/*STX+Len*/
}



unsigned int SendDataCount = 0;

void SendPacket(unsigned char func, unsigned short param, const unsigned char *pData, unsigned int len)
{
    unsigned char packet[500];
    unsigned int nPacketLen;
    nPacketLen = MakePacket(func, param, NodeEnv.Segment.GroupID, NodeEnv.Segment.NodeID, pData, len, packet);
    if (nPacketLen < 256) {
        BSP_Ser1_WriteBuffer(packet, nPacketLen-1);
        SendDataCount++;        
    }
    else {
        BSP_Ser1_WriteBuffer(packet, nPacketLen);
    }
}

//=====================Serial to lan end========================================//




