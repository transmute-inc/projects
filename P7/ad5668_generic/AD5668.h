/***************************************************************************//**
 *   @file   AD5668.h
 *   @brief  Header file of AD5668 Driver.
 *   @author Bancisor Mihai
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 629
*******************************************************************************/
#ifndef _AD5668_H_
#define _AD5668_H_

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "Communication.h"

/******************************************************************************/
/* AD5668                                                                    */
/******************************************************************************/
#define AD5668_CMD(x)   (((unsigned long)(x) & 0xF) << 24)
#define AD5668_ADR(x)   (((unsigned long)(x) & 0xF) << 20)
#define AD5668_DATA(x)  (((unsigned long)(x) & 0xFFFF) << 4)

#define AD5668_CMD_WRITE_IN_REG             0x00
#define AD5668_CMD_UPDATE_DAC               0x01
#define AD5668_CMD_WRITE_IN_REG_UPDATE_ALL  0x02
#define AD5668_CMD_WRITE_IN_REG_UPDATE      0x03
#define AD5668_CMD_PWR_DOWN_UP              0x04
#define AD5668_CMD_LOAD_CLEAR               0x05
#define AD5668_CMD_LOAD_LDAC                0x06
#define AD5668_CMD_RESET                    0x07
#define AD5668_CMD_INT_REF                  0x08

#define AD5668_ADR_DAC_A                    0x00
#define AD5668_ADR_DAC_B                    0x01
#define AD5668_ADR_DAC_C                    0x02
#define AD5668_ADR_DAC_D                    0x03
#define AD5668_ADR_DAC_E                    0x04
#define AD5668_ADR_DAC_F                    0x05
#define AD5668_ADR_DAC_G                    0x06
#define AD5668_ADR_DAC_H                    0x07
#define AD5668_ADR_ALL                      0x0F
/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/
/* Initializes the SPI communication peripheral. */
unsigned char AD5668_Init(void);
/* Writes to the DAC register. */
void AD5668_SetRegisterValue(unsigned long regValue);

#endif // _AD5668_H
