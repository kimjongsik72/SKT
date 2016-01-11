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
 * \addtogroup srf06-common-peripherals
 * @{
 *
 * \file
 * Driver for the SmartRF06EB LEDs when a CC13xx/CC26xx EM is mounted on it
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "gpiorelay.h"
#include "ti-lib.h"
/*---------------------------------------------------------------------------*/

static int inited_relay = 0;
/*---------------------------------------------------------------------------*/
void
gpio_relay_init(void)
{
  if(inited_relay) {
    return;
  }
  inited_relay = 1;

  ti_lib_ioc_pin_type_gpio_output(SENSOR_POWER);
  ti_lib_ioc_pin_type_gpio_output(ADC_CT_SELECT);
  ti_lib_gpio_pin_write(RELAY_ALL, 0);
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/

void relay_all_clear(void)
{
        
    ti_lib_gpio_pin_write(RELAY_ALL, 0);
    
}

void sensor_power_set_on(void)
{
  /* Clear everything */ 
  ti_lib_gpio_pin_write(SENSOR_POWER_ENABLE, 1);
}

void sensor_power_set_off(void)
{
  /* Clear everything */ 
  ti_lib_gpio_pin_write(SENSOR_POWER_ENABLE, 0);
}


void adc_ct_select_set_on(void)
{
  /* Clear everything */ 
  ti_lib_gpio_pin_write(ADC_CT_SELECT_ENABLE, 1);
}

void adc_ct_select_set_off(void)
{
  /* Clear everything */ 
  ti_lib_gpio_pin_write(ADC_CT_SELECT_ENABLE, 0);
}

/*---------------------------------------------------------------------------*/
/** @} */
