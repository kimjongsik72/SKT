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
 * \addtogroup cc26xx-platforms
 * @{
 *
 * \defgroup cc26xx-examples CC26xx Example Projects
 *
 * Example projects for CC26xx-based platforms.
 * @{
 *
 * \defgroup cc26xx-demo CC26xx Demo Project
 *
 *   Example project demonstrating the CC26xx platforms
 *
 *   This example will work for the following boards:
 *   - srf06-cc26xx: SmartRF06EB + CC26XX EM
 *   - sensortag-cc26xx: CC26XX sensortag
 *
 *   By default, the example will build for the srf06-cc26xx board. To switch
 *   between platforms:
 *   - make clean
 *   - make BOARD=sensortag-cc26xx savetarget
 *
 *     or
 *
 *     make BOARD=srf06-cc26xx savetarget
 *
 *   This is an IPv6/RPL-enabled example. Thus, if you have a border router in
 *   your installation (same RDC layer, same PAN ID and RF channel), you should
 *   be able to ping6 this demo node.
 *
 *   This example also demonstrates CC26xx BLE operation. The process starts
 *   the BLE beacon daemon (implemented in the RF driver). The daemon will
 *   send out a BLE beacon periodically. Use any BLE-enabled application (e.g.
 *   LightBlue on OS X or the TI BLE Multitool smartphone app) and after a few
 *   seconds the cc26xx device will be discovered.
 *
 * - etimer/clock : Every CC26XX_DEMO_LOOP_INTERVAL clock ticks the LED defined
 *                  as CC26XX_DEMO_LEDS_PERIODIC will toggle and the device
 *                  will print out readings from some supported sensors
 * - sensors      : Some sensortag sensors are read asynchronously (see sensor
 *                  documentation). For those, this example will print out
 *                  readings in a staggered fashion at a random interval
 * - Buttons      : CC26XX_DEMO_SENSOR_1 button will toggle CC26XX_DEMO_LEDS_BUTTON
 *                - CC26XX_DEMO_SENSOR_2 turns on LEDS_REBOOT and causes a
 *                  watchdog reboot
 *                - The remaining buttons will just print something
 *                - The example also shows how to retrieve the duration of a
 *                  button press (in ticks). The driver will generate a
 *                  sensors_changed event upon button release
 * - Reed Relay   : Will toggle the sensortag buzzer on/off
 *
 * @{
 *
 * \file
 *     Example demonstrating the cc26xx platforms
 */
#include "contiki.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "random.h"
#include "button-sensor.h"
#include "batmon-sensor.h"
#include "board-peripherals.h"
#include "rf-core/rf-ble.h"
#include "net/rime/broadcast.h"
#include "driverlib/aux_adc.h"
#include "driverlib/aux_wuc.h"
#include "dev/watchdog.h"
#include "ti-lib.h"
#include "gpiorelay.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>

/*---------------------------------------------------------------------------*/
#define LOOP_INTERVAL       CLOCK_SECOND
#define LEDS_OFF_HYSTERISIS (RTIMER_SECOND >> 1)
#define LEDS_PERIODIC       LEDS_YELLOW
#define LEDS_BUTTON         LEDS_RED
#define LEDS_SERIAL_IN      LEDS_ORANGE
#define LEDS_REBOOT         LEDS_ALL
#define LEDS_RF_RX          (LEDS_YELLOW | LEDS_ORANGE)
#define BROADCAST_CHANNEL   129

/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_LOOP_INTERVAL       (CLOCK_SECOND * 20)
#define CC26XX_DEMO_LEDS_PERIODIC       LEDS_YELLOW
#define CC26XX_DEMO_LEDS_BUTTON         LEDS_RED
#define CC26XX_DEMO_LEDS_REBOOT         LEDS_ALL
/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_SENSOR_NONE         (void *)0xFFFFFFFF

#define CC26XX_DEMO_SENSOR_1     &button_left_sensor
#define CC26XX_DEMO_SENSOR_2     &button_right_sensor

#if BOARD_SENSORTAG
#define CC26XX_DEMO_SENSOR_3     CC26XX_DEMO_SENSOR_NONE
#define CC26XX_DEMO_SENSOR_4     CC26XX_DEMO_SENSOR_NONE
#define CC26XX_DEMO_SENSOR_5     &reed_relay_sensor
#else
#define CC26XX_DEMO_SENSOR_3     &button_up_sensor
#define CC26XX_DEMO_SENSOR_4     &button_down_sensor
#define CC26XX_DEMO_SENSOR_5     &button_select_sensor
#endif



/*---------------------------------------------------------------------------*/
static struct etimer et;
static struct rtimer rt;
static uint16_t counter;

/*---------------------------------------------------------------------------*/
PROCESS(cc26xx_demo_process, "cc26xx demo process");
AUTOSTART_PROCESSES(&cc26xx_demo_process);
/*---------------------------------------------------------------------------*/
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  leds_toggle(LEDS_RF_RX);
 
}
/*---------------------------------------------------------------------------*/
static const struct broadcast_callbacks bc_rx = { broadcast_recv };
static struct broadcast_conn bc;
/*---------------------------------------------------------------------------*/
void
rt_callback(struct rtimer *t, void *ptr)
{
  leds_off(LEDS_PERIODIC);
}
/*---------------------------------------------------------------------------*/


void bubble_sort(uint16_t list[], int n)
{
  int c, d, t;
 
  for (c = 0 ; c < ( n - 1 ); c++)
  {
    for (d = 0 ; d < n - c - 1; d++)
    {
      if (list[d] > list[d+1])
      {        
        t         = list[d];
        list[d]   = list[d+1];
        list[d+1] = t;
      }
    }
  }
}

#define ADC_Average 2000
#define ADC_Average_Max 5
#define ADC_Average_Min 5


uint16_t Current_Average(uint16_t ADC_array[]) 
{
    uint32_t sum=0;
   // CPU_INT32U RPM_array[RPM_Average];
    uint16_t ADC_result=0; 
      
    bubble_sort(ADC_array,ADC_Average);
    
    for(int i=(ADC_Average_Min);i<(ADC_Average-ADC_Average_Max);i++)
    {
      sum += ADC_array[i];
      
    }
    
    ADC_result = (uint16_t)(sum/(ADC_Average-(ADC_Average_Max+ADC_Average_Min)));
    
    return ADC_result;
  
}



uint16_t Adc_Average(uint16_t ADC_array[]) 
{
    uint32_t sum=0;
   
    uint16_t ADC_result=0; 
  
    
    for(int i=0;i<ADC_Average;i++)
    {
      sum += ADC_array[i];
      
    }
    
    ADC_result = (uint16_t)(sum/ADC_Average);
    
    return ADC_result;
  
}





/*---------------------------ADC-------------------------------------------*/

static uint16_t Get_ADC_reading(void)
{
      uint16_t  result_data = 0;
      uint16_t ADC_array[ADC_Average] = {0};
      int32_t adc_temp=0;      
      double result=0;
      sensor_power_set_on();
      adc_ct_select_set_on();
      // Enable AUX 
      ti_lib_aon_wuc_aux_wakeup_event(AONWUC_AUX_WAKEUP);
      while(!(ti_lib_aon_wuc_power_status_get() & AONWUC_AUX_POWER_ON))
      { }

      

      // Enable clocks
      ti_lib_aux_wuc_clock_enable(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK);
      while(ti_lib_aux_wuc_clock_status(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK) != AUX_WUC_CLOCK_READY)
      { }

      
      // Select input
      AUXADCSelectInput(ADC_COMPB_IN_AUXIO5);
      

      // Configure and enable
    //  AUXADCEnableSync(AUXADC_REF_VDDS_REL,  AUXADC_SAMPLE_TIME_10P6_US, AUXADC_TRIGGER_MANUAL);
      AUXADCEnableSync(AUXADC_REF_VDDS_REL,  AUXADC_SAMPLE_TIME_170_US, AUXADC_TRIGGER_MANUAL);    
     // AUXADCEnableSync(AUXADC_REF_FIXED,  AUXADC_SAMPLE_TIME_10P6_US, AUXADC_TRIGGER_MANUAL);

  
     
          
      // Read
      for(int i=0;i<ADC_Average;i++)
      {
        AUXADCGenManualTrigger();
        
        adc_temp = AUXADCReadFifo();
        if(adc_temp<2048)
        {
           result = adc_temp - 2048;        
          
        }else{
          
           result = adc_temp;
          
        }
        
        ADC_array[i] = (uint16_t)(abs(result));
       //  AUXADCReadFifo();
        
      }
      
      result_data = Adc_Average(ADC_array);
      
     // result_data = upcount++;
    //  AUXADCGenManualTrigger();
    //  result_data = AUXADCReadFifo();
      
      
      


      // Disable ADC
      AUXADCDisable();
      adc_ct_select_set_off();
     // relay_all_clear();
     // sensor_power_set_off();
      
      return result_data;
      
}

/*---------------------------------------------------------------------------*/


PROCESS_THREAD(cc26xx_demo_process, ev, data)
{
  PROCESS_EXITHANDLER(broadcast_close(&bc))

  PROCESS_BEGIN();

  gpio_relay_init();
  relay_all_clear();
  
  
  counter = 0;
  broadcast_open(&bc, BROADCAST_CHANNEL, &bc_rx);

  etimer_set(&et, CLOCK_SECOND);
  
  while(1) {

    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER) {
      leds_on(LEDS_PERIODIC);
     
      etimer_set(&et, CLOCK_SECOND*5);
      rtimer_set(&rt, RTIMER_NOW() + LEDS_OFF_HYSTERISIS, 1,
                 rt_callback, NULL);
      counter = Get_ADC_reading();
      packetbuf_copyfrom(&counter, sizeof(counter));
      broadcast_send(&bc);
     // printf("adc data value : %d \r\n",counter);
        
    } 
     watchdog_periodic();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
