/*
 * Copyright (c) 2011, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"

#include "simple-udp.h"
#include "dev/button-sensor.h"
#include "dev/light-sensor.h"
#include "dev/sht11/sht11-sensor.h"
#include "dev/leds.h"

#include <stdio.h>
#include <string.h>

#define UDP_PORT 1234

#define SEND_INTERVAL		(2 * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))

static struct simple_udp_connection broadcast_connection;
static process_event_t event_arlam;
static process_event_t event_broadcast;
static process_event_t event_button;

/*---------------------------------------------------------------------------*/
PROCESS(light_sensor_montitor_process, "light_sensor_montitor_process");
PROCESS(reset_button_monitor_process, "Mreset_button_monitor_process");
PROCESS(actuate_arlam_process, "actuate_arlam_process");
PROCESS(broadcast_intrusion_process, "broadcast_intrusion_process");
//AUTOSTART_PROCESSES(&light_sensor_montitor_process);
AUTOSTART_PROCESSES(&light_sensor_montitor_process, &reset_button_monitor_process, &actuate_arlam_process, &broadcast_intrusion_process);
/*---------------------------------------------------------------------------*/
static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
  
         
    static char light_data[4];
    static char arlam_data[4];
    int arlam_flag;
    
    printf("Data: %s received on port %d from port %d with length %d\n",
         data, receiver_port, sender_port, datalen);

    strncpy(light_data, data, 3);
    
    //strncpy(arlam_data, &data[4], 3);
    arlam_data[0] = data[4];
    arlam_data[1] = data[5];
    arlam_data[2] = data[6];
    arlam_data[3] = '\0';
    printf("Received light_data message with: %s\n", light_data);
    printf("Received arlam_data message with: %s\n", arlam_data);
    process_post(&actuate_arlam_process, event_arlam, &arlam_data);
    
    arlam_flag = atoi(arlam_data);
    printf("Received arlam_flag with value: %d\n", arlam_flag);
    
    if(arlam_flag == 112) {
      leds_on(LEDS_RED);
    } else if(arlam_flag == 110) {
       leds_off(LEDS_RED);
   }
    
    process_post(&broadcast_intrusion_process, event_broadcast, data);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(light_sensor_montitor_process, ev, data)
{
  static struct etimer periodic_timer;
  static struct etimer send_timer;
  uip_ipaddr_t addr;
  static char message[8];
  int arlam_flag;
  static char arlam_data[4];
  
  
  
  PROCESS_BEGIN();
  SENSORS_ACTIVATE(button_sensor);
	SENSORS_ACTIVATE(light_sensor);
	SENSORS_ACTIVATE(sht11_sensor);

  simple_udp_register(&broadcast_connection, UDP_PORT,
                      NULL, UDP_PORT,
                      receiver);

  etimer_set(&periodic_timer, SEND_INTERVAL);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
    etimer_reset(&periodic_timer);
    etimer_set(&send_timer, SEND_TIME);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_timer));
    uip_create_linklocal_allnodes_mcast(&addr);
    
    if(light_sensor.value(SHT11_SENSOR_TEMP) < 50) 
		{
      int send_data = light_sensor.value(0);
        
       //int send_data = sht11_sensor.value(SHT11_SENSOR_TEMP);
      //printf("Sending raw  data: %d\n", send_data);
      //sprintf(message,"hello");
      arlam_flag = 112;
      sprintf(arlam_data, "%d", arlam_flag);
      process_post(&actuate_arlam_process, event_arlam, &arlam_data);
      
      sprintf(message, "%03d.%03d", send_data, arlam_flag);
      printf("Sending message with data: %s\n", message);

      process_post(&broadcast_intrusion_process, event_broadcast, &message);
      
      
      
    
		}

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(reset_button_monitor_process, ev, data)
{
  static int arlam_flag;
  static char arlam_data[4];
  static char message[8];
  PROCESS_BEGIN();
  SENSORS_ACTIVATE(button_sensor);
  while(1) {
   //PROCESS_WAIT_EVENT_UNTIL(ev == event_button);
   PROCESS_WAIT_EVENT_UNTIL(data == &button_sensor);
   
   printf("Reset button detected\n");
   //leds_off(LEDS_RED);
   arlam_flag = 110;
   sprintf(arlam_data, "%d", arlam_flag);
   
   process_post(&actuate_arlam_process, event_arlam, &arlam_data);
   
   sprintf(message, "000.%03d", arlam_flag);
   printf("Sending message with data: %s\n", message);

   process_post(&broadcast_intrusion_process, event_broadcast, &message);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(actuate_arlam_process, ev, data)
{
  int arlam_flag;
  char arlam_data[4];
  PROCESS_BEGIN();

  while(1) {
   PROCESS_WAIT_EVENT_UNTIL(ev == event_arlam);
   
   strncpy(arlam_data, data, 4);
   printf("Arlam Event wake up with raw data: %s\n", arlam_data);
   arlam_flag = atoi(arlam_data);
   printf("Arlam Event wake up with arlam flag: %d\n", arlam_flag);
   
   if(arlam_flag == 112) {
    leds_on(LEDS_RED);
   } else if(arlam_flag == 110) {
    leds_off(LEDS_RED);
   }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(broadcast_intrusion_process, ev, data)
{
  
  static struct etimer periodic_timer;
  static struct etimer send_timer;
  uip_ipaddr_t addr;
  char message[7];
  
  
  
  PROCESS_BEGIN();

  simple_udp_register(&broadcast_connection, UDP_PORT,
                      NULL, UDP_PORT,
                      receiver);


  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == event_broadcast);
    printf("Broadcast Event wake up with raw data: %s\n", data);
    strncpy(message, data, 8);
    //message[3] = '\0';
    printf("Broadcast Event wake up with message: %s\n", message);
    uip_create_linklocal_allnodes_mcast(&addr);

    simple_udp_sendto(&broadcast_connection, message, 8, &addr);
    

  }

  PROCESS_END();
}

