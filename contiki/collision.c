// /**
//  * \addtogroup timesynch
//  * @{
//  */


/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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


/**
 * \file
 *        Resilient Clustering in Contiki
 * \author
 *         Nitin Shivaraman <nitin.shivaraman@tum-create.edu.sg>
 */


#include "contiki.h"
#include "net/rime/rime.h"
#include "dev/cc2420/cc2420.h"
#include "sys/rtimer.h"
#include "node-id.h"
#include "net/packetbuf.h"
#include "net/netstack.h"
#include <stdio.h>

// #include "project-conf.h"


#define FALSE                      0      // Error state
#define TRUE                       1      // OK state
#define UNKNOWN_NODE               0xFF   // Default value of a node
#define RSSI_THRESHOLD             65     // Signal strength for neighbours
#define RSSI_OFFSET                -45    // As mentioned in the datasheet of cc2420
#define MAX_NEIGHBOURS             64     // Maximum number of neighbour nodes for each node
#define XFER_CHANNEL               25      // Channel used for data transfer
#define EPOCH                      10     // Number of rounds per epoch

// A message is sent every 1.1 seconds by the root
// #define MIN_INTERVAL CLOCK_SECOND * 0.6
#define MIN_INTERVAL CLOCK_SECOND * 0.5

/**
 *  Message format
 */
struct message_t
{
    char msg[24];
    // uint16_t serial_no;
};

PROCESS(collision_process, "Collision Detection");
AUTOSTART_PROCESSES(&collision_process);

/*---------------------------------------------------------------------------*/

static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
    // synchronization message
    struct message_t msg;
    int16_t received_rssi = cc2420_last_rssi;
    received_rssi += RSSI_OFFSET;

    /* The packetbuf_dataptr() returns a pointer to the first data byte
       in the received packet. */

    memcpy(&msg, packetbuf_dataptr(), sizeof(struct message_t));
    // printf("Received RSSI is %d\n", received_rssi);

    // printf("BYTE RECV: %s\n", msg.msg);
    // printf("SIZE RECV: %d\n", packetbuf_datalen());
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(collision_process, ev, data)
{

  static struct etimer sendtimer;
  static clock_time_t interval;
  struct message_t beacon;
  // uint16_t count = 0;

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
  PROCESS_BEGIN();

  broadcast_open(&broadcast, XFER_CHANNEL, &broadcast_call);
  interval = MIN_INTERVAL;

  while(1)
  {

    etimer_set(&sendtimer, interval);
    PROCESS_WAIT_UNTIL(etimer_expired(&sendtimer));
    
      memcpy(&beacon.msg, "NTUUUUUUUUUUUUUUUUUUUU!", 24);
      // NUSSSSSSSSSSSSSSSSSSSS
      // NTUUUUUUUUUUUUUUUUUUUU
    // beacon.serial_no = count++;

    packetbuf_copyfrom(&beacon, sizeof(beacon));
    broadcast_send(&broadcast);
  }

  PROCESS_END();
  return 1;
}
