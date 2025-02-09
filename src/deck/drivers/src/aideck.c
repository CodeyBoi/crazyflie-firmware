/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * aideck.c - Deck driver for the AIdeck
 */
#define DEBUG_MODULE "AIDECK"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "aideck.h"
#include "config.h"
#include "console.h"
#include "uart1.h"
#include "uart2.h"
#include "debug.h"
#include "deck.h"
#include "esp_deck_flasher.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "log.h"
#include "param.h"
#include "queue.h"
#include "stm32fxxx.h"
#include "system.h"

#include "aideck.h"
#include "aideck-router.h"

static bool isInit = false;
static uint8_t byte;

#define ESP_TX_QUEUE_LENGTH 4
#define ESP_RX_QUEUE_LENGTH 4

static xQueueHandle espTxQueue;
static xQueueHandle espRxQueue;


// Length of start + payloadLength
#define UART_HEADER_LENGTH 2
#define UART_CRC_LENGTH 1
#define UART_META_LENGTH (UART_HEADER_LENGTH + UART_CRC_LENGTH)

typedef struct {
  CPXTarget_t destination : 3;
  CPXTarget_t source : 3;
  bool lastPacket : 1;
  bool reserved : 1;
  CPXFunction_t function : 8;
} __attribute__((packed)) CPXRoutingPacked_t;

#define CPX_ROUTING_PACKED_SIZE (sizeof(CPXRoutingPacked_t))

typedef struct {
    CPXRoutingPacked_t route;
    uint8_t data[AIDECK_UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE];
} __attribute__((packed)) uartTransportPayload_t;

typedef struct {
    uint8_t start;
    uint8_t payloadLength; // Excluding start and crc
    union {
        uartTransportPayload_t routablePayload;
        uint8_t payload[AIDECK_UART_TRANSPORT_MTU];
    };

    uint8_t crcPlaceHolder; // Not actual position. CRC is added after the last byte of payload
} __attribute__((packed)) uart_transport_packet_t;

// Used when sending/receiving data on the UART
static uart_transport_packet_t espTxp;
static CPXPacket_t cpxTxp;
static uart_transport_packet_t espRxp;

static EventGroupHandle_t evGroup;
#define ESP_CTS_EVENT (1 << 0)
#define ESP_CTR_EVENT (1 << 1)
#define ESP_TXQ_EVENT (1 << 2)

static void assemblePacket(const CPXPacket_t *packet, uart_transport_packet_t * txp);

static uint8_t calcCrc(const uart_transport_packet_t* packet) {
  const uint8_t* start = (const uint8_t*) packet;
  const uint8_t* end = &packet->payload[packet->payloadLength];

  uint8_t crc = 0;
  for (const uint8_t* p = start; p < end; p++) {
    crc ^= *p;
  }

  return crc;
}

static void ESP_RX(void *param)
{
  systemWaitStart();

  while (1)
  {
    // Wait for start!
    do
    {
      uart2GetDataWithTimeout(&espRxp.start, (TickType_t)portMAX_DELAY);
    } while (espRxp.start != 0xFF);

    uart2GetDataWithTimeout(&espRxp.payloadLength, (TickType_t)portMAX_DELAY);

    if (espRxp.payloadLength == 0)
    {
      xEventGroupSetBits(evGroup, ESP_CTS_EVENT);
    }
    else
    {
      for (int i = 0; i < espRxp.payloadLength; i++)
      {
        uart2GetDataWithTimeout(&espRxp.payload[i], (TickType_t)portMAX_DELAY);
      }

      uint8_t crc;
      uart2GetDataWithTimeout(&crc, (TickType_t)portMAX_DELAY);
      ASSERT(crc == calcCrc(&espRxp));

      xQueueSend(espRxQueue, &espRxp, portMAX_DELAY);
      xEventGroupSetBits(evGroup, ESP_CTR_EVENT);
    }
  }
}

static void ESP_TX(void *param)
{
  systemWaitStart();

  uint8_t ctr[] = {0xFF, 0x00};
  EventBits_t evBits = 0;

  // We need to hold off here to make sure that the RX task
  // has started up and is waiting for chars, otherwise we might send
  // CTR and miss CTS (which means that the ESP32 will stop sending CTS
  // too early and we cannot sync)
  vTaskDelay(100);

  // Sync with ESP32 so both are in CTS
  do
  {
    uart2SendData(sizeof(ctr), (uint8_t *)&ctr);
    vTaskDelay(100);
    evBits = xEventGroupGetBits(evGroup);
  } while ((evBits & ESP_CTS_EVENT) != ESP_CTS_EVENT);

  while (1)
  {
    // If we have nothing to send then wait, either for something to be
    // queued or for a request to send CTR
    if (uxQueueMessagesWaiting(espTxQueue) == 0)
    {
      evBits = xEventGroupWaitBits(evGroup,
                                   ESP_CTR_EVENT | ESP_TXQ_EVENT,
                                   pdTRUE,  // Clear bits before returning
                                   pdFALSE, // Wait for any bit
                                   portMAX_DELAY);
      if ((evBits & ESP_CTR_EVENT) == ESP_CTR_EVENT)
      {
        uart2SendData(sizeof(ctr), (uint8_t *)&ctr);
      }
    }

    if (uxQueueMessagesWaiting(espTxQueue) > 0)
    {
      // Dequeue and wait for either CTS or CTR
      xQueueReceive(espTxQueue, &cpxTxp, 0);
      espTxp.start = 0xFF;
      assemblePacket(&cpxTxp, &espTxp);
      do
      {
        evBits = xEventGroupWaitBits(evGroup,
                                     ESP_CTR_EVENT | ESP_CTS_EVENT,
                                     pdTRUE,  // Clear bits before returning
                                     pdFALSE, // Wait for any bit
                                     portMAX_DELAY);
        if ((evBits & ESP_CTR_EVENT) == ESP_CTR_EVENT)
        {
          uart2SendData(sizeof(ctr), (uint8_t *)&ctr);
        }
      } while ((evBits & ESP_CTS_EVENT) != ESP_CTS_EVENT);
      uart2SendData((uint32_t) espTxp.payloadLength + UART_META_LENGTH, (uint8_t *)&espTxp);
    }
  }
}

static void Gap8Task(void *param)
{
  systemWaitStart();
  vTaskDelay(M2T(1000));

  // Read out the byte the Gap8 sends and immediately send it to the console.
  while (1)
  {
    uart1GetDataWithDefaultTimeout(&byte);
    consolePutchar(byte);
  }
}

static void assemblePacket(const CPXPacket_t *packet, uart_transport_packet_t * txp) {
  ASSERT((packet->route.destination >> 4) == 0);
  ASSERT((packet->route.source >> 4) == 0);
  ASSERT((packet->route.function >> 8) == 0);
  ASSERT(packet->dataLength <= AIDECK_UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);

  txp->payloadLength = packet->dataLength + CPX_ROUTING_PACKED_SIZE;
  txp->routablePayload.route.destination = packet->route.destination;
  txp->routablePayload.route.source = packet->route.source;
  txp->routablePayload.route.lastPacket = packet->route.lastPacket;
  txp->routablePayload.route.function = packet->route.function;
  memcpy(txp->routablePayload.data, &packet->data, packet->dataLength);
  txp->payload[txp->payloadLength] = calcCrc(txp);
}

void cpxReceivePacketBlocking(CPXPacket_t *packet)
{
  static uart_transport_packet_t cpxRxp;

  xQueueReceive(espRxQueue, &cpxRxp, portMAX_DELAY);

  packet->dataLength = (uint32_t) cpxRxp.payloadLength - CPX_ROUTING_PACKED_SIZE;
  packet->route.destination = cpxRxp.routablePayload.route.destination;
  packet->route.source = cpxRxp.routablePayload.route.source;
  packet->route.function = cpxRxp.routablePayload.route.function;
  packet->route.lastPacket = cpxRxp.routablePayload.route.lastPacket;
  memcpy(&packet->data, cpxRxp.routablePayload.data, packet->dataLength);
}

void cpxSendPacketBlocking(const CPXPacket_t *packet)
{
  xQueueSend(espTxQueue, packet, portMAX_DELAY);
  xEventGroupSetBits(evGroup, ESP_TXQ_EVENT);
}

bool cpxSendPacket(const CPXPacket_t *packet, uint32_t timeoutInMS)
{
  bool packetWasSent = false;
  if (xQueueSend(espTxQueue, packet, M2T(timeoutInMS)) == pdTRUE)
  {
    xEventGroupSetBits(evGroup, ESP_TXQ_EVENT);
    packetWasSent = true;
  }
  return packetWasSent;
}

void cpxInitRoute(const CPXTarget_t source, const CPXTarget_t destination, const CPXFunction_t function, CPXRouting_t* route) {
    route->source = source;
    route->destination = destination;
    route->function = function;
    route->lastPacket = true;
}

static void aideckInit(DeckInfo *info)
{

  if (isInit)
    return;

  // Initialize task for the GAP8
  xTaskCreate(Gap8Task, AI_DECK_GAP_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
              AI_DECK_TASK_PRI, NULL);

  espTxQueue = xQueueCreate(ESP_TX_QUEUE_LENGTH, sizeof(CPXPacket_t));
  espRxQueue = xQueueCreate(ESP_RX_QUEUE_LENGTH, sizeof(uart_transport_packet_t));

  evGroup = xEventGroupCreate();

  // Pull reset for GAP8/ESP32
  pinMode(DECK_GPIO_IO4, OUTPUT);
  digitalWrite(DECK_GPIO_IO4, LOW);
  //Initialize UARTs while GAP8/ESP32 is held in reset
  uart1Init(115200);
  uart2Init(115200);

  // Initialize task for the ESP while it's held in reset
  xTaskCreate(ESP_RX, AIDECK_ESP_RX_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
              AI_DECK_TASK_PRI, NULL);
  xTaskCreate(ESP_TX, AIDECK_ESP_TX_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
              AI_DECK_TASK_PRI, NULL);

  // Release reset for GAP8/ESP32
  digitalWrite(DECK_GPIO_IO4, HIGH);
  pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

  aideckRouterInit();

  isInit = true;
}

static bool aideckTest()
{

  return true;
}

static const DeckMemDef_t memoryDef = {
    .write = espDeckFlasherWrite,
    .read = 0,
    .properties = espDeckFlasherPropertiesQuery,
    .supportsUpgrade = true,

    .requiredSize = ESP_BITSTREAM_SIZE,
    // .requiredHash = ESP_BITSTREAM_CRC,
};

static const DeckDriver aideck_deck = {
    .vid = 0xBC,
    .pid = 0x12,
    .name = "bcAI",

    .usedGpio = DECK_USING_IO_4,
    .usedPeriph = DECK_USING_UART1,

    .memoryDef = &memoryDef,

    .init = aideckInit,
    .test = aideckTest,
};

LOG_GROUP_START(aideck)
LOG_ADD(LOG_UINT8, receivebyte, &byte)
LOG_GROUP_STOP(aideck)

/** @addtogroup deck
*/
PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [AI deck](%https://store.bitcraze.io/collections/decks/products/ai-deck-1-1) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcAIDeck, &isInit)

PARAM_GROUP_STOP(deck)

DECK_DRIVER(aideck_deck);
