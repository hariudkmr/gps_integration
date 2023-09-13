/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "app.h"

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_eusart.h"
#include "em_gpio.h"

// Size of the buffer for received data
#define BUFLEN  80

// Receive data buffer
uint8_t buffer[BUFLEN];

// Current position in buffer
uint32_t inpos = 0;
uint32_t outpos = 0;

// True while receiving data (waiting for CR or BUFLEN characters)
bool data_received = false;

/**************************************************************************//**
 * @brief
 *    CMU initialization
 *****************************************************************************/
void initCMU(void)
{
  // Enable clock to GPIO and EUSART1
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_EUSART1, true);
}

/**************************************************************************//**
 * @brief
 *    GPIO initialization
 *****************************************************************************/
void initGPIO(void)
{
  // Configure the EUSART TX pin to the board controller as an output
  GPIO_PinModeSet(GPS_TXPORT, GPS_TXPIN, gpioModePushPull, 1);

  // Configure the EUSART RX pin to the board controller as an input
  GPIO_PinModeSet(GPS_RXPORT, GPS_RXPIN, gpioModeInput, 0);

}

/**************************************************************************//**
 * @brief
 *    EUSART1 initialization
 *****************************************************************************/
void initEUSART1(void)
{
  // Default asynchronous initializer (115.2 Kbps, 8N1, no flow control)
  EUSART_UartInit_TypeDef euartInit;
  euartInit.enable = eusartEnable;               /* Enable RX/TX when initialization completed. */                      \
  euartInit.refFreq =    0;                          /* Use current configured reference clock for configuring baud rate.*/ \
  euartInit.baudrate =    9600;                       /* 9600 bits/s. */                                                     \
  euartInit.oversampling =   eusartOVS16;                 /* Oversampling disabled. */                                           \
  euartInit.databits =   eusartDataBits8;            /* 8 data bits. */                                                     \
  euartInit.parity =   eusartNoParity;             /* No parity. */                                                       \
  euartInit.stopbits  =  eusartStopbits1;            /* 1 stop bit. */                                                      \
  euartInit.majorityVote  =  eusartMajorityVoteDisable;  /* Majority vote enabled. */                                           \
  euartInit.loopbackEnable =  eusartLoopbackDisable;
  euartInit.advancedSettings = 0;

  // Route EUSART1 TX and RX to the board controller TX and RX pins
  GPIO->EUSARTROUTE[1].TXROUTE = (GPS_TXPORT << _GPIO_EUSART_TXROUTE_PORT_SHIFT)
      | (GPS_TXPIN << _GPIO_EUSART_TXROUTE_PIN_SHIFT);
  GPIO->EUSARTROUTE[1].RXROUTE = (GPS_RXPORT << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
      | (GPS_RXPIN << _GPIO_EUSART_RXROUTE_PIN_SHIFT);

  // Enable RX and TX signals now that they have been routed
  GPIO->EUSARTROUTE[1].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN | GPIO_EUSART_ROUTEEN_TXPEN;

  // Configure and enable EUSART1 for high-frequency (EM0/1) operation
  EUSART_UartInitHf(EUSART1, &euartInit);

  // Enable NVIC USART sources
  NVIC_ClearPendingIRQ(EUSART1_RX_IRQn);
  NVIC_EnableIRQ(EUSART1_RX_IRQn);
  NVIC_ClearPendingIRQ(EUSART1_TX_IRQn);
  NVIC_EnableIRQ(EUSART1_TX_IRQn);

  EUSART_IntEnable(EUSART1, EUSART_IEN_RXFL);
}

/**************************************************************************//**
 * @brief
 *    The EUSART1 receive interrupt saves incoming characters.
 *****************************************************************************/
void EUSART1_RX_IRQHandler(void)
{
  // Get the character just received
  buffer[inpos] = EUSART1->RXDATA;

  // Exit loop on new line or buffer full
  if ((buffer[inpos] != '\r') && (inpos < BUFLEN))
    inpos++;
  else
    data_received = true;   // Stop receiving on CR

  /*
   * The EUSART differs from the USART in that explicit clearing of
   * RX interrupt flags is required even after emptying the RX FIFO.
   */
  EUSART_IntClear(EUSART1, EUSART_IF_RXFL);
}


/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
  initCMU();
  initGPIO();
  initEUSART1();

}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
  if(data_received == true)
    {
      data_received = false;
      inpos =0;
    }

}
