/*****************************************************************************
 * @file spi_master.c
 * @brief DMA SPI master transmit/receive example
 * @author Silicon Labs
 * @version 2.06
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "em_dma.h"
#include "em_cmu.h"
#include "em_emu.h"
//#include "em_int.h"
#include "dmactrl.h"

#define DMA_CHANNEL_TX   0
#define DMA_CHANNEL_RX   1
#define DMA_CHANNELS     2

/* DMA Callback structure */
DMA_CB_TypeDef spiCallback;

/* Transfer Flags */
volatile bool rxActive;
volatile bool txActive;

/* SPI Data Buffers */

const uint16_t spiTxData2[] = {
                                0b000000000010,
                                0b000000000101,
                                0b010000000000, 
                                0b011000000000,
                                0x0123,
                                0x0456, 0x0DEF, 0x0ABA, 0x0CAB, 0x0B0B };
int spiTxData2Len = (sizeof(spiTxData2)/sizeof(uint16_t));


// The uptime in milliseconds, maintained by the SysTick timer.
volatile uint32_t uptime_millis;

// This functions is injected into the Interrupt Vector Table, and will be
// called whenever the SysTick timer fires (whose interval is configured inside
// main() further below).
void SysTick_Handler() {
  uptime_millis++;
}
// simple delay() 
void SpinDelay(uint32_t millis) {
  // Calculate the time at which we need to finish "sleeping".
  uint32_t sleep_until = uptime_millis + millis;

  // Spin until the requested time has passed.
  while (uptime_millis < sleep_until);
}
void setupSysTick() {
    // Sets up and enable the `SysTick_Handler' interrupt to fire once every 1ms.
  // ref: http://community.silabs.com/t5/Official-Blog-of-Silicon-Labs/Chapter-5-MCU-Clocking-Part-2-The-SysTick-Interrupt/ba-p/145297
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) {
    // Something went wrong.
    while (1);
  }
}

/**************************************************************************//**
 * @brief  Call-back called when transfer is complete
 *****************************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user)
{
  (void) primary;
  (void) user;
  
  /* Clear flag to indicate complete transfer */
  if (channel == DMA_CHANNEL_TX) {
    txActive = false;  
  }
}

/**************************************************************************//**
 * @brief  Enabling clocks
 *****************************************************************************/
void setupCmu(void)
{  
  /* Enabling clocks */
  CMU_ClockEnable(cmuClock_DMA, true);  
  CMU_ClockEnable(cmuClock_GPIO, true);  
  CMU_ClockEnable(cmuClock_USART0, true);  
}

/**************************************************************************//**
 * @brief  Setup SPI as Master
 *****************************************************************************/
void setupSpi(void)
{
  USART_InitSync_TypeDef usartInit = USART_INITSYNC_DEFAULT;  
  
  /* Initialize SPI */
  //usartInit.databits = usartDatabits8;
  usartInit.databits = usartDatabits12;
  //usartInit.baudrate = 1000000;
  usartInit.baudrate = 2400000;
  usartInit.msbf = true;
  //usartInit.baudrate = 12000000;
  //usartInit.baudrate = 20000000;
  USART_InitSync(USART0, &usartInit);
  
  /* Turn on automatic Chip Select control */
  //USART0->CTRL |= USART_CTRL_AUTOCS;
  
  /* Enable SPI transmit and receive */
  USART_Enable(USART0, usartEnable);
  
  // Configure GPIO pins for SPI 
  GPIO_PinModeSet(gpioPortE, 12, gpioModePushPull, 0); // CLK
  GPIO_PinModeSet(gpioPortE, 10, gpioModePushPull, 0); // MOSI 
  //GPIO_PinModeSet(gpioPortD, 1, gpioModeInput,    0); // MISO
  //GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 1); // CS 
 
  // Route USART clock to display clock and USART TX to display SI
  USART0->ROUTE = USART_ROUTE_LOCATION_LOC0 |
                  USART_ROUTE_CLKPEN |
                  USART_ROUTE_TXPEN;
  
  /*
  // Enable routing for SPI pins from USART to location 1 
  USART0->ROUTE = USART_ROUTE_TXPEN | 
                  USART_ROUTE_RXPEN | 
                  USART_ROUTE_CSPEN | 
                  USART_ROUTE_CLKPEN | 
                  USART_ROUTE_LOCATION_LOC1;
  */
}



/**************************************************************************//**
 * @brief Configure DMA in basic mode for both TX and RX to/from USART
 *****************************************************************************/
void setupDma(void)
{
  /* Initialization structs */
  DMA_Init_TypeDef        dmaInit;
  DMA_CfgChannel_TypeDef  txChnlCfg;
  DMA_CfgDescr_TypeDef    txDescrCfg;
  
  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);
  
  /* Setup call-back function */  
  spiCallback.cbFunc  = transferComplete;
  spiCallback.userPtr = NULL;
  

  /*** Setting up TX DMA ***/

  /* Setting up channel */
  txChnlCfg.highPri   = false;
  txChnlCfg.enableInt = true;
  txChnlCfg.select    = DMAREQ_USART0_TXBL;
  txChnlCfg.cb        = &spiCallback;
  DMA_CfgChannel(DMA_CHANNEL_TX, &txChnlCfg);

  /* Setting up channel descriptor */
  txDescrCfg.dstInc  = dmaDataIncNone;
  txDescrCfg.srcInc  = dmaDataInc2;
  txDescrCfg.size    = dmaDataSize2;
  txDescrCfg.arbRate = dmaArbitrate1;
  txDescrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_TX, true, &txDescrCfg);
}

//
//void spiDmaTransferOut(uint8_t *txBuffer, int bytes)
void spiDmaTransferOut2(uint8_t *txBuffer, int count)
{
  // Setting flag to indicate that TX is in progress
  // will be cleared by call-back function 
  txActive = true;
  
  // Clear TX regsiters 
  USART0->CMD = USART_CMD_CLEARTX;
  
  // Activate TX channel 
  DMA_ActivateBasic(DMA_CHANNEL_TX,
                    true,
                    false,
                    (void *)&(USART0->TXDOUBLE),
                    txBuffer,
                    count - 1);
}




/**************************************************************************//**
 * @brief  Returns if an SPI transfer is active
 *****************************************************************************/
bool spiDmaIsActive(void)
{
  bool temp;
  temp = rxActive;
  temp = temp | txActive;
  return temp;
}



/**************************************************************************//**
 * @brief  Sleep in EM1 until DMA transfer is done
 *****************************************************************************/
void sleepUntilDmaDone(void)
{
  /* Enter EM1 while DMA transfer is active to save power. Note that
   * interrupts are disabled to prevent the ISR from being triggered
   * after checking the transferActive flag, but before entering
   * sleep. If this were to happen, there would be no interrupt to wake
   * the core again and the MCU would be stuck in EM1. While the 
   * core is in sleep, pending interrupts will still wake up the 
   * core and the ISR will be triggered after interrupts are enabled
   * again. 
   */ 
  bool isActive = false;
  
  while(1)
  {
    //    INT_Disable();
    isActive = spiDmaIsActive();
    if ( isActive ) {
      EMU_EnterEM1(); 
    }
    //INT_Enable();
    
    /* Exit the loop if transfer has completed */
    if ( !isActive ) {
      break;
    }
  }  
}


typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} rgb_t;

static const uint16_t bits[] = {
    0b100100100100,
    0b100100100110,
    0b100100110100,
    0b100100110110,
    0b100110100100,
    0b100110100110,
    0b100110110100,
    0b100110110110,
    0b110100100100,
    0b110100100110,
    0b110100110100,
    0b110100110110,
    0b110110100100,
    0b110110100110,
    0b110110110100,
    0b110110110110,
};

#define spiSend(x) USART_TxDouble( USART0, x)

static void sendByte (int value)
{
    spiSend( bits[value >> 4] );
    spiSend( bits[value & 0xF] );
}

static void sendRGB (int r, int g, int b)
{
    sendByte(g);
    sendByte(r);
    sendByte(b);
}

static void sendLEDs(rgb_t* leds, int num)
{
  for( int i=0; i<num; i++ ) {
    sendByte( leds[i].g );
    sendByte( leds[i].r );
    sendByte( leds[i].b );
  }
}


/**************************************************************************//**
 * @brief  Main function
 * This example sets up the DMA to transfer outbound and incoming data from the
 * SPI (USART0) to/from the source/destination buffers. Three tests are done:
 * 1) Transmit data (string) without reading received data
 * 2) Transmit data (string) and transfer received data to RAM buffer
 * 3) Transmit dummy data and transfer received data to RAM buffer
 *****************************************************************************/
int main(void)
{ 
  // Initialize chip 
  CHIP_Init();
  
  // Disable the watchdog that the bootloader started.
  WDOG->CTRL = 0;
  
  // Switch on the clock for GPIO. Even though there's no immediately obvious
  // timing stuff going on beyond the SysTick below, it still needs to be
  // enabled for the GPIO to work.
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(gpioPortF, 4, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortF, 5, gpioModePushPull, 0);

  // Configuring clocks in the Clock Management Unit (CMU) 
  setupCmu();
  
  // Configura USART for SPI 
  setupSpi();

  // configure SysTick handler for millis 
  setupSysTick();

  
  GPIO_PinOutSet(gpioPortF, 4);
  /*
  while( 1 ) { 
    GPIO_PinOutSet(gpioPortF, 4);
    SpinDelay(100);
    GPIO_PinOutClear(gpioPortF, 4);
    SpinDelay(100);
  }
  */

  /*
  while(1) {
    for( int i=0x100; i<0x350; i++) {
      USART_TxDouble( USART0, i);
    }
  }
  */

  rgb_t leds[8]=
    {
      {0x40, 0x00, 0x00},
      {0x00, 0x40, 0x00},
      {0x00, 0x00, 0x40},
      {0x40, 0x40, 0x00},
      {0x40, 0x00, 0x40},
      {0x00, 0x40, 0x40},
      {0x40, 0x40, 0x40},
      {0x00, 0x00, 0x00},
    };

  while( 1 ) {  
    sendLEDs( leds, 8);

    rgb_t l = leds[0];
    for( int i=0; i<7; i++) {
      leds[i] = leds[i+1];
    }
    leds[7] = l;

    SpinDelay(100);
  }
  
  while( 1 ) {
    
    sendRGB( 0x40, 0x00, 0x00 );
    sendRGB( 0x00, 0x40, 0x00 );
    sendRGB( 0x00, 0x00, 0x40 );
    sendRGB( 0x40, 0x00, 0x40 );
    sendRGB( 0x40, 0x40, 0x00 );
    sendRGB( 0x00, 0x40, 0x40 );
    sendRGB( 0x40, 0x40, 0x40 );
    sendRGB( 0x10, 0x10, 0x10 );
    SpinDelay(500);
    
    sendRGB( 0x00, 0x00, 0x00 );
    sendRGB( 0x00, 0x00, 0x00 );
    sendRGB( 0x00, 0x00, 0x00 );
    sendRGB( 0x00, 0x00, 0x00 );
    sendRGB( 0x00, 0x00, 0x00 );
    sendRGB( 0x00, 0x00, 0x00 );
    sendRGB( 0x00, 0x00, 0x00 );
    sendRGB( 0x00, 0x00, 0x00 );
    SpinDelay(500); 
 }
  
  /* Configure DMA transfer from RAM to SPI using ping-pong */      
  setupDma();

  while( 1 ) {
    // Send data out
    spiDmaTransferOut2((uint8_t*) spiTxData2, spiTxData2Len);
  
    GPIO_PinOutSet(gpioPortF, 5);
    
    // Sleep until DMA is done 
    sleepUntilDmaDone();
  }
  
 
  /* Cleaning up after DMA transfers */
  DMA_Reset();

  /* Done */
  while (1);
}
