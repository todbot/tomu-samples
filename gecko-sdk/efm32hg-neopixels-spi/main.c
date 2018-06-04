/*****************************************************************************
 *
 * EFM32HG Neopixels using SPI
 *
 * 2018, Tod E. Kurt, http://todbot.com/blog
 * 
 * Technique from https://jeelabs.org/book/1450d/
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

// DMA Callback structure 
DMA_CB_TypeDef spiCallback;

// Transfer Flags 
volatile bool txActive;


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
    while (1);      // Something went wrong.
  }
}

/**********************************************************************
 * @brief  Call-back called when transfer is complete
 **********************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user)
{
  (void) primary;
  (void) user;
  
  /* Clear flag to indicate complete transfer */
  if (channel == DMA_CHANNEL_TX) {
    txActive = false;  
  }
}

/**********************************************************************
 * @brief  Setup SPI as Master
 **********************************************************************/
void setupSpi(void)
{
  USART_InitSync_TypeDef usartInit = USART_INITSYNC_DEFAULT;  
  
  // Initialize SPI 
  usartInit.databits = usartDatabits12;
  //usartInit.baudrate = 2400000; // 2.4MHz
  usartInit.baudrate = 3000000; // 3.0MHz (better for SK6812)
  usartInit.msbf = true;

  USART_InitSync(USART0, &usartInit);
  
  // Enable SPI transmit and receive 
  USART_Enable(USART0, usartEnable);
  
  // Configure GPIO pins for SPI
  // These are the values on the EFM32HG dev board
  GPIO_PinModeSet(gpioPortE, 12, gpioModePushPull, 0); // CLK
  GPIO_PinModeSet(gpioPortE, 10, gpioModePushPull, 0); // MOSI 
 
  // Route USART clock and USART TX to LOC0 (PortE12, PortE10)
  USART0->ROUTE = USART_ROUTE_LOCATION_LOC0 |
                  USART_ROUTE_CLKPEN |
                  USART_ROUTE_TXPEN;
}


/**********************************************************************
 * @brief Configure DMA in basic mode for both TX and RX to/from USART
 **********************************************************************/
void setupDma(void)
{
  // Initialization structs
  DMA_Init_TypeDef        dmaInit;
  DMA_CfgChannel_TypeDef  txChnlCfg;
  DMA_CfgDescr_TypeDef    txDescrCfg;
  
  // Initializing the DMA 
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);
  
  // Setup call-back function 
  spiCallback.cbFunc  = transferComplete;
  spiCallback.userPtr = NULL;

  /*** Setting up TX DMA ***/

  // Setting up channel 
  txChnlCfg.highPri   = false;
  txChnlCfg.enableInt = true;
  txChnlCfg.select    = DMAREQ_USART0_TXBL;
  txChnlCfg.cb        = &spiCallback;
  DMA_CfgChannel(DMA_CHANNEL_TX, &txChnlCfg);

  // Setting up channel descriptor 
  txDescrCfg.dstInc  = dmaDataIncNone;
  txDescrCfg.srcInc  = dmaDataInc2;  // Note double-wide
  txDescrCfg.size    = dmaDataSize2; // Note double-wide
  txDescrCfg.arbRate = dmaArbitrate1;
  txDescrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_TX, true, &txDescrCfg);
}

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
                    (void *)&(USART0->TXDOUBLE), // Note double-wide
                    txBuffer,
                    count - 1);
}

/**********************************************************************
 * @brief  Returns if an SPI transfer is active
 **********************************************************************/
bool spiDmaIsActive(void)
{
  bool temp;
  temp = txActive;
  return temp;
}

/***********************************************************************
 * @brief  Sleep in EM1 until DMA transfer is done
 ***********************************************************************/
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


// this idea from JeeLabs:
// https://jeelabs.org/book/1450d/

// @ 2.4MHz, 3 bits for each ws2812 bit
// ws2812 0 bit = 0b10000
// ws2812 1 bit = 0b11110
// => 12 bits carry 4 ws2812 bits
// To send one ws2812 byte, send two 12-bit transfers
// concept from: https://jeelabs.org/book/1450d/
static const uint16_t bits[] = {
    0b100100100100, // => 0b0000 in ws2812 bits
    0b100100100110, // => 0b0001 in ws2812 bits
    0b100100110100, // => 0b0010 in ws2812 bits
    0b100100110110, // => 0b0011 in ws2812 bits
    0b100110100100, // => 0b0100 in ws2812 bits
    0b100110100110, // => 0b0101 in ws2812 bits
    0b100110110100, // => 0b0110 in ws2812 bits
    0b100110110110, // => 0b0111 in ws2812 bits
    0b110100100100, // => 0b1000 in ws2812 bits
    0b110100100110, // => 0b1001 in ws2812 bits
    0b110100110100, // => 0b1010 in ws2812 bits
    0b110100110110, // => 0b1011 in ws2812 bits
    0b110110100100, // => 0b1100 in ws2812 bits
    0b110110100110, // => 0b1101 in ws2812 bits
    0b110110110100, // => 0b1110 in ws2812 bits
    0b110110110110, // => 0b1111 in ws2812 bits
};
// note double-wide
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
 *****************************************************************************/
int main(void)
{ 
  // Initialize chip 
  CHIP_Init();
  
  // Disable the watchdog that the bootloader started.
  WDOG->CTRL = 0;
  
  // Configuring clocks in the Clock Management Unit (CMU) 
  CMU_ClockEnable(cmuClock_DMA, true);  
  CMU_ClockEnable(cmuClock_USART0, true);  
  // Switch on the clock for GPIO. Even though there's no immediately obvious
  // timing stuff going on beyond the SysTick below, it still needs to be
  // enabled for the GPIO to work.
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configura USART for SPI 
  setupSpi();

  // configure SysTick handler for millis 
  setupSysTick();

  sendRGB( 0,0,0 ); // just to test

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
  
  /*  
  // more tests
  const uint16_t spiTxData2[] = {
                                0b000000000010,
                                0b000000000101,
                                0b010000000000, 
                                0b011000000000,
                                0x0123,
                                0x0456, 0x0DEF, 0x0ABA, 0x0CAB, 0x0B0B };
  int spiTxData2Len = (sizeof(spiTxData2)/sizeof(uint16_t));

  // Configure DMA transfer from RAM to SPI using ping-pong
  setupDma();

  while( 1 ) {
    // Send data out
    spiDmaTransferOut2((uint8_t*) spiTxData2, spiTxData2Len);
    // Sleep until DMA is done 
    sleepUntilDmaDone();
  }

  */
  
  // Cleaning up after DMA transfers 
  DMA_Reset();

  // Done 
  while (1);
}

// Alternate idea:
/*
// @ 4MHz, 5-bits for each ws2812 bit
// ws2812 0 bit = 0b10000
// ws2812 1 bit = 0b11110
// 15 bits carry 3 ws2812 bits
// => 45 bits carry 6 ws bits
// => 90 bits carry 12 ws bits
// 10 bits carry 2 ws2812 bits
// => 20 bits c
// concept from: https://www.pjrc.com/non-blocking-ws2812-led-library/
// (but SPI instead of UART because we can't invert the output?)
// NAH FORGET THIS: look up 'TXINV' in USARTn_CTRL
static const uint16_t bits4M[] = {
  0b100001000010000, // => 0b000
  0b100001000011110, // => 0b001
  0b100001111010000, // => 0b010
  0b100001111011110, // => 0b011
  0b111101000010000, // => 0b100
  0b111101000011110, // => 0b101
  0b111101111010000, // => 0b110
  0b111101111011110, // => 0b111
};
*/
