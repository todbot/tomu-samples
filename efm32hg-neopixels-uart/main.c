/*****************************************************************************
 *
 * EFM32HG Neopixels using 4Mbps UART
 *
 * 2018, Tod E. Kurt, http://todbot.com/blog
 * 
 * Technique from https://www.pjrc.com/non-blocking-ws2812-led-library/
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
#include <em_core.h>
#include "dmactrl.h"

#define DMA_CHANNEL_TX   0
#define DMA_CHANNEL_RX   1
#define DMA_CHANNELS     2

// DMA Callback structure 
DMA_CB_TypeDef dmaCallback;

// DMA Transfer Flag 
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

/***********************************************************************
 * @brief  Call-back called when transfer is complete
 ***********************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user)
{
  (void) primary;  (void) user;

  // Clear flag to indicate complete transfer 
  if (channel == DMA_CHANNEL_TX) {
    txActive = false;  
  }
}


/**********************************************************************
 * @brief Set up the USART0 to be 4Mbps 8N1
 **********************************************************************/
static void setupSerialPort(void)
{
  USART_InitAsync_TypeDef init  = USART_INITASYNC_DEFAULT;

  // Configure GPIO pins. 
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(gpioPortE, 10, gpioModePushPull, 0); // MOSI / TX

  // Enable peripheral clocks. 
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_USART0, true);

  // Configure UART for basic async operation. 
  init.enable = usartDisable;
  init.databits = usartDatabits8;
  init.baudrate = 4000000;  // 4MHz
  init.oversampling = usartOVS4; // allows for faster baudrate
  USART_InitAsync( USART0, &init);

  // invert output so start bit goes high
  USART0->CTRL |= USART_CTRL_TXINV;

  // Route USART TX to LOC0 ( PortE10)
  USART0->ROUTE = USART_ROUTE_LOCATION_LOC0 |
                  USART_ROUTE_TXPEN;

  // Finally enable it 
  USART_Enable( USART0, usartEnable);
}


/**********************************************************************
 * @brief Configure DMA in basic mode for TX from USART
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
  dmaCallback.cbFunc  = transferComplete;
  dmaCallback.userPtr = NULL;

  // Setting up TX channel 
  txChnlCfg.highPri   = false;
  txChnlCfg.enableInt = true;
  txChnlCfg.select    = DMAREQ_USART0_TXBL;
  txChnlCfg.cb        = &dmaCallback;
  DMA_CfgChannel(DMA_CHANNEL_TX, &txChnlCfg);

  // Setting up TX channel descriptor 
  txDescrCfg.dstInc  = dmaDataIncNone;
  txDescrCfg.srcInc  = dmaDataInc1;
  txDescrCfg.size    = dmaDataSize1;
  txDescrCfg.arbRate = dmaArbitrate1;
  txDescrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_TX, true, &txDescrCfg);
}

/**********************************************************************
 * @brief Send a byte buffer out USART0 via DMA, sets txActive
 **********************************************************************/
void dmaTransferOut(uint8_t *txBuffer, int count)
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
                    (void *)&(USART0->TXDATA),
                    txBuffer,
                    count - 1);
}


/**********************************************************************
 * @brief  Returns if an DMA transfer is active
 **********************************************************************/
bool dmaIsActive(void)
{
  bool temp;
  temp = txActive;
  return temp;
}


/*********************************************************************
 * @brief  Sleep in EM1 until DMA transfer is done
 *********************************************************************/
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
  
  while(1) {
    // INT_Disable();  
    CORE_irqState_t is = CORE_EnterCritical();
    isActive = dmaIsActive();
    if ( isActive ) {
      EMU_EnterEM1(); 
    }
    CORE_ExitCritical(is);
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

#define nLEDs 8
rgb_t drawBuffer[nLEDs]; //  3 bytes per RGB LED
uint8_t frameBuffer[nLEDs*3*4]; // 12 bytes per RGB LED

// this code from: https://www.pjrc.com/non-blocking-ws2812-led-library/
// and https://github.com/PaulStoffregen/WS2812Serial

// WS2812Serial::clear()
void clearLEDs()
{
  memset(drawBuffer, 0, nLEDs * sizeof(rgb_t));
} 	

// WS2812Serial::setPixel()
void setLED(uint32_t num, uint8_t red, uint8_t green, uint8_t blue)
{
		if (num >= nLEDs) return;
		drawBuffer[num].r = red;
		drawBuffer[num].g = green;
		drawBuffer[num].b = blue;
}
// WS2812Serial::show()
static void showLEDs()
{
	// copy drawing buffer to frame buffer
	const uint8_t *p = (uint8_t*)drawBuffer;
	const uint8_t *end = p + (nLEDs * 3);
	uint8_t *fb = frameBuffer;
	while (p < end) {
		uint8_t r = *p++; // FIXME: assuming rgb_t aligned
		uint8_t g = *p++;
		uint8_t b = *p++;
    // color ordering of our LEDs is GRB
		uint32_t n  = n = (g << 16) | (r << 8) | b;
		const uint8_t *stop = fb + 12;
		do {
			uint8_t x = 0x08;
			if (!(n & 0x00800000)) x |= 0x07;
			if (!(n & 0x00400000)) x |= 0xE0;
			n <<= 2;
			*fb++ = x;
		} while (fb < stop);
	}
  dmaTransferOut( frameBuffer, nLEDs * 12);
}

/***********************************************************************
 * @brief  Main function
 ***********************************************************************/
int main(void)
{ 
  // Initialize chip 
  CHIP_Init();
  
  // Disable the watchdog that the bootloader started.
  WDOG->CTRL = 0;
  
  // Configuring clocks in the Clock Management Unit (CMU) 
  CMU_ClockEnable(cmuClock_DMA, true);  
  CMU_ClockEnable(cmuClock_USART0, true);  
  CMU_ClockEnable(cmuClock_GPIO, true);

  // these are debug LEDs on the dev board
  GPIO_PinModeSet(gpioPortF, 4, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortF, 5, gpioModePushPull, 0);

  GPIO_PinOutSet(gpioPortF, 4);  // say we are alive

  // configure SysTick handler for millis 
  setupSysTick();

  // configure USART0
  setupSerialPort();

  // Configure DMA transfer from RAM to SPI using ping-pong
  setupDma();
  
  /*
  // just playing around, testing if oscilloscope can resolve this
  while(1) {
    for( uint8_t c = 'a'; c< 'z'; c++)  {
      GPIO_PinOutSet(gpioPortF, 5);
      USART_Tx( USART0, c);
      GPIO_PinOutClear(gpioPortF, 5);
    }
  }    
  */
  /*
  // more scope playing, this time with DMA
  char str[] = "hello tod this is todbot";
  while( 1 ) {
    dmaTransferOut( (uint8_t*)str, strlen(str) );
    GPIO_PinOutSet(gpioPortF, 5);
    sleepUntilDmaDone();
  }
  */

  for( int i=0; i< nLEDs; i++) {
    setLED( i, 0,0,0 );
  }
  showLEDs();
  sleepUntilDmaDone();
  SpinDelay(1000);
  
  setLED( 0, 0x40, 0x00, 0x00 );
  setLED( 1, 0x00, 0x40, 0x00 );
  setLED( 2, 0x00, 0x00, 0x40 );
  setLED( 3, 0x40, 0x40, 0x00 );
  setLED( 4, 0x00, 0x40, 0x40 );
  setLED( 5, 0x00, 0x40, 0x40 );
  setLED( 6, 0x40, 0x40, 0x40 );
  setLED( 7, 0x00, 0x00, 0x00 );

  while( 1 ) {
    rgb_t l = drawBuffer[0];
    for( int i=1; i< nLEDs; i++ ) {
      drawBuffer[i-1] = drawBuffer[i];
    }
    drawBuffer[nLEDs-1] = l;
    
    showLEDs();
    sleepUntilDmaDone();
    SpinDelay(300);
  }

   
  // Cleaning up after DMA transfers 
  DMA_Reset();

  // Done 
  while (1);
}
