#include <stdint.h>
#include <stdbool.h>

#include "capsense.h"
#include "usbconfig.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_usb.h"
#include "em_wdog.h"
#include "em_leuart.h"

#include "callbacks.h"
#include "descriptors.h"

#define LEUART_LOCATION    LEUART_ROUTE_LOCATION_LOC1
#define LEUART_TXPORT      gpioPortB         // LEUART transmission port 
#define LEUART_TXPIN       13                // LEUART transmission pin  
#define LEUART_RXPORT      gpioPortB         // LEUART reception port    
#define LEUART_RXPIN       14                // LEUART reception pin     

// The uptime of this application in milliseconds, maintained by the SysTick
// timer.
volatile uint32_t uptime_millis;

// This functions is injected into the Interrupt Vector Table, and will be
// called whenever the SysTick timer fires (whose interval is configured inside
// main() further below).
void SysTick_Handler() {
  uptime_millis++;
}

void SpinDelay(uint32_t millis) {
  // Calculate the time at which we need to finish "sleeping".
  uint32_t sleep_until = uptime_millis + millis;

  // Spin until the requested time has passed.
  while (uptime_millis < sleep_until);
}

// Write a single character out LEUART0
int write_char(int c) {
  while (!(LEUART0->STATUS & LEUART_STATUS_TXBL))
        ;
  LEUART0->TXDATA = (uint32_t)c & 0xffUL;
  return 0;
}
// Write a null-terminated string out LEUART0
int write_str(const char *s) {
  while (*s)
    write_char(*s++);
  return 0;
}
// Set up LEUART0 on Tomu. LEUART0 is only 9600bps.
void setupLeuart(void) {  
  LEUART_Init_TypeDef leuart_init = LEUART_INIT_DEFAULT;

  // Enable peripheral clocks 
  CMU_ClockEnable(cmuClock_HFPER, true);
  // Enable CORE LE clock in order to access LE modules 
  CMU_ClockEnable(cmuClock_CORELE, true);
  // Select LFXO for LEUARTs (and wait for it to stabilize) 
  // set to internal RC 32kHz oscillator (normally its LFXO in docs, but no ext LF crystal)
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFRCO); 
  CMU_ClockEnable(cmuClock_LEUART0, true);
  // Set location of TX & RX pins and enable them
  LEUART0->ROUTE = LEUART_LOCATION |
                   LEUART_ROUTE_TXPEN |
                   LEUART_ROUTE_RXPEN;
  // Enable the pins 
  GPIO_PinModeSet(LEUART_TXPORT, LEUART_TXPIN, gpioModePushPull, 1);
  GPIO_PinModeSet(LEUART_RXPORT, LEUART_RXPIN, gpioModeInput, 0);
  // Configure the LEUART
  LEUART_Init(LEUART0, &leuart_init);
}



int main() {
  // Runs the Silicon Labs chip initialisation stuff, that also deals with
  // errata (implements workarounds, etc).
  CHIP_Init();

  // Disable the watchdog that the bootloader started.
  WDOG->CTRL = 0;

  // Switch on the clock for GPIO. Even though there's no immediately obvious
  // timing stuff going on beyond the SysTick below, it still needs to be
  // enabled for the GPIO to work.
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Sets up and enable the `SysTick_Handler' interrupt to fire once every 1ms.
  // ref: http://community.silabs.com/t5/Official-Blog-of-Silicon-Labs/Chapter-5-MCU-Clocking-Part-2-The-SysTick-Interrupt/ba-p/145297
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) {
    // Something went wrong.
    while (1);
  }
  // Set up two pins with the GPIO controller and configure them to be open
  // drain:
  //  - PA0 == green
  //  - PB7 == red
  GPIO_PinModeSet(gpioPortA, 0, gpioModeWiredAnd, 0);
  GPIO_PinModeSet(gpioPortB, 7, gpioModeWiredAnd, 0);

  setupLeuart();
  write_str("hello from efm32hg-blinky-usb-leuart\n");
  
  // Enable the capacitive touch sensor. Remember, this consumes TIMER0 and
  // TIMER1, so those are off-limits to us.
  CAPSENSE_Init();

  // Enable the USB controller. Remember, this consumes TIMER2 as per
  // -DUSB_TIMER=USB_TIMER2 in Makefile because TIMER0 and TIMER1 are already
  // taken by the capacitive touch sensors.
  USBD_Init(&initstruct);

  // Blink infinitely, in an aviation-like pattern.
  while (1) {
    write_str("blinking...");
    // Clear the PA0 bit, allowing the FET to sink to ground and thus lighting
    // up the green LED.
    GPIO_PinOutClear(gpioPortA, 0);
    SpinDelay(100);

    // Set the PA0 bit, preventing the FET from sinking to ground and thus
    // switching the green LED off.
    GPIO_PinOutSet(gpioPortA, 0);
    SpinDelay(100);

    // Repeat for the red LED on port PB7, but do so twice so that those that
    // self-assemble their boards can be sure they've got the green and red LEDs
    // around the right way.
    GPIO_PinOutClear(gpioPortB, 7);
    SpinDelay(100);
    GPIO_PinOutSet(gpioPortB, 7);
    SpinDelay(100);
    GPIO_PinOutClear(gpioPortB, 7);
    SpinDelay(100);
    GPIO_PinOutSet(gpioPortB, 7);
    SpinDelay(100);

    SpinDelay(500);

    // Capture/sample the state of the capacitive touch sensors.
    CAPSENSE_Sense();

    // Analyse the sample, and if the touch-pads on the green LED side is
    // touched, rapid blink the green LED ten times.
    if (CAPSENSE_getPressed(BUTTON0_CHANNEL) &&
        !CAPSENSE_getPressed(BUTTON1_CHANNEL)) {
      int i;
      for (i = 10; i > 0; i--) {
        GPIO_PinOutClear(gpioPortA, 0);
        SpinDelay(100);
        GPIO_PinOutSet(gpioPortA, 0);
        SpinDelay(100);
      }
      write_str("touch BUTTON0\n");
    // Analyse the same sample, and if the touch-pads on the red LED side is
    // touched, rapid blink the red LED ten times.
    } else if (CAPSENSE_getPressed(BUTTON1_CHANNEL) &&
               !CAPSENSE_getPressed(BUTTON0_CHANNEL)) {
      int i;
      for (i = 10; i > 0; i--) {
        GPIO_PinOutClear(gpioPortB, 7);
        SpinDelay(100);
        GPIO_PinOutSet(gpioPortB, 7);
        SpinDelay(100);
      }
      write_str("touch BUTTON1\n");
    }
    SpinDelay(500);
  }
}

