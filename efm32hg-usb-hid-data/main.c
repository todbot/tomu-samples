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

#include <em_leuart.h>

#include <stdio.h>


// LEUART Rx/Tx Port/Pin Location 
#define LEUART_LOCATION    0
#define LEUART_TXPORT      gpioPortD        // LEUART transmission port 
#define LEUART_TXPIN       4                // LEUART transmission pin  
#define LEUART_RXPORT      gpioPortD        // LEUART reception port    
#define LEUART_RXPIN       5                // LEUART reception pin     


#include "callbacks.h"
#include "descriptors.h"


char sbuf[150] = "hi!";
char dbgstr[30];

int write_char(int c)
{
    while (!(LEUART0->STATUS & LEUART_STATUS_TXBL))
        ;
    LEUART0->TXDATA = (uint32_t)c & 0xffUL;
    return 0;
}
int write_str(const char *s)
{
    while (*s)
        write_char(*s++);
    write_char('\r');
    write_char('\n');
    return 0;
}

/***************************************************************************//**
 * @brief  Setting up LEUART
 ******************************************************************************/
void setupLeuart(void)
{
  /* Enable peripheral clocks */
  CMU_ClockEnable(cmuClock_HFPER, true);
  /* Configure GPIO pins */
  CMU_ClockEnable(cmuClock_GPIO, true);
  /* To avoid false start, configure output as high */
  GPIO_PinModeSet(LEUART_TXPORT, LEUART_TXPIN, gpioModePushPull, 1);
  GPIO_PinModeSet(LEUART_RXPORT, LEUART_RXPIN, gpioModeInput, 0);

  LEUART_Init_TypeDef init = LEUART_INIT_DEFAULT;

  /* Enable CORE LE clock in order to access LE modules */
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Select LFXO for LEUARTs (and wait for it to stabilize) */
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_LEUART0, true);

  /* Do not prescale clock */
  CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1);

  /* Configure LEUART */
  init.enable = leuartDisable;
  //init.baudrate = BAUDRATE;
      
  LEUART_Init(LEUART0, &init);

  /* Enable pins at default location */
  LEUART0->ROUTE = LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN | LEUART_LOCATION;

  /* Set RXDMAWU to wake up the DMA controller in EM2 */
//LEUART_RxDmaInEM2Enable(LEUART0, true);

  /* Clear previous RX interrupts */
  LEUART_IntClear(LEUART0, LEUART_IF_RXDATAV);
  NVIC_ClearPendingIRQ(LEUART0_IRQn);

  /* Finally enable it */
  LEUART_Enable(LEUART0, leuartEnable);
}


int setupCmd(const USB_Setup_TypeDef *setup);

/* Define callbacks that are called by the USB stack on different events. */
static const USBD_Callbacks_TypeDef callbacks =
{
  .usbReset        = NULL,              /* Called whenever USB reset signalling is detected on the USB port. */
  .usbStateChange  = stateChange,       /* Called whenever the device change state.  */
  .setupCmd        = setupCmd,          /* Called on each setup request received from host. */
  .isSelfPowered   = NULL,              /* Called whenever the device stack needs to query if the device is currently self- or bus-powered. */
  .sofInt          = NULL               /* Called at each SOF (Start of Frame) interrupt. If NULL, the device stack will not enable the SOF interrupt. */
};

/* Fill the init struct. This struct is passed to USBD_Init() in order 
 * to initialize the USB Stack */
static const USBD_Init_TypeDef initstruct =
{
  .deviceDescriptor    = &deviceDesc,
  .configDescriptor    = configDesc,
  .stringDescriptors   = strings,
  .numberOfStrings     = sizeof(strings)/sizeof(void*),
  .callbacks           = &callbacks,
  .bufferingMultiplier = bufferingMultiplier,
  .reserved            = 0
};


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



// HID keyboard input report definition. 
SL_PACK_START(1)
typedef struct {
  uint8_t args[8];
} SL_ATTRIBUTE_PACKED HIDReport_t;
SL_PACK_END()

static void  *hidDescriptor = NULL;
static uint8_t   inbuff[16]; // FIXME: REPORT_COUNT

// The last keyboard report reported to the driver. 
SL_ALIGN(4)
static uint8_t reportToSend[REPORT_COUNT] SL_ATTRIBUTE_ALIGN(4);
//static HIDReport_t reportToSend SL_ATTRIBUTE_ALIGN(4);


int main()
{
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
  
  setupLeuart();
  write_str("startup...\n");
  
  // Set up two pins with the GPIO controller and configure them to be open
  // drain:
  //  - PA0 == green
  //  - PB7 == red
  // GPIO_PinModeSet(gpioPortA, 0, gpioModeWiredAnd, 0);
  //GPIO_PinModeSet(gpioPortB, 7, gpioModeWiredAnd, 0);
  /*
  */
  GPIO_PinModeSet(gpioPortF, 4, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortF, 5, gpioModePushPull, 0);
  
  // Enable the capacitive touch sensor. Remember, this consumes TIMER0 and
  // TIMER1, so those are off-limits to us.
  //CAPSENSE_Init();

  hidDescriptor = (void*) USBDESC_HidDescriptor; // FIXME
  
  // Enable the USB controller. Remember, this consumes TIMER2 as per
  // -DUSB_TIMER=USB_TIMER2 in Makefile because TIMER0 and TIMER1 are already
  // taken by the capacitive touch sensors.
  USBD_Init(&initstruct);

  /*
   * When using a debugger it is practical to uncomment the following three
   * lines to force host to re-enumerate the device.
   */
   USBD_Disconnect();      
   USBTIMER_DelayMs(1000); 
   USBD_Connect();         


  // Blink infinitely, in an aviation-like pattern.
  while (1) {

    write_str(sbuf);

    SpinDelay(500);
   
    /*
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
    }
    SpinDelay(500);
    */
  }
}


/**************************************************************************//**
 * @brief
 *   Callback function called when the data stage of a USB_HID_SET_REPORT
 *   setup command has completed.
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
v *
 * @return USB_STATUS_OK.
 *****************************************************************************/
static int OutputReportReceived(USB_Status_TypeDef status,
                                uint32_t xferred,
                                uint32_t remaining)
{
  (void) remaining;

  if ((status           == USB_STATUS_OK)
      && (xferred       == REPORT_COUNT) ) {
      //      && (setReportFunc != NULL) ) {
      //setReportFunc( (uint8_t)tmpBuffer);
    sprintf(dbgstr, "%x,%x,%x,%x,%x,%x,%x,%x\n", 
            inbuff[0],inbuff[1],inbuff[2],inbuff[3],inbuff[4],inbuff[5],inbuff[6],inbuff[7] );
    write_str(dbgstr);
    uint8_t cmd = inbuff[1];

    // 1, 76, 0, 0, ...
    if( cmd == 'v' ) {
      GPIO_PinOutSet(gpioPortF, 5);
      memcpy( (void*)reportToSend, (void*)inbuff, REPORT_COUNT);
      reportToSend[3] = '4';
      reportToSend[4] = '5';
    }
  }

  return USB_STATUS_OK;
}

/**************************************************************************//**
 * @brief
 *   Handle USB setup commands. Implements HID class specific commands.
 *   This function must be called each time the device receive a setup command.
 *
 *
 * @param[in] setup Pointer to the setup packet received.
 *
 * @return USB_STATUS_OK if command accepted,
 *         USB_STATUS_REQ_UNHANDLED when command is unknown. In the latter case
 *         the USB device stack will handle the request. 
 ****************************************************************************/
int setupCmd(const USB_Setup_TypeDef *setup)
{
  STATIC_UBUF(hidDesc, USB_HID_DESCSIZE);

  int retVal = USB_STATUS_REQ_UNHANDLED;

  //setup->bmRequestType == 0x81) {
  if (  (setup->Type         == USB_SETUP_TYPE_STANDARD)
        && (setup->Direction == USB_SETUP_DIR_IN)
        && (setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE)    ) {
      
    /* A HID device must extend the standard GET_DESCRIPTOR command   */
    /* with support for HID descriptors.                              */
    switch (setup->bRequest) {
    case GET_DESCRIPTOR:

      if ( (setup->wValue >> 8) == USB_HID_REPORT_DESCRIPTOR ) {
        USBD_Write(0, (void*)MyHIDReportDescriptor,
                   SL_MIN(sizeof(MyHIDReportDescriptor), setup->wLength),
                   NULL);
        retVal = USB_STATUS_OK;
      }
      else if ( (setup->wValue >> 8) == USB_HID_DESCRIPTOR ) {
        /* The HID descriptor might be misaligned ! */
        memcpy(hidDesc, hidDescriptor, USB_HID_DESCSIZE);
        USBD_Write(0, hidDesc, SL_MIN(USB_HID_DESCSIZE, setup->wLength),
                   NULL);
        retVal = USB_STATUS_OK;
      }
      break;
    }
  }
  else {

      if ( (setup->Type         == USB_SETUP_TYPE_CLASS)
           && (setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE) ) { 
        // && (setup->wIndex    == HIDKBD_INTERFACE_NO)    ) {

        // sprintf(sbuf, "%s\n%x/%x/%x/%x/%x",
        // sbuf,setup->Type, setup->Direction, setup->Recipient, setup->bRequest, setup->wValue);
        // sprintf(sbuf, "%s\n%x/%x/%x/%x",
        //        sbuf,setup->wValue>>8, setup->wValue &0xff, setup->wLength, setup->Direction);
  
        // Implement the necessary HID class specific commands.           
        switch ( setup->bRequest ) {
        case USB_HID_SET_REPORT:           // 0x09, receive data from host
          
          GPIO_PinOutSet(gpioPortF, 4);
          /*
          if ( ( (setup->wValue >> 8)      == 3)              // FEATURE report 
          if ( ( (setup->wValue >> 8)      == 2)              // OUTPUT report 
               && ( (setup->wValue & 0xFF) == 1)              // Report ID  
               && (setup->wLength          == 1)              // Report length 
               && (setup->Direction        != USB_SETUP_DIR_OUT)    ) { // 
          */
          
          if( ((setup->wValue & 0xFF) == REPORT_ID ) ) {      // Report ID
            USBD_Read(0, (void*)&inbuff, REPORT_COUNT, OutputReportReceived);
            retVal = USB_STATUS_OK;
          }
          break;

        case USB_HID_GET_REPORT:           // 0x01, send data to host

          /*
          if ( ( (setup->wValue >> 8)       == 1)             // INPUT report  
               && ( (setup->wValue & 0xFF)  == 1)             // Report ID     
               //               && (setup->wLength           == 8)             // Report length 
               //               && (setup->Direction         == USB_SETUP_DIR_IN)    ) {
               ) {
          */
          if( ((setup->wValue & 0xFF) == REPORT_ID ) ) { 
            USBD_Write(0, &reportToSend, REPORT_COUNT, NULL);
            retVal = USB_STATUS_OK;
          }
          break;
          /*
      case USB_HID_SET_IDLE:
        // ********************
          if ( ( (setup->wValue & 0xFF)    == 0)              // Report ID     
             && (setup->wLength          == 0)
             && (setup->Direction        != USB_SETUP_DIR_IN)    ) {
          idleRate = setup->wValue >> 8;
          if ( (idleRate != 0) && (idleRate < (HIDKBD_POLL_RATE / 4) ) ) {
            idleRate = HIDKBD_POLL_RATE / 4;
          }
          USBTIMER_Stop(HIDKBD_IDLE_TIMER);
          if ( idleRate != 0 ) {
            IdleTimeout();
          }
          retVal = USB_STATUS_OK;
        }
        break;

      case USB_HID_GET_IDLE:
        // ******************
          if ( (setup->wValue       == 0)                     // Report ID     
             && (setup->wLength   == 1)
             && (setup->Direction == USB_SETUP_DIR_IN)    ) {
          *(uint8_t*)&tmpBuffer = idleRate;
          USBD_Write(0, (void*)&tmpBuffer, 1, NULL);
          retVal = USB_STATUS_OK;
        }
        break;
          */
        } // swtich bRequest
        
      } // if
  } // else


   
  return retVal;
}


