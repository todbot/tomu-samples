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
#include "em_system.h"
#include <em_leuart.h>

#include <stdio.h>


#include "callbacks.h"
#include "descriptors.h"

// LEUART Rx/Tx Port/Pin Location 
#define LEUART_LOCATION    0
#define LEUART_TXPORT      gpioPortD        // LEUART transmission port 
#define LEUART_TXPIN       4                // LEUART transmission pin  
#define LEUART_RXPORT      gpioPortD        // LEUART reception port    
#define LEUART_RXPIN       5                // LEUART reception pin     

#include "leuart.h"

//char sbuf[150] = "hi!";
char dbgstr[30];



int setupCmd(const USB_Setup_TypeDef *setup);

/* Define callbacks that are called by the USB stack on different events. */
static const USBD_Callbacks_TypeDef callbacks =
{
  .usbReset        = NULL,          // Called whenever USB reset signalling is detected  
  .usbStateChange  = stateChange,   // Called whenever the device change state.  
  .setupCmd        = setupCmd,      // Called on each setup request received from host. 
  .isSelfPowered   = NULL,          // Called when the device stack needs to query if the device is currently self- or bus-powered. 
  .sofInt          = NULL           // Called at each SOF interrupt. If NULL, device stack will not enable the SOF interrupt. 
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
static uint8_t   inbuf[REPORT2_COUNT]; // FIXME: REPORT_COUNT

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
  
  uint64_t uniqid = SYSTEM_GetUnique(); // is 64-bit but we'll only use 32-bits
  
  setupLeuart();
  
  write_str("startup...\n");
  
  GPIO_PinModeSet(gpioPortF, 4, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortF, 5, gpioModePushPull, 0);
  
  // Enable the capacitive touch sensor. Remember, this consumes TIMER0 and
  // TIMER1, so those are off-limits to us.
  //CAPSENSE_Init();

  char serbuf[17];
  sprintf(serbuf, "%8llx", uniqid);
  write_str("uniqid:");write_str(serbuf);

  sprintf(serbuf, "3%8lx", (uint32_t)uniqid );
  write_str("serbuf:");write_str(serbuf);
  iSerialNumber[2]  = serbuf[0]; // mk3
  iSerialNumber[4]  = serbuf[1];
  iSerialNumber[6]  = serbuf[2];
  iSerialNumber[8]  = serbuf[3];
  iSerialNumber[10] = serbuf[4];
  iSerialNumber[12] = serbuf[5];
  iSerialNumber[14] = serbuf[6];
  iSerialNumber[16] = serbuf[7];
  
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

    //write_str(sbuf);
    write_char('.');

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

/**
 * handleMessage(char* inbuf) -- main command router
 *
 * inbuf[] is 8 bytes long
 *  byte0 = report-id
 *  byte1 = command
 *  byte2..byte7 = args for command
 *
 * Available commands:
 *    - Fade to RGB color       format: { 1, 'c', r,g,b,     th,tl, n }
 *    - Set RGB color now       format: { 1, 'n', r,g,b,       0,0, n } (*)
 *    - Read current RGB color  format: { 1, 'r', n,0,0,       0,0, n } (2)
 *    - Serverdown tickle/off   format: { 1, 'D', on,th,tl,  st,sp,ep } (*)
 *    - PlayLoop                format: { 1, 'p', on,sp,ep,c,    0, 0 } (2)
 *    - Playstate readback      format: { 1, 'S', 0,0,0,       0,0, 0 } (2)
 *    - Set color pattern line  format: { 1, 'P', r,g,b,     th,tl, p }
 *    - Save color patterns     format: { 1, 'W', 0,0,0,       0,0, 0 } (2)
 *    - read color pattern line format: { 1, 'R', 0,0,0,       0,0, p }
 *    - Set ledn                format: { 1, 'l', n,0,0,       0,0, 0 } (2+)
 *    - Read EEPROM location    format: { 1, 'e', ad,0,0,      0,0, 0 } (1)
 *    - Write EEPROM location   format: { 1, 'E', ad,v,0,      0,0, 0 } (1)
 *    - Get version             format: { 1, 'v', 0,0,0,       0,0, 0 }
 *    - Test command            format: { 1, '!', 0,0,0,       0,0, 0 }
 *
 *  Fade to RGB color        format: { 1, 'c', r,g,b,      th,tl, ledn }
 *  Set RGB color now        format: { 1, 'n', r,g,b,        0,0, ledn }
 *  Play/Pause, with pos     format: { 1, 'p', {1/0},pos,0,  0,0,    0 }
 *  Play/Pause, with pos     format: { 1, 'p', {1/0},pos,endpos, 0,0,0 }
 *  Write color pattern line format: { 1, 'P', r,g,b,      th,tl,  pos }
 *  Read color pattern line  format: { 1, 'R', 0,0,0,        0,0, pos }
 *  Server mode tickle       format: { 1, 'D', {1/0},th,tl, {1,0},0, 0 }
 *  Get version              format: { 1, 'v', 0,0,0,        0,0, 0 }
 *
 **/
static void handleMessage(uint8_t reportId)
{
  sprintf(dbgstr, "%d:%x,%x,%x,%x,%x,%x,%x,%x\n", reportId,
          inbuf[0],inbuf[1],inbuf[2],inbuf[3],inbuf[4],inbuf[5],inbuf[6],inbuf[7] );
  write_str(dbgstr);

  // pre-load response with request, contains report id
  uint8_t count = (reportId==REPORT_ID) ? REPORT_COUNT : REPORT2_COUNT;
  memcpy( (void*)reportToSend, (void*)inbuf, count);
  
  uint8_t cmd = inbuf[1];
  
  // 1, 76, 0, 0, ...
  if( cmd == 'v' ) {
    GPIO_PinOutSet(gpioPortF, 5);
    reportToSend[3] = '2';
    reportToSend[4] = '1';
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
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/
static int ReportReceived(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining)
{
  (void) remaining;

  if ((status   == USB_STATUS_OK) &&
      (xferred  == REPORT_COUNT) ) {
      //      && (setReportFunc != NULL) ) {
      //setReportFunc( (uint8_t)tmpBuffer);
    handleMessage(REPORT_ID);
  }

  return USB_STATUS_OK;
}

/*****************************************************************************
 *
 *****************************************************************************/
static int Report2Received(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining)
{
  (void) remaining;
  //(void) xferred;  (void) status;
  
  if ((status   == USB_STATUS_OK) &&
      (xferred  == REPORT2_COUNT) ) {
    GPIO_PinOutSet(gpioPortF, 4);
    handleMessage(REPORT2_ID);
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
  
        // Implement the necessary HID class specific commands.           
        switch ( setup->bRequest ) {
        case USB_HID_SET_REPORT:           // 0x09, receive data from host
          
          /*
          if ( ( (setup->wValue >> 8)      == 3)              // FEATURE report 
          if ( ( (setup->wValue >> 8)      == 2)              // OUTPUT report 
               && ( (setup->wValue & 0xFF) == 1)              // Report ID  
               && (setup->wLength          == 1)              // Report length 
               && (setup->Direction        != USB_SETUP_DIR_OUT)    ) { // 
          */
          
          if( (setup->wValue & 0xFF) == REPORT_ID ) { 
            USBD_Read(0, (void*)&inbuf, REPORT_COUNT, ReportReceived);
            retVal = USB_STATUS_OK;
          }
          else if( (setup->wValue & 0xFF) == REPORT2_ID ) {
            USBD_Read(0, (void*)&inbuf, REPORT2_COUNT, Report2Received);
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


