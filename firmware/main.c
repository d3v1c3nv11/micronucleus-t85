/* Name: main.c
 * Project: Micronucleus
 * Author: Jenna Fox
 * Creation Date: 2007-12-08
 * Tabsize: 4
 * Copyright: (c) 2012 Jenna Fox
 * Portions Copyright: (c) 2007 by OBJECTIVE DEVELOPMENT Software GmbH (USBaspLoader)
 * Portions Copyright: (c) 2012-2013 Louis Beaudoin (USBaspLoader-tiny85 and new contributions)
 * License: GNU GPL v2 (see License.txt)
 */
 
#define MICRONUCLEUS_VERSION_MAJOR 2
#define MICRONUCLEUS_VERSION_MINOR 0
// how many milliseconds should host wait till it sends another erase or write?
// needs to be above 9 (and a whole integer) as avr freezes for up to 9ms during runSpmOperation()
// in HID mode need to wait the USB poll interval plus the erase/write time
#define MICRONUCLEUS_WRITE_SLEEP (USB_CFG_INTR_POLL_INTERVAL + 9)


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/boot.h>
//#include <avr/eeprom.h>
#include <util/delay.h>
//#include <string.h>

static void leaveBootloader() __attribute__((__noreturn__));

#include "bootloaderconfig.h"
#include "usbdrv/usbdrv.c"

/* ------------------------------------------------------------------------ */

#ifndef ulong
#   define ulong    unsigned long
#endif
#ifndef uint
#   define uint     unsigned int
#endif

#ifndef BOOTLOADER_CAN_EXIT
#   define  BOOTLOADER_CAN_EXIT     0
#endif

/* allow compatibility with avrusbboot's bootloaderconfig.h: */
#ifdef BOOTLOADER_INIT
#   define bootLoaderInit()         BOOTLOADER_INIT
#   define bootLoaderExit()
#endif
#ifdef BOOTLOADER_CONDITION
#   define bootLoaderCondition()    BOOTLOADER_CONDITION
#endif

/* device compatibility: */
#ifndef GICR    /* ATMega*8 don't have GICR, use MCUCR instead */
#   define GICR     MCUCR
#endif

/* ------------------------------------------------------------------------ */

//#define addr_t uint
// typedef union longConverter{
//     addr_t  l;
//     uint    w[sizeof(addr_t)/2];
//     uchar   b[sizeof(addr_t)];
// } longConverter_t;

//////// Stuff Bluebie Added
// postscript are the few bytes at the end of programmable memory which store tinyVectors
// and used to in USBaspLoader-tiny85 store the checksum iirc
#define POSTSCRIPT_SIZE 4
#define PROGMEM_SIZE (BOOTLOADER_ADDRESS - POSTSCRIPT_SIZE) /* max size of user program */

// verify the bootloader address aligns with page size
#if BOOTLOADER_ADDRESS % SPM_PAGESIZE != 0
#  error "BOOTLOADER_ADDRESS in makefile must be a multiple of chip's pagesize"
#endif

#ifdef AUTO_EXIT_MS
#  if AUTO_EXIT_MS < (MICRONUCLEUS_WRITE_SLEEP * (BOOTLOADER_ADDRESS / SPM_PAGESIZE))
#    warning "AUTO_EXIT_MS is shorter than the time it takes to perform erase function - might affect reliability?"
#    warning "Try increasing AUTO_EXIT_MS if you have stability problems"
#  endif
#endif

// events system schedules functions to run in the main loop
static uchar events = 0; // bitmap of events to run
#define EVENT_EXECUTE 1
#define EVENT_SPM_OPERATION 2
#define EVENT_SPM_OPERATION_NODELAY 4

// controls state of events
#define fireEvent(event) events |= (event)
#define isEvent(event)   (events & (event))
#define clearEvents()    events = 0
#define clearEvent(event)   events &= ~(event)

uint16_t idlePolls = 0; // how long have we been idle?

/* USB report descriptor */
PROGMEM const char usbHidReportDescriptor[] = {
  0x06, 0xa0, 0xff, // USAGE_PAGE (Vendor Defined Page 1)
  0x09, 0x01,       // USAGE (Vendor Usage 1)
  0xa1, 0x01,       // COLLECTION (Application)

  // Input Report
  0x09, 0x02,       // Usage ID - vendor defined
  0x15, 0x00,       // Logical Minimum (0)
  0x26, 0xFF, 0x00, // Logical Maximum (255)
  0x75, 0x08,       // Report Size (8 bits)
  0x95, 0x08,       // Report Count (8 fields)
  0x81, 0x02,       // Input (Data, Variable, Absolute)

  0xc0              // END_COLLECTION
};

/* ------------------------------------------------------------------------ */
static uchar usbFunctionSetup(uchar data[8]);
static uchar usbFunctionWrite(uchar *data, uchar length);
static inline void initForUsbConnectivity(void);
static inline void tiny85FlashInit(void);
static inline void leaveBootloader(void);

// clear memory which stores data to be written by next writeFlashPage call
#define __boot_page_fill_clear()   \
(__extension__({                                 \
    __asm__ __volatile__                         \
    (                                            \
        "sts %0, %1\n\t"                         \
        "spm\n\t"                                \
        :                                        \
        : "i" (_SFR_MEM_ADDR(__SPM_REG)),        \
          "r" ((uint8_t)(__BOOT_PAGE_FILL | (1 << CTPB)))     \
    );                                           \
}))

/* ------------------------------------------------------------------------ */
static uint16_t spmAddress;
static uint16_t spmData;
static uchar spmCommand;

static uchar usbFunctionSetup(uchar data[8]) {
    usbRequest_t *rq = (void *)data;
    idlePolls = 0; // reset idle polls when we get usb traffic
    
    static uchar replyBuffer[6] = {
        (((uint)PROGMEM_SIZE) >> 8) & 0xff,
        ((uint)PROGMEM_SIZE) & 0xff,
        SPM_PAGESIZE,
        MICRONUCLEUS_WRITE_SLEEP,
        POSTSCRIPT_SIZE,
        0
    };

    if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {    /* HID class request */
        if (rq->bRequest == USBRQ_HID_GET_REPORT) {  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* since we have only one report type, we can ignore the report-ID */
            // get device info
            usbMsgPtr = (usbMsgPtr_t)replyBuffer;
        replyBuffer[5] = OSCCAL;
        return sizeof(replyBuffer);
        } else if(rq->bRequest == USBRQ_HID_SET_REPORT) {
            /* since we have only one report type, we can ignore the report-ID */
            return USB_NO_MSG;
        }
    }        

// TODO: add support for exit bootloader message
//#       if BOOTLOADER_CAN_EXIT
//            fireEvent(EVENT_EXECUTE);
//#endif
    
    return 0;
}

// read in a page over usb, and write it in to the flash write buffer
static uchar usbFunctionWrite(uchar *data, uchar length) {
    PORTB|=(1<<0);
        spmCommand = data[0] & ~(SPM_COMMAND_MASK);
        spmAddress = data[2] + (data[1] * 256);
        spmData = data[4] + (data[3] * 256);
        if (data[0] & SPM_COMMAND_MASK)
            fireEvent(EVENT_SPM_OPERATION);       
        else
            fireEvent(EVENT_SPM_OPERATION_NODELAY);
    PORTB&=~(1<<0);

    return 1;
}

/* ------------------------------------------------------------------------ */

// this tells the linker to call this function right after the stack is initialized, before main()
void PushMagicWord (void) __attribute__ ((naked)) __attribute__ ((section (".init3")));

// put the word "B007" at the bottom of the stack (RAMEND - RAMEND-1)
void PushMagicWord (void) {
    asm volatile("ldi r16, 0xB0"::);
    asm volatile("push r16"::);
    asm volatile("ldi r16, 0x07"::);
    asm volatile("push r16"::);
}

/* ------------------------------------------------------------------------ */

static inline void initForUsbConnectivity(void) {
    usbInit();
    /* enforce USB re-enumerate: */
    usbDeviceDisconnect();  /* do this while interrupts are disabled */
    _delay_ms(500);
    usbDeviceConnect();
    sei();
}

// this function should be called with interrupts disabled as the boot_page instructions can have
// no break between loading SPMCSR and executing SPM instruction
static inline void tiny85FlashInit(void) {
    // check for erased first page (no bootloader interrupt vectors), add vectors if missing
    // this needs to happen for usb communication to work later - essential to first run after bootloader
    // being installed
    if(pgm_read_word(RESET_VECTOR_OFFSET * 2) != 0xC000 + (BOOTLOADER_ADDRESS/2) - 1 ||
            pgm_read_word(USBPLUS_VECTOR_OFFSET * 2) != 0xC000 + (BOOTLOADER_ADDRESS/2) - 1) {
        boot_page_fill(RESET_VECTOR_OFFSET * 2, 0xC000 + (BOOTLOADER_ADDRESS/2) - 1);
        boot_page_fill(USBPLUS_VECTOR_OFFSET * 2, 0xC000 + (BOOTLOADER_ADDRESS/2) - 1);
        boot_page_write(0);
        boot_spm_busy_wait(); // Wait until the memory is written.
    }
}

// reset system to a normal state and launch user program
static inline void leaveBootloader(void) {
    _delay_ms(10); // removing delay causes USB errors
    
    //DBG1(0x01, 0, 0);
    bootLoaderExit();
    cli();
    USB_INTR_ENABLE = 0;
    USB_INTR_CFG = 0;       /* also reset config bits */

    // clear magic word from bottom of stack before jumping to the app
    *(uint8_t*)(RAMEND) = 0x00;
    *(uint8_t*)(RAMEND-1) = 0x00;
    
    // adjust clock to previous calibration value, so user program always starts with same calibration
    // as when it was uploaded originally
    // TODO: Test this and find out, do we need the +1 offset?
    unsigned char stored_osc_calibration = pgm_read_byte(BOOTLOADER_ADDRESS - TINYVECTOR_OSCCAL_OFFSET);
    if (stored_osc_calibration != 0xFF && stored_osc_calibration != 0x00) {
        //OSCCAL = stored_osc_calibration; // this should really be a gradual change, but maybe it's alright anyway?
        // do the gradual change - failed to score extra free bytes anyway in 1.06
        while (OSCCAL > stored_osc_calibration) OSCCAL--;
        while (OSCCAL < stored_osc_calibration) OSCCAL++;
    }

    // jump to application reset vector at end of flash
    asm volatile ("rjmp __vectors - 4");
}

#define run_spm_operation_load(command, address, data)   \
(__extension__({                                 \
    __asm__ __volatile__                         \
    (                                            \
        "movw  r0, %3\n\t"                       \
        "sts %0, %1\n\t"                         \
        "spm\n\t"                                \
        "clr  r1\n\t"                            \
        :                                        \
        : "i" (_SFR_MEM_ADDR(__SPM_REG)),        \
          "r" ((uint8_t)(command)),              \
          "z" ((uint16_t)(address)),             \
          "r" ((uint16_t)(data))                 \
        : "r0"                                   \
    );                                           \
}))

static void runSpmOperation(void) {
    // make sure an interrupt can't separate loading the SPMCSR register and executing the SPM instruction
    cli();
    run_spm_operation_load(spmCommand, spmAddress, spmData);
    boot_spm_busy_wait(); // Wait until the memory is written.
    
    // load the interrupt vectors in case they were just erased
    tiny85FlashInit();
    sei();
}


int main(void) {
DDRB|=(1<<0);
DDRB|=(1<<1);

    /* initialize  */
    #ifdef RESTORE_OSCCAL
        uint8_t osccal_default = OSCCAL;
    #endif
    #if (!SET_CLOCK_PRESCALER) && LOW_POWER_MODE
        uint8_t prescaler_default = CLKPR;
    #endif
    
    wdt_disable();      /* main app may have enabled watchdog */
    

    // clean up the boot page registers, in case there was a partial page already loaded
    // is there a better place to put this? tiny85FlashInit() seemed good, but is now called
    // when there is valid data in the buffer
    __boot_page_fill_clear();

    tiny85FlashInit();
    bootLoaderInit();
    
    if (bootLoaderStartCondition()) {
        #if LOW_POWER_MODE
            // turn off clock prescalling - chip must run at full speed for usb
            // if you might run chip at lower voltages, detect that in bootLoaderStartCondition
            CLKPR = 1 << CLKPCE;
            CLKPR = 0;
        #endif
        
        initForUsbConnectivity();
        do {
            usbPoll();
            
            // delay to allow any current USB traffic to complete before disabling interrupts, and to space out calls to usbPoll
            _delay_us(IDLE_POLL_DELAY_US);
            idlePolls++;

            // runSpmOperation will freeze the chip for up to 9ms, breaking usb protocol, so host
            // needs to wait > 9ms before next usb request

            if(isEvent(EVENT_SPM_OPERATION_NODELAY)) {
                runSpmOperation();
                clearEvent(EVENT_SPM_OPERATION_NODELAY);
            }

            if(usbInterruptIsReady()){
PORTB|=(1<<1);
                if (isEvent(EVENT_SPM_OPERATION)) {
                    runSpmOperation();
                    clearEvent(EVENT_SPM_OPERATION);
                }
                usbSetInterrupt((void*)0x00, 0);
PORTB&=~(1<<1);
            }
#           if BOOTLOADER_CAN_EXIT            
                if (isEvent(EVENT_EXECUTE)) { // when host requests device run uploaded program
                    break;
                }
#           endif
        } while(bootLoaderCondition());  /* main event loop runs so long as bootLoaderCondition remains truthy */
    }
    
    // set clock prescaler to desired clock speed (changing from clkdiv8, or no division, depending on fuses)
    #if LOW_POWER_MODE
        #ifdef SET_CLOCK_PRESCALER
            CLKPR = 1 << CLKPCE;
            CLKPR = SET_CLOCK_PRESCALER;
        #else
            CLKPR = 1 << CLKPCE;
            CLKPR = prescaler_default;
        #endif
    #endif
    
    // slowly bring down OSCCAL to it's original value before launching in to user program
    #ifdef RESTORE_OSCCAL
        while (OSCCAL > osccal_default) { OSCCAL -= 1; }
    #endif
    leaveBootloader();
}

/* ------------------------------------------------------------------------ */
