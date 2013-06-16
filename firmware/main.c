/* Name: main.c
 * Project: Micronucleus
 * Author: Jenna Fox
 * Creation Date: 2007-12-08
 * Tabsize: 4
 * Copyright: (c) 2012 Jenna Fox
 * Portions Copyright: (c) 2007 by OBJECTIVE DEVELOPMENT Software GmbH (USBaspLoader)
 * Portions Copyright: (c) 2012 Louis Beaudoin (USBaspLoader-tiny85)
 * License: GNU GPL v2 (see License.txt)
 */

#define MICRONUCLEUS_VERSION_MAJOR 1
#define MICRONUCLEUS_VERSION_MINOR 6
// how many milliseconds should host wait till it sends another erase or write?
// needs to be above 4.5 (and a whole integer) as avr freezes for 4.5m,
// plus the poll interval, as the write doesn't start until there's an HID poll
#define MICRONUCLEUS_WRITE_SLEEP (USB_CFG_INTR_POLL_INTERVAL + 5)


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

#define addr_t uint

/* allow compatibility with avrusbboot's bootloaderconfig.h: */
#ifdef BOOTLOADER_INIT
#   define bootLoaderInit()         BOOTLOADER_INIT
#   define bootLoaderExit()
#endif
#ifdef BOOTLOADER_CONDITION
#   define bootLoaderCondition()    BOOTLOADER_CONDITION
#endif

/* ------------------------------------------------------------------------ */

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

uchar eraseEvent, writeEvent, executeEvent;

// events system schedules functions to run in the main loop
#define EVENT_ERASE_APPLICATION eraseEvent
#define EVENT_WRITE_PAGE writeEvent
#define EVENT_EXECUTE executeEvent

// controls state of events
#define fireEvent(event) event = 1
#define isEvent(event)   (event & 1)
// no need to clear executeEvent, as it exits the loop
#define clearEvents()    {eraseEvent = writeEvent = 0;}

uint16_t idlePolls = 0; // how long have we been idle?

#if (USB_HID_SUPPORT == 1)
uchar usbHidPollFlag = 0;

/* USB report descriptor */
PROGMEM char usbHidReportDescriptor[] = {
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
#endif

static uint16_t vectorTemp[2]; // remember data to create tinyVector table before BOOTLOADER_ADDRESS
static addr_t currentAddress; // current progmem address, used for erasing and writing
static uchar currentCommand; // current command received in a USB report

/* ------------------------------------------------------------------------ */
static inline void eraseApplicationPage(void);
static void writeFlashPage(void);
static void writeWordToPageBuffer(addr_t address, uint16_t data);
static void fillFirstPageWithVectors(void);
static uchar usbFunctionSetup(uchar data[8]);
static uchar usbFunctionWrite(uchar *data, uchar length);
static inline void initForUsbConnectivity(void);
static inline void tiny85FlashInit(void);
static inline void tiny85FlashWrites(void);
//static inline void tiny85FinishWriting(void);
static inline void leaveBootloader(void);

// resets micro if anything but 0xFF's were found after page 0 in the application
static inline void eraseSafetyCheck(void) {
    addr_t tempaddr;

    for (tempaddr = SPM_PAGESIZE; tempaddr < BOOTLOADER_ADDRESS; tempaddr++) {
        if (pgm_read_byte(tempaddr) != 0xFF) {

            // flash is not stable, restart system
            wdt_enable(WDTO_15MS);
            while(1);
        }
    }
}

// erase the page at currentAddress, and write in jumps for usb interrupt and reset to bootloader if the first page was erased
//  - Because flash can be erased once and programmed several times, we can write the bootloader
//  - vectors in now, and write in the application stuff around them later.
//  - if vectors weren't written back in immediately, usb would fail.
static inline void eraseApplicationPage(void) {
    // special case when erasing the first page, check to make sure the rest of the application has been erased already
    // while the vectors don't matter for usb comms as interrupts are disabled during erase, it's important
    // to minimise the chance of leaving the device in a state where the bootloader wont run, if there's power failure
    // during upload
    cli();
    if (!currentAddress)
        eraseSafetyCheck();

    boot_page_erase(currentAddress);
    boot_spm_busy_wait();

    if (!currentAddress)
        fillFirstPageWithVectors();

    sei();
}

// simply write currently stored page in to already erased flash memory
static void writeFlashPage(void) {
    uint8_t previous_sreg = SREG; // backup current interrupt setting
    cli();
    boot_page_write(currentAddress);
    boot_spm_busy_wait(); // Wait until the memory is written.
    SREG = previous_sreg; // restore interrupts to previous state
}

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

// write a word in to the page buffer, doing interrupt table modifications where they're required
static void writeWordToPageBuffer(addr_t address, uint16_t data) {
    uint8_t previous_sreg;

    // first two interrupt vectors get replaced with a jump to the bootloader's vector table
    if (address == (RESET_VECTOR_OFFSET * 2)) {
        vectorTemp[0] = data;
        data = 0xC000 + (BOOTLOADER_ADDRESS/2) - 1;
    } else if (address == (USBPLUS_VECTOR_OFFSET * 2)) {
        vectorTemp[1] = data;
        data = 0xC000 + (BOOTLOADER_ADDRESS/2) - 1;
    }

    // at end of page just before bootloader, write in tinyVector table
    // see http://embedded-creations.com/projects/attiny85-usb-bootloader-overview/avr-jtag-programmer/
    // for info on how the tiny vector table works
    if (address == BOOTLOADER_ADDRESS - TINYVECTOR_RESET_OFFSET) {
        data = vectorTemp[0] + ((FLASHEND + 1) - BOOTLOADER_ADDRESS)/2 + 2 + RESET_VECTOR_OFFSET;
    } else if (address == BOOTLOADER_ADDRESS - TINYVECTOR_USBPLUS_OFFSET) {
        data = vectorTemp[1] + ((FLASHEND + 1) - BOOTLOADER_ADDRESS)/2 + 1 + USBPLUS_VECTOR_OFFSET;
    } else if (address == BOOTLOADER_ADDRESS - TINYVECTOR_OSCCAL_OFFSET) {
        data = OSCCAL;
    }


    // clear page buffer as a precaution before filling the buffer on the first page
    // in case the bootloader somehow ran after user program and there was something
    // in the page buffer already
    if (address == 0x0000) __boot_page_fill_clear();

    previous_sreg = SREG; // backup previous interrupt settings
    cli(); // ensure interrupts are disabled
    boot_page_fill(address, data);
    SREG = previous_sreg; // restore previous interrupt setting
}

static void fillFirstPageWithVectors(void) {
    uchar i;

    currentAddress = 0x0000;
    // fill page with nothing, let writeWordToPageBuffer fill in the vectors
    for(i=0; i < SPM_PAGESIZE; i+=2)
        writeWordToPageBuffer(i, 0xFFFF);

    writeFlashPage();
}

/* ------------------------------------------------------------------------ */

#define MICRONUCLEUS_COMMAND_GETINFO    0
#define MICRONUCLEUS_COMMAND_PAGELOAD   1
#define MICRONUCLEUS_COMMAND_ERASE      2
#define MICRONUCLEUS_COMMAND_STARTAPP   3
#define MICRONUCLEUS_COMMAND_PAGEWRITE  4

static uchar usbFunctionSetup(uchar data[8]) {
    usbRequest_t *rq = (void *)data;
    idlePolls = 0; // reset idle polls when we get usb traffic

    static uchar replyBuffer[4] = {
        (((uint)PROGMEM_SIZE) >> 8) & 0xff,
        ((uint)PROGMEM_SIZE) & 0xff,
        SPM_PAGESIZE,
        MICRONUCLEUS_WRITE_SLEEP
    };

    currentCommand = (uchar)rq->wValue.bytes[0];

    // assume everything coming in is a CLASS-type report (safe?)
    //if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {    /* HID class request */
        if (rq->bRequest == USBRQ_HID_GET_REPORT) {  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* since we have only one report type, we can ignore the report-ID */
            // get device info
            usbMsgPtr = replyBuffer;
            return sizeof(replyBuffer);
        } else if (rq->bRequest == USBRQ_HID_SET_REPORT) {
            if (currentCommand == MICRONUCLEUS_COMMAND_STARTAPP) {
#               if BOOTLOADER_CAN_EXIT
                    fireEvent(EVENT_EXECUTE);
                    return 0;
#               endif
            }

            /* some reports will be followed by data, hand off to usbFunctionWrite */
            return USB_NO_MSG;
        }
    //}

    return 0;
}


// read in the common report structure, and process it based on the currentCommand
static uchar usbFunctionWrite(uchar *data, uchar length) {
    // byte 0 is address0
    // byte 1 is address1
    // byte 2 is data0 for load
    // byte 3 is data1 for load

    currentAddress = *(unsigned short *)(data);
    unsigned short progdata = *(unsigned short *)(data + 2);

    if (currentCommand == MICRONUCLEUS_COMMAND_PAGELOAD) {
        // make sure we don't write over the bootloader!
        if (currentAddress >= BOOTLOADER_ADDRESS) {
            return 1;
        }
        writeWordToPageBuffer(currentAddress, progdata);
    } else if (currentCommand == MICRONUCLEUS_COMMAND_PAGEWRITE) {
        fireEvent(EVENT_WRITE_PAGE);
    } else if (currentCommand == MICRONUCLEUS_COMMAND_ERASE) {
        fireEvent(EVENT_ERASE_APPLICATION);
    }

    return 1;
}

/* ------------------------------------------------------------------------ */

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

static inline void tiny85FlashInit(void) {
    // check for erased first page (no bootloader interrupt vectors), add vectors if missing
    // this needs to happen for usb communication to work later - essential to first run after bootloader
    // being installed
    if (pgm_read_word(RESET_VECTOR_OFFSET * 2) != 0xC000 + (BOOTLOADER_ADDRESS/2) - 1 ||
            pgm_read_word(USBPLUS_VECTOR_OFFSET * 2) != 0xC000 + (BOOTLOADER_ADDRESS/2) - 1) {


        fillFirstPageWithVectors();
    }
}

static inline void tiny85FlashWrites(void) {
    // write page to flash, interrupts will be disabled for > 4.5ms including erase
    writeFlashPage();
}

// reset system to a normal state and launch user program
static inline void leaveBootloader(void) {
    _delay_ms(10); // removing delay causes USB errors

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

int main(void) {
#if USB_HID_SUPPORT
    DDRB|=(1<<0);
    DDRB|=(1<<1);
#endif

    /* initialize  */
    #ifdef RESTORE_OSCCAL
        uint8_t osccal_default = OSCCAL;
    #endif
    #if (!SET_CLOCK_PRESCALER) && LOW_POWER_MODE
        uint8_t prescaler_default = CLKPR;
    #endif

    wdt_disable();      /* main app may have enabled watchdog */
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
            _delay_us(100);
            idlePolls++;

            // only process commands that halt the CPU after a HID poll has been recently received
            // to minimize the chances of another poll coming while the CPU is halted
#if (USB_HID_SUPPORT == 1)
            if (usbHidPollFlag) {
PORTB|=(1<<1);
#endif

                // these next two freeze the chip for ~ 4.5ms, breaking usb protocol
                // and usually both of these will activate in the same loop, so host
                // needs to wait > 9ms before next usb request
                if (isEvent(EVENT_ERASE_APPLICATION)) eraseApplicationPage();
                if (isEvent(EVENT_WRITE_PAGE)) tiny85FlashWrites();

#               if BOOTLOADER_CAN_EXIT
                    if (isEvent(EVENT_EXECUTE)) { // when host requests device run uploaded program
                        break;
                    }
#               endif

                clearEvents();

#if (USB_HID_SUPPORT == 1)
                usbHidPollFlag = 0;
PORTB&=~(1<<1);
            }
#endif
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
