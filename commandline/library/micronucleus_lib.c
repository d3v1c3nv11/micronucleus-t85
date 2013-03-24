/*
  Created: September 2012
  by ihsan Kehribar <ihsan@kehribar.me>
  
  Permission is hereby granted, free of charge, to any person obtaining a copy of
  this software and associated documentation files (the "Software"), to deal in
  the Software without restriction, including without limitation the rights to
  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
  of the Software, and to permit persons to whom the Software is furnished to do
  so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.  
*/

/***************************************************************/
/* See the micronucleus_lib.h for the function descriptions/comments */
/***************************************************************/
#include "micronucleus_lib.h"
#include "littleWire_util.h"

micronucleus* micronucleus_connect() {
  micronucleus *nucleus = NULL;
  struct usb_bus *busses;
  
  // intialise usb and find micronucleus device
  usb_init();
  usb_find_busses();
  usb_find_devices();
  
  busses = usb_get_busses();
  struct usb_bus *bus;
  for (bus = busses; bus; bus = bus->next) {
    struct usb_device *dev;
    
    for (dev = bus->devices; dev; dev = dev->next) {
      /* Check if this device is a micronucleus */
      if (dev->descriptor.idVendor == MICRONUCLEUS_VENDOR_ID && dev->descriptor.idProduct == MICRONUCLEUS_PRODUCT_ID)  {
        nucleus = malloc(sizeof(micronucleus));
        nucleus->version.major = (dev->descriptor.bcdDevice >> 8) & 0xFF;
        nucleus->version.minor = dev->descriptor.bcdDevice & 0xFF;
        
        if (nucleus->version.major > MICRONUCLEUS_MAX_MAJOR_VERSION) {
	        fprintf(stderr, "Warning: device with unknown new version of Micronucleus detected.\n");
	        fprintf(stderr, "This tool doesn't know how to upload to this new device. Updates may be available.\n");
	        fprintf(stderr, "Device reports version as: %d.%d\n", nucleus->version.major, nucleus->version.minor);
	        return NULL;
        }
        
        nucleus->device = usb_open(dev);
        
        // get nucleus info
        unsigned char buffer[6];
        #if 0
        int res = usb_control_msg(nucleus->device, 
                   USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                   0, 
                   0, 0,
                   buffer, 6,
                   MICRONUCLEUS_USB_TIMEOUT);
        #else
        int res = usb_control_msg(nucleus->device, 
                   USB_ENDPOINT_IN | USB_TYPE_CLASS | USB_RECIP_DEVICE,
                   USBRQ_HID_GET_REPORT, 
                   USB_HID_REPORT_TYPE_FEATURE << 8 | (0 & 0xff), 0,
                   buffer, 6,
                   MICRONUCLEUS_USB_TIMEOUT);
        #endif

        assert(res >= 4);
        
        nucleus->flash_size = (buffer[0]<<8) + buffer[1];
        nucleus->page_size = buffer[2];
        nucleus->pages = (nucleus->flash_size / nucleus->page_size);
        if (nucleus->pages * nucleus->page_size < nucleus->flash_size) nucleus->pages += 1;
        nucleus->write_sleep = buffer[3];
        if (nucleus->version.major < 2)
          nucleus->erase_sleep = nucleus->write_sleep * nucleus->pages;
        else
          nucleus->erase_sleep = nucleus->write_sleep;
        if(res >= 6) {
          nucleus->bootloader_start_addr = nucleus->flash_size + buffer[4];
          nucleus->osccal_value = buffer[5];
        }
        else {
          nucleus->bootloader_start_addr = 0;
          nucleus->osccal_value = 0;
        }
      }
    }
  }

  return nucleus;
}

int micronucleus_addVectorsToFlash(micronucleus* deviceHandle, unsigned char* buffer, int *endAddr) {
  // nothing to do for versions less than 2
  if(deviceHandle->version.major < 2)
    return 0;

  unsigned int tempword;
  unsigned int originalResetVector;
  unsigned int originalIntVector;

  originalResetVector = buffer[MICRONUCLEUS_VECTORS_RESET_VECTOR_OFFSET * 2] + 
    (buffer[MICRONUCLEUS_VECTORS_RESET_VECTOR_OFFSET * 2 + 1] * 256);

  originalIntVector = buffer[MICRONUCLEUS_VECTORS_USBPLUS_VECTOR_OFFSET * 2] + 
    (buffer[MICRONUCLEUS_VECTORS_USBPLUS_VECTOR_OFFSET * 2 + 1] * 256);

  tempword = 0xC000 + (deviceHandle->bootloader_start_addr/2) - 1;

  // store instructions redirecting the RESET and USBPLUS vectors to the bootloader section
  buffer[MICRONUCLEUS_VECTORS_RESET_VECTOR_OFFSET * 2] = (unsigned char)(tempword);
  buffer[MICRONUCLEUS_VECTORS_RESET_VECTOR_OFFSET * 2 + 1] = (unsigned char)(tempword / 256);
  buffer[MICRONUCLEUS_VECTORS_USBPLUS_VECTOR_OFFSET * 2] = (unsigned char)(tempword);
  buffer[MICRONUCLEUS_VECTORS_USBPLUS_VECTOR_OFFSET * 2 + 1] = (unsigned char)(tempword / 256);

  originalResetVector += ((MICRONUCLEUS_VECTORS_FLASHEND + 1) - deviceHandle->bootloader_start_addr)/2 + 2 + MICRONUCLEUS_VECTORS_RESET_VECTOR_OFFSET;
  originalIntVector +=  ((MICRONUCLEUS_VECTORS_FLASHEND + 1) - deviceHandle->bootloader_start_addr)/2 + 1 + MICRONUCLEUS_VECTORS_USBPLUS_VECTOR_OFFSET;

  // at end of page just before bootloader, write in tinyVector table
  // see http://embedded-creations.com/projects/attiny85-usb-bootloader-overview/avr-jtag-programmer/
  // for info on how the tiny vector table works
  buffer[deviceHandle->bootloader_start_addr - MICRONUCLEUS_VECTORS_TINYVECTOR_RESET_OFFSET] = (unsigned char)(originalResetVector);
  buffer[deviceHandle->bootloader_start_addr - MICRONUCLEUS_VECTORS_TINYVECTOR_RESET_OFFSET + 1] = (unsigned char)(originalResetVector / 256);
  buffer[deviceHandle->bootloader_start_addr - MICRONUCLEUS_VECTORS_TINYVECTOR_USBPLUS_OFFSET] = (unsigned char)(originalIntVector);
  buffer[deviceHandle->bootloader_start_addr - MICRONUCLEUS_VECTORS_TINYVECTOR_USBPLUS_OFFSET + 1] = (unsigned char)(originalIntVector / 256);
  buffer[deviceHandle->bootloader_start_addr - MICRONUCLEUS_VECTORS_TINYVECTOR_OSCCAL_OFFSET] = (unsigned char)(deviceHandle->osccal_value);
  buffer[deviceHandle->bootloader_start_addr - MICRONUCLEUS_VECTORS_TINYVECTOR_OSCCAL_OFFSET + 1] = (unsigned char)(deviceHandle->osccal_value / 256);

  // now the entire application section (including tinyvector table) needs to be written to flash, not just what the user's program contains
  *endAddr = deviceHandle->bootloader_start_addr;
}

int micronucleus_eraseFlash(micronucleus* deviceHandle, micronucleus_callback progress) {
  int res;
  if(deviceHandle->version.major < 2) {
    res = usb_control_msg(deviceHandle->device,
           USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
           2,
           0, 0,
           NULL, 0,
           MICRONUCLEUS_USB_TIMEOUT);
    
    // give microcontroller enough time to erase all writable pages and come back online
    float i = 0;
    while (i < 1.0) {
      // update progress callback if one was supplied
      if (progress) progress(i);
      
      delay(((float) deviceHandle->erase_sleep) / 100.0f);
      i += 0.01;
    }
    
    /* Under Linux, the erase process is often aborted with errors such as:
     usbfs: USBDEVFS_CONTROL failed cmd micronucleus rqt 192 rq 2 len 0 ret -84
     This seems to be because the erase is taking long enough that the device
     is disconnecting and reconnecting.  Under Windows, micronucleus can see this
     and automatically reconnects prior to uploading the program.  To get the
     the same functionality, we must flag this state (the "-84" error result) by
     converting the return to -2 for the upper layer.
     
     On Mac OS a common error is -34 = epipe, but adding it to this list causes:
     Assertion failed: (res >= 4), function micronucleus_connect, file library/micronucleus_lib.c, line 63.
    */
    if (res == -5 || res == -34 || res == -84) {
      if (res = -34) {
        usb_close(deviceHandle->device);
        deviceHandle->device = NULL;
      }
    
      return 1; // recoverable errors
    } else {
      return res;
    }
  } else{
    unsigned int currentAddress = deviceHandle->bootloader_start_addr;

    while(currentAddress) {
      currentAddress -= deviceHandle->page_size;

      // ask microcontroller to erase a page - we start from the end of the application section
      // and work toward the beginning for safety reasons: the bootloader can recover from an interruption
      // that erases the first page of flash before loading the vectors only if the rest of application flash
      // is erased (allowing the program counter to reach the bootloader eventually after running through 
      // pages of 0xFF's)
#if 0
      res = usb_control_msg(deviceHandle->device,
             USB_ENDPOINT_OUT| USB_TYPE_VENDOR | USB_RECIP_DEVICE,
             MICRONUCLEUS_SPM_COMMAND_MASK | MICRONUCLEUS_SPM_COMMAND_ERASE,
             0, currentAddress,
             0, 0,
             MICRONUCLEUS_USB_TIMEOUT);

      delay(deviceHandle->erase_sleep);

      if(res < 0)
        return -1;
#else
      unsigned char eraseCommand[5] = {MICRONUCLEUS_SPM_COMMAND_MASK | MICRONUCLEUS_SPM_COMMAND_ERASE,
                                      (unsigned char)(currentAddress/256), 
                                      (unsigned char)currentAddress, 
                                      0, 0};
      res = usb_control_msg(deviceHandle->device,
             USB_ENDPOINT_OUT| USB_TYPE_CLASS | USB_RECIP_DEVICE,
             USBRQ_HID_SET_REPORT,
             USB_HID_REPORT_TYPE_FEATURE << 8 | (0 & 0xff), 0,
             eraseCommand, 5,
             MICRONUCLEUS_USB_TIMEOUT);

      delay(deviceHandle->erase_sleep);

      if(res < 5)
        return -1;
#endif
    }

    return 0;
  }
}

int micronucleus_writeFlash(micronucleus* deviceHandle, unsigned int program_size, unsigned char* program, micronucleus_callback prog) {
  unsigned char page_length = deviceHandle->page_size;
  unsigned char page_buffer[page_length];
  unsigned int  address; // overall flash memory address
  unsigned int  page_address; // address within this page when copying buffer
  unsigned int  res;

  for (address = 0; address < deviceHandle->flash_size; address += deviceHandle->page_size) {
    // work around a bug in older bootloader versions
    if (deviceHandle->version.major == 1 && deviceHandle->version.minor <= 2
        && address / deviceHandle->page_size == deviceHandle->pages - 1) {
      page_length = deviceHandle->flash_size % deviceHandle->page_size;
    }
    
    // copy in bytes from user program
    for (page_address = 0; page_address < page_length; page_address += 1) {
      if (address + page_address > program_size) {
        page_buffer[page_address] = 0xFF; // pad out remainder with unprogrammed bytes
      } else {
        page_buffer[page_address] = program[address + page_address]; // load from user program
      }
    }
    
    if(deviceHandle->version.major < 2) {
      // ask microcontroller to write this page's data
      res = usb_control_msg(deviceHandle->device,
             USB_ENDPOINT_OUT| USB_TYPE_VENDOR | USB_RECIP_DEVICE,
             1,
             page_length, address,
             page_buffer, page_length,
             MICRONUCLEUS_USB_TIMEOUT);
    }
    else {
#if 0
      // ask microcontroller to load this page's data into the write buffer
      res = usb_control_msg(deviceHandle->device,
             USB_ENDPOINT_OUT| USB_TYPE_VENDOR | USB_RECIP_DEVICE,
             1,
             page_length, 0,
             page_buffer, page_length,
             MICRONUCLEUS_USB_TIMEOUT);

      if (res != page_length) return -1;
#else
      int i;
      for(i=0; i<page_length; i+=2) {
        unsigned char loadCommand[5] = {MICRONUCLEUS_SPM_COMMAND_LOAD,
                                        0, i, 
                                        page_buffer[i+1], page_buffer[i]};

        res = usb_control_msg(deviceHandle->device,
               USB_ENDPOINT_OUT| USB_TYPE_CLASS | USB_RECIP_DEVICE,
               USBRQ_HID_SET_REPORT,
               USB_HID_REPORT_TYPE_FEATURE << 8 | (0 & 0xff), 0,
               loadCommand, 5,
               MICRONUCLEUS_USB_TIMEOUT);
      }

#endif
#if 0
      // ask microcontroller to write the data to flash
      res = usb_control_msg(deviceHandle->device,
             USB_ENDPOINT_OUT| USB_TYPE_VENDOR | USB_RECIP_DEVICE,
             MICRONUCLEUS_SPM_COMMAND_MASK | MICRONUCLEUS_SPM_COMMAND_WRITE,
             0, address,
             0, 0,
             MICRONUCLEUS_USB_TIMEOUT);
#else
      unsigned char writeCommand[5] = {MICRONUCLEUS_SPM_COMMAND_MASK | MICRONUCLEUS_SPM_COMMAND_WRITE,
                                      (unsigned char)(address/256), (unsigned char)address, 
                                      0, 0};

      res = usb_control_msg(deviceHandle->device,
             USB_ENDPOINT_OUT| USB_TYPE_CLASS | USB_RECIP_DEVICE,
             USBRQ_HID_SET_REPORT,
             USB_HID_REPORT_TYPE_FEATURE << 8 | (0 & 0xff), 0,
             writeCommand, 5,
             MICRONUCLEUS_USB_TIMEOUT);

#endif

    }

    // call progress update callback if that's a thing
    if (prog) prog(((float) address) / ((float) deviceHandle->flash_size));
    
    // give microcontroller enough time to write this page and come back online
    delay(deviceHandle->write_sleep);
    
    if(deviceHandle->version.major < 2) {
      if (res != page_length) return -1;
    } else {
      if(res < 0) return -1;
    }
  }
  
  // call progress update callback with completion status
  if (prog) prog(1.0);
  
  return 0;
}

int micronucleus_startApp(micronucleus* deviceHandle) {
  int res;
  res = usb_control_msg(deviceHandle->device,
         USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
         4, 
         0, 0, 
         NULL, 0, 
         MICRONUCLEUS_USB_TIMEOUT);
  
  if(res!=0)  
    return -1;
  else 
    return 0;
}



