/*
 * shellcfg.c
 *
 *  Created on: Apr 4, 2020
 *      Author: Luczia
 */
#include "ch.h"
#include "hal.h"

#include "shell.h"
#include "chprintf.h"

#include "usb_otg/usbcfg.h"


//#include "ichausmu/icmu_utils.h"
//#include "ichausmu/icmu_utils.hpp"

#define TEST BUS_SIZE

/*===========================================================================*/
/* Command line/shell related.                                                     */
/*===========================================================================*/



/* Can be measured using dd if=/dev/xxxx of=/dev/null bs=512 count=10000.*/
void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) {
  static uint8_t buf[] =
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: write\r\n");
    return;
  }

  while (chnGetTimeout((BaseChannel *)chp, TIME_IMMEDIATE) == Q_TIMEOUT) {
#if 1
    /* Writing in channel mode.*/
    chnWrite(&SDU1, buf, sizeof buf - 1);
#else
    /* Writing in buffer mode.*/
    (void) obqGetEmptyBufferTimeout(&SDU1.obqueue, TIME_INFINITE);
    memcpy(SDU1.obqueue.ptr, buf, SERIAL_USB_BUFFERS_SIZE);
    obqPostFullBuffer(&SDU1.obqueue, SERIAL_USB_BUFFERS_SIZE);
#endif
  }
  chprintf(chp, "\r\n\nstopped\r\n");
}

void cmd_led(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argv;
   if (argc > 0) {
     chprintf(chp, "Usage: led\r\n Will Toggle the RED LED \r\n");
     return;
   }
   palToggleLine(LINE_LED6);
}

#define TEST TETA

void cmd_spi(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argv;
   if (argc > 0) {
     chprintf(chp, "Usage: led\r\n Will Send a SPI frame  \r\n");
     return;
   }
  //basicSpiInitFrame();
   basicSpiComTest();
   test();
   sdPutTimeout(&SD2, (int8_t)'t',TIME_MS2I(50));
}


const ShellCommand commands[] = {
  {"write", cmd_write},
  {"led", cmd_led},
  {"spi", cmd_spi},
  {NULL, NULL}
};

const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};
