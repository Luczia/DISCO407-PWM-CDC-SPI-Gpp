/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.hpp"
#include "hal.h"

#include "rt_test_root.h"
#include "oslib_test_root.h"

#include "shell.h"
#include "chprintf.h"

#include "usb_otg/usbcfg.h"

using namespace chibios_rt;

#include "ichausmu/IcHausMu.hpp"
#include "ichausmu/icmu_utils.h"
#include "ichausmu/icmu.h"

#include "spi_conf.h"
#include "pwm_conf.h"
#include "shell_conf.h"

#define BLUE_LED LINE_LED6 //=> Manually activated LED
#define RED_LED LINE_LED5   // => Shell active
#define GREEN_LED LINE_LED4 // => HeartBeat
#define ORANGE_LED LINE_LED3 // => OTG_USB_Com active


/*===========================================================================*/
/* Shell Handler thread to spawn a shell                                                             */
/*===========================================================================*/

static THD_WORKING_AREA(waThread2, 2048);
static THD_FUNCTION(Thread2, arg) {
  (void)arg;
  chRegSetThreadName("shell_handler");

  while (true){
    if (SDU1.config->usbp->state == USB_ACTIVE) {
                palSetLine(LINE_LED5);
                thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                                        "shell", NORMALPRIO + 1,
                                                        shellThread, (void *)&shell_cfg1);
                chThdWait(shelltp);               // Waiting termination.
                palClearLine(LINE_LED5);
    }
    chThdSleepMilliseconds(1000);
  }
}

/* =============================================================================
 * USB com status blinker thread, times are in milliseconds.
 ===============================================================================*/
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
  (void)arg;
  chRegSetThreadName("com_status");
  while (true) {
    systime_t time;
    time = serusbcfg.usbp->state == USB_ACTIVE ? 250 : 1000;
    palClearLine(LINE_LED3);
    chThdSleepMilliseconds(time);
    palSetLine(LINE_LED3);
    chThdSleepMilliseconds(time);
  }
}

/*=============================================================================
 * LED blinker heartbeat thread, times are in milliseconds.
 =============================================================================*/
static THD_WORKING_AREA(waThread3, 128);
static THD_FUNCTION(Thread3, arg) {
  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palClearPad(GPIOD, GPIOD_LED4);
    chThdSleepMilliseconds(800);
    palSetPad(GPIOD, GPIOD_LED4);
    chThdSleepMilliseconds(200);
  }
}


/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */

  halInit();
  //chSysInit();
  System::init();

  // Initializes a serial-over-USB CDC driver./
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /** Activates the USB driver and then the USB bus pull-up on D+.
       * Note, a delay is inserted in order to not have to disconnect the cable
       * after a reset.
       */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  // Shell manager initialization.
  shellInit();

  //Initialize pads and AF(Alternate functions) for PWM
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1)); //PWM on PA8(dico) or D7(nucleo/arduino header)
  //Initialize pads and AF for Serial UART 2 */
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));
  /*Initialize pads and AF for SPI */
  palSetPadMode(PORT_SPI2_SCK, PIN_SPI2_SCK,PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);    /* New SCK */
  palSetPadMode(PORT_SPI2_MISO, PIN_SPI2_MISO,PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);    /* New MISO*/
  palSetPadMode(PORT_SPI2_MOSI, PIN_SPI2_MOSI, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);    /* New MOSI*/
  palSetPadMode(PORT_SPI2_CS, PIN_SPI2_CS, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);//PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST); /* New CS*/

  //Start Serial on UART 2
  sdStart(&SD2, NULL);
  //Start PWM
  pwmStart(&PWMD1, &pwmcfg);
  //Start SPI
  spiStart(&SPID2, &cs_spicfg); //Power Up the clock signal and start the driver
  //Confirm boot to the computer
  sdWriteTimeout(&SD2, (uint8_t*)"\r\n!System Initialized!\r\n", 25, TIME_MS2I(50));


  //Start PWM
  pwmChangePeriod(&PWMD1, 500);
  /*always call a pwmEnableChanel after a PeriodChange because according to doc: "If a period is specified that is shorter than the pulse width
    *          programmed in one of the channels then the behavior is not;
    *          guaranteed." */
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000));

  // Creates the usb status blinker thread.
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  // Creates the shell spawning thread.
  chThdCreateStatic(waThread2, sizeof(waThread2), LOWPRIO, Thread2, NULL);
  //Create the heartbeat thread
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, Thread3, NULL);

  ///Init encoder;
  encoder.csFunctionAttach(ChangeCS);
  encoder.streamFunctionAttach(TransferIcMU);



  while (true) {
    if (palReadPad(GPIOA, GPIOA_BUTTON)) {
        fctStatus init_encoder = encoder.init();
        encoder.setAutomaticGain(true);

        sdWriteTimeout(&SD2, (uint8_t*)"Enc status:\r\n", 13, TIME_MS2I(50)); //Example to write a frame on UART 2
        sdPutTimeout(&SD2, (int8_t)init_encoder,TIME_MS2I(50)); //Example to write a single byte on UART 2
        sdPutTimeout(&SD2, (int8_t)'\n',TIME_MS2I(50));

        basicSpiComTest(); //example to call a function of a C++ library
        sdPutTimeout(&SD2, (int8_t)'t',TIME_MS2I(50));
        uint8_t msg[] = "\r\nD: SPI frame sent";
        chnWrite(&SDU1, msg, sizeof msg); //Example to send Debug Message on the VCP (Virtual Com Port
    }

    BaseThread::sleep(TIME_MS2I(200)); //chThdSleepMilliseconds(200) in C;
  }

  return 0;
}
