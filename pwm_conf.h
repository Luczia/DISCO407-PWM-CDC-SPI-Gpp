/*
 * pwmconf.h
 *
 *  Created on: Apr 5, 2020
 *      Author: Luczia
 */

#ifndef PWM_CONF_H_
#define PWM_CONF_H_

/*Demo PWM @1Hz */
extern PWMConfig pwmcfg = {
  10000,                                    /* 10kHz PWM clock frequency.     */
  10000,                                    /* Initial PWM period 1s. or 10k ticks(100us)        */
  NULL,                                     /* Period callback.               */
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},          /* CH1 mode and callback.         */
   {PWM_OUTPUT_DISABLED, NULL},             /* CH2 mode and callback.         */
   {PWM_OUTPUT_DISABLED, NULL},             /* CH3 mode and callback.         */
   {PWM_OUTPUT_DISABLED, NULL}              /* CH4 mode and callback.         */
  },
  0,                                        /* Control Register 2.            */
  0                                         /* DMA/Interrupt Enable Register. */
};


#endif /* PWM_CONF_H_ */
