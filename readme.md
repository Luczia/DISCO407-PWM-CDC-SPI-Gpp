*****************************************************************************
** ChibiOS/RT base devlopment code for F407 Discovery board.      
*****************************************************************************

 This repo contains all files for a clean development on a STM32 F407 Discovery board. 
 It includes files in C and C++ to show example of multiple language compilation, as well as a example files for :
    --usb_otg/* to create a virtual com port on the FS USB port of the board
    --shel_conf.c/h to create a shell engine in the chip whcih answers basic commands (threads, mem, turn on and off LED)
    --pwm_conf.h to enable PWM
    --spi_conf.h to enable SPI with 3 differents clock speed
    -- an example of a C++ library wrapper
    
  It runs on ChibiOS 19.1 but should be easily ported to 18.2 as the project was originally created on that branch.
  
  Just modidy the makefile variable CHIBIOS to adjust the location of your chibiOS source files.
  
  
  Hopefully it can be usefull to some wanderes lost in the ChibiOS world, it took me quite soem times to get all thos example running with this beautiful but complex RTOS.
  
  


