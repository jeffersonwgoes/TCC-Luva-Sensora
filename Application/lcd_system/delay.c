/**
  ******************************************************************************
  * @file       delay.c
  * @author Yohanes Erwin Setiawan
  * @date       10 January 2016
  ******************************************************************************
  */
    
#include "delay.h"


// SysTick_Handler function will be called every 1 us

void DelayUs(uint32_t us)
{
    unsigned char x = us*72;
    while(x--);
}

void DelayMs(uint32_t ms)
{
    // Wait until ms reach zero
    while (ms--)
    {
        // Delay 1ms
        DelayUs(1000);
    }
}

/********************************* END OF FILE ********************************/
/******************************************************************************/
