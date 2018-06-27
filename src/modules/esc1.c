/*
 * esc1.c
 *
 *  Created on: 25 de mai de 2018
 *      Author: Girardin
 */

#include "stm32f10x.h"

#include "esc1.h"

static int calibration_state = -1;
static int safe_esc1         = 0;
static int danger_esc1       = 0;
static int cur_p             = 0;

void esc1_update()
{
    if(calibration_state >= 0 && calibration_state<=1375)
    {
        switch(calibration_state)
        {
            case 0:
                GPIOB->ODR &= ~GPIO_ODR_ODR8;  //turn off ESC
                break;

            case 25:                           //0.02 sec * 25 = 0.5 sec
                GPIOB->ODR |= GPIO_ODR_ODR9;  //turn on SET
                GPIOB->ODR |= GPIO_ODR_ODR8;   //turn on ESC
                break;

            case 150:                          //0.02sec * 125 = 2.5 sec
                GPIOB->ODR &= ~GPIO_ODR_ODR9; //turn off set
                TIM4->CCR1 = PWM_HIGH;
                break;

            case 550:                          //0.02sec * 400 = 8.0 sec
                TIM4->CCR1 = PWM_LOW;
                break;

            case 950:                          //0.02sec * 400 = 8.0 sec
                TIM4->CCR1 = PWM_NEUTRAL;
                break;

            case 1350:                         //0.02sec * 400 = 8.0 sec
                GPIOB->ODR &= ~GPIO_ODR_ODR8;  //turn off ESC
                break;

            case 1375:
                GPIOB->ODR |= GPIO_ODR_ODR8;   //turn on ESC
                calibration_state = -1;
                danger_esc1 = 0;
        }
        if(calibration_state != -1)
            calibration_state++;
    }
    else
    {
        if(cur_p > 10)
        {
            danger_esc1 = 1;
            safe_esc1 = 0;
        }

        else if(danger_esc1)
        {
            safe_esc1++;
            if(safe_esc1 == 100)
            {
                danger_esc1 = 0;
                safe_esc1 = 0;
            }

        }
        TIM4->CCR1 = 3072 + (!danger_esc1 || (cur_p>10) ? cur_p : 0);
    }

}

void esc1_control(int p)
{
    cur_p = p;
}

void esc1_calibration_routine()
{
    if(calibration_state == -1)
    {
        calibration_state = 0;
    }
}

int esc1_calibration_done()
{
    return calibration_state == -1;
}
