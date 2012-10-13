/* 
 * General configuration of the device
 * 
 * Karl Palsson <karlp@tweak.net.au> 2012
 */

#ifndef SYSCFG_H
#define	SYSCFG_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>


#define USART_CONSOLE USART2
#define USE_NASTYLOG 1

    // Discovery board LED is PC8 (blue led)
#define LED_DISCO_GREEN_PORT GPIOC
#define LED_DISCO_GREEN_PIN GPIO9
#define LED_DISCO_BLUE_PORT GPIOC
#define LED_DISCO_BLUE_PIN GPIO8

#define BUTTON_DISCO_USER_PORT GPIOA
#define BUTTON_DISCO_USER_PIN GPIO0
#define BUTTON_DISCO_USER_EXTI EXTI0
#define BUTTON_DISCO_USER_isr exti0_isr
#define BUTTON_DISCO_USER_NVIC NVIC_EXTI0_IRQ

#define RHT_PORT GPIOB
#define RHT_PIN GPIO9
#define RHT_EXTI EXTI9
#define RHT_isr exti9_5_isr
#define RHT_NVIC NVIC_EXTI9_5_IRQ


    struct state_t {
        volatile int seconds;
        int last_start;
        volatile int bitcount;
        volatile u16 timings[100];  // more than 40*2
        volatile bool rht_timeout;
        volatile int milliticks;
    };


#ifdef	__cplusplus
}
#endif

#endif	/* SYSCFG_H */

