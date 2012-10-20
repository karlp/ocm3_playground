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

#define RHT_INTER_BIT_TIMEOUT_USEC 500
// Falling edge-falling edge times less than this are 0, else 1
#define RHT_LOW_HIGH_THRESHOLD 100

#define RHT_PORT GPIOB
#define RHT_PIN GPIO9
#define RHT_EXTI EXTI9
#define RHT_isr exti9_5_isr
#define RHT_NVIC NVIC_EXTI9_5_IRQ


    struct state_t {
        int seconds;
        int last_start;
        unsigned bitcount;
        bool seen_startbit;
        uint8_t rht_bytes[5];
        bool rht_timeout;
        int milliticks;
    };


#ifdef	__cplusplus
}
#endif

#endif	/* SYSCFG_H */

