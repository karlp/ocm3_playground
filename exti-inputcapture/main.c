/*
 * Karl Palsson, 2012 <karlp@tweak.net.au
 */

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/flash.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/systick.h>
#include <libopencm3/stm32/timer.h>

#include "syscfg.h"
#include "nastylog.h"

__attribute__( ( always_inline ) ) static inline void __WFI(void)
{
  __asm volatile ("wfi");
}

#define LOG_TAG __FILE__
#define DLOG(format, args...)         nastylog(LOG_TAG ":DEBUG", format, ## args)
#define ILOG(format, args...)         nastylog(LOG_TAG ":INFO", format, ## args)
#define WLOG(format, args...)         nastylog(LOG_TAG ":WARN", format, ## args)

static struct state_t state;


void clock_setup(void) {
    rcc_clock_setup_in_hsi_out_24mhz();
    /* Lots of things on all ports... */
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);

    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);
    // oh, and dma!
    rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);
    // and timers...
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM6EN);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM7EN);
    /* Enable AFIO clock. */
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
}

void usart_enable_all_pins(void) {
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);
}

void usart_console_setup(void) {
    usart_set_baudrate(USART_CONSOLE, 115200);
    usart_set_databits(USART_CONSOLE, 8);
    usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
    usart_set_mode(USART_CONSOLE, USART_MODE_TX);
    usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
    usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);
    usart_enable(USART_CONSOLE);
}

/**
 * Use USART_CONSOLE as a console.
 * @param file
 * @param ptr
 * @param len
 * @return 
 */
int _write(int file, char *ptr, int len) {
    int i;

    if (file == STDOUT_FILENO || file == STDERR_FILENO) {
        for (i = 0; i < len; i++) {
            if (ptr[i] == '\n') {
                usart_send_blocking(USART_CONSOLE, '\r');
            }
            usart_send_blocking(USART_CONSOLE, ptr[i]);
        }
        return i;
    }
    errno = EIO;
    return -1;
}

void BUTTON_DISCO_USER_isr(void) {
    exti_reset_request(BUTTON_DISCO_USER_EXTI);
    if (state.falling) {
        state.falling = false;
        exti_set_trigger(BUTTON_DISCO_USER_EXTI, EXTI_TRIGGER_RISING);
        ILOG("held: %d\n", TIM_CNT(TIM7));
    } else {
        TIM_CNT(TIM7) = 0;
        state.falling = true;
        exti_set_trigger(BUTTON_DISCO_USER_EXTI, EXTI_TRIGGER_FALLING);
    }
}

void setup_buttons(void) {
    /* Enable EXTI0 interrupt. */
    nvic_enable_irq(NVIC_EXTI0_IRQ);

    gpio_set_mode(BUTTON_DISCO_USER_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, BUTTON_DISCO_USER_PIN);

    /* Configure the EXTI subsystem. */
    exti_select_source(BUTTON_DISCO_USER_EXTI, BUTTON_DISCO_USER_PORT);
    state.falling = false;
    exti_set_trigger(BUTTON_DISCO_USER_EXTI, EXTI_TRIGGER_RISING);
    exti_enable_request(BUTTON_DISCO_USER_EXTI);
}

static volatile int t6ovf = 0;
void tim6_isr(void) {
    TIM_SR(TIM6) &= ~TIM_SR_UIF;
    if (t6ovf++ > 1000) {
        ILOG("TICK %d\n", state.bitcount++);
        t6ovf = 0;
    }
}

void setup_tim6(void) {
    
    timer_reset(TIM6);
    // 24Mhz / 10khz -1.
    timer_set_prescaler(TIM6, 2399);  // 24Mhz/10000hz - 1
    // 10khz for 10 ticks = 1 khz overflow = 1ms overflow interrupts
    timer_set_period(TIM6, 10);
    
    nvic_enable_irq(NVIC_TIM6_IRQ);
    //nvic_set_priority(NVIC_TIM&_IRQ, 1);
    timer_enable_update_event(TIM6);  // default at reset!
    timer_enable_irq(TIM6, TIM_DIER_UIE);
    timer_enable_counter(TIM6);
}

// Just free running please...
void setup_tim7(void) {
    timer_reset(TIM7);
    timer_set_prescaler(TIM7, 23999); // 24Mhz/1000hz - 1
    timer_set_period(TIM7, 0xffff);
    timer_enable_counter(TIM7);
}

int main(void) {

    memset(&state, 0, sizeof (state));

    clock_setup();
    usart_enable_all_pins();
    usart_console_setup();
    setup_tim6();
    setup_tim7();
    setup_buttons();
    while (1) {
        //__WFI();  // This breaks texane/stlink badly!
        ;
    }

    return 0;
}
