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

#define LOG_TAG __FILE__
#define DLOG(format, args...)         nastylog(LOG_TAG ":DEBUG", format, ## args)
#define ILOG(format, args...)         nastylog(LOG_TAG ":INFO", format, ## args)
#define WLOG(format, args...)         nastylog(LOG_TAG ":WARN", format, ## args)

static struct state_t volatile state;

volatile uint64_t ksystick;

uint64_t millis(void)
{
    return ksystick;
}

/**
 * Busy loop for X ms USES INTERRUPTS
 * @param ms
 */
void delay_ms(unsigned int ms) {
    uint64_t now = millis();
    while (millis() - ms < now) {
        ;
    }
}

void sys_tick_handler(void)
{
    ksystick++;
    state.milliticks++;
    if (state.milliticks >= 1000) {
        state.milliticks = 0;
        ILOG("Tick: %d: %d\n", state.seconds++, TIM7_CNT);
    }
}

void clock_setup(void) {
    rcc_clock_setup_in_hsi_out_24mhz();
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM7EN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
}

void setup_console(void) {
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);
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

void setup_tim7(void) {
    state.timed_out = false;
//    timer_reset(TIM7);
    // 1ms ticks, 23999
    // 1usec ticks, 23
    TIM7_CNT = 1;
    TIM7_PSC = 23999;
    TIM7_ARR = 5000;
    TIM7_SR &= ~TIM_SR_UIF;
    TIM7_DIER |= TIM_DIER_UIE;
    nvic_enable_irq(NVIC_TIM7_IRQ);
    TIM7_CR1 |= TIM_CR1_CEN;
    ILOG("Starting oneshot!\n");
}


void tim7_isr(void) {
    timer_disable_irq(TIM7, TIM_DIER_UIE);
    timer_disable_counter(TIM7);
    nvic_disable_irq(NVIC_TIM7_IRQ);
    state.timed_out = true;
    timer_clear_flag(TIM7, TIM_SR_UIF);
}


static void systick_setup(void) {
    /* 24MHz / 8 => 3000000 counts per second. */
    systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8);
    /* 3000000/3000 = 1000 overflows per second - every 1ms one interrupt */
    systick_set_reload(3000);
    systick_interrupt_enable();
    systick_counter_enable();
}


int main(void) {

    clock_setup();
    setup_console();
    systick_setup();
    setup_tim7();
    while (1) {
        if (state.seconds - state.last_start > 10) {
            state.last_start = state.seconds;
            setup_tim7();
        }
        if (state.timed_out) {
            ILOG("Finished!\n");
            state.timed_out = false;
        }
    }

    return 0;
}
