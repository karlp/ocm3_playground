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
        ILOG("Tick: %d\n", state.seconds++);
    }
}

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

void RHT_isr(void) {
    exti_reset_request(RHT_EXTI);
    state.timings[state.bitcount++] = TIM7_CNT;
    TIM7_CNT = 0;
}

// We want to count uSecs.  2^16 usecs should be a timeout on interrupt
void setup_tim7(void) {
#if ORIGINAL
    timer_reset(TIM7);
    timer_set_prescaler(TIM7, 23); // 24Mhz/1Mhz - 1
    timer_set_period(TIM7, 0xffff);
    nvic_enable_irq(NVIC_TIM7_IRQ);
    //nvic_set_priority(NVIC_TIM&_IRQ, 1);
    timer_enable_update_event(TIM7);  // default at reset!
    timer_enable_irq(TIM7, TIM_DIER_UIE);
    timer_enable_counter(TIM7);
#else
    TIM7_CNT = 1;
    timer_set_prescaler(TIM7, 23);
    timer_set_period(TIM7, 0xfff0);
    timer_clear_flag(TIM7, TIM_SR_UIF);
    timer_enable_irq(TIM7, TIM_DIER_UIE);
    nvic_enable_irq(NVIC_TIM7_IRQ);
    timer_enable_counter(TIM7);
#endif
}

void start_rht_read(void) {
    // First, move the pins up and down to get it going...
    gpio_set_mode(RHT_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, RHT_PIN);
    gpio_clear(RHT_PORT, RHT_PIN);
    delay_ms(20); // docs say 1-10ms is enough....
    gpio_set(RHT_PORT, RHT_PIN);
    // want to wait for 40us here, but we're ok with letting some code delay us..
    state.bitcount = 0;
    memset(state.timings, 0, sizeof(state.timings));
    nvic_enable_irq(RHT_NVIC);
    // pull up will finish the job here for us.
    gpio_set_mode(RHT_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, RHT_PIN);
    exti_select_source(RHT_EXTI, RHT_PORT);
    exti_set_trigger(RHT_EXTI, EXTI_TRIGGER_FALLING);
    exti_enable_request(RHT_EXTI);
    state.rht_timeout = false;
    setup_tim7();
}

void tim7_isr(void) {
#if ORIGINAL
    TIM_SR(TIM7) &= ~TIM_SR_UIF;
    state.rht_timeout = true;
    nvic_disable_irq(NVIC_TIM7_IRQ);
    //nvic_set_priority(NVIC_TIM&_IRQ, 1);
    timer_disable_irq(TIM7, TIM_DIER_UIE);
    timer_disable_counter(TIM7);
#else
    timer_disable_irq(TIM7, TIM_DIER_UIE);
    timer_disable_counter(TIM7);
    nvic_disable_irq(NVIC_TIM7_IRQ);
    state.rht_timeout = true;
    timer_clear_flag(TIM7, TIM_SR_UIF);
#endif
    
}


void systick_setup(void) {
    /* 24MHz / 8 => 3000000 counts per second. */
    systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8);

    /* 3000000/3000 = 1000 overflows per second - every 1ms one interrupt */
    systick_set_reload(3000);

    systick_interrupt_enable();

    /* Start counting. */
    systick_counter_enable();
}

void wait_for_shit(void) {
    while (1) {
        if (state.rht_timeout) {
            ILOG("timeout\n"); // eventually, we just go til we get all our bits
            return;
        }
        if (state.bitcount > 115) {
            ILOG("too many bits!\n");
            return;
        }
    }
}

int main(void) {

    memset(&state, 0, sizeof (state));

    clock_setup();
    usart_enable_all_pins();
    usart_console_setup();
    systick_setup();
    while (1) {
        if (state.seconds - state.last_start > 3) {
            state.last_start = state.seconds;
            ILOG("Start!\n");
            // Start an rht03 reading...
            start_rht_read();
            wait_for_shit();
            for (int i = 0; i < state.bitcount; i++)  {
                ILOG("bit[%d] = %d\n", i, state.timings[i]);
            }
        }
        //__WFI();  // This breaks texane/stlink badly!
        ;
    }

    return 0;
}
