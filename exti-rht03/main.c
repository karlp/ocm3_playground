/*
 * Karl Palsson, 2012 <karlp@tweak.net.au
 */

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
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

    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);
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

void stuff_bit(int bitnumber, int timing, volatile uint8_t *bytes) {
    int byte_offset = bitnumber / 8;
    int bit = 7 - (bitnumber % 8); // Stuff MSB first.
    if (timing < RHT_LOW_HIGH_THRESHOLD) {
        bytes[byte_offset] &= ~(1 << bit);
    } else {
        bytes[byte_offset] |= (1 << bit);
    }
}

void RHT_isr(void) {
    exti_reset_request(RHT_EXTI);
    int cnt = TIM7_CNT;
    TIM7_CNT = 0;
    // Skip catching ourself pulsing the start line until the 150uS start.
    if (!state.seen_startbit) {
        if (cnt < RHT_LOW_HIGH_THRESHOLD) {
            return;
        } else {
            state.seen_startbit = true;
        }
    }
    if (state.bitcount > 0) {  // but skip that start bit...
        stuff_bit(state.bitcount - 1, cnt, state.rht_bytes);
    }
    state.bitcount++;
}

/**
 * We set this timer to count uSecs.
 * The interrupt is only to indicate that it timed out and to shut itself off.
 */
void setup_tim7(void) {
    timer_clear_flag(TIM7, TIM_SR_UIF);
    TIM7_CNT = 1;
    timer_set_prescaler(TIM7, 23); // 24Mhz/1Mhz - 1
    timer_set_period(TIM7, RHT_INTER_BIT_TIMEOUT_USEC);
    timer_enable_irq(TIM7, TIM_DIER_UIE);
    nvic_enable_irq(NVIC_TIM7_IRQ);
    timer_enable_counter(TIM7);
}

void start_rht_read(void) {
    // First, move the pins up and down to get it going...
    gpio_set_mode(RHT_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, RHT_PIN);
    gpio_clear(RHT_PORT, RHT_PIN);
    delay_ms(20); // docs say 1-10ms is enough....
    gpio_set(RHT_PORT, RHT_PIN);
    // want to wait for 40us here, but we're ok with letting some code delay us..
    state.bitcount = 0;
    state.seen_startbit = false;
    state.rht_timeout = false;
    // don't need, let bitcount declare what's valid!
    // memset(state.timings, 0, sizeof(state.timings));
    nvic_enable_irq(RHT_NVIC);
    // pull up will finish the job here for us.
    gpio_set_mode(RHT_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, RHT_PIN);
    exti_select_source(RHT_EXTI, RHT_PORT);
    exti_set_trigger(RHT_EXTI, EXTI_TRIGGER_FALLING);
    exti_enable_request(RHT_EXTI);
    setup_tim7();
}

void tim7_isr(void) {
    timer_clear_flag(TIM7, TIM_SR_UIF);
    state.rht_timeout = true;
    nvic_disable_irq(NVIC_TIM7_IRQ);
    //nvic_set_priority(NVIC_TIM&_IRQ, 1);
    timer_disable_irq(TIM7, TIM_DIER_UIE);
    timer_disable_counter(TIM7);
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
            return;
        }
        if (state.bitcount >= 40) {
            return;
        }
        __WFI();
    }
}

void loop_forever(void) {
    if (state.seconds - state.last_start > 3) {
        state.last_start = state.seconds;
        ILOG("Start!\n");
        start_rht_read();
        wait_for_shit();
        if (state.rht_timeout) {
            ILOG("timeout\n");
            return;
        }
        ILOG("All bits found!\n");
        unsigned chksum = state.rht_bytes[0] + state.rht_bytes[1] + state.rht_bytes[2] + state.rht_bytes[3];
        chksum &= 0xff;
        DLOG("%x %x %x %x sum: %x == %x\n", 
            state.rht_bytes[0], state.rht_bytes[1], state.rht_bytes[2], state.rht_bytes[3],
            chksum, state.rht_bytes[4]);
        if (chksum != state.rht_bytes[4]) {
            ILOG("CHKSUM failed, ignoring: \n");
            return;
        }

        int rh = (state.rht_bytes[0] << 8 | state.rht_bytes[1]);
        int temp = (state.rht_bytes[2] << 8 | state.rht_bytes[3]);
        ILOG("orig: temp = %d, rh = %d\n", temp, rh);
        ILOG("Temp: %d.%d C, RH = %d.%d %%\n", temp / 10, temp % 10, rh / 10, rh % 10);

    }
    // texane/stlink will have problems debugging through this.
    __WFI();
}

int main(void) {
    clock_setup();
    usart_enable_all_pins();
    usart_console_setup();
    systick_setup();
    setup_tim7();
    while (1) {
        loop_forever();
    }

    return 0;
}
