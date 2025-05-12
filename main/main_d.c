#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>


#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"   // stdlib 
#include "hardware/irq.h"  // interrupts
#include "hardware/pwm.h"  // pwm
#include "hardware/sync.h" // wait for interrupt
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"  // adc

#include "ring.h"

#define AUDIO_PIN 28
#define BTN_PIN 14

#define SAMPLE_RATE 8000
#define DATA_LENGTH 16000
#define FREQ 8000

char audio[DATA_LENGTH];
int wav_position = 0;

SemaphoreHandle_t xSemaphorePlayInit;
SemaphoreHandle_t xSemaphorePlayDone;
SemaphoreHandle_t xSemaphoreRecordDone;

/*
 * PWM Interrupt Handler which outputs PWM level and advances the
 * current sample.
 *
 * We repeat the same value for 8 cycles this means sample rate etc
 * adjust by factor of 8 (this is what bitshifting <<3 is doing)
 *
 */
void pwm_interrupt_handler() {
    pwm_clear_irq(pwm_gpio_to_slice_num(AUDIO_PIN));
    if (wav_position < (DATA_LENGTH<<3) - 1) {
        // set pwm level
        // allow the pwm value to repeat for 8 cycles this is >>3
        pwm_set_gpio_level(AUDIO_PIN, WAV_DATA[wav_position>>3]);
        wav_position++;
    } else {
        // Acabou de reproduzir o áudio
        //wav_position = 0;
        xSemaphoreGiveFromISR(xSemaphorePlayDone, 0);
    }
}

volatile int a = 0;
bool timer_0_callback(repeating_timer_t *rt) {
  //  printf("timerrrr \n");
    if (wav_position < DATA_LENGTH) {
        a = 1;
        audio[wav_position++] = adc_read();
        return true; // keep repeating
    } else {
        // Acabou de gravar o áudio
        xSemaphoreGiveFromISR(xSemaphoreRecordDone, 0);
        return false; // stop repeating
    }
}

void btn_callback(uint gpio, uint32_t events) {
    xSemaphoreGiveFromISR(xSemaphorePlayInit, 0);
    printf("Hello! \n");
}

void sin_task() {
    int freq = 1000;
    for( int i = 0; i < SAMPLE_RATE; i++ ) {
        float sineValue = sin(2.0 * M_PI * freq * i / SAMPLE_RATE);
        int sample = (int)(((sineValue + 1.0) / 2.0) * 255.0);
        audio[i] = (char) sample;
    }

    vTaskDelay(100);
    xSemaphoreGive(xSemaphorePlayInit);

    while(1) {
    }
}

void mic_task() {
    adc_gpio_init(26);
    adc_init();
    // adc_fifo_setup(true, false, 0, false, false);
    adc_select_input(0);
    // adc_run(true);

    wav_position = 0;
    int timer_0_hz = 11000;
    repeating_timer_t timer_0;

    if (!add_repeating_timer_us(1000000 / timer_0_hz, 
                                timer_0_callback,
                                NULL, 
                                &timer_0)) {
        printf("Failed to add timer\n");
    }


    if (xSemaphoreTake(xSemaphoreRecordDone, portMAX_DELAY) == pdTRUE) {
    }

    printf("terminou\n");
    xSemaphoreGive(xSemaphorePlayInit);

    while(1) {
    }
}

void play_task() {
    /*
     * Based on: https://github.com/rgrosset/pico-pwm-audio/
     * Overclocking for fun but then also so the system clock is a
     * multiple of typical audio sampling rates.
     */
    set_sys_clock_khz(176000, true);
    gpio_set_function(AUDIO_PIN, GPIO_FUNC_PWM);

    int audio_pin_slice = pwm_gpio_to_slice_num(AUDIO_PIN);

    // Setup PWM interrupt to fire when PWM cycle is complete
    pwm_clear_irq(audio_pin_slice);
    pwm_set_irq_enabled(audio_pin_slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_interrupt_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_config config = pwm_get_default_config();

    /* Base clock 176,000 Hz divide by wrap 250 then the clock divider further divides
     * to set the interrupt rate.
     *
     * 11 KHz is fine for speech. Phone lines generally sample at 8 KHz
     *
     *
     * So clkdiv should be as follows for given sample rate
     *  8.0f for 11 KHz
     *  4.0f for 22 KHz
     *  2.0f for 44 KHz etc
     */
    pwm_config_set_clkdiv(&config, 8.0f);
    pwm_config_set_wrap(&config, 250);

    while(1) {
        if (xSemaphoreTake(xSemaphorePlayInit, pdMS_TO_TICKS(500)) == pdTRUE) {
            wav_position = 0;
            pwm_init(audio_pin_slice, &config, true);
            pwm_set_gpio_level(AUDIO_PIN, 0);
        }

        if (xSemaphoreTake(xSemaphorePlayDone, pdMS_TO_TICKS(500)) == pdTRUE) {
            pwm_set_enabled(audio_pin_slice, false);
        }
    }
}

int main() {
    stdio_init_all();
    printf("oi\n");


  gpio_init(BTN_PIN);
  gpio_set_dir(BTN_PIN, GPIO_IN);
  gpio_pull_up(BTN_PIN);
  gpio_set_irq_enabled_with_callback(BTN_PIN, 
                                     GPIO_IRQ_EDGE_FALL, 
                                     true,
                                     &btn_callback);

    xSemaphorePlayInit = xSemaphoreCreateBinary();
    xSemaphorePlayDone = xSemaphoreCreateBinary();
    xSemaphoreRecordDone = xSemaphoreCreateBinary();

    xTaskCreate(play_task, "Play Task", 4095, NULL, 1, NULL);
    //xTaskCreate(mic_task, "Mic Task", 4095, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
