// main.c

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/timer.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stream_buffer.h"

#define AUDIO_OUT_PIN    26
#define AUDIO_IN_PIN     27
#define SAMPLE_RATE      8000
#define DATA_LENGTH      (SAMPLE_RATE * 2)

static StreamBufferHandle_t xAudioStream;
static SemaphoreHandle_t    xSemaphorePlayInit;
static SemaphoreHandle_t    xSemaphorePlayDone;
static SemaphoreHandle_t    xSemaphoreRecordDone;

static volatile size_t      record_count;
static int                  audio_pin_slice;

//------------------------------------------------------------------------------
// Timer callback: read ADC and push into stream buffer until we've got DATA_LENGTH
bool mic_timer_callback(repeating_timer_t *rt) {
    if (record_count < DATA_LENGTH) {
        uint8_t sample = (adc_read() >> 4);  // scale 12-bit → 8-bit
        BaseType_t woken = pdFALSE;
        xStreamBufferSendFromISR(xAudioStream, &sample, 1, &woken);
        record_count++;
        if (woken) portYIELD_FROM_ISR();
        return true;  // keep the timer running
    } else {
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(xSemaphoreRecordDone, &woken);
        if (woken) portYIELD_FROM_ISR();
        return false; // stop the timer
    }
}

//------------------------------------------------------------------------------
// PWM IRQ handler: pull next sample from stream buffer, drive PWM,
// or signal playback done when buffer empties.
void pwm_interrupt_handler() {
    pwm_clear_irq(audio_pin_slice);

    uint8_t level;
    BaseType_t woken = pdFALSE;
    size_t bytes = xStreamBufferReceiveFromISR(
        xAudioStream, &level, 1, &woken
    );

    if (bytes == 1) {
        pwm_set_gpio_level(AUDIO_OUT_PIN, level);
    } else {
        // buffer empty → stop PWM and notify done
        pwm_set_enabled(audio_pin_slice, false);
        xSemaphoreGiveFromISR(xSemaphorePlayDone, &woken);
    }

    if (woken) portYIELD_FROM_ISR();
}

//------------------------------------------------------------------------------
// Task: wait for voice, record into stream buffer via timer, then trigger playback
void mic_task(void *pvParameters) {
    // ADC setup
    adc_gpio_init(AUDIO_IN_PIN);
    adc_init();
    adc_select_input(AUDIO_IN_PIN - 26);

    repeating_timer_t timer_cfg;
    for (;;) {
        // --- Voice detection loop ---
        printf("Aguardando detecção de voz...\n");
        bool detected = false;
        while (!detected) {
            int sum = 0;
            int values[50];
            for (int i = 0; i < 50; i++) {
                values[i] = adc_read();
                sum += values[i];
                vTaskDelay(pdMS_TO_TICKS(1));  // ~1 ms
            }
            int mean = sum / 50;
            int energy = 0;
            for (int i = 0; i < 50; i++) {
                energy += abs(values[i] - mean);
            }
            energy /= 50;
            printf("Signal energy: %d\n", energy);
            if (energy > 200) {
                detected = true;
                printf("Voz detectada!\n");
            }
        }

        // --- Start recording via repeating_timer ---
        record_count = 0;
        if (!add_repeating_timer_us(
                - (1000000 / SAMPLE_RATE),  // negative = periodic
                mic_timer_callback,
                NULL,
                &timer_cfg
             )) {
            printf("Erro ao iniciar timer de gravação\n");
        }

        // wait until callback signals buffer full
        xSemaphoreTake(xSemaphoreRecordDone, portMAX_DELAY);
        cancel_repeating_timer(&timer_cfg);

        // (Opcional) — aqui você poderia aplicar filtro de média móvel
        // diretamente na stream antes de tocar, se desejar.

        // --- Trigger playback ---
        xSemaphoreGive(xSemaphorePlayInit);
        // wait until playback ISR signals done
        xSemaphoreTake(xSemaphorePlayDone, portMAX_DELAY);
    }
}

//------------------------------------------------------------------------------
// Task: configure PWM and wait for play-init signal to start output
void play_task(void *pvParameters) {
    // PWM pin → PWM function
    gpio_set_function(AUDIO_OUT_PIN, GPIO_FUNC_PWM);
    audio_pin_slice = pwm_gpio_to_slice_num(AUDIO_OUT_PIN);

    // hook up IRQ
    pwm_clear_irq(audio_pin_slice);
    pwm_set_irq_enabled(audio_pin_slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_interrupt_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // PWM config: divide clock so wrap=250 yields ~8 kHz interrupt
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 8.0f);
    pwm_config_set_wrap(&config, 250);

    for (;;) {
        // wait for record → playback handoff
        if (xSemaphoreTake(xSemaphorePlayInit, portMAX_DELAY) == pdTRUE) {
            // start PWM
            pwm_init(audio_pin_slice, &config, true);
            pwm_set_gpio_level(AUDIO_OUT_PIN, 0);
        }
    }
}

//------------------------------------------------------------------------------
// Application entry point
int main() {
    stdio_init_all();
    printf("Iniciando sistema de gravação e reprodução...\n");

    // create RTOS primitives
    xAudioStream        = xStreamBufferCreate(DATA_LENGTH, 1);
    xSemaphorePlayInit  = xSemaphoreCreateBinary();
    xSemaphorePlayDone  = xSemaphoreCreateBinary();
    xSemaphoreRecordDone= xSemaphoreCreateBinary();

    // spawn tasks
    xTaskCreate(play_task, "Play Task", 4096, NULL, 1, NULL);
    xTaskCreate(mic_task,  "Mic  Task", 4096, NULL, 1, NULL);

    // start scheduler
    vTaskStartScheduler();

    // should never reach here
    while (true) { tight_loop_contents(); }
    return 0;
}