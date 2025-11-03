// main.c
// Atividade 08 - Leitura do sensor de temperatura NTC + alarme
// ESP-IDF (compatível com ESP32-S3)
// Ajuste os defines de pinos abaixo conforme sua placa/Wokwi.

// Inclua no CMakeLists.txt: idf_component_register(SRCS "main.c" INCLUDE_DIRS ".")
// Compilar com ESP-IDF v4.x ou v5.x.

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_system.h"

// ----------------- CONFIGURÁVEIS (ajuste conforme sua placa/Wokwi) -----------------
#define TAG "ATV08"

// ADC (NTC)
#define ADC_UNIT                ADC_UNIT_1
#define ADC_CHANNEL             ADC1_CHANNEL_0    // alterar se necessário
#define ADC_ATTEN               ADC_ATTEN_DB_11
#define ADC_WIDTH               ADC_WIDTH_BIT_12
#define V_REF                  3.3f               // tensão de referência (aprox.)
#define ADC_MAX_VALUE          4095.0f             // 12-bit

// Hardware pins (exemplos — adapte no Wokwi se necessário)
#define BUTTON_A_GPIO          18      // incrementa alarme
#define BUTTON_B_GPIO          19      // decrementa alarme
#define LED1_GPIO              2
#define LED2_GPIO              3
#define LED3_GPIO              4
#define LED4_GPIO              5
#define BUZZER_GPIO            21      // saída PWM (LEDC)
#define I2C_SDA_GPIO           8
#define I2C_SCL_GPIO           9
#define I2C_PORT               I2C_NUM_0
#define I2C_FREQ_HZ            100000  // 100kHz

// PWM/LEDC (buzzer)
#define LEDC_TIMER             LEDC_TIMER_0
#define LEDC_MODE              LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL           LEDC_CHANNEL_0
#define LEDC_DUTY_RES          LEDC_TIMER_10_BIT
#define BUZZER_FREQ_HZ         2000    // frequência do alarme (2kHz)
#define BUZZER_DUTY            512     // duty para 10-bit (meio)

// Debounce
#define DEBOUNCE_MS            50

// NTC / divisor de tensão
#define R_FIXED_OHMS           10000.0f   // resistor fixo do divisor (10k)
#define NTC_R0                 10000.0f   // NTC R @ 25°C (10k)
#define NTC_BETA               3950.0f    // Beta do NTC (ex.: 3950)
#define TEMP_REF_KELVIN        298.15f    // 25°C em Kelvin

// Atualizações
#define LCD_UPDATE_MS          500
#define LED_BLINK_MS           500

// Temperatura de alarme default
static int alarm_temperature_c = 25; // default 25 °C

// ----------------- Variáveis e filas -----------------
static QueueHandle_t gpio_evt_queue = NULL;
static volatile bool alarm_active = false;
static volatile float latest_temp_c = 0.0f;

// ---------- Simple I2C LCD (PCF8574 backpack) - comandos mínimos ----------
/*
 This is a compact PCF8574-based I2C->HD44780 driver with 4-bit mode.
 It implements initialization and simple prints for 2x16.
 Adapte se seu módulo I2C for diferente.
*/

// PCF8574 bit mapping (ajuste se seu módulo usar outra ordem)
#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04
#define LCD_RW        0x02
#define LCD_RS        0x01

static uint8_t pcf_addr = 0x27; // endereço comum: 0x27 ou 0x3F

static esp_err_t i2c_write_byte(uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err = i2c_master_start(cmd);
    if (err != ESP_OK) { i2c_cmd_link_delete(cmd); return err; }
    i2c_master_write_byte(cmd, (pcf_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return err;
}
static void lcd_pulse(uint8_t data) {
    i2c_write_byte(data | LCD_ENABLE | LCD_BACKLIGHT);
    ets_delay_us(1);
    i2c_write_byte((data & ~LCD_ENABLE) | LCD_BACKLIGHT);
    ets_delay_us(50);
}
static void lcd_write4(uint8_t nibble, uint8_t flags) {
    uint8_t data = (nibble << 4) | flags | LCD_BACKLIGHT;
    i2c_write_byte(data);
    lcd_pulse(data);
}
static void lcd_cmd(uint8_t cmd) {
    lcd_write4((cmd >> 4) & 0x0F, 0x00);
    lcd_write4(cmd & 0x0F, 0x00);
    vTaskDelay(pdMS_TO_TICKS(2));
}
static void lcd_data(uint8_t data) {
    lcd_write4((data >> 4) & 0x0F, LCD_RS);
    lcd_write4(data & 0x0F, LCD_RS);
    vTaskDelay(pdMS_TO_TICKS(2));
}
static void lcd_init(void) {
    vTaskDelay(pdMS_TO_TICKS(50));
    // init sequence (per HD44780)
    i2c_write_byte(0x00); // ensure device reachable
    // force 4-bit mode
    lcd_write4(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write4(0x03, 0);
    ets_delay_us(150);
    lcd_write4(0x03, 0);
    lcd_write4(0x02, 0); // 4-bit
    // Function set: 2 lines, 5x8 dots
    lcd_cmd(0x28);
    // Display on, cursor off, blink off
    lcd_cmd(0x0C);
    // Clear
    lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
    // Entry mode set
    lcd_cmd(0x06);
}
static void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    addr += col;
    lcd_cmd(0x80 | addr);
}
static void lcd_print_line(uint8_t row, const char *s) {
    lcd_set_cursor(row, 0);
    for (int i = 0; i < 16; ++i) {
        char c = s[i];
        if (c == 0) break;
        lcd_data(c);
    }
    // Fill rest with spaces
    int len = strlen(s);
    for (int i = len; i < 16; ++i) lcd_data(' ');
}

// ----------------- Funções utilitárias -----------------

// Converte leitura ADC (raw) para temperatura C usando divisor + equação Beta
static float adc_raw_to_temp_c(int raw) {
    // tensão no pino
    float voltage = (raw / ADC_MAX_VALUE) * V_REF;
    // R_ntc = R_fixed * V / (Vref - V)
    if (voltage <= 0.0f) return -273.15f;
    float r_ntc = R_FIXED_OHMS * (voltage / (V_REF - voltage));
    // Steinhart-Hart (beta formula)
    float inv_T = (1.0f / TEMP_REF_KELVIN) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0);
    float t_kelvin = 1.0f / inv_T;
    float t_c = t_kelvin - 273.15f;
    return t_c;
}

// Leitura ADC com média simples para reduzir ruído
static int read_adc_avg(int samples) {
    int sum = 0;
    for (int i = 0; i < samples; ++i) {
        int val = adc1_get_raw(ADC_CHANNEL);
        sum += val;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return sum / samples;
}

// ----------------- ISR e tratamento de botões com debounce -----------------
typedef struct {
    uint32_t gpio_num;
    int64_t time_ms;
} gpio_evt_t;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    // push minimal info to queue
    uint32_t gpio = gpio_num;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(gpio_evt_queue, &gpio, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

static void button_task(void* arg) {
    const TickType_t xDebounceTicks = pdMS_TO_TICKS(DEBOUNCE_MS);
    uint32_t last_time = 0;
    uint32_t gpio;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &gpio, portMAX_DELAY)) {
            // simple software debounce: wait DEBOUNCE_MS and read pin stable state
            vTaskDelay(xDebounceTicks);
            int level = gpio_get_level((gpio_num_t)gpio);
            if (level == 0) { // assuming pull-up and active-low buttons
                if (gpio == BUTTON_A_GPIO) {
                    alarm_temperature_c += 5;
                    ESP_LOGI(TAG, "Botao A pressed - alarm -> %d C", alarm_temperature_c);
                } else if (gpio == BUTTON_B_GPIO) {
                    alarm_temperature_c -= 5;
                    ESP_LOGI(TAG, "Botao B pressed - alarm -> %d C", alarm_temperature_c);
                }
            }
        }
    }
}

// ----------------- LED blink task para alarme -----------------
static void led_blink_task(void* arg) {
    const TickType_t delay = pdMS_TO_TICKS(LED_BLINK_MS);
    gpio_num_t leds[4] = {LED1_GPIO, LED2_GPIO, LED3_GPIO, LED4_GPIO};
    bool blink_state = false;
    while (1) {
        if (alarm_active) {
            blink_state = !blink_state;
            for (int i = 0; i < 4; ++i) {
                gpio_set_level(leds[i], blink_state ? 1 : 0);
            }
        }
        vTaskDelay(delay);
    }
}

// ----------------- LCD update task -----------------
static void lcd_task(void* arg) {
    char line1[17], line2[17];
    while (1) {
        snprintf(line1, sizeof(line1), "Temp: %6.2f C", latest_temp_c);
        snprintf(line2, sizeof(line2), "Alarme: %3d C", alarm_temperature_c);
        lcd_print_line(0, line1);
        lcd_print_line(1, line2);
        vTaskDelay(pdMS_TO_TICKS(LCD_UPDATE_MS));
    }
}

// ----------------- Main application task -----------------
static void sensor_task(void* arg) {
    while (1) {
        int raw = read_adc_avg(8);
        float temp = adc_raw_to_temp_c(raw);
        latest_temp_c = temp;

        // Verifica proximidade e controla LEDs (não piscando) conforme distância
        float diff = alarm_temperature_c - temp; // quão abaixo está temp (positivo = abaixo)
        // ligar LEDs progressivamente:
        // 4 LEDs - 20°C
        // 3 LEDs - 15°C
        // 2 LEDs - 10°C
        // 1 LED  - 2°C
        // nota: se temp >= alarm -> alarm_active e piscará (led_blink_task)
        if (temp >= alarm_temperature_c) {
            // alarm on
            alarm_active = true;
            // buzzer ON (LEDC)
            ledc_set_freq(LEDC_MODE, LEDC_TIMER, BUZZER_FREQ_HZ);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, BUZZER_DUTY);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        } else {
            // alarm off
            alarm_active = false;
            // turn off buzzer
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            // set LEDs per aproximação (apenas estado estático aqui)
            int leds_on = 0;
            float near = alarm_temperature_c - temp; // se near >= x então ligar
            if (near >= 20.0f) leds_on = 1;
            if (near >= 15.0f) leds_on = 2;
            if (near >= 10.0f) leds_on = 3;
            if (near >= 2.0f)  leds_on = 4;
            // Mapeamento: leds_on = 1 => apenas LED1 ligado ; 4 => LED1..LED4 ligados
            gpio_set_level(LED1_GPIO, (leds_on >= 1) ? 1 : 0);
            gpio_set_level(LED2_GPIO, (leds_on >= 2) ? 1 : 0);
            gpio_set_level(LED3_GPIO, (leds_on >= 3) ? 1 : 0);
            gpio_set_level(LED4_GPIO, (leds_on >= 4) ? 1 : 0);
        }

        vTaskDelay(pdMS_TO_TICKS(800));
    }
}

// ----------------- Inicializações -----------------
static void init_adc(void) {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
    ESP_LOGI(TAG, "ADC initialized (channel raw read sample)");
}

static void init_leds(void) {
    gpio_config_t io_conf = {0};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<LED1_GPIO)|(1ULL<<LED2_GPIO)|(1ULL<<LED3_GPIO)|(1ULL<<LED4_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(LED1_GPIO, 0);
    gpio_set_level(LED2_GPIO, 0);
    gpio_set_level(LED3_GPIO, 0);
    gpio_set_level(LED4_GPIO, 0);
}

static void init_buttons(void) {
    gpio_config_t io_conf = {0};
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // botão ativo LOW (pull-up)
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<BUTTON_A_GPIO) | (1ULL<<BUTTON_B_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_A_GPIO, gpio_isr_handler, (void*) BUTTON_A_GPIO);
    gpio_isr_handler_add(BUTTON_B_GPIO, gpio_isr_handler, (void*) BUTTON_B_GPIO);
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
}

static void init_buzzer(void) {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = BUZZER_FREQ_HZ,
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL,
        .duty       = 0,
        .gpio_num   = BUZZER_GPIO,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER
    };
    ledc_channel_config(&ledc_channel);
}

static void init_i2c_lcd(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    lcd_init();
}

// ----------------- app_main -----------------
void app_main(void) {
    ESP_LOGI(TAG, "Atividade 08 - Inicializando...");

    init_adc();
    init_leds();
    init_buttons();
    init_buzzer();
    init_i2c_lcd();

    // Criar tasks
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 8, NULL);
    xTaskCreate(lcd_task, "lcd_task", 4096, NULL, 5, NULL);
    xTaskCreate(led_blink_task, "led_blink_task", 2048, NULL, 6, NULL);

    ESP_LOGI(TAG, "Sistema iniciado. Alarme default: %d C", alarm_temperature_c);
}