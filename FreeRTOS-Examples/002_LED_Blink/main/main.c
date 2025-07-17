/**
 * @file    main.c
 * @brief   ESP32-PICO LED Control with FreeRTOS Multi-Core Tasks
 * @author  Rahul B. 
 * @version 2.0
 * @date    17th July 2025
 * 
 * @description
* A simplified implementation to demonstrate running two independent tasks on
 * separate ESP32 cores.
 * - Task 1 runs on Core 0, blinking a RED LED every 1 second.
 * - Task 2 runs on Core 1, blinking a WHITE LED every 2 seconds.
 * This version removes complex structures for clarity and ease of understanding.

 * 
 * @hardware
 * - Target: ESP32-PICO Development Board
 * - RED LED: GPIO 2
 * - WHITE LED: GPIO 4
 * 
 * @features
 * - Thread-safe LED operations
 * - Configurable timing intervals
 * - Real-time core monitoring
 * - Structured parameter passing
 * - Comprehensive logging system
 */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_idf_version.h"

/*============================================================================
 * PREPROCESSOR DEFINITIONS
 *============================================================================*/

// Hardware GPIO Pin Definitions
#define RED_LED_PIN                     (2U)    /**< Red LED GPIO pin */
#define WHITE_LED_PIN                   (4U)    /**< White LED GPIO pin */

// Define blink delays in milliseconds
#define RED_LED_DELAY_MS                (2000U) /**< Red LED Delay in milliseconds */
#define WHITE_LED_DELAY_MS              (1000U) /**< White LED Delay in milliseconds */

// Task Configuration
#define LED_TASK_PRIORITY               (1U)    /**< Task priority level */
#define LED_TASK_STACK_SIZE             (2048U) /**< Stack size in bytes */

// LED States
#define LED_STATE_OFF                   (0U)    /**< LED OFF state */
#define LED_STATE_ON                    (1U)    /**< LED ON state */

// Logging tags
static const char *TAG  = "LED";
static const char *TAG_RED   = "RED_LED";
static const char *TAG_WHITE = "WHITE_LED";


/* Reset reason lookup table */
static const char *reset_reasons[] = {
                                        "ESP_RST_UNKNOWN", 
                                        "ESP_RST_POWERON", 
                                        "ESP_RST_EXT", 
                                        "ESP_RST_SW",
                                        "ESP_RST_PANIC", 
                                        "ESP_RST_INT_WDT", 
                                        "ESP_RST_TASK_WDT", 
                                        "ESP_RST_WDT",
                                        "ESP_RST_DEEPSLEEP", 
                                        "ESP_RST_BROWNOUT", 
                                        "ESP_RST_SDIO"
};

/* ────── GPIO INIT ────── */
static void led_gpio_init(gpio_num_t gpio_pin) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << gpio_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(gpio_pin, 0);  // Set LED OFF initially
}

/* ────── Shutdown Handler ────── */
void FirmwareShutdownHandler(void) {
    char task_info[1024];
    TaskHandle_t current = xTaskGetCurrentTaskHandle();
    printf("Shutdown from: %s\n", pcTaskGetName(current));
    vTaskList(task_info);
    printf("Active Tasks:\n%s\n", task_info);
}

/* ────── RED LED Task (Core 0) ────── */
void red_led_task(void *param) {
    gpio_num_t pin = RED_LED_PIN;
    led_gpio_init(pin);
    while (1) {
        gpio_set_level(pin, 1);
        ESP_LOGI(TAG_RED, "LED ON");
        vTaskDelay(pdMS_TO_TICKS(RED_LED_DELAY_MS / 2));
        gpio_set_level(pin, 0);
        ESP_LOGI(TAG_RED, "LED OFF");
        vTaskDelay(pdMS_TO_TICKS(RED_LED_DELAY_MS / 2));
    }
}


/* ────── WHITE LED Task (Core 1) ────── */
void white_led_task(void *param) {
    
    gpio_num_t pin = WHITE_LED_PIN;
    led_gpio_init(pin);
    while (1) {
        gpio_set_level(pin, 1);
        ESP_LOGI(TAG_WHITE, "LED ON");
        vTaskDelay(pdMS_TO_TICKS(WHITE_LED_DELAY_MS / 2));
        gpio_set_level(pin, 0);
        ESP_LOGI(TAG_WHITE, "LED OFF");
        vTaskDelay(pdMS_TO_TICKS(WHITE_LED_DELAY_MS / 2));
    }
}


/* ────── Initializer ────── */
static esp_err_t initializer(void) {
    ESP_LOGI(TAG, "GPIO initialization...");
    led_gpio_init(RED_LED_PIN);
    led_gpio_init(WHITE_LED_PIN);
    return ESP_OK;
}

void start_led_task(void)
{

    // Create RED LED task on Core 0
    xTaskCreatePinnedToCore(red_led_task, "Red LED Task", LED_TASK_STACK_SIZE, NULL,
                            LED_TASK_PRIORITY, NULL, 0);

    // Create WHITE LED task on Core 1
    xTaskCreatePinnedToCore(white_led_task, "White LED Task", LED_TASK_STACK_SIZE, NULL,
                            LED_TASK_PRIORITY, NULL, 1);
}

/*============================================================================
 * PUBLIC FUNCTION IMPLEMENTATIONS
 *============================================================================*/

/**
 * @brief Print project and system details in banner format
 */
static void print_project_banner(void)
{
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "🔥 Starting ESP32 Dual LED Controller");
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "Project       : ESP32-PICO LED Blinker");
    ESP_LOGI(TAG, "Author        : Rahul B.");
    ESP_LOGI(TAG, "Version       : 2.0");
    ESP_LOGI(TAG, "Board         : ESP32-PICO Development Board");
    ESP_LOGI(TAG, "FreeRTOS      : %s", tskKERNEL_VERSION_NUMBER);
    ESP_LOGI(TAG, "ESP-IDF       : %s", esp_get_idf_version());
    ESP_LOGI(TAG, "================================================");
}

/**
 * @brief Initialize system-level settings and handlers with error handling
 */
static void initialize_system(void)
{
    printf("Software Start...\n");

    // Set log level
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    // Log reset reason
    esp_reset_reason_t reason = esp_reset_reason();
    printf("Reset Reason: %s\n", reset_reasons[reason]);

    // Register shutdown handler
    esp_err_t ret = esp_register_shutdown_handler(FirmwareShutdownHandler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register shutdown handler: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Shutdown handler registered successfully");
    }

    // Initialize system components
    ret = initializer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "System initializer failed: %s", esp_err_to_name(ret));
        esp_restart();
    }

    ESP_LOGD(TAG, "System initialized successfully");
}

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    initialize_system();
    print_project_banner();
    start_led_task();

}
