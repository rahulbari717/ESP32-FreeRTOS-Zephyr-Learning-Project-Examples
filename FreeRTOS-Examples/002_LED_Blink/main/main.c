/**
 * @file    main.c
 * @brief   ESP32-PICO LED Control with FreeRTOS Multi-Core Tasks
 * @author  Rahul B. 
 * @version 1.0
 * @date    17th July 2025
 * 
 * @description
 * LED controller implementation featuring:
 * - Multi-core task distribution (Core 0 & Core 1)
 * - Independent LED control with different intervals
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"

/*============================================================================
 * PREPROCESSOR DEFINITIONS
 *============================================================================*/

// Hardware GPIO Pin Definitions
#define RED_LED_GPIO_PIN        (2U)    /**< Red LED GPIO pin */
#define WHITE_LED_GPIO_PIN      (4U)    /**< White LED GPIO pin */

// Task Configuration
#define LED_TASK_PRIORITY       (1U)    /**< Task priority level */
#define LED_TASK_STACK_SIZE     (2048U) /**< Stack size in bytes */

// Timing Configuration (in seconds)
#define RED_LED_INTERVAL_SEC    (1U)    /**< Red LED blink interval */
#define WHITE_LED_INTERVAL_SEC  (2U)    /**< White LED blink interval */

// Core Assignment
#define CORE_0                  (0U)    /**< Core 0 identifier */
#define CORE_1                  (1U)    /**< Core 1 identifier */

// LED States
#define LED_STATE_OFF           (0U)    /**< LED OFF state */
#define LED_STATE_ON            (1U)    /**< LED ON state */

// Conversion Macros
#define SECONDS_TO_TICKS(sec)   ((sec) * 1000 / portTICK_PERIOD_MS)

/*============================================================================
 * TYPE DEFINITIONS
 *============================================================================*/

/**
 * @brief LED color enumeration
 */
typedef enum {
    LED_COLOR_RED = 0,      /**< Red LED identifier */
    LED_COLOR_WHITE,        /**< White LED identifier */
    LED_COLOR_MAX           /**< Maximum LED count */
} led_color_t;

/**
 * @brief LED state enumeration
 */
typedef enum {
    LED_OFF = LED_STATE_OFF,    /**< LED is turned off */
    LED_ON = LED_STATE_ON       /**< LED is turned on */
} led_state_t;

/**
 * @brief Core assignment enumeration
 */
typedef enum {
    CORE_ZERO = CORE_0,     /**< Core 0 assignment */
    CORE_ONE = CORE_1       /**< Core 1 assignment */
} core_id_t;

/**
 * @brief LED configuration structure
 * 
 * Contains all necessary parameters for LED control task
 */
typedef struct {
    gpio_num_t gpio_pin;            /**< GPIO pin number */
    uint32_t blink_interval_sec;    /**< Blink interval in seconds */
    const char* led_name;           /**< Human-readable LED name */
    led_color_t led_color;          /**< LED color identifier */
    core_id_t assigned_core;        /**< Assigned processor core */
    bool is_active;                 /**< LED active status */
} led_config_t;

/**
 * @brief Task statistics structure
 */
typedef struct {
    uint32_t blink_count;           /**< Total blink cycles */
    uint32_t uptime_seconds;        /**< Task uptime in seconds */
    core_id_t current_core;         /**< Current executing core */
} led_task_stats_t;

/**
 * @brief Complete LED task parameters
 */
typedef struct {
    led_config_t config;            /**< LED configuration */
    led_task_stats_t stats;         /**< Task statistics */
} led_task_params_t;

/*============================================================================
 * STATIC VARIABLES
 *============================================================================*/

static const char *TAG = "DUAL_LED_CONTROLLER";

/**
 * @brief LED configuration table
 * 
 * Centralized configuration for all LED instances
 */
static led_config_t led_configs[LED_COLOR_MAX] = {
    [LED_COLOR_RED] = {
        .gpio_pin = RED_LED_GPIO_PIN,
        .blink_interval_sec = RED_LED_INTERVAL_SEC,
        .led_name = "RED_LED",
        .led_color = LED_COLOR_RED,
        .assigned_core = CORE_ZERO,
        .is_active = true
    },
    [LED_COLOR_WHITE] = {
        .gpio_pin = WHITE_LED_GPIO_PIN,
        .blink_interval_sec = WHITE_LED_INTERVAL_SEC,
        .led_name = "WHITE_LED",
        .led_color = LED_COLOR_WHITE,
        .assigned_core = CORE_ONE,
        .is_active = true
    }
};

/*============================================================================
 * STATIC FUNCTION PROTOTYPES
 *============================================================================*/

static void led_gpio_initialize(gpio_num_t gpio_pin);
static void led_set_state(gpio_num_t gpio_pin, led_state_t state);
static void led_blink_task(void *pvParameters);
static void print_system_info(void);
static void print_led_configuration(const led_config_t *config);
static const char* get_core_name(core_id_t core_id);
static const char* get_led_state_name(led_state_t state);

/*============================================================================
 * STATIC FUNCTION IMPLEMENTATIONS
 *============================================================================*/

/**
 * @brief Initialize GPIO pin for LED control
 * 
 * @param gpio_pin GPIO pin number to initialize
 */
static void led_gpio_initialize(gpio_num_t gpio_pin)
{
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << gpio_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t result = gpio_config(&gpio_conf);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "GPIO %d initialized successfully", gpio_pin);
    } else {
        ESP_LOGE(TAG, "Failed to initialize GPIO %d: %s", gpio_pin, esp_err_to_name(result));
    }
    
    // Set initial state to OFF
    led_set_state(gpio_pin, LED_OFF);
}

/**
 * @brief Set LED state (ON/OFF)
 * 
 * @param gpio_pin GPIO pin number
 * @param state Desired LED state
 */
static void led_set_state(gpio_num_t gpio_pin, led_state_t state)
{
    gpio_set_level(gpio_pin, (uint32_t)state);
}

/**
 * @brief Get human-readable core name
 * 
 * @param core_id Core identifier
 * @return const char* Core name string
 */
static const char* get_core_name(core_id_t core_id)
{
    switch (core_id) {
        case CORE_ZERO: return "CORE_0";
        case CORE_ONE:  return "CORE_1";
        default:        return "UNKNOWN_CORE";
    }
}

/**
 * @brief Get human-readable LED state name
 * 
 * @param state LED state
 * @return const char* State name string
 */
static const char* get_led_state_name(led_state_t state)
{
    switch (state) {
        case LED_OFF: return "OFF";
        case LED_ON:  return "ON";
        default:      return "UNKNOWN";
    }
}

/**
 * @brief Print system information
 */
static void print_system_info(void)
{
    ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          SYSTEM INFORMATION            ║");
    ESP_LOGI(TAG, "╠════════════════════════════════════════╣");
    ESP_LOGI(TAG, "║ Target: ESP32-PICO Development Board   ║");
    ESP_LOGI(TAG, "║ Available Cores: %d                    ║", portNUM_PROCESSORS);
    ESP_LOGI(TAG, "║ FreeRTOS Version: %s               ║", tskKERNEL_VERSION_NUMBER);
    ESP_LOGI(TAG, "║ Heap Size: %d bytes                ║", esp_get_free_heap_size());
    ESP_LOGI(TAG, "╚════════════════════════════════════════╝");
}

/**
 * @brief Print LED configuration details
 * 
 * @param config LED configuration structure
 */
static void print_led_configuration(const led_config_t *config)
{
    ESP_LOGI(TAG, "┌─ %s Configuration ─────────────────┐", config->led_name);
    ESP_LOGI(TAG, "│ GPIO Pin: %d                           │", config->gpio_pin);
    ESP_LOGI(TAG, "│ Interval: %d seconds                   │", config->blink_interval_sec);
    ESP_LOGI(TAG, "│ Core: %s                           │", get_core_name(config->assigned_core));
    ESP_LOGI(TAG, "│ Status: %s                           │", config->is_active ? "ACTIVE" : "INACTIVE");
    ESP_LOGI(TAG, "└────────────────────────────────────────┘");
}

/**
 * @brief LED blink task implementation
 * 
 * @param pvParameters Task parameters (led_task_params_t*)
 */
static void led_blink_task(void *pvParameters)
{
    if (pvParameters == NULL) {
        ESP_LOGE(TAG, "Invalid task parameters received");
        vTaskDelete(NULL);
        return;
    }
    
    led_task_params_t *params = (led_task_params_t *)pvParameters;
    led_config_t *config = &params->config;
    led_task_stats_t *stats = &params->stats;
    
    // Initialize GPIO
    led_gpio_initialize(config->gpio_pin);
    
    // Initialize statistics
    stats->blink_count = 0;
    stats->uptime_seconds = 0;
    stats->current_core = (core_id_t)xPortGetCoreID();
    
    // Task startup log
    ESP_LOGI(TAG, "🚀 %s task started successfully", config->led_name);
    ESP_LOGI(TAG, "   ├─ Running on %s", get_core_name(stats->current_core));
    ESP_LOGI(TAG, "   ├─ GPIO Pin: %d", config->gpio_pin);
    ESP_LOGI(TAG, "   └─ Blink Interval: %d seconds", config->blink_interval_sec);
    
    // Main task loop
    while (config->is_active) {
        // Turn LED ON
        led_set_state(config->gpio_pin, LED_ON);
        ESP_LOGI(TAG, "💡 %s: %s | Cycle: %d | Core: %s | Uptime: %ds",
                 config->led_name,
                 get_led_state_name(LED_ON),
                 stats->blink_count,
                 get_core_name(stats->current_core),
                 stats->uptime_seconds);
        
        // Wait for specified interval
        vTaskDelay(SECONDS_TO_TICKS(config->blink_interval_sec));
        
        // Turn LED OFF
        led_set_state(config->gpio_pin, LED_OFF);
        ESP_LOGI(TAG, "💡 %s: %s | Cycle: %d | Core: %s | Uptime: %ds",
                 config->led_name,
                 get_led_state_name(LED_OFF),
                 stats->blink_count,
                 get_core_name(stats->current_core),
                 stats->uptime_seconds);
        
        // Wait for specified interval
        vTaskDelay(SECONDS_TO_TICKS(config->blink_interval_sec));
        
        // Update statistics
        stats->blink_count++;
        stats->uptime_seconds += (config->blink_interval_sec * 2); // ON + OFF time
        
        // Periodic statistics log (every 10 cycles)
        if (stats->blink_count % 10 == 0) {
            ESP_LOGI(TAG, "📊 %s Statistics: %d cycles completed, %d seconds uptime",
                     config->led_name, stats->blink_count, stats->uptime_seconds);
        }
    }
    
    // Task cleanup
    ESP_LOGI(TAG, "🛑 %s task terminated", config->led_name);
    led_set_state(config->gpio_pin, LED_OFF);
    vTaskDelete(NULL);
}

/*============================================================================
 * PUBLIC FUNCTION IMPLEMENTATIONS
 *============================================================================*/

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "🔥 Starting ESP32 Dual LED Controller");
    ESP_LOGI(TAG, "=====================================");
    
    // Print system information
    print_system_info();
    
    // Task parameters for both LEDs
    static led_task_params_t led_task_params[LED_COLOR_MAX];
    
    // Initialize task parameters
    for (int i = 0; i < LED_COLOR_MAX; i++) {
        led_task_params[i].config = led_configs[i];
        memset(&led_task_params[i].stats, 0, sizeof(led_task_stats_t));
    }
    
    // Print LED configurations
    ESP_LOGI(TAG, "\n📋 LED CONFIGURATION DETAILS:");
    for (int i = 0; i < LED_COLOR_MAX; i++) {
        print_led_configuration(&led_configs[i]);
    }
    
    // Create RED LED task on Core 0
    BaseType_t red_task_result = xTaskCreatePinnedToCore(
        led_blink_task,                    // Task function
        "RED_LED_TASK",                    // Task name
        LED_TASK_STACK_SIZE,               // Stack size
        &led_task_params[LED_COLOR_RED],   // Task parameters
        LED_TASK_PRIORITY,                 // Priority
        NULL,                              // Task handle
        CORE_0                             // Core ID
    );
    
    // Create WHITE LED task on Core 1
    BaseType_t white_task_result = xTaskCreatePinnedToCore(
        led_blink_task,                      // Task function
        "WHITE_LED_TASK",                    // Task name
        LED_TASK_STACK_SIZE,                 // Stack size
        &led_task_params[LED_COLOR_WHITE],   // Task parameters
        LED_TASK_PRIORITY,                   // Priority
        NULL,                                // Task handle
        CORE_1                               // Core ID
    );
    
    // Verify task creation
    if (red_task_result == pdPASS) {
        ESP_LOGI(TAG, "✅ RED LED task created successfully on Core 0");
    } else {
        ESP_LOGE(TAG, "❌ Failed to create RED LED task");
    }
    
    if (white_task_result == pdPASS) {
        ESP_LOGI(TAG, "✅ WHITE LED task created successfully on Core 1");
    } else {
        ESP_LOGE(TAG, "❌ Failed to create WHITE LED task");
    }
    
    // Final status
    if (red_task_result == pdPASS && white_task_result == pdPASS) {
        ESP_LOGI(TAG, "🎉 All LED tasks initialized successfully!");
        ESP_LOGI(TAG, "================================================");
        ESP_LOGI(TAG, "🔴 RED LED   -> GPIO %d | %ds interval | Core 0", 
                 RED_LED_GPIO_PIN, RED_LED_INTERVAL_SEC);
        ESP_LOGI(TAG, "⚪ WHITE LED -> GPIO %d | %ds interval | Core 1", 
                 WHITE_LED_GPIO_PIN, WHITE_LED_INTERVAL_SEC);
        ESP_LOGI(TAG, "================================================");
    } else {
        ESP_LOGE(TAG, "💥 Task creation failed! System halted.");
    }
    
    // Delete main task as LED control tasks are now independent
    ESP_LOGI(TAG, "🏁 Main task completed. LED control tasks are now autonomous.");
    vTaskDelete(NULL);
}
