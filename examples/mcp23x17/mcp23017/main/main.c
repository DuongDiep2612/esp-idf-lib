#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mcp23x17.h>
#include <driver/gpio.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "COUNTER";
xQueueHandle icounter_queue;
int16_t countSensor = 0;

void IRAM_ATTR intr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    ets_printf("Interupt in pin %d", gpio_num);
    xQueueSendFromISR(icounter_queue, &gpio_num, NULL);
}

void test(void *pvParameters)
{
    mcp23x17_t dev;
    memset(&dev, 0, sizeof(mcp23x17_t));

    icounter_queue = xQueueCreate(100, sizeof(uint32_t));

    ESP_ERROR_CHECK(mcp23x17_init_desc(&dev, CONFIG_EXAMPLE_I2C_ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // Setup PORTB (12-15) as output
    mcp23x17_set_mode(&dev, 15, MCP23X17_GPIO_OUTPUT);
    mcp23x17_set_mode(&dev, 14, MCP23X17_GPIO_OUTPUT);
    mcp23x17_set_mode(&dev, 13, MCP23X17_GPIO_OUTPUT);
    mcp23x17_set_mode(&dev, 12, MCP23X17_GPIO_OUTPUT);

    // Setup PORTA(0-8), PORTB(9-11) as input
    // Setup PORTB (12-15) as output
    mcp23x17_set_mode(&dev, 0, MCP23X17_GPIO_INPUT);
    mcp23x17_set_mode(&dev, 1, MCP23X17_GPIO_INPUT);
    mcp23x17_set_mode(&dev, 2, MCP23X17_GPIO_INPUT);
    mcp23x17_set_mode(&dev, 3, MCP23X17_GPIO_INPUT);
    mcp23x17_set_mode(&dev, 4, MCP23X17_GPIO_INPUT);
    mcp23x17_set_mode(&dev, 5, MCP23X17_GPIO_INPUT);
    mcp23x17_set_mode(&dev, 6, MCP23X17_GPIO_INPUT);
    mcp23x17_set_mode(&dev, 7, MCP23X17_GPIO_INPUT);
    mcp23x17_set_mode(&dev, 8, MCP23X17_GPIO_INPUT);
    mcp23x17_set_mode(&dev, 9, MCP23X17_GPIO_INPUT);
    mcp23x17_set_mode(&dev, 10, MCP23X17_GPIO_INPUT);
    mcp23x17_set_mode(&dev, 11, MCP23X17_GPIO_INPUT);

    // Setup interrupt on it
    mcp23x17_set_interrupt(&dev, 0, MCP23X17_INT_HIGH_EDGE);
    mcp23x17_set_interrupt(&dev, 10, MCP23X17_INT_HIGH_EDGE);
    mcp23x17_set_interrupt(&dev, 11, MCP23X17_INT_HIGH_EDGE);
    mcp23x17_set_interrupt(&dev, 3, MCP23X17_INT_HIGH_EDGE);
    gpio_set_direction(CONFIG_EXAMPLE_INTA_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(CONFIG_EXAMPLE_INTA_GPIO, GPIO_INTR_POSEDGE);
    gpio_set_direction(32, GPIO_MODE_INPUT);
    gpio_set_intr_type(32, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CONFIG_EXAMPLE_INTA_GPIO, intr_handler, NULL);
    gpio_isr_handler_add(32, intr_handler, NULL);

    // Setup PORTB0 as output
    // mcp23x17_set_mode(&dev, 8, MCP23X17_GPIO_OUTPUT);
    // do some blinking
    bool on = true;
    while (1)
    {
        mcp23x17_set_level(&dev, 15, on);
        mcp23x17_set_level(&dev, 14, on);
        mcp23x17_set_level(&dev, 13, on);
        mcp23x17_set_level(&dev, 12, on);
        on = !on;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void counter_task2(void)
{
    uint32_t io_num;
    while (1)
    {
        ESP_LOGI(TAG, "Current counter value start .....:%d", countSensor);
        if (xQueueReceive(icounter_queue, &io_num, 10))
        {
            switch (io_num)
            {
                case 27:
                    countSensor++;
                    ESP_LOGI(TAG, "Current counter value :%d", countSensor);
                    break;
                default:
                    break;
            }
            ESP_LOGI(TAG, "Current counter value on pin :%d", io_num);
        }
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
    xTaskCreate(counter_task2, "counter_task2", 2048, NULL, 6, NULL);
}
