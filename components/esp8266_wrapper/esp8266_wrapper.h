/*
 * Wrapper module for source code compatibility with esp-open-rtos.
 */

#ifndef __ESP8266_WRAPPER_H__
#define __ESP8266_WRAPPER_H__

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/spi_common.h"

/*
 * esp-open-rtos SDK function wrapper 
 */

uint32_t sdk_system_get_time ();

#define GPIO_INTTYPE_NONE       GPIO_INTR_DISABLE
#define GPIO_INTTYPE_EDGE_POS   GPIO_INTR_POSEDGE
#define GPIO_INTTYPE_EDGE_NEG   GPIO_INTR_NEGEDGE
#define GPIO_INTTYPE_EDGE_ANY   GPIO_INTR_ANYEDGE
#define GPIO_INTTYPE_LEVEL_LOW  GPIO_INTR_LOW_LEVEL
#define GPIO_INTTYPE_LEVEL_HIGH GPIO_INTR_HIGH_LEVEL

// Set it true, if isr_service is already installed anywhere else or should
// not be installed, otherwise, *gpio_set_interrupt* install it when it is
// called first time.
extern bool gpio_isr_service_installed;

// pull-up, pull-down configuration used for next call of *gpio_set_interrupt*
extern bool auto_pull_up;   // default false;
extern bool auto_pull_down; // default true;

esp_err_t gpio_set_interrupt(gpio_num_t gpio, 
                             gpio_int_type_t type, 
                             gpio_isr_t handler);

/*
 * I2C interface wrapper
 */

void i2c_init (int bus, gpio_num_t scl, gpio_num_t sda, uint32_t freq);

int i2c_slave_write (uint8_t bus, uint8_t addr, const uint8_t *reg, 
                     uint8_t *data, uint32_t len);

int i2c_slave_read (uint8_t bus, uint8_t addr, const uint8_t *reg, 
                    uint8_t *data, uint32_t len);

/*
 * SPI interface wrapper
 */

bool spi_bus_init (spi_host_device_t host, 
                   uint8_t sclk , uint8_t miso, uint8_t mosi);

bool spi_device_init (uint8_t bus, uint8_t cs);

size_t spi_transfer_pf(uint8_t bus, uint8_t cs, 
                       const uint8_t *mosi, uint8_t *miso, uint16_t len);

/*
 * freertos api wrapper
 */

#define QueueHandle_t xQueueHandle

#ifdef __cplusplus
}
#endif

#endif  // ESP_PLATFORM

#endif  // __ESP8266_WRAPPER_H__
