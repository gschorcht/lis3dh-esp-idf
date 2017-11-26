/**
 * Simple example with one sensor connected to I2C or SPI. It demonstrates the
 * different approaches to fetch the data. Either one of the interrupt signals
 * is used or new data are fetched periodically.
 *
 * Harware configuration:
 *
 *   I2C   +-------------------------+     +----------+
 *         | ESP8266  Bus 0          |     | LIS3DH   |
 *         |          GPIO 5 (SCL)   ------> SCL      |
 *         |          GPIO 4 (SDA)   ------- SDA      |
 *         |          GPIO 13        <------ INT1     |
 *         +-------------------------+     +----------+
 *
 *         +-------------------------+     +----------+
 *         | ESP32    Bus 0          |     | LIS3DH   |
 *         |          GPIO 16 (SCL)  >-----> SCL      |
 *         |          GPIO 17 (SDA)  ------- SDA      |
 *         |          GPIO 22        <------ INT1     |
 *         +-------------------------+     +----------+
 *
 *   SPI   +-------------------------+     +----------+
 *         | ESP8266  Bus 1          |     | LIS3DH   |
 *         |          GPIO 14 (SCK)  ------> SCK      |
 *         |          GPIO 13 (MOSI) ------> SDI      |
 *         |          GPIO 12 (MISO) <------ SDO      |
 *         |          GPIO 2  (CS)   ------> CS       |
 *         |          GPIO 5         <------ INT1     |
 *         +-------------------------+     +----------+
 *
 *         +-------------------------+     +----------+
 *         | ESP32    Bus 0          |     | LIS3DH   |
 *         |          GPIO 16 (SCK)  ------> SCK      |
 *         |          GPIO 17 (MOSI) ------> SDI      |
 *         |          GPIO 18 (MISO) <------ SDO      |
 *         |          GPIO 19 (CS)   ------> CS       |
 *         |          GPIO 22        <------ INT1     |
 *         +-------------------------+     +----------+
 */

// use following constants to define the example mode

// #define SPI_USED
   #define DATA_INT     // data ready and FIFO status interrupts
// #define EVENT_INT    // wake-up, free fall or 6D/4D orientation detection 
// #define CLICK_INT    // click detection interrupt
   #define FIFO_MODE    // multiple sample read mode

#include <string.h>

/* -- platform dependent includes ----------------------------- */

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp8266_wrapper.h"

#include "lis3dh.h"

#else  // ESP8266 (esp-open-rtos)

#define TASK_STACK_DEPTH 256

#include <stdio.h>

#include "espressif/esp_common.h"
#include "espressif/sdk_private.h"

#include "esp/uart.h"
#include "i2c/i2c.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "lis3dh/lis3dh.h"

#endif  // ESP_PLATFORM

/** -- platform dependent definitions ------------------------------ */

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

// user task stack depth
#define TASK_STACK_DEPTH 4096

// define SPI interface for LIS3DH  sensors
#define SPI_BUS       HSPI_HOST
#define SPI_SCK_GPIO  16
#define SPI_MOSI_GPIO 17
#define SPI_MISO_GPIO 18
#define SPI_CS_GPIO   19

// define I2C interfaces for LIS3DH  sensors
#define I2C_BUS       0
#define I2C_SCL_PIN   16
#define I2C_SDA_PIN   17
#define I2C_FREQ      400000

// define GPIOs for interrupt
#define INT1_PIN      22
#define INT2_PIN      23

#else  // ESP8266 (esp-open-rtos)

// user task stack depth
#define TASK_STACK_DEPTH 256

// define SPI interface for LIS3DH  sensors
#define SPI_BUS       1
#define SPI_CS_GPIO   2   // GPIO 15, the default CS of SPI bus 1, can't be used

// define I2C interfaces for LIS3DH  sensors
#define I2C_BUS       0
#define I2C_SCL_PIN   5
#define I2C_SDA_PIN   4
#define I2C_FREQ      I2C_FREQ_100K

// define GPIOs for interrupt
#ifdef SPI_USED
#define INT1_PIN      5
#define INT2_PIN      4
#else
#define INT1_PIN      13
#define INT2_PIN      12
#endif  // SPI_USED

#endif  // ESP_PLATFORM

/* -- user tasks ---------------------------------------------- */

static lis3dh_sensor_t* sensor;

/**
 * Common function used to get sensor data.
 */
void read_data ()
{
    #ifdef FIFO_MODE

    lis3dh_float_data_fifo_t fifo;

    if (lis3dh_new_data (sensor))
    {
        uint8_t num = lis3dh_get_float_data_fifo (sensor, fifo);

        printf("%.3f LIS3DH num=%d\n", (double)sdk_system_get_time()*1e-3, num);

        for (int i=0; i < num; i++)
            // max. full scale is +-16 g and resolution is 1 mg, i.e. 5 digits
            printf("%.3f LIS3DH (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
                   (double)sdk_system_get_time()*1e-3, 
                   fifo[i].ax, fifo[i].ay, fifo[i].az);
    }

    #else

    lis3dh_float_data_t  data;

    if (lis3dh_new_data (sensor) &&
        lis3dh_get_float_data (sensor, &data))
        // max. full scale is +-16 g and resolution is 1 mg, i.e. 5 digits
        printf("%.3f LIS3DH (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
               (double)sdk_system_get_time()*1e-3, 
                data.ax, data.ay, data.az);
        
    #endif // FIFO_MODE
}


#if defined(DATA_INT) || defined(EVENT_INT) || defined(CLICK_INT)
/**
 * In this case, any of the possible interrupts on interrupt signal *INT1* is
 * used to fetch the data.
 *
 * When interrupts are used, the user has to define interrupt handlers that
 * either fetches the data directly or triggers a task which is waiting to
 * fetch the data. In this example, the interrupt handler sends an event to
 * a waiting task to trigger the data gathering.
 */

static QueueHandle_t gpio_evt_queue = NULL;

// User task that fetches the sensor values.

void user_task_interrupt (void *pvParameters)
{
    uint32_t gpio_num;

    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
        {
            lis3dh_int_event_source_t event_src;
            lis3dh_int_data_source_t  data_src;
            lis3dh_int_click_source_t click_src;

            // get the source of the interrupt and reset INT signals
            lis3dh_get_int_event_source (sensor, &event_src, lis3dh_int1_signal);
            lis3dh_get_int_data_source  (sensor, &data_src);
            lis3dh_get_int_click_source (sensor, &click_src);
    
            // in case of DRDY interrupt or event interrupt read one data sample
            if (data_src.data_ready || event_src.active)
                read_data ();
   
            // in case of FIFO interrupts read the whole FIFO
            else  if (data_src.fifo_watermark || data_src.fifo_overrun)
                read_data ();
    
            else if (click_src.active)
               printf("%.3f LIS3DH %s\n", (double)sdk_system_get_time()*1e-3, 
                      click_src.s_click ? "single click" : "double click");
        }
    }
}

// Interrupt handler which resumes user_task_interrupt on interrupt

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)
static void IRAM_ATTR int_signal_handler(void* arg)
{
    uint32_t gpio = (uint32_t) arg;
#else  // ESP8266 (esp-open-rtos)
void int_signal_handler (uint8_t gpio)
{
#endif
    // send an event with GPIO to the interrupt user task
    xQueueSendFromISR(gpio_evt_queue, &gpio, NULL);
}

#else

/*
 * In this example, user task fetches the sensor values every seconds.
 */

void user_task_periodic(void *pvParameters)
{
    vTaskDelay (10);
    
    while (1)
    {
        // read sensor data
        read_data ();
        
        // passive waiting until 1 second is over
        vTaskDelay(100);
    }
}

#endif

/* -- main program ---------------------------------------------- */

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)
void app_main()
#else  // ESP8266 (esp-open-rtos)
void user_init(void)
#endif
{
    #ifdef ESP_OPEN_RTOS  // ESP8266
    // Set UART Parameter.
    uart_set_baud(0, 115200);
    #endif

    vTaskDelay(1);

    /** -- MANDATORY PART -- */

    #ifdef SPI_USED

    // init the sensor connnected to SPI
    #ifdef ESP_OPEN_RTOS
    sensor = lis3dh_init_sensor (SPI_BUS, 0, SPI_CS_GPIO);
    #else
    spi_bus_config_t spi_bus_cfg = {
        .miso_io_num=SPI_MISO_GPIO,
        .mosi_io_num=SPI_MOSI_GPIO,
        .sclk_io_num=SPI_SCK_GPIO,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    if (spi_bus_initialize(SPI_BUS, &spi_bus_cfg, 1) == ESP_OK)
        sensor = lis3dh_init_sensor (SPI_BUS, 0, SPI_CS_GPIO);
    #endif
    
    #else

    // init all I2C bus interfaces at which LIS3DH  sensors are connected
    i2c_init (I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);
    
    // init the sensor with slave address LIS3DH_I2C_ADDRESS_2 connected I2C_BUS.
    sensor = lis3dh_init_sensor (I2C_BUS, LIS3DH_I2C_ADDRESS_1, 0);

    #endif
    
    if (sensor)
    {
        // --- PLATFORM DEPENDENT PART ----
        
        #if !defined(DATA_INT) && !defined(EVENT_INT) && !defined(CLICK_INT)

        // create a user task that fetches data from sensor periodically
        xTaskCreate(user_task_periodic, "user_task_periodic", TASK_STACK_DEPTH, NULL, 2, NULL);

        #else // INT1_USED || INT2_USED

        // create a task that is triggered only in case of interrupts to fetch the data
        xTaskCreate(user_task_interrupt, "user_task_interrupt", TASK_STACK_DEPTH, NULL, 2, NULL);

        // create event queue
        gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

        #ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)
        
        // configure interupt pins for *INT1* and *INT2* signals
        gpio_config_t gpio_cfg = {
            .pin_bit_mask = ((uint64_t)(((uint64_t)1)<< INT1_PIN)),
            .mode = GPIO_MODE_INPUT,
            .pull_down_en = true,
            .intr_type = GPIO_INTR_POSEDGE
        };
        gpio_config(&gpio_cfg);

        // set interrupt handler
        gpio_install_isr_service(0);
        gpio_isr_handler_add(INT1_PIN, int_signal_handler, (void*)INT1_PIN);

        #else  // ESP8266 (esp-open-rtos)

        // configure interupt pins for *INT1* and *INT2* signals and set the interrupt handler
        gpio_set_interrupt(INT1_PIN, GPIO_INTTYPE_EDGE_POS, int_signal_handler);

        #endif  // ESP_PLATFORM 
        
        #endif  // !defined(INT1_USED) && !defined(INT2_USED)
        
        // -- SENSOR CONFIGURATION PART ---

        // Interrupt configuration has to be done before the sensor is set
        // into measurement mode
        
        // set polarity of INT signals if necessary
        // lis3dh_config_int_signals (sensor, lis3dh_high_active);

        #ifdef DATA_INT
        // enable data interrupts on INT1 (data ready or FIFO events)
        // data ready and FIFO interrupts must not be enabled at the same time
        #ifdef FIFO_MODE
        lis3dh_enable_int_data (sensor, lis3dh_fifo_overrun, true);
        lis3dh_enable_int_data (sensor, lis3dh_fifo_watermark, true);
        #else
        lis3dh_enable_int_data (sensor, lis3dh_data_ready, true);
        #endif // FIFO_MODE
        #endif // DATA_INT
        
        #ifdef EVENT_INT
        // enable data interrupts on INT1 
        lis3dh_int_event_config_t act_config;
    
        act_config.event = lis3dh_wake_up;
        // act_config.event = lis3dh_free_fall;
        // act_config.event = lis3dh_6d_movement;
        // act_config.event = lis3dh_6d_position;
        // act_config.event = lis3dh_4d_movement;
        // act_config.event = lis3dh_4d_position;
        act_config.threshold = 10;
        act_config.x_low_enabled  = false;
        act_config.x_high_enabled = true;
        act_config.y_low_enabled  = false;
        act_config.y_high_enabled = true;
        act_config.z_low_enabled  = false;
        act_config.z_high_enabled = true;
        act_config.duration = 0;
        act_config.latch = true;
        
        lis3dh_set_int_event_config (sensor, lis3dh_int1_signal, &act_config);
        #endif // EVENT_INT

        #ifdef CLICK_INT
        // enable click interrupt on INT1
        lis3dh_int_click_config_t click_config;
        
        click_config.threshold = 10;
        click_config.x_single = false;
        click_config.x_double = false;        
        click_config.y_single = false;
        click_config.y_double = false;        
        click_config.z_single = true;
        click_config.z_double = false;
        click_config.latch = true;
        click_config.time_limit   = 1;
        click_config.time_latency = 1;
        click_config.time_window  = 3;
        
        lis3dh_set_int_click_config (sensor, lis3dh_int1_signal, &click_config);
        #endif // CLICK_INT

        #ifdef FIFO_MODE
        // clear FIFO and activate FIFO mode if needed
        lis3dh_set_fifo_mode (sensor, lis3dh_bypass,  0, lis3dh_int1_signal);
        lis3dh_set_fifo_mode (sensor, lis3dh_stream, 10, lis3dh_int1_signal);
        #endif

        // configure HPF and reset the reference by dummy read
        lis3dh_config_hpf (sensor, lis3dh_hpf_normal, 0, true, true, true, true);
        lis3dh_get_hpf_ref (sensor);
        
        // enable ADC inputs and temperature sensor for ADC input 3
        lis3dh_enable_adc (sensor, true, true);
        
        // LAST STEP: Finally set scale and mode to start measurements
        lis3dh_set_scale(sensor, lis3dh_scale_2g);
        lis3dh_set_mode (sensor, lis3dh_odr_10, lis3dh_high_res, true, true, true);

        vTaskDelay (20);

        // -- SENSOR CONFIGURATION PART ---
    }
}

