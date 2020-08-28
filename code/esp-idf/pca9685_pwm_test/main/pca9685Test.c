#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "pca9685.h"

#include "sdkconfig.h"

#define I2C_EXAMPLE_MASTER_SCL_IO   22    /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO   23    /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_FREQ_HZ  100000     /*!< I2C master clock frequency */
#define I2C_EXAMPLE_MASTER_NUM      I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */


#define I2C_ADDRESS     0x40    /*!< lave address for PCA9685 */

#define ACK_CHECK_EN    0x1     /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS   0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL         0x0     /*!< I2C ack value */
#define NACK_VAL        0x1     /*!< I2C nack value */

static char tag[] = "PCA9685";

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

static void i2c_example_master_init(void);

/**
 * @brief i2c master initialization
 */
static void i2c_example_master_init(void)
{
    ESP_LOGD(tag, ">> PCA9685");
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0));
}

void task_PCA9685(void *ignore)
{
    printf("Executing on core %d\n", xPortGetCoreID());

    esp_err_t ret;

    i2c_example_master_init();

    set_pca9685_adress(I2C_ADDRESS);
    resetPCA9685();
    setFrequencyPCA9685(50);  // 1000 Hz

    printf("Finished setup, entering loop now\n");

    while(1)
    {

        vTaskDelay(1000 / portTICK_RATE_MS);

        printf("Left Front Shoulder to default position\n");

        ret = setPWM(2, 0, 290);

        if(ret == ESP_ERR_TIMEOUT)
        {
            printf("I2C timeout\n");
        }
        else if(ret == ESP_OK)
        {
            // all good
        }
        else
        {
            printf("No ack, sensor not connected...skip...\n");
        }

        vTaskDelay(1000/portTICK_PERIOD_MS);

        printf("Move leg up: %d\n", 2);
        setPWM(2, 0, 300);
        

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(task_PCA9685, "task_PCA9685", 1024 * 2, (void* ) 0, 10, NULL);
}

