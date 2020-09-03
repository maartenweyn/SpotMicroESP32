/*
Test IK implementation
Author Maarten Weyn
*/

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>


#include "esp_dsp.h"

#include "pca9685.h"
#include "i2c_app.h"
#include "sdkconfig.h"
#include "spot_ik.h"

#include "servo.h"
#include "config.h"

static char tag[] = "IKTEST";

// const servo_settings_t servo_settings[12] = {{150, 400, 1}, {130, 420, 0}, {125, 430, 1},
//                                           {150, 510, 0}, {100, 480, 1}, {130, 505, 0},
//                                           {130, 500, 0}, {90, 490, 0}, {125, 430, 1},
//                                           {120, 410, 1}, {150, 430, 1}, {145, 440, 0}};

const int16_t servo_min[12] = {163,153,151,121,126,124,131,73,125,143,185,140};
const float servo_conversion[12] = {1.927778,1.427778,1.705556,2.050000,2.050000,2.061111,2.038889,2.177778,1.650000,1.750000,2.233333,1.694444};
                                //LF     //RF   //LB    //RB
const int8_t servo_invert[12] = {1,0,1, 0,1,0,  0,0,1,  1,1,0};
const float theta_range[3][2] = {{-M_PI / 3, M_PI/3}, {-2 * M_PI/3, M_PI/3}, {0, M_PI}};

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

/**
 * @brief pac 9685 initialization
 */
void init_pca9685() {
    i2c_example_master_init();
    set_pca9685_adress(I2C_ADDRESS);
    resetPCA9685();
    setFrequencyPCA9685(50); 
}

float omega = 0; //Rx
float psi = 0;  // Rz
float height = 200;

int16_t servo_angles[4][3] = {0,};
float p[4][3] = {{-L1, -L3-L2, L4},{-L1, -L3-L2, L4},{-L1, -L3-L2, L4},{-L1, -L3-L2, L4}};

#define RAD2DEGREES 57.295779513082321 // 180 / PI



void test_servo() {
    esp_err_t ret = 0;

    int16_t start = 60;
    int16_t end = 120;
    int16_t step = 5;
    int16_t current = (start+end) / 2;
    while(1)
    {
        vTaskDelay(500 / portTICK_RATE_MS);

        //ret = set_servo(2, current);
        //ret = set_servo(8, 180-current);

        if(ret == ESP_FAIL) ESP_LOGD(tag, "set servo error");

        current += step;
        if (current > end) {
            step *= -1;
            current+= 2 * step;
        } else if (current < start) {
            step *= -1;
            current+= 2 * step;
        }
    }
}

void calculate_foot_points() {
    float tan_omega = tan(omega);
    float tan_psi = tan(psi);
    // Front has impact of omega
    
    float h_offset = (W/2.0 + L1) * tan_omega;

    //Front Left Leg
    p[0][1]= - (height - h_offset);
    p[0][0] = -L1 + p[0][1]* tan_omega ;
    p[0][2] = p[0][1]* tan_psi;

    // Front Right leg
    p[1][1]= - (height + h_offset);
    p[1][0] = L1 - p[1][1]* tan_omega ;
    p[1][2] = p[1][1]* tan_psi;

    // Rear correct based on psi
    float height_rear = height + L * tan_psi;
    //Front Left Leg
    p[2][1]= - (height_rear - h_offset);
    p[2][0] = -L1 + p[2][1]* tan_omega;
    p[2][2] = p[2][1]* tan_psi;

    // Front Right leg
    p[3][1]= - (height_rear + h_offset);
    p[3][0] = L1 - p[3][1]* tan_omega ;
    p[3][2] = p[3][1]* tan(psi);

    ESP_LOGI(tag, "calculate_foot_points ( onega %d, psi %d, height %.1f mm):", (int) (RAD2DEGREES * omega), (int) (RAD2DEGREES * psi),  height);
    ESP_LOGI(tag, "Front Left Leg (x,z,y) (%.1f,%.1f,%.1f)",  p[0][0], p[0][2], p[0][1]);
    ESP_LOGI(tag, "Front Right Leg (x,z,y) (%.1f,%.1f,%.1f)", p[1][0], p[1][2], p[1][1]);
    ESP_LOGI(tag, "Rear Left Leg (x,z,y) (%.1f,%.1f,%.1f)",   p[2][0], p[2][2], p[2][1]);
    ESP_LOGI(tag, "Rear Right Leg (x,z,y) (%.1f,%.1f,%.1f)",  p[3][0], p[3][2], p[3][1]);
}

esp_err_t calculate_leg_positions() {
    for (int l = 0; l<4; l++) {
        esp_err_t ret = leg_IK(p[l], l, servo_angles[l]);
        ESP_LOGI(tag, "IK (x,z,y) (%.1f, %.1f, %.1f) -> (%d, %d, %d) (%d)", p[l][0], p[l][2], p[l][1], servo_angles[l][0], servo_angles[l][1], servo_angles[l][2], ret);
        if (ret != ESP_OK) return ESP_FAIL;
    }

    return ESP_OK;
}

void set_leg_servos() {
    //for (int l = 0; l<4; l++) {
    for (int l = 1; l<2; l++) {    
        for (int s=0;s<3;s++) {
            set_servo(l*3 + s, servo_angles[l][s]);
        }
    }
}

inline void set_foot_position(uint8_t leg_id, float x, float z, float y) {
    p[leg_id][0] = x;
    p[leg_id][2] = z;
    p[leg_id][1]= y;
}

void sleep_position()
{
    // // sleep position
    // set_servo(0, 90);
    // set_servo(1, 150);
    // set_servo(2, 0);

    // set_servo(3, 90);
    // set_servo(4, 30);
    // set_servo(5, 180);

    // set_servo(6, 90);
    // set_servo(7, 150);
    // set_servo(8, 0);

    // set_servo(9, 90);
    // set_servo(10, 30);
    // set_servo(11, 180);

    set_foot_position(LEG_LF, -L1, L4-L3, -L2);
    set_foot_position(LEG_RF, L1, L4-L3, -L2);  
    set_foot_position(LEG_LB, -L1, L4-L3, -L2);  
    set_foot_position(LEG_RB, L1, L4-L3, -L2);  
    esp_err_t ret = calculate_leg_positions();
    if (ret == ESP_OK) set_leg_servos();
}

void reset_position() {
    omega = 0;
    psi = 0;
    height = 100;

    calculate_foot_points();
    esp_err_t ret = calculate_leg_positions();
    if (ret == ESP_OK) set_leg_servos();
    // for (int l = 0; l<4; l++) {
    //     p[l][0] = -L1;
    //     p[l][1]= height;
    //     p[l][2] = 0;
    //     esp_err_t ret = leg_IK(p[l], l, servo_angles[l]);
    //     SP_LOGI(tag, "IK (%d, %d, %d) -> (%d, %d, %d) (%d)", p[l][0], p[l][1], p[l][2], servo_angles[l][0], servo_angles[l][1], servo_angles[l][2], ret);
    //     for (int s=0;s<3;s++)
    //         if (ret==ESP_OK) set_servo(l*3 + s, servo_angles[l][s]);
    // }
    

}

esp_err_t set_legs() {
    calculate_foot_points();
    esp_err_t ret = calculate_leg_positions();
    if (ret == ESP_OK) set_leg_servos();

    return ret;
}

void task_walk(void *ignore)
{
    ESP_LOGI(tag, "Executing on core %d", xPortGetCoreID());
    esp_err_t ret;

    //sleep_position();

    reset_position();
    
    // for (int l=0; l<4; l++) {
    //     esp_err_t ret = leg_IK(p[l], l, servo_angles[l]);
    //     ESP_LOGI(tag, "IK (%d, %d, %d) -> (%d, %d, %d) (%d)", p[l][0], p[l][1], p[l][2], servo_angles[l][0], servo_angles[l][1], servo_angles[l][2], ret);
    //     for (int s=0;s<3;s++)
    //         if (ret==ESP_OK) set_servo(l*3 + s, servo_angles[l][s]);
    // }

    while(1)
    {
        vTaskDelay(2000 / portTICK_RATE_MS);

        for (int i = 0; i < 10; i++) {
            height -= 2;
            ret = set_legs();
            vTaskDelay(10 / portTICK_RATE_MS);
        }


        vTaskDelay(1000 / portTICK_RATE_MS);

        for (int i = 0; i < 10; i++) {
            height += 2;
            ret = set_legs();
            vTaskDelay(10 / portTICK_RATE_MS);
        }

        

    }

    vTaskDelete(NULL);
}

void app_main()
{
    init_pca9685();
    init_servos();

    // ESP_LOGI(tag, "Start Example.");

    // float A[2][3] = {{0, 1, 0} , {0, 1, 2}};
    // float B[3][2] = {{0, 0}, {1, 1}, {0, 1}};
    // float C[2][2];
    // dspm_mult_f32_ae32((float*) A, (float*) B,  (float*) C, 2, 2, 2);

    // ESP_LOGD(tag, "%.2f %.2f", C[0][0], C[0][1]);
    // ESP_LOGD(tag, "%.2f %.2f", C[1][0], C[1][1]);

    esp_err_t ret = spot_IK(0, 0, 0, 0, 0, 20, servo_angles);

    ESP_LOGD(tag, "Valid IK %d", ret==ESP_OK);
    print_int_matrix((int16_t*) servo_angles, 4, 3, "servo_angles");

    while(1) {};

    xTaskCreate(task_walk, "task_walk", 1024 * 2, (void* ) 0, 10, NULL);
}

