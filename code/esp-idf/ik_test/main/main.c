/*
Test IK implementation
Author Maarten Weyn
*/

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "pca9685.h"
#include "i2c_app.h"
#include "sdkconfig.h"

#include "servo.h"
#include "config.h"

static char tag[] = "IKTEST";

// const servo_settings_t servo_settings[12] = {{150, 400, 1}, {130, 420, 0}, {125, 430, 1},
//                                           {150, 510, 0}, {100, 480, 1}, {130, 505, 0},
//                                           {130, 500, 0}, {90, 490, 0}, {125, 430, 1},
//                                           {120, 410, 1}, {150, 430, 1}, {145, 440, 0}};

const int16_t servo_min[12] = {163,153,151,121,126,124,131,73,125,143,185,140};
const float servo_conversion[12] = {1.927778,1.427778,1.705556,2.050000,2.050000,2.061111,2.038889,2.177778,1.650000,1.750000,2.233333,1.694444};
const int8_t servo_invert[12] = {1,0,1, 0,1,0, 0,0,1,  1,1,0};

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

typedef struct {
    float x;
    float y;
    float z;
} point;

float omega = 0; //Rx
float psi = 0;  // Rz
float height = 200;

int16_t servo_angles[3][4] = {0,};
point p[4] = {{-L1, -L3-L2, L4},{-L1, -L3-L2, L4},{-L1, -L3-L2, L4},{-L1, -L3-L2, L4}};

#define RAD2DEGREES 57.295779513082321 // 180 / PI

esp_err_t leg_IK(point p, uint8_t leg_id, int16_t servo_angles[3]) {
    float x = (leg_id == FR || leg_id == RR) ? -p.x : p.x;
    // Rxy Lenght of shoulder-point on x/y plane only
    float Rxy = sqrt(p.x*p.x + p.y*p.y - L1L1);
    // Dxy Distance we need to cover in xy plane
    float Dxy = Rxy - L2;
    // Dxyz 3D Distance we need to cover
    float Dxyz = sqrt(Dxy*Dxy+p.z*p.z);

    // ESP_LOGD(tag, "IK Rxy,Dxy,Dxyz %.2f %.2f %.2f", Rxy, Dxy, Dxyz);

    // The angle we need to cover - the angle already covered because of the offset of the servo
    float theta1 = -atan2(p.y, x) - atan2(Rxy, -L1);
    if (isnan(theta1)) return -1;

    servo_angles[0] = 90 + (int16_t) (theta1 * RAD2DEGREES);
    if (servo_invert[leg_id*3]) servo_angles[0] = 180 - servo_angles[0];

    float d=(Dxyz*Dxyz - L3L3 - L4L4)/(LL34);
    float theta3 = acos(d);
    if (isnan(theta3)) return -3;

    servo_angles[2] = (int16_t) (theta3 * RAD2DEGREES);
    if (servo_invert[leg_id*3 + 2]) servo_angles[2] = 180 - servo_angles[2];

    // ESP_LOGD(tag, "IK D %.2f, T3 %.2f -> %d", d, theta3, servo_angles[2]);

    float theta2 = atan2(p.z, Dxy) - atan2(L4 * sin(theta3), L3 + L4 * cos(theta3));
    if (isnan(theta2)) return -2;

    servo_angles[1] = 60 - (int16_t) (theta2 * RAD2DEGREES);
    if (servo_invert[leg_id*3 + 1]) servo_angles[1] = 180 - servo_angles[1];
    if (servo_angles[1] < 0) servo_angles[1] += 360;
    if (servo_angles[1] > 360) servo_angles[1] -= 360;

    // ESP_LOGD(tag, "IK %.2f - atan2( %.2f, %.2f) (%.2f)", atan2(p.z, Dxy), L4 * sin(theta3), L3 + L4 * cos(theta3), atan2(L4 * sin(theta3), L3 + L4 * cos(theta3)));
    // ESP_LOGD(tag, "IK T2 %.2f -> %d", theta2, servo_angles[1]);

    return ESP_OK;
}

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
    p[0].y = - (height - h_offset);
    p[0].x = -L1 + p[0].y * tan_omega ;
    p[0].z = p[0].y * tan_psi;

    // Front Right leg
    p[1].y = - (height + h_offset);
    p[1].x = L1 - p[1].y * tan_omega ;
    p[1].z = p[1].y * tan_psi;

    // Rear correct based on psi
    float height_rear = height + L * tan_psi;
    //Front Left Leg
    p[2].y = - (height_rear - h_offset);
    p[2].x = -L1 + p[2].y * tan_omega;
    p[2].z = p[2].y * tan_psi;

    // Front Right leg
    p[3].y = - (height_rear + h_offset);
    p[3].x = L1 - p[3].y * tan_omega ;
    p[3].z = p[3].y * tan(psi);

    ESP_LOGI(tag, "calculate_foot_points ( onega %d, psi %d, height %.1f mm):", (int) (RAD2DEGREES * omega), (int) (RAD2DEGREES * psi),  height);
    ESP_LOGI(tag, "Front Left Leg (x,z,y) (%.1f,%.1f,%.1f)",  p[0].x, p[0].z, p[0].y);
    ESP_LOGI(tag, "Front Right Leg (x,z,y) (%.1f,%.1f,%.1f)", p[1].x, p[1].z, p[1].y);
    ESP_LOGI(tag, "Rear Left Leg (x,z,y) (%.1f,%.1f,%.1f)",   p[2].x, p[2].z, p[2].y);
    ESP_LOGI(tag, "Rear Right Leg (x,z,y) (%.1f,%.1f,%.1f)",  p[3].x, p[3].z, p[3].y);
}

esp_err_t calculate_leg_positions() {
    for (int l = 0; l<4; l++) {
        esp_err_t ret = leg_IK(p[l], l, servo_angles[l]);
        ESP_LOGI(tag, "IK (x,z,y) (%.1f, %.1f, %.1f) -> (%d, %d, %d) (%d)", p[l].x, p[l].z, p[l].y, servo_angles[l][0], servo_angles[l][1], servo_angles[l][2], ret);
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
    p[leg_id].x = x;
    p[leg_id].z = z;
    p[leg_id].y = y;
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

    set_foot_position(FL, -L1, L4-L3, -L2);
    set_foot_position(FR, L1, L4-L3, -L2);  
    set_foot_position(RL, -L1, L4-L3, -L2);  
    set_foot_position(RR, L1, L4-L3, -L2);  
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
    //     p[l].x = -L1;
    //     p[l].y = height;
    //     p[l].z = 0;
    //     esp_err_t ret = leg_IK(p[l], l, servo_angles[l]);
    //     SP_LOGI(tag, "IK (%d, %d, %d) -> (%d, %d, %d) (%d)", p[l].x, p[l].y, p[l].z, servo_angles[l][0], servo_angles[l][1], servo_angles[l][2], ret);
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
    //     ESP_LOGI(tag, "IK (%d, %d, %d) -> (%d, %d, %d) (%d)", p[l].x, p[l].y, p[l].z, servo_angles[l][0], servo_angles[l][1], servo_angles[l][2], ret);
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

    xTaskCreate(task_walk, "task_walk", 1024 * 2, (void* ) 0, 10, NULL);
}

