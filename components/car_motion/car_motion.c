#include "car_motion.h"


#include "stdio.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor.h"


car_motion_t micro_car;

static float g_robot_apb = ROBOT_APB;

// 线速度和角速度
static float line_vx = 0;
static float line_vy = 0;
static float angular_v = 0;

static float speed_L1_setup = 0;
static float speed_L2_setup = 0;
static float speed_R1_setup = 0;
static float speed_R2_setup = 0;


void Motion_Set_Calibration(float robot_width_m, float robot_length_m)
{
    g_robot_apb = (robot_width_m + robot_length_m) / 2.0f;
}

void Motion_Stop(uint8_t brake)
{
    Motor_Stop(brake);
}

// 控制小车运动
// Control car motion
void Motion_Ctrl(float V_x, float V_y, float V_z)
{
    line_vx = V_x;
    line_vy = V_y;
    angular_v = V_z;
    speed_L1_setup = line_vx - line_vy - angular_v * g_robot_apb;
    speed_L2_setup = line_vx + line_vy - angular_v * g_robot_apb;
    speed_R1_setup = line_vx + line_vy + angular_v * g_robot_apb;
    speed_R2_setup = line_vx - line_vy + angular_v * g_robot_apb;
    Motor_Set_Speed(speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);
}

// 获取小车运动的速度
// Get the speed of the car's motion
void Motion_Get_Speed(car_motion_t* car)
{
    float speed_m1 = 0, speed_m2 = 0, speed_m3 = 0, speed_m4 = 0;
    Motor_Get_Speed(&speed_m1, &speed_m2, &speed_m3, &speed_m4);

    car->Vx = (speed_m1 + speed_m2 + speed_m3 + speed_m4) / 4;
    car->Vy = -(speed_m1 - speed_m2 - speed_m3 + speed_m4) / 4;;
    car->Wz = -(speed_m1 + speed_m2 - speed_m3 - speed_m4) / 4.0f / g_robot_apb;
    if(car->Wz == 0) car->Wz = 0;
}

// 控制小车的运动状态
// Control the motion state of the car
void Motion_Ctrl_State(uint8_t state, float speed)
{
    if (speed < 0) speed = -speed;
    if (speed > 1.0) speed = 1.0;
    switch (state)
    {
    case MOTION_STOP:
        Motion_Stop(STOP_COAST);
        break;
    case MOTION_RUN:
        Motion_Ctrl(speed, 0, 0);
        break;
    case MOTION_BACK:
        Motion_Ctrl(-speed, 0, 0);
        break;
    case MOTION_LEFT:
        Motion_Ctrl(0, speed, 0);
        break;
    case MOTION_RIGHT:
        Motion_Ctrl(0, -speed, 0);
        break;
    case MOTION_SPIN_LEFT:
        Motion_Ctrl(0, 0, speed*ROBOT_SPIN_SCALE);
        break;
    case MOTION_SPIN_RIGHT:
        Motion_Ctrl(0, 0, -speed*ROBOT_SPIN_SCALE);
        break;
    case MOTION_BRAKE:
        Motion_Stop(STOP_BRAKE);
        break;
    default:
        break;
    }
}
