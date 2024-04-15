#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>

#include "lidar_ms200.h"
#include "ms200.h"
#include "uart1.h"
#include "icm42670p.h"
#include "car_motion.h"
#include "beep.h"


#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }


#define ROS_NAMESPACE      CONFIG_MICRO_ROS_NAMESPACE
#define ROS_DOMAIN_ID      CONFIG_MICRO_ROS_DOMAIN_ID
#define ROS_AGENT_IP       CONFIG_MICRO_ROS_AGENT_IP
#define ROS_AGENT_PORT     CONFIG_MICRO_ROS_AGENT_PORT



static const char *TAG = "MAIN";

rcl_publisher_t publisher_imu;
sensor_msgs__msg__Imu msg_imu;
rcl_timer_t timer_imu;

rcl_publisher_t publisher_odom;
nav_msgs__msg__Odometry msg_odom;
rcl_timer_t timer_odom;

rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;

rcl_subscription_t buzzer_subscriber;
std_msgs__msg__UInt16 msg_beep;

rcl_publisher_t publisher_lidar;
sensor_msgs__msg__LaserScan msg_lidar;
rcl_timer_t timer_lidar;


unsigned long long time_offset = 0;
unsigned long prev_odom_update = 0;

float x_pos_ = 0.0;
float y_pos_ = 0.0;
float heading_ = 0.0;

car_motion_t car_motion;

// Initializes the ROS topic information for lidar
void lidar_ros_init(void)
{
    int i;

    msg_lidar.angle_min = -180*M_PI/180.0;
    msg_lidar.angle_max = 180*M_PI/180.0;

    msg_lidar.angle_increment = 1*M_PI/180.0;
    msg_lidar.range_min = 0.12;
    msg_lidar.range_max = 8.0;

    msg_lidar.ranges.data = (float *)malloc(360 * sizeof(float));
    msg_lidar.ranges.size = 360;
    for (i = 0; i < msg_lidar.ranges.size; i++)
    {
        msg_lidar.ranges.data[i] = 0;
    }
    
    msg_lidar.intensities.data = (float *)malloc(360 * sizeof(float));
    msg_lidar.intensities.size = 360;
    for (i = 0; i < msg_lidar.intensities.size; i++)
    {
        msg_lidar.intensities.data[i] = 10.0;
    }

    char* content_frame_id = "laser_frame";
    int len_namespace = strlen(ROS_NAMESPACE);
    int len_frame_id_max = len_namespace + strlen(content_frame_id) + 2;
    // ESP_LOGI(TAG, "lidar frame len:%d", len_frame_id_max);
    char* frame_id = malloc(len_frame_id_max);
    if (len_namespace == 0)
    {
        // The ROS namespace is empty characters
        sprintf(frame_id, "%s", content_frame_id);
    }
    else
    {
        // Concatenate the namespace and frame id
        sprintf(frame_id, "%s/%s", ROS_NAMESPACE, content_frame_id);
    }
    msg_lidar.header.frame_id = micro_ros_string_utilities_set(msg_lidar.header.frame_id, frame_id);
    free(frame_id);
}

// lidar update data task
void lidar_update_data_task(void *arg)
{
    uint16_t distance_mm[MS200_POINT_MAX] = {0};
    uint8_t intensity[MS200_POINT_MAX] = {0};
    uint16_t index = 0;
    int i = 0;

    while (1)
    {
        index = 0;
        for (i = 0; i < MS200_POINT_MAX; i++)
        {
            distance_mm[i] = Lidar_Ms200_Get_Distance(i);
            intensity[i] = Lidar_Ms200_Get_Intensity(i);
        }
        for (i = 0; i < MS200_POINT_MAX; i++)
        {
            index = (MS200_POINT_MAX-i) % MS200_POINT_MAX;
            if (index >= 180)
            {
                index = (index - 180) % MS200_POINT_MAX;
            }
            else
            {
                index = (index + 180) % MS200_POINT_MAX;
            }
            msg_lidar.ranges.data[i] = (float)(distance_mm[index] / 1000.0);
            msg_lidar.intensities.data[i] = (float)(intensity[index]);
        }
        //printf("lidar distance:%.2f\n", msg_lidar.ranges.data[0]);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    vTaskDelete(NULL);
}



// Initializes the ROS topic information for IMU
void imu_ros_init(void)
{
    msg_imu.angular_velocity.x = 0;
    msg_imu.angular_velocity.y = 0;
    msg_imu.angular_velocity.z = 0;

    msg_imu.linear_acceleration.x = 0;
    msg_imu.linear_acceleration.y = 0;
    msg_imu.linear_acceleration.z = 0;

    msg_imu.orientation.x = 0;
    msg_imu.orientation.y = 0;
    msg_imu.orientation.z = 0;
    msg_imu.orientation.w = 1;

    char* content_frame_id = "imu_frame";
    int len_namespace = strlen(ROS_NAMESPACE);
    int len_frame_id_max = len_namespace + strlen(content_frame_id) + 2;
    // ESP_LOGI(TAG, "imu frame len:%d", len_frame_id_max);
    char* frame_id = malloc(len_frame_id_max);
    if (len_namespace == 0)
    {
        // The ROS namespace is empty characters
        sprintf(frame_id, "%s", content_frame_id);
    }
    else
    {
        // Concatenate the namespace and frame id
        sprintf(frame_id, "%s/%s", ROS_NAMESPACE, content_frame_id);
    }
    msg_imu.header.frame_id = micro_ros_string_utilities_set(msg_imu.header.frame_id, frame_id);
    free(frame_id);
}

// IMU update data task
void imu_update_data_task(void *arg)
{
    int16_t gyro_raw[3] = {0};
    int16_t accel_raw[3] = {0};
    float imu_accel_g[3] = {0};
    float imu_gyro_dps[3] = {0};

    while (1)
    {
        Icm42670p_Get_Gyro_RawData(gyro_raw);
        Icm42670p_Get_Accel_RawData(accel_raw);
        Icm42670p_Get_Accel_g(imu_accel_g);
        Icm42670p_Get_Gyro_dps(imu_gyro_dps);
        msg_imu.angular_velocity.x = imu_gyro_dps[0];
        msg_imu.angular_velocity.y = imu_gyro_dps[1];
        msg_imu.angular_velocity.z = imu_gyro_dps[2];

        msg_imu.linear_acceleration.x = imu_accel_g[0];
        msg_imu.linear_acceleration.y = imu_accel_g[1];
        msg_imu.linear_acceleration.z = imu_accel_g[2];
        //printf("imu z %f\n",msg_imu.linear_acceleration.z);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}


// Initializes the ROS topic information for odom
void odom_ros_init(void)
{
    char* content_frame_id = "odom_frame";
    char* content_child_frame_id = "base_footprint";
    int len_namespace = strlen(ROS_NAMESPACE);
    int len_frame_id_max = len_namespace + strlen(content_frame_id) + 2;
    int len_child_frame_id_max = len_namespace + strlen(content_child_frame_id) + 2;
    char* frame_id = malloc(len_frame_id_max);
    char* child_frame_id = malloc(len_child_frame_id_max);
    if (len_namespace == 0)
    {
        // The ROS namespace is empty characters
        sprintf(frame_id, "%s", content_frame_id);
        sprintf(child_frame_id, "%s", content_child_frame_id);
    }
    else
    {
        // Concatenate the namespace and frame id
        sprintf(frame_id, "%s/%s", ROS_NAMESPACE, content_frame_id);
        sprintf(child_frame_id, "%s/%s", ROS_NAMESPACE, content_child_frame_id);
    }
    msg_odom.header.frame_id = micro_ros_string_utilities_set(msg_odom.header.frame_id, frame_id);
    msg_odom.child_frame_id = micro_ros_string_utilities_set(msg_odom.child_frame_id, child_frame_id);
    free(frame_id);
    free(child_frame_id);
}

// Euler's angular revolution quaternion
void odom_euler_to_quat(float roll, float pitch, float yaw, float *q)
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
}

// Update odom data
void odom_update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z)
{
    float delta_heading = angular_vel_z * vel_dt; // radians
    float cos_h = cos(heading_);
    float sin_h = sin(heading_);
    float delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt; // m
    float delta_y = (linear_vel_x * sin_h + linear_vel_y * cos_h) * vel_dt; // m

    // calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    // calculate robot's heading in quaternion angle
    // ROS has a function to calculate yaw in quaternion angle
    float q[4];
    odom_euler_to_quat(0, 0, heading_, q);

    // robot's position in x,y, and z
    msg_odom.pose.pose.position.x = x_pos_;
    msg_odom.pose.pose.position.y = y_pos_;
    msg_odom.pose.pose.position.z = 0.0;

    // robot's heading in quaternion
    msg_odom.pose.pose.orientation.x = (double)q[1];
    msg_odom.pose.pose.orientation.y = (double)q[2];
    msg_odom.pose.pose.orientation.z = (double)q[3];
    msg_odom.pose.pose.orientation.w = (double)q[0];

    msg_odom.pose.covariance[0] = 0.001;
    msg_odom.pose.covariance[7] = 0.001;
    msg_odom.pose.covariance[35] = 0.001;

    // linear speed from encoders
    msg_odom.twist.twist.linear.x = linear_vel_x;
    msg_odom.twist.twist.linear.y = linear_vel_y;
    msg_odom.twist.twist.linear.z = 0.0;

    // angular speed from encoders
    msg_odom.twist.twist.angular.x = 0.0;
    msg_odom.twist.twist.angular.y = 0.0;
    msg_odom.twist.twist.angular.z = angular_vel_z;

    msg_odom.twist.covariance[0] = 0.0001;
    msg_odom.twist.covariance[7] = 0.0001;
    msg_odom.twist.covariance[35] = 0.0001;
}


// Gets the number of seconds since boot
unsigned long get_millisecond(void)
{
    return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

// Calculate the time difference between the microROS agent and the MCU
static void sync_time(void)
{
    // unsigned long now = get_millisecond();
    // unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // time_offset = ros_time_ms - now;

    unsigned long now = get_millisecond();
    RCSOFTCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

// Get timestamp
struct timespec get_timespec(void)
{
    struct timespec tp = {0};
    // 同步时间 deviation of synchronous time
    unsigned long long now = get_millisecond() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}


// Timer callback function
void timer_odom_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        struct timespec time_stamp = get_timespec();
        unsigned long now = get_millisecond();
        float vel_dt = (now - prev_odom_update) / 1000.0;
        prev_odom_update = now;
        Motion_Get_Speed(&car_motion);
        odom_update(
            vel_dt,
            car_motion.Vx,
            car_motion.Vy,
            car_motion.Wz);
        msg_odom.header.stamp.sec = time_stamp.tv_sec;
        msg_odom.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&publisher_odom, &msg_odom, NULL));
    }
}

// Timer callback function
void timer_imu_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        struct timespec time_stamp = get_timespec();
        msg_imu.header.stamp.sec = time_stamp.tv_sec;
        msg_imu.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&publisher_imu, &msg_imu, NULL));
    }
}

// Timer callback function
void timer_lidar_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        struct timespec time_stamp = get_timespec();
        msg_lidar.header.stamp.sec = time_stamp.tv_sec;
        msg_lidar.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&publisher_lidar, &msg_lidar, NULL));
    }
}

void twist_Callback(const void *msgin)
{
    ESP_LOGI(TAG, "cmd_vel:%.2f, %.2f, %.2f", twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);
    Motion_Ctrl(twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);
}

// Subscriber callback function
void beep_callback(const void * msgin)
{
	const std_msgs__msg__UInt16 * msg = (const std_msgs__msg__UInt16 *)msgin;
	ESP_LOGI(TAG, "Beep: %d\n",  (int)  msg->data);
    // Control the state of the buzzer based on the received data
    Beep_On_Time(msg->data);
}

// micro ros processes tasks
void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Create init_options.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    // change ros domain id
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

    // Initialize the rmw options
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Setup static agent IP and port
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, ROS_AGENT_PORT, rmw_options));

    // Try to connect to the agent. If the connection succeeds, go to the next step.
    int state_agent = 0;
    while (1)
    {
        ESP_LOGI(TAG, "Connecting agent: %s:%s", CONFIG_MICRO_ROS_AGENT_IP, ROS_AGENT_PORT);
        state_agent = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
        if (state_agent == ESP_OK)
        {
            ESP_LOGI(TAG, "Connected agent: %s:%s", CONFIG_MICRO_ROS_AGENT_IP, ROS_AGENT_PORT);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // create ROS2 node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "YB_Car_Node", ROS_NAMESPACE, &support));

    // create publisher_odom
    RCCHECK(rclc_publisher_init_default(
        &publisher_odom,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom_raw"));

    // Create subscriber cmd_vel
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

	RCCHECK(rclc_subscription_init_default(
		&buzzer_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
		"beep"));

    // create publisher_imu
    RCCHECK(rclc_publisher_init_default(
        &publisher_imu,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu"));

    // create timer. Set the publish frequency to 11HZ
    const unsigned int odom_timer_timeout = 90;
    RCCHECK(rclc_timer_init_default(
        &timer_odom,
        &support,
        RCL_MS_TO_NS(odom_timer_timeout),
        timer_odom_callback));

    // create timer. Set the publish frequency to 25HZ
    const unsigned int imu_timer_timeout = 40;
    RCCHECK(rclc_timer_init_default(
        &timer_imu,
        &support,
        RCL_MS_TO_NS(imu_timer_timeout),
        timer_imu_callback));

        // 创建发布者
    // create publisher_lidar
    RCCHECK(rclc_publisher_init_default(
        &publisher_lidar,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "scan"));

    // create timer. Set the publish frequency to 11HZ
    const unsigned int lidar_timer_timeout = 90;
    RCCHECK(rclc_timer_init_default(
        &timer_lidar,
        &support,
        RCL_MS_TO_NS(lidar_timer_timeout),
        timer_lidar_callback));

    // create executor. Three of the parameters are the number of actuators controlled that is greater than or equal to the number of subscribers and publishers added to the executor.
    rclc_executor_t executor;
    int handle_num = 6;
    RCCHECK(rclc_executor_init(&executor, &support.context, handle_num, &allocator));
    
    // Adds the publisher_odom's timer to the executor
    RCCHECK(rclc_executor_add_timer(&executor, &timer_odom));
    // Adds the publisher_lidar's timer to the executor
    RCCHECK(rclc_executor_add_timer(&executor, &timer_lidar));

    // Adds the publisher_imu's timer to the executor
    RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));

    // Add a subscriber twist to the executor
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twist_Callback,
        ON_NEW_DATA));

    // Add subscriber to executor.
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &buzzer_subscriber,
        &msg_beep,
        &beep_callback,
        ON_NEW_DATA));


    sync_time();

    // Loop the microROS task
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(1000);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher_odom, &node));
    RCCHECK(rcl_subscription_fini(&twist_subscriber, &node));
    RCCHECK(rcl_subscription_fini(&buzzer_subscriber,&node));
    RCCHECK(rcl_publisher_fini(&publisher_imu, &node));
    RCCHECK(rcl_publisher_fini(&publisher_lidar, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void)
{

    Uart1_Init();
    // Initialize the Lidar
    Lidar_Ms200_Init();
    Beep_Init();
    Motor_Init();
    Icm42670p_Init();
    // Initialize serial port 1

   
    // Initialize the network and connect the WiFi signal
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    imu_ros_init();
    odom_ros_init();
    lidar_ros_init();

    // Start microROS tasks
    xTaskCreate(micro_ros_task,
                "micro_ros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);

    // Start imu tasks
    xTaskCreatePinnedToCore(imu_update_data_task,
                "imu_update_data_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL, 1);

    // Start lidar tasks
    xTaskCreatePinnedToCore(lidar_update_data_task,
                "lidar_update_data_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL, 1);
}

