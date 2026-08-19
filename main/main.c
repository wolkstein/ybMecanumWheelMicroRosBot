#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/uart.h"

#if CONFIG_MICRO_ROS_ESP_NETIF_WLAN || CONFIG_MICRO_ROS_ESP_NETIF_ENET
#include <uros_network_interfaces.h>
#endif
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>
#include <micro_ros_utilities/string_utilities.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>

#include "icm42670p.h"
#include "car_motion.h"
#include "beep.h"
#include "calibration.h"
#include "battery.h"

// Custom UART transport for micro-ROS
#define UART_BUFFER_SIZE 512

static bool transport_serial_open(struct uxrCustomTransport *transport)
{
    size_t *uart_port = (size_t *)transport->args;
    uart_config_t uart_config = {
        .baud_rate  = CONFIG_MICRO_ROS_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    if (uart_param_config(*uart_port, &uart_config) != ESP_OK) return false;
    if (uart_set_pin(*uart_port, CONFIG_MICRO_ROS_UART_TXD, CONFIG_MICRO_ROS_UART_RXD,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return false;
    if (uart_driver_install(*uart_port, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0) != ESP_OK) return false;
    return true;
}

static bool transport_serial_close(struct uxrCustomTransport *transport)
{
    size_t *uart_port = (size_t *)transport->args;
    return uart_driver_delete(*uart_port) == ESP_OK;
}

static size_t transport_serial_write(struct uxrCustomTransport *transport,
                                     const uint8_t *buf, size_t len, uint8_t *err)
{
    size_t *uart_port = (size_t *)transport->args;
    return uart_write_bytes(*uart_port, (const char *)buf, len);
}

static size_t transport_serial_read(struct uxrCustomTransport *transport,
                                    uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    size_t *uart_port = (size_t *)transport->args;
    return uart_read_bytes(*uart_port, buf, len, timeout / portTICK_PERIOD_MS);
}


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
#if CONFIG_MICRO_ROS_ESP_NETIF_WLAN || CONFIG_MICRO_ROS_ESP_NETIF_ENET
#define ROS_AGENT_IP       CONFIG_MICRO_ROS_AGENT_IP
#define ROS_AGENT_PORT     CONFIG_MICRO_ROS_AGENT_PORT
#endif



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

rcl_subscription_t calibrate_subscriber;
geometry_msgs__msg__Vector3 msg_calibrate;

rcl_publisher_t publisher_battery;
sensor_msgs__msg__BatteryState msg_battery;
rcl_timer_t timer_battery;

rcl_subscription_t battery_config_subscriber;
std_msgs__msg__Int32MultiArray msg_battery_config;


unsigned long long time_offset = 0;
unsigned long prev_odom_update = 0;

float x_pos_ = 0.0;
float y_pos_ = 0.0;
float heading_ = 0.0;

car_motion_t car_motion;



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

/**
 * @brief Set up the static/one-time fields of the /battery_state message.
 *
 * Sets the frame_id (namespace-prefixed like the odom/imu frames), the
 * required-non-NULL location/serial_number strings, the design_capacity
 * derived from the current battery config, and the fields we can't measure
 * (temperature/current/charge/capacity => NaN, no such sensors on this
 * board). The per-cycle fields (voltage, percentage, health, timestamp) are
 * filled in by timer_battery_callback() on every publish.
 *
 * Must be called once during startup, after Battery_Init() (so
 * Battery_Get_CapacityMah() returns the loaded config) and before the
 * micro-ROS task starts publishing.
 */
void battery_ros_init(void)
{
    char* content_frame_id = "battery_frame";
    int len_namespace = strlen(ROS_NAMESPACE);
    int len_frame_id_max = len_namespace + strlen(content_frame_id) + 2;
    char* frame_id = malloc(len_frame_id_max);
    if (len_namespace == 0)
    {
        sprintf(frame_id, "%s", content_frame_id);
    }
    else
    {
        sprintf(frame_id, "%s/%s", ROS_NAMESPACE, content_frame_id);
    }
    msg_battery.header.frame_id = micro_ros_string_utilities_set(msg_battery.header.frame_id, frame_id);
    free(frame_id);

    // location/serial_number must be a valid (non-NULL-data) rosidl string,
    // never left zero-initialized -- see MicroRosServoControlBoard crash notes.
    msg_battery.location = micro_ros_string_utilities_init("");
    msg_battery.serial_number = micro_ros_string_utilities_init("");

    // cell_voltage/cell_temperature stay zero-initialized (size=0): we only
    // measure total pack voltage via one ADC pin, no per-cell taps.
    msg_battery.design_capacity = Battery_Get_CapacityMah() / 1000.0f; // mAh -> Ah
    msg_battery.capacity = NAN;    // last full capacity: unmeasured
    msg_battery.charge = NAN;      // current charge: unmeasured
    msg_battery.current = NAN;     // pack current: no shunt/sense hardware
    msg_battery.temperature = NAN; // no temperature sensor
    msg_battery.present = true;
    msg_battery.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN; // no charge-detection hardware
    msg_battery.power_supply_technology = (uint8_t)Battery_Get_Technology();
}

/** Number of int32 values expected in a /battery_config message, and their
 *  order -- must match the parameter order of Battery_Save() exactly:
 *  [cell_count, capacity_mah, cell_voltage_max_mv, cell_voltage_warn_mv,
 *   cell_voltage_cutoff_mv, technology, adc_divider_factor_x1000] */
#define BATTERY_CONFIG_PARAM_COUNT 7

/**
 * @brief Pre-allocate the /battery_config subscription's data buffer.
 *
 * The micro-ROS deserializer writes incoming Int32MultiArray elements
 * directly into msg_battery_config.data.data, so that buffer must already
 * exist (with capacity >= BATTERY_CONFIG_PARAM_COUNT) before the executor
 * starts spinning -- there is no reallocation on receive. layout.dim stays
 * zero-initialized/empty since we don't use multi-dimensional layout
 * metadata, just a flat fixed-size array.
 *
 * Must be called once during startup, before the micro-ROS task's executor
 * starts spinning (see app_main()).
 */
void battery_config_ros_init(void)
{
    msg_battery_config.data.data = malloc(BATTERY_CONFIG_PARAM_COUNT * sizeof(int32_t));
    msg_battery_config.data.size = 0;
    msg_battery_config.data.capacity = BATTERY_CONFIG_PARAM_COUNT;
}

/**
 * @brief Subscriber callback for /battery_config.
 *
 * Expects an Int32MultiArray with exactly BATTERY_CONFIG_PARAM_COUNT values
 * in the order documented at BATTERY_CONFIG_PARAM_COUNT. Messages with a
 * different length are rejected (logged, not applied) rather than read
 * out-of-bounds. Valid messages are forwarded to Battery_Save(), which
 * applies them immediately and persists them to NVS.
 *
 * @param msgin Pointer to the received std_msgs__msg__Int32MultiArray.
 */
void battery_config_callback(const void *msgin)
{
    const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;
    if (msg->data.size != BATTERY_CONFIG_PARAM_COUNT)
    {
        ESP_LOGE(TAG, "battery_config: expected %d values, got %d",
                 BATTERY_CONFIG_PARAM_COUNT, (int)msg->data.size);
        return;
    }
    Battery_Save(msg->data.data[0], msg->data.data[1], msg->data.data[2],
                 msg->data.data[3], msg->data.data[4], msg->data.data[5],
                 msg->data.data[6]);
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

/**
 * @brief 1Hz timer callback: publish /battery_state and drive the low-battery beep.
 *
 * Computes pack-level max/warn/cutoff voltages from the per-cell config
 * (Battery_Get_CellVoltageMaxMV() etc. times the cell count), fills in the
 * per-cycle fields of msg_battery (voltage, percentage, health, timestamp)
 * and publishes it. If the pack voltage is below the warn threshold, also
 * triggers a short buzzer pulse -- since this callback re-fires every
 * second, that naturally produces a "beep once per second" warning pattern
 * for as long as the voltage stays low, with no separate timer needed.
 *
 * @param timer          The firing rcl_timer_t (timer_battery). NULL-checked
 *                        defensively, mirroring the other timer callbacks.
 * @param last_call_time Unused; required by the rclc timer callback signature.
 */
void timer_battery_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        struct timespec time_stamp = get_timespec();
        msg_battery.header.stamp.sec = time_stamp.tv_sec;
        msg_battery.header.stamp.nanosec = time_stamp.tv_nsec;

        float voltage = Battery_Get_Voltage();
        int cells = Battery_Get_CellCount();
        float v_max = cells * (Battery_Get_CellVoltageMaxMV() / 1000.0f);
        float v_warn = cells * (Battery_Get_CellVoltageWarnMV() / 1000.0f);
        float v_cutoff = cells * (Battery_Get_CellVoltageCutoffMV() / 1000.0f);

        msg_battery.voltage = voltage;
        float pct = (voltage - v_cutoff) / (v_max - v_cutoff);
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 1.0f) pct = 1.0f;
        msg_battery.percentage = pct;
        msg_battery.power_supply_health = (voltage <= v_cutoff)
            ? sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_DEAD
            : sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_GOOD;

        RCSOFTCHECK(rcl_publish(&publisher_battery, &msg_battery, NULL));

        // Periodic low-battery warning: short beep once per timer tick (1Hz)
        // while below the warn threshold. Re-triggered every tick, so it
        // naturally stops as soon as voltage recovers above v_warn.
        if (voltage > 0.0f && voltage < v_warn)
        {
            Beep_On_Time(150);
        }
    }
}

void twist_Callback(const void *msgin)
{
    ESP_LOGI(TAG, "cmd_vel:%.2f, %.2f, %.2f", twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);
    Motion_Ctrl(twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);
}

// x=wheel_diameter_mm, y=robot_width_m, z=robot_length_m
void calibrate_callback(const void *msgin)
{
    const geometry_msgs__msg__Vector3 *msg = (const geometry_msgs__msg__Vector3 *)msgin;
    float circ_mm = (float)(3.14159265f * msg->x);
    Calibration_Save((float)msg->x, (float)msg->y, (float)msg->z);
    Motor_Set_WheelCirc(circ_mm);
    Motion_Set_Calibration((float)msg->y, (float)msg->z);
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
    // Disable logging and release UART0 so micro-ROS can take it exclusively
    esp_log_level_set("*", ESP_LOG_NONE);
    uart_driver_delete(UART_NUM_0);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Create init_options.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    // change ros domain id
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

    // Initialize the rmw options
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Register custom serial transport
    static size_t uart_port = CONFIG_MICRO_ROS_UART_NUM;
    RCCHECK(rmw_uros_options_set_custom_transport(
        true, (void *) &uart_port,
        transport_serial_open, transport_serial_close,
        transport_serial_write, transport_serial_read,
        rmw_options));

    // Setup static agent IP and port for network transport.
#if CONFIG_MICRO_ROS_ESP_NETIF_WLAN || CONFIG_MICRO_ROS_ESP_NETIF_ENET
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, ROS_AGENT_PORT, rmw_options));
#endif

    // Try to connect to the agent. If the connection succeeds, go to the next step.
    rcl_ret_t state_agent = RCL_RET_ERROR;
    while (1)
    {
#if CONFIG_MICRO_ROS_ESP_NETIF_WLAN || CONFIG_MICRO_ROS_ESP_NETIF_ENET
        ESP_LOGI(TAG, "Connecting agent: %s:%s", CONFIG_MICRO_ROS_AGENT_IP, ROS_AGENT_PORT);
#else
        ESP_LOGI(TAG, "Connecting agent over UART transport");
#endif
        state_agent = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
        if (state_agent == RCL_RET_OK)
        {
#if CONFIG_MICRO_ROS_ESP_NETIF_WLAN || CONFIG_MICRO_ROS_ESP_NETIF_ENET
            ESP_LOGI(TAG, "Connected agent: %s:%s", CONFIG_MICRO_ROS_AGENT_IP, ROS_AGENT_PORT);
#else
            ESP_LOGI(TAG, "Connected agent over UART transport");
#endif
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

    // Create subscriber /calibrate  (Vector3: x=wheel_diam_mm, y=robot_width_m, z=robot_len_m)
    RCCHECK(rclc_subscription_init_default(
        &calibrate_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "calibrate"));

    // Create subscriber /battery_config (Int32MultiArray, see battery_config_callback)
    RCCHECK(rclc_subscription_init_default(
        &battery_config_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "battery_config"));

    // create publisher_imu
    RCCHECK(rclc_publisher_init_default(
        &publisher_imu,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu"));

    // create publisher_battery
    RCCHECK(rclc_publisher_init_default(
        &publisher_battery,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        "battery_state"));

    // create timer. Set the publish frequency to 11HZ
    const unsigned int odom_timer_timeout = 90;
    RCCHECK(rclc_timer_init_default2(
        &timer_odom,
        &support,
        RCL_MS_TO_NS(odom_timer_timeout),
        timer_odom_callback, true));

    // create timer. Set the publish frequency to 25HZ
    const unsigned int imu_timer_timeout = 40;
    RCCHECK(rclc_timer_init_default2(
        &timer_imu,
        &support,
        RCL_MS_TO_NS(imu_timer_timeout),
        timer_imu_callback, true));

    // create timer. Set the publish frequency to 1HZ (battery changes slowly)
    const unsigned int battery_timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default2(
        &timer_battery,
        &support,
        RCL_MS_TO_NS(battery_timer_timeout),
        timer_battery_callback, true));

    // create executor. Three of the parameters are the number of actuators controlled that is greater than or equal to the number of subscribers and publishers added to the executor.
    rclc_executor_t executor;
    int handle_num = 7;
    RCCHECK(rclc_executor_init(&executor, &support.context, handle_num, &allocator));

    // Adds the publisher_odom's timer to the executor
    RCCHECK(rclc_executor_add_timer(&executor, &timer_odom));

    // Adds the publisher_imu's timer to the executor
    RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));

    // Adds the publisher_battery's timer to the executor
    RCCHECK(rclc_executor_add_timer(&executor, &timer_battery));

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

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &calibrate_subscriber,
        &msg_calibrate,
        &calibrate_callback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &battery_config_subscriber,
        &msg_battery_config,
        &battery_config_callback,
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
    RCCHECK(rcl_subscription_fini(&calibrate_subscriber, &node));
    RCCHECK(rcl_subscription_fini(&battery_config_subscriber, &node));
    RCCHECK(rcl_publisher_fini(&publisher_imu, &node));
    RCCHECK(rcl_publisher_fini(&publisher_battery, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void)
{
    Calibration_Init();
    float wheel_circ = 3.14159265f * Calib_Get_WheelDiameterMM();
    Motor_Set_WheelCirc(wheel_circ);
    Motion_Set_Calibration(Calib_Get_RobotWidth(), Calib_Get_RobotLength());

    Beep_Init();
    Motor_Init();
    Icm42670p_Init();
    Battery_Init();

    // Initialize the network and connect the WiFi signal
#if CONFIG_MICRO_ROS_ESP_NETIF_WLAN || CONFIG_MICRO_ROS_ESP_NETIF_ENET
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    imu_ros_init();
    odom_ros_init();
    battery_ros_init();
    battery_config_ros_init();

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

}

