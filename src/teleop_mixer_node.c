#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
// ten program steruje silnikiem i serwem przy uzyciu tego czy uklad dziala czy nie i wyhylen joysticka
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

#define JOY_CENTER    2048 //srodek przetwornika adc 
#define JOY_DEADZONE  300 //szumy 

static bool is_armed = false; //na start zakladamy ze system jest zablokowany
static int32_t raw_joy_x = JOY_CENTER;
static int32_t raw_joy_y = JOY_CENTER;

static rcl_publisher_t pub_servo;
static rcl_publisher_t pub_motor;

static rcl_subscription_t sub_state;
static rcl_subscription_t sub_joy_x;
static rcl_subscription_t sub_joy_y;

static std_msgs__msg__Bool msg_state;
static std_msgs__msg__Int32 msg_joy_x;
static std_msgs__msg__Int32 msg_joy_y;
static std_msgs__msg__Int32 msg_servo;
static std_msgs__msg__Int32 msg_motor;

static void cb_state(const void * msvin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msvin;
    is_armed = msg->data;
}

static void cb_joy_x(const void * msvin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msvin;
    raw_joy_x = msg->data;
}

static void cb_joy_y(const void * msvin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msvin;
    raw_joy_y = msg->data;
}

static void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) timer;
    (void) last_call_time;

    int32_t target_servo = 90; // Serwo na środku
    int32_t target_motor = 0;  // Silnik zatrzymany

    if (is_armed) {
        // 1. Skręcanie serwem: Oś X (0..4095) mapowana na 0..180 stopni
        if (abs(raw_joy_x - JOY_CENTER) > JOY_DEADZONE) {
            target_servo = (raw_joy_x * 180) / 4095;
            if (target_servo < 0) target_servo = 0;
            if (target_servo > 180) target_servo = 180;
        }

        // 2. Napęd silnika: Oś Y (0..4095) mapowana na -255..+255
        if (abs(raw_joy_y - JOY_CENTER) > JOY_DEADZONE) {
            target_motor = ((raw_joy_y - JOY_CENTER) * 255) / (4095 - JOY_CENTER);
            if (target_motor > 255) target_motor = 255;
            if (target_motor < -255) target_motor = -255;
        }
    }

    msg_servo.data = target_servo; //przypisanie nowej instrukcji
    rcl_ret_t ret = rcl_publish(&pub_servo, &msg_servo, NULL); //to bedziemy przesylac na topiki
    (void)ret;

    msg_motor.data = target_motor;
    ret = rcl_publish(&pub_motor, &msg_motor, NULL);
    (void)ret; //to ze nas ret nie interesuje swiadomie
}

int main(int argc, const char * const * argv) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
    (void)ret;

    rclc_support_init_with_options(&support, argc, argv, &init_options, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "teleop_mixer_node", "", &support);

    // Publishery dla ESP32
    rclc_publisher_init_default(
        &pub_servo, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/actuators/servo_angle"
    );
    rclc_publisher_init_default(
        &pub_motor, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/actuators/motor_speed"
    );

    // Subskrypcje
    rclc_subscription_init_default(
        &sub_state, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/system/state"
    );
    rclc_subscription_init_default(
        &sub_joy_x, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/esp32/joy_x"
    );
    rclc_subscription_init_default(
        &sub_joy_y, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/esp32/joy_y"
    );

    rcl_timer_t timer;
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), (rcl_timer_callback_t)timer_callback); // 50 Hz

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 4, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &sub_state, &msg_state, &cb_state, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_joy_x, &msg_joy_x, &cb_joy_x, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_joy_y, &msg_joy_y, &cb_joy_y, ON_NEW_DATA);

    printf("[teleop_mixer_node] Mikser uruchomiony (50Hz). Oczekuje na ARMED...\n");

    while (rcl_context_is_valid(&support.context)) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)); //petla while odpowiednik loop
    }

    ret = rclc_executor_fini(&executor);
    (void)ret;
    ret = rcl_node_fini(&node);
    (void)ret;
    ret = rclc_support_fini(&support);
    (void)ret;
    return 0;
}