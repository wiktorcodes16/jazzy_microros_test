#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
//ten program to tylko interface dlatego ma tylko subskrybcje swoje

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

static bool system_armed = false;
static float distance_cm = 0.0f;
static int32_t joy_x = 2048;
static int32_t joy_y = 2048;
static int32_t servo_ang = 90;
static int32_t motor_spd = 0;

static rcl_subscription_t sub_state;
static rcl_subscription_t sub_dist;
static rcl_subscription_t sub_jx;
static rcl_subscription_t sub_jy;
static rcl_subscription_t sub_servo;
static rcl_subscription_t sub_motor;

static std_msgs__msg__Bool msg_state;
static std_msgs__msg__Float32 msg_dist;
static std_msgs__msg__Int32 msg_jx;
static std_msgs__msg__Int32 msg_jy;
static std_msgs__msg__Int32 msg_servo;
static std_msgs__msg__Int32 msg_motor;

static void cb_state(const void * msvin) { system_armed = ((const std_msgs__msg__Bool *)msvin)->data; }
static void cb_dist(const void * msvin)  { distance_cm = ((const std_msgs__msg__Float32 *)msvin)->data; }
static void cb_jx(const void * msvin)    { joy_x = ((const std_msgs__msg__Int32 *)msvin)->data; }
static void cb_jy(const void * msvin)    { joy_y = ((const std_msgs__msg__Int32 *)msvin)->data; }
static void cb_servo(const void * msvin) { servo_ang = ((const std_msgs__msg__Int32 *)msvin)->data; }
static void cb_motor(const void * msvin) { motor_spd = ((const std_msgs__msg__Int32 *)msvin)->data; }

static void draw_bar(int val, int min, int max, int width) {
    if (val < min) val = min;
    if (val > max) val = max;
    int pos = ((val - min) * width) / (max - min);
    printf("[");
    for (int i = 0; i < width; i++) {
        if (i == pos) printf("|");
        else if (i == width / 2) printf(":");
        else printf("-");
    }
    printf("]");
}

static void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) timer;
    (void) last_call_time;

    // Czyszczenie ekranu terminala (ANSI)
    printf("\033[H\033[J");

    printf("=================================================================\n");
    printf("                PANEL TELEMETRII I KONTROLI ROS 2                \n");
    printf("=================================================================\n\n");

    printf(" STATUS SYSTEMU: ");
    if (system_armed) {
        printf("\033[1;32m[   ARMED / ODBLOKOWANY   ]\033[0m  (Zielona LED ON)\n");
    } else {
        printf("\033[1;31m[   LOCKED / ZABLOKOWANY  ]\033[0m  (Czerwona LED ON)\n");
    }
    printf("\n");

    printf(" [HC-SR04] Odleglosc:  %5.1f cm  ", distance_cm);
    draw_bar((int)distance_cm, 0, 50, 20);
    printf("\n\n");

    printf(" [JOYSTICK] Oś X:       %5d     ", joy_x);
    draw_bar(joy_x, 0, 4095, 20);
    printf("\n");
    printf(" [JOYSTICK] Oś Y:       %5d     ", joy_y);
    draw_bar(joy_y, 0, 4095, 20);
    printf("\n\n");

    printf("------------------------ WYJŚCIA WYKONAWCZE ---------------------\n");
    printf(" [SG90]    Kąt serwa:   %5d deg  ", servo_ang);
    draw_bar(servo_ang, 0, 180, 20);
    printf("\n");
    printf(" [L293D]   Ciąg silnika:%5d      ", motor_spd);
    draw_bar(motor_spd, -255, 255, 20);
    printf("\n");
    printf("=================================================================\n");
    printf(" Instrukcja: Trzymaj dlon < 10 cm przez 5s, by odblokowac.\n");
    printf(" Brak ruchu joysticka przez 5s powoduje automatyczna blokade.\n");
    fflush(stdout);
}

int main(int argc, const char * const * argv) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
    (void)ret;

    rclc_support_init_with_options(&support, argc, argv, &init_options, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "terminal_ui_node", "", &support);

    rclc_subscription_init_default(&sub_state, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/system/state");
    rclc_subscription_init_default(&sub_dist,  &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/esp32/distance");
    rclc_subscription_init_default(&sub_jx,    &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/esp32/joy_x");
    rclc_subscription_init_default(&sub_jy,    &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/esp32/joy_y");
    rclc_subscription_init_default(&sub_servo, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/actuators/servo_angle");
    rclc_subscription_init_default(&sub_motor, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/actuators/motor_speed");

    rcl_timer_t timer;
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), (rcl_timer_callback_t)timer_callback); // 10 Hz

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 7, &allocator); // 1 timer + 6 subs = 7 slotów
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &sub_state, &msg_state, &cb_state, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_dist,  &msg_dist,  &cb_dist,  ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_jx,    &msg_jx,    &cb_jx,    ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_jy,    &msg_jy,    &cb_jy,    ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_servo, &msg_servo, &cb_servo, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_motor, &msg_motor, &cb_motor, ON_NEW_DATA);

    while (rcl_context_is_valid(&support.context)) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    ret = rclc_executor_fini(&executor);
    (void)ret;
    ret = rcl_node_fini(&node);
    (void)ret;
    ret = rclc_support_fini(&support);
    (void)ret;
    return 0;
}