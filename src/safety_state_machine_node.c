#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
//
//
//
//TUTAJ JUZ PISZEMY NA PC, NIE MA SETUP ANI LOOP WIEC OBIEKTY JAK NODE SUPPORT ALLOCATOR DEFINIUJEMY I SA POTRZEBNE TYLKO W FUNKCJI MAIN!!!!!
//
///
//
#define DISTANCE_THRESHOLD_CM   10.0f // max dystans przy ktorym zaczyna zliczac czas czujnik ultrasound 
#define ARM_REQUIRED_TIME_S     5.0f  //ile sekund trzeba trzymac dlon
#define INACTIVITY_TIMEOUT_S    5.0f //ile czasu trzeba nic nie robic aby przerwac proces
#define JOY_CENTER              2048  //  pozycja spoczynkowa joysticka
#define JOY_DEADZONE            400  //margines szumu (10%) powyzej jego ruch jest aktywnoscia

typedef enum {
    STATE_LOCKED = 0,
    STATE_ARMED  = 1
} system_state_t; //maszyna albo moze dzialac albo byc zablokowana

static system_state_t current_state = STATE_LOCKED;  //aktualny stan maszyny
static float distance_cm = 400.0f;     //odczyty z esp 400 to max odleglosc dzialania naszego czujnika , gdyby ustawic zero , czujnik zaczalby odliczac 
static int32_t joy_x_val = JOY_CENTER;
static int32_t joy_y_val = JOY_CENTER;

static float arming_timer_s = 0.0f;    //odmierzanie czasu wykrycia
static float inactivity_timer_s = 0.0f; //odmierzanie czasu bez aktywnosci

static rcl_publisher_t pub_system_state; // publisher stan systemu
static rcl_publisher_t pub_led_status;  //publisher kolor diody

static rcl_subscription_t sub_distance; //subskrybent odczytu z czujnika ultradzwiekowego
static rcl_subscription_t sub_joy_x; //subskrybent odczytu z joysticka
static rcl_subscription_t sub_joy_y;

static std_msgs__msg__Float32 msg_distance; //bufory
static std_msgs__msg__Int32 msg_joy_x;
static std_msgs__msg__Int32 msg_joy_y;

static std_msgs__msg__Bool msg_system_state;
static std_msgs__msg__Bool msg_led_status;
//callbacki subskrybentow
static void cb_distance(const void * msvin) {    //nie mozna przeciazac funkcji w C sharp wiec robimy tak
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msvin;
    distance_cm = msg->data;
}

static void cb_joy_x(const void * msvin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msvin;
    joy_x_val = msg->data;
}

static void cb_joy_y(const void * msvin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msvin;
    joy_y_val = msg->data;
}
static void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) timer;
    (void) last_call_time;
    const float dt = 0.05f; //50ms

    bool joy_is_active = (abs(joy_x_val - JOY_CENTER) > JOY_DEADZONE) ||  //flaga na to czy ruszamy w ogole galka
                         (abs(joy_y_val - JOY_CENTER) > JOY_DEADZONE);

    switch (current_state) {
        case STATE_LOCKED:
            if (distance_cm >= 1.0f && distance_cm <= DISTANCE_THRESHOLD_CM) { // czy wykrwyamy sygnal od 1cm do 10cm od czujnika (od 1 cm aby wywalic bledy szumy itd)
                arming_timer_s += dt; //dodanie kroku timera (0.05s)
                int progress = (int)((arming_timer_s / ARM_REQUIRED_TIME_S) * 20.0f); //wypelnienie paska w konsoli
                printf("\r[LOCKED] Trzymaj dlon! [");// \r przesuwa kursor na start tej samej lini
                for (int i = 0; i < 20; i++) { //petlafor rysuje ====>
                    if (i < progress) printf("=");
                    else if (i == progress) printf(">");
                    else printf(" ");
                }
                printf("] %.1f s / 5.0 s (HC-SR04: %.1f cm)   ", arming_timer_s, distance_cm); //odliczanie 5 sekund
                fflush(stdout); //wyrzucenie tekstu na ekran

                if (arming_timer_s >= ARM_REQUIRED_TIME_S) { //jak dobije timer do 5 sekund
                    current_state = STATE_ARMED; //uzbraja sie program
                    arming_timer_s = 0.0f;// zeruje timer
                    inactivity_timer_s = 0.0f;// ten tez
                    printf("\n\n>>> [SYSTEM ARMED] Odblokowano! ZIELONA DIODA ON <<<\n\n"); //no i komunikat wjezdza
                }
            } else {
                if (arming_timer_s > 0.0f) {//gdy zabierzemy dlon przed czasem
                    arming_timer_s -= dt * 2.0f;//chwilowe wygaszanie 0.10s na kazdy krok
                    if (arming_timer_s < 0.0f) arming_timer_s = 0.0f; //aby uniknac chwilowych wartosci ujemnych bo odejmujemy
                    printf("\r[LOCKED] Przerwano! Reset: %.1f s (HC-SR04: %.1f cm)     ", arming_timer_s, distance_cm);
                    fflush(stdout);
                }
            }
            break;// konczy wykonywanie bloku STATE_LOCKED
             case STATE_ARMED:
            if (joy_is_active) {
                inactivity_timer_s = 0.0f;
            } else {
                inactivity_timer_s += dt; //gdy przestajemy dotykac joysticka zaczyna sie sumowac czas +0.05s
                printf("\r[ARMED] Aktywny. Czas bezczynnosci: %.1f s / 5.0 s     ", inactivity_timer_s); //i tu wyswietlac
                fflush(stdout);

                if (inactivity_timer_s >= INACTIVITY_TIMEOUT_S) {// przekroczenie limitu
                    current_state = STATE_LOCKED; //zmiana stanu
                    inactivity_timer_s = 0.0f;//zerowanie timerow
                    arming_timer_s = 0.0f;
                    printf("\n\n>>> [SYSTEM LOCKED] Brak ruchu joysticka. CZERWONA DIODA ON <<<\n\n");
                }
            }
            break;
    }

    msg_system_state.data = (current_state == STATE_ARMED);// to jest pytanie logiczne, jesli state_armed jest to jest true, jesli nie to false
    rcl_ret_t ret = rcl_publish(&pub_system_state, &msg_system_state, NULL);//rcl publish przesyla paczke danych z bufora z msg system state przez publishera 
    //pub_system_state i do reszty ROS2 jako topik system/state
    (void)ret; //swiadomie nie korzystam z tego po to void

    msg_led_status.data = (current_state == STATE_ARMED); 
    ret = rcl_publish(&pub_led_status, &msg_led_status, NULL);
    (void)ret;
}

int main(int argc, const char * const * argv) {
    rcl_allocator_t allocator = rcl_get_default_allocator(); //allocator wraz z definicja rcl_allocator_t (obiektem)
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options(); // struktura przechowujaca opcje startowe ROS2,  to na zolto zapewnia ze nie ma smieci z poprzednich procesow (zapelnia zerami)
    rcl_ret_t ret = rcl_init_options_init(&init_options, allocator); //przygotowuje opcje do uzycia dajac pamiec za pomoca allocatora, ret przechowuje wynik operaji
    (void)ret; //z ret nie korzystamy ale normalnie moglbydawac np RCL_RET_OK

    rclc_support_init_with_options(&support, argc, argv, &init_options, &allocator); //kontekst wykonawczy, support,argumenty z wiersza main, wskaznik na
    //strukture zawierjaca np numer ROS_DOMAIN_ID, wskaznik na konkretny allocator

    rcl_node_t node;
    rclc_node_init_default(&node, "safety_state_machine_node", "", &support);// stworzenie nowego node'a

    rclc_publisher_init_default(
        &pub_system_state, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/system/state"
    );
    rclc_publisher_init_default(
        &pub_led_status, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/actuators/led_status"
    );

    rclc_subscription_init_default(
        &sub_distance, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/esp32/distance"
    );
    rclc_subscription_init_default(
        &sub_joy_x, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/esp32/joy_x"  //NAZWA TOPIKA
    );
    rclc_subscription_init_default(
        &sub_joy_y, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), //to wszystko tak jak zawsze czyli rejestracja publisherow i subsription
        "/esp32/joy_y"
    );

    rcl_timer_t timer;
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), (rcl_timer_callback_t)timer_callback);

    rclc_executor_t executor; //obiekt egzekutora
    rclc_executor_init(&executor, &support.context, 4, &allocator); //inicjalizacja z 4 zadankami
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &sub_distance, &msg_distance, &cb_distance, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_joy_x, &msg_joy_x, &cb_joy_x, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_joy_y, &msg_joy_y, &cb_joy_y, ON_NEW_DATA);

    printf("[safety_state_machine_node] Kierownik uruchomiony na PC. Czekam na dlon...\n");

    while (rcl_context_is_valid(&support.context)) { //w petli sprawdza naplywajace wiadomosci (while zamiast loopa)
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    ret = rclc_executor_fini(&executor);
    (void)ret;
    ret = rcl_node_fini(&node);
    (void)ret;
    ret = rclc_support_fini(&support);
    (void)ret;
    return 0;
    //sprzatanie pamieci jesli zostanie program zakonczony chociazby przez ctrl c\
    //tworzylismy support node executor wiec sprzatamy w odwrotnej kolejnosci
}