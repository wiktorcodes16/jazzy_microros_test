cmake_minimum_required(VERSION 3.8)
project(esp32_interface C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(std_msgs REQUIRED)

# Węzeł 2: Nadzorca bezpieczeństwa
add_executable(safety_state_machine_node src/safety_state_machine_node.c)
target_link_libraries(safety_state_machine_node
  rcl::rcl
  rclc::rclc
  std_msgs::std_msgs
)

# Węzeł 3: Mikser joysticka na serwo i silnik
add_executable(teleop_mixer_node src/teleop_mixer_node.c)
target_link_libraries(teleop_mixer_node
  rcl::rcl
  rclc::rclc
  std_msgs::std_msgs
)

# Węzeł 4: Interfejs telemetryczny
add_executable(terminal_ui_node src/terminal_ui_node.c)
target_link_libraries(terminal_ui_node
  rcl::rcl
  rclc::rclc
  std_msgs::std_msgs
)

install(TARGETS
  safety_state_machine_node
  teleop_mixer_node
  terminal_ui_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()