#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#define LED_PIN 13

rcl_publisher_t publisher;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

std_msgs__msg__Int32 sensor_msg;

bool led_state = false;
unsigned long previous_time = 0;
const unsigned long interval_ms = 1000;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  
  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();

  rclc_support_init(
    &support,
    0,
    NULL,
    &allocator);

  rclc_node_init_default(
    &node,
    "teensy_node",
    "",
    &support);

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      std_msgs,
      msg,
      Int32),
    "/sensor_status");
}

void loop()
{
  unsigned long current_time = millis();

  if (current_time - previous_time >= interval_ms)
  {
    previous_time = current_time;

    led_state = !led_state;

    digitalWrite(
      LED_PIN,
      led_state ? HIGH : LOW);

    sensor_msg.data =
      led_state ? 1 : 0;

    rcl_publish(
      &publisher,
      &sensor_msg,
      NULL);
  }
}
