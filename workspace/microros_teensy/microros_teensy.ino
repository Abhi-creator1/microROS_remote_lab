#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

std_msgs__msg__Int32 sensor_msg;

const int LED_PIN = 13;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;

  if (timer != NULL)
  {
    sensor_msg.data++;

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    rcl_publish(&publisher, &sensor_msg, NULL);
  }
}

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
    "sensor_node",
    "",
    &support);

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/sensor_status");

  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(1000),
    timer_callback);

  rclc_executor_init(
    &executor,
    &support.context,
    1,
    &allocator);

  rclc_executor_add_timer(
    &executor,
    &timer);

  sensor_msg.data = 0;
}

void loop()
{
  rclc_executor_spin_some(
    &executor,
    RCL_MS_TO_NS(100));

  delay(100);
}
