#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/color_rgba.h>
#include <rmw_microros/rmw_microros.h>
#include <my_interfaces/srv/set_color.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include <PicoLed.hpp>

const uint LED_PIN = 0;
const uint LED_LENGTH = 16;
const uint BTN_PIN = 15;

PicoLed::PicoLedController *ledStripPtr;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_subscription_t subscriber;
std_msgs__msg__ColorRGBA sub_msg;

rcl_service_t service;
my_interfaces__srv__SetColor_Request response_msg;
my_interfaces__srv__SetColor_Response request_msg;

uint service_counter = 0;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    // msg.data++;
}

void subscription_callback(const void * msgin)
{
    // Cast received message to used type
    const std_msgs__msg__ColorRGBA * msg = (const std_msgs__msg__ColorRGBA *)msgin;

    ledStripPtr->fill( PicoLed::RGB(msg->r*255, msg->g*255, msg->b*255) );
    ledStripPtr->show();
}

void service_callback(const void * request_msg, void * response_msg){
  // Cast messages to expected types
  my_interfaces__srv__SetColor_Request * req_in =
    (my_interfaces__srv__SetColor_Request *) request_msg;
  my_interfaces__srv__SetColor_Response * res_in =
    (my_interfaces__srv__SetColor_Response *) response_msg;

  // Handle request message and set the response message values

    ledStripPtr->fill( PicoLed::RGB(req_in->color.r*255, req_in->color.g*255, req_in->color.b*255) );
    ledStripPtr->show();

    service_counter++;

    res_in->count = service_counter;
}


int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);


    stdio_init_all();
    auto ledStrip = PicoLed::addLeds<PicoLed::WS2812B>(pio0, 0, LED_PIN, LED_LENGTH, PicoLed::FORMAT_GRB);
    ledStripPtr = &ledStrip;
    ledStrip.setBrightness(255);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");

    rcl_ret_t rc = rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
        "button_color");

    rc = rclc_service_init_default(
        &service, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(my_interfaces, srv, SetColor),
        "color_service");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    // Add subscription to the executor
    rc = rclc_executor_add_subscription(
        &executor, &subscriber, &sub_msg,
        &subscription_callback, ON_NEW_DATA);

    rc = rclc_executor_add_service(
        &executor, &service, &request_msg,
        &response_msg, service_callback);


    auto last_btn_time = to_ms_since_boot(get_absolute_time());
    msg.data = 0;
    while (true)
    {
        if (!gpio_get(BTN_PIN))
        {
            auto new_time = to_ms_since_boot(get_absolute_time());
            if (new_time - last_btn_time > 1000)
            {
                rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
                msg.data++;
                last_btn_time = new_time;
            }
        }

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
