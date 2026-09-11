/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include "sys/time.h"
#include "iwdg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, int timeout, uint8_t * err);

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_5

rcl_subscription_t led_control_subscriber;
rcl_subscription_t fade_speed_subscriber;
rcl_subscription_t custom_message_subscriber;

std_msgs__msg__Bool   led_control_msg;
std_msgs__msg__Int32  fade_speed_msg;
std_msgs__msg__String custom_message_msg;
char custom_message_buffer[50];


rcl_publisher_t counter_publisher;
rcl_publisher_t string_publisher;
std_msgs__msg__Int32 counter_msg;
std_msgs__msg__String string_msg;
char string_buffer[50];
rcl_timer_t timer;

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

volatile bool led_manual_override = false;
volatile bool led_manual_state = false;
volatile int32_t fade_step = 10;          // default step size, same as before
volatile bool custom_message_active = false;
char custom_message_storage[50];

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Fade */
osThreadId_t FadeHandle;
const osThreadAttr_t Fade_attributes = {
  .name = "Fade",
  .stack_size = 200 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Fade */
  FadeHandle = osThreadNew(StartTask02, NULL, &Fade_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {

    if (led_manual_override) {
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, led_manual_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
      HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    }

    rcl_ret_t ret1 = rcl_publish(&counter_publisher, &counter_msg, NULL);
    (void)ret1;
    counter_msg.data++;

    if (custom_message_active) {
      snprintf(string_buffer, sizeof(string_buffer), "%s", custom_message_storage);
    } else {
      snprintf(string_buffer, sizeof(string_buffer), "Hello from Nucleo! count=%ld", (long)counter_msg.data);
    }
    string_msg.data.data = string_buffer;
    string_msg.data.size = strlen(string_buffer);
    string_msg.data.capacity = sizeof(string_buffer);
    rcl_ret_t ret2 = rcl_publish(&string_publisher, &string_msg, NULL);
    (void)ret2;
  }
}

int _gettimeofday(struct timeval *tv, void *tzvp) {
  (void)tzvp;
  uint32_t tick = HAL_GetTick();
  tv->tv_sec = tick / 1000;
  tv->tv_usec = (tick % 1000) * 1000;
  return 0;
}

void led_control_callback(const void *msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *) msgin;
  led_manual_override = true;
  led_manual_state = msg->data;
}

void fade_speed_callback(const void *msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *) msgin;
  if (msg->data >= 0 && msg->data <= 999) {
    fade_step = msg->data;
  }
}

void custom_message_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *) msgin;
  size_t len = msg->data.size;
  if (len >= sizeof(custom_message_storage)) {
    len = sizeof(custom_message_storage) - 1;
  }
  memcpy(custom_message_storage, msg->data.data, len);
  custom_message_storage[len] = '\0';
  custom_message_active = true;
}
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart2,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);

	  rcl_allocator_t allocator = rcl_get_default_allocator();
	  rclc_support_t support;
	  rclc_support_init(&support, 0, NULL, &allocator);

	  rcl_node_t node;
	  rclc_node_init_default(&node, "nucleo_led_node", "", &support);

	  HAL_IWDG_Refresh(&hiwdg);
	  rclc_publisher_init_default(
	    &counter_publisher,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	    "led_blink_count");
	  HAL_IWDG_Refresh(&hiwdg);
	  rclc_publisher_init_default(
	    &string_publisher,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
	    "nucleo_chatter");
	  HAL_IWDG_Refresh(&hiwdg);
	  rclc_subscription_init_default(
		&led_control_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
		"led_control"
	  );
	  HAL_IWDG_Refresh(&hiwdg);
	  rclc_subscription_init_default(
		&fade_speed_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"fade_speed"
	  );
	  HAL_IWDG_Refresh(&hiwdg);
	  rclc_subscription_init_default(
			  &custom_message_subscriber, &node,
			  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg, String),
			  "custom_message"
			  );
	  HAL_IWDG_Refresh(&hiwdg);
	  const unsigned int timer_period_ms = 500;
	  rclc_timer_init_default2(
	    &timer,
	    &support,
	    RCL_MS_TO_NS(timer_period_ms),
	    timer_callback,
		true);
	  HAL_IWDG_Refresh(&hiwdg);
	  rclc_executor_t executor;
	  rclc_executor_init(&executor, &support.context, 4, &allocator);
	  rclc_executor_add_timer(&executor, &timer);

	  rclc_executor_add_subscription(
	    &executor, &led_control_subscriber, &led_control_msg,
	    &led_control_callback, ON_NEW_DATA);

	  rclc_executor_add_subscription(
	    &executor, &fade_speed_subscriber, &fade_speed_msg,
	    &fade_speed_callback, ON_NEW_DATA);
	  HAL_IWDG_Refresh(&hiwdg);
	  custom_message_msg.data.data = custom_message_buffer;
	    custom_message_msg.data.size = 0;
	    custom_message_msg.data.capacity = sizeof(custom_message_buffer);

	    rclc_executor_add_subscription(
	      &executor, &custom_message_subscriber, &custom_message_msg,
	      &custom_message_callback, ON_NEW_DATA);

	  counter_msg.data = 0;
	  HAL_IWDG_Refresh(&hiwdg);
  /* Infinite loop */
  for(;;)
  {
	  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
	  HAL_IWDG_Refresh(&hiwdg);
	  osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Fade thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	  uint32_t max_duty = 999;  // 999, from ARR
	  int32_t duty = 0;
	  int8_t direction = 1;
  /* Infinite loop */
  for(;;)
  {
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);

	      duty += direction * 10;
	      if (duty >= (int32_t)max_duty) {
	        duty = max_duty;
	        direction = -1;
	      } else if (duty <= 0) {
	        duty = 0;
	        direction = 1;
	      }

	      osDelay(5);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

