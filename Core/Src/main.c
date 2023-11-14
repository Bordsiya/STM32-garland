/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_TIMEOUT 10
#define BUF_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern void initialise_monitor_handles(void);

bool flag = true;
bool pred_flag = true;

void set_green_led(bool on) { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }

void set_yellow_led(bool on) { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }

void set_red_led(bool on) { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }

uint32_t last_pressed_time = 0;

int is_btn_clicked() {
	// GPIO_PIN_RESET means pressed
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET && HAL_GetTick() - last_pressed_time > 500) {
		last_pressed_time = HAL_GetTick();
		printf("Button pressed\n");
		return 1;
	} else return 0;
}

void set_no_one_led() {
	set_green_led(false);
	set_yellow_led(false);
	set_red_led(false);
}


enum LED {
    LED_NO_ONE = 0,
    LED_RED = 1,
    LED_GREEN = 2,
    LED_YELLOW = 3,
};

struct LightState {
    enum LED color;
    uint32_t timeout;
};

typedef struct LightState LightState;

struct GarlandMode {
    uint8_t light_states_count;
    LightState states[8];
};

typedef struct GarlandMode GarlandMode;
GarlandMode new_mode;

GarlandMode modes[8] = {
		{
			.light_states_count = 3,
            .states = {
                {
                    .color = LED_YELLOW,
                    .timeout = 100,
                },
                {
                    .color = LED_GREEN,
                    .timeout = 100,
                },
				{
				    .color = LED_RED,
				    .timeout = 100,
				}
            }
        },
        {
            .light_states_count = 6,
            .states = {
                {
                    .color = LED_RED,
                    .timeout = 700,
                },
                {
                    .color = LED_NO_ONE,
                    .timeout = 700,
                },
                {
                    .color = LED_YELLOW,
                    .timeout = 700,
                },
                {
                    .color = LED_NO_ONE,
                    .timeout = 700,
                },
                {
                    .color = LED_GREEN,
                    .timeout = 700,
                },
                {
                    .color = LED_NO_ONE,
                    .timeout = 700,
                },
            }
        },
		{
		            .light_states_count = 4,
		            .states = {
		                {
		                    .color = LED_GREEN,
		                    .timeout = 1500,
		                },
		                {
		                    .color = LED_NO_ONE,
		                    .timeout = 1500,
		                },
		                {
		                    .color = LED_RED,
		                    .timeout = 1500,
		                },
		                {
		                    .color = LED_NO_ONE,
		                    .timeout = 1500,
		                },
		            }
		  },
		  {
		            .light_states_count = 5,
		            .states = {
		                {
		                    .color = LED_RED,
		                    .timeout = 2100,
		                },
		                {
		                    .color = LED_YELLOW,
		                    .timeout = 2100,
		                },
		                {
		                    .color = LED_NO_ONE,
		                    .timeout = 2100,
		                },
		                {
		                    .color = LED_GREEN,
		                    .timeout = 2100,
		                },
		                {
		                    .color = LED_NO_ONE,
		                    .timeout = 2100,
		                },
		            }
		   }
};

uint32_t pred_led_numbers[8];
uint32_t start_time, current_time;
uint32_t current_delay;
uint32_t current_mode = 0;
uint32_t current_led_number = 0;
uint8_t MODES_COUNT = 4;
bool remaining_timeouts_input = false;
uint8_t prev_mode_n = 5;

void garland_modes(void){
			current_time = HAL_GetTick();

		  	  if (is_btn_clicked()) {
		  		  pred_led_numbers[current_mode] = current_led_number;
		  		  current_mode = (current_mode + 1) % MODES_COUNT;
		  		  current_delay = modes[current_mode].states[pred_led_numbers[current_mode]].timeout;
		  		  current_led_number = pred_led_numbers[current_mode];

		  		  printf("current_mode - %d\n", current_mode);
		  		  printf("current_delay - %d\n", current_delay);
		  		  printf("current_led_number - %d\n", current_led_number);
		  	  }

		  	  if (current_time - start_time >= current_delay) {
		  		  int modes_count = modes[current_mode].light_states_count;
		  		  if (modes_count <= current_led_number || modes_count < 0) return;
		  		  if (modes[current_mode].states[current_led_number].color == LED_NO_ONE) {
		  				 set_no_one_led();
		  		  }
		  		  else if (modes[current_mode].states[current_led_number].color == LED_RED) {
		  			  	 set_green_led(false);
		  			  	 set_yellow_led(false);
		  				 set_red_led(true);
		  		  }
		  		  else if (modes[current_mode].states[current_led_number].color == LED_GREEN) {
		  			  	 set_red_led(false);
		  			  	 set_yellow_led(false);
		  				 set_green_led(true);
		  		  }
		  		  else {
		  			  	 set_red_led(false);
		  			  	 set_green_led(false);
		  				 set_yellow_led(true);
		  		  }
		  		  current_led_number = (current_led_number + 1) % modes[current_mode].light_states_count;
		  		  start_time = HAL_GetTick();
		  	  }
}

struct RingBuffer {
	char data[BUF_SIZE];
	uint8_t head;
	uint8_t tail;
	bool empty;
};
typedef struct RingBuffer RingBuffer;
static RingBuffer buf_transmit;
static RingBuffer buf_receive;

static void buf_init(RingBuffer *buf) {
  buf->head = 0;
  buf->tail = 0;
  buf->empty = true;
}

static void buf_push(RingBuffer* buf, char* el) {
	pred_flag = flag;
	flag = false;
	  uint64_t size = strlen(el);
	  if (buf->head + size > BUF_SIZE) {
	    buf->head = 0;
	  }

	  strcpy(&buf->data[buf->head], el);
	  buf->head += size;

	  if (buf->head == BUF_SIZE) {
	    buf->head = 0;
	  }

	  buf->empty = false;
	flag = pred_flag;
}


static bool buf_pop(RingBuffer *buf, /* out */ char *el) {
	pred_flag = flag;
		flag = false;
  if (buf->empty) {
    return false;
  }
  uint64_t size = strlen(&buf->data[buf->tail]);

  strcpy(el, &buf->data[buf->tail]);
  buf->tail += size;
 // TODO: почему \0?
  if (buf->tail == BUF_SIZE) {
    buf->tail = 0;
  }

  if (buf->tail == buf->head) {
    buf->empty = true;
  }
  flag = pred_flag;

  return true;
}

struct InterruptStatus {
    bool interrupt_enable;
    uint32_t pmask;
};
typedef struct InterruptStatus InterruptStatus;
static InterruptStatus interrupt_status;

void enable_interrupt(InterruptStatus* status) {
	HAL_UART_Abort(&huart6);
	HAL_UART_Abort_IT(&huart6);
	HAL_NVIC_EnableIRQ(USART6_IRQn);
	status->interrupt_enable = true;
}

void disable_interrupt(InterruptStatus* status) {
	HAL_UART_Abort(&huart6);
	HAL_UART_Abort_IT(&huart6);
	HAL_NVIC_DisableIRQ(USART6_IRQn);
	status->interrupt_enable = false;
}

bool transmit_busy = false;

void transmit_uart(InterruptStatus* status, char* buf, size_t siz) {
	if (status->interrupt_enable) {
		if (transmit_busy) {
			buf_push(&buf_transmit, buf);
		}
		else {
			transmit_busy = true;
			HAL_UART_Transmit_IT(&huart6, buf, siz);
		}
	}
	else {
		// чтобы не прерывало ничего другое наше это ожидание
		pred_flag = flag;
			flag = false;
		HAL_UART_Transmit(&huart6, buf, siz, 100);
		flag = pred_flag;
	}
}

char el;
bool pred_it = false;

void receive_uart(InterruptStatus* status) {
	if (status->interrupt_enable) {
		HAL_UART_Receive_IT(&huart6, (uint8_t*) &el, sizeof(char));
	}
	else {
		HAL_StatusTypeDef stat = HAL_UART_Receive(&huart6, &el, sizeof(char), 0);
		 switch (stat) {
		  case HAL_OK: {
		    if (el != '\r') {
				buf_push(&buf_receive, &el);
				transmit_uart(&interrupt_status, &el, 1);
			}
			else {
				pred_flag = flag;
					flag = false;
					pred_it = status->interrupt_enable;
				handle_command_line();
				//если мы были в без прерываний то true
				//если мы были в прерываниях то пока false меняем в колбеке
				if (transmit_busy) flag = false;
				else flag = true;
			}
		    break;
		  }
		  case HAL_ERROR:
		  case HAL_BUSY:
		  case HAL_TIMEOUT:
		    return;
		  }
	}
}

void transmit_uart_nl(const struct InterruptStatus *status, char *buf, size_t size) {
	pred_flag = flag;
		flag = false;
  transmit_uart(status, buf, size);
  transmit_uart(status, "\r\n", 2);
  flag = pred_flag;
}

bool string_equals(const char * a, const char * b) {
	return strcmp(a, b) == 0;
}

bool starts_with(const char * prefix, const char * str) { return strncmp(prefix, str, strlen(prefix)) == 0; }
char interrupt_enabled_msg[] = {"Interrupts ON"};
char interrupt_disabled_msg[] = {"Interrupts OFF"};
char timeout_input_msg[] = {"Print timeout for new_mode - 100, 500 or 1500"};
char incorrect_command_msg[] = {"No such command"};
char incorrect_parameters_amount_msg[] = {"Parameters amount must be from 2 to 8"};
char incorrect_led_light_msg[] = {"Incorrect led light parameter. Must be: r, g, y or n"};
char incorrect_timeout_msg[] = {"Incorrect timeout"};
char incorrect_mode_number_msg[] = {"Incorrect mode number"};

void handle_new_command(const char* command) {
	const char* const pattern = command + 4; // set pointer after 'new '
	const uint32_t pattern_length = strlen(pattern);
	if (pattern_length < 2 || pattern_length > 8) {
		transmit_uart_nl(&interrupt_status, incorrect_parameters_amount_msg, sizeof(incorrect_parameters_amount_msg));
		return;
	}
	new_mode.light_states_count = pattern_length;
	for (uint8_t i = 0; i < pattern_length; ++i)
		switch (pattern[i]) {
			case 'n':
				new_mode.states[i].color = LED_NO_ONE;
				break;
			case 'r':
				new_mode.states[i].color = LED_RED;
				break;
			case 'g':
				new_mode.states[i].color = LED_GREEN;
				break;
			case 'y':
				new_mode.states[i].color = LED_YELLOW;
				break;
			default:
				transmit_uart_nl(&interrupt_status, incorrect_led_light_msg, sizeof(incorrect_led_light_msg));
				return;
		}
	transmit_uart_nl(&interrupt_status, timeout_input_msg, sizeof(timeout_input_msg));
	remaining_timeouts_input = true;
}

void handle_new_command_timeout(const char* command) {
	uint32_t timeout;
	uint8_t index;
	if (sscanf(command, "%lu", &timeout) != 1) {
		transmit_uart_nl(&interrupt_status, incorrect_timeout_msg, sizeof(incorrect_timeout_msg));
		return;
	}
	if (timeout != 100 && timeout != 500 && timeout != 1500) {
		transmit_uart_nl(&interrupt_status, incorrect_timeout_msg, sizeof(incorrect_timeout_msg));
		return;
	}
	for (uint8_t i = 0; i < new_mode.light_states_count; i++) {
		new_mode.states[i].timeout = timeout;
	}
	if (MODES_COUNT == 8) {
		memcpy(modes + prev_mode_n, &(new_mode), sizeof(new_mode));
		index = prev_mode_n;
		if(prev_mode_n < 8){
			prev_mode_n++;
		}
		else{
			prev_mode_n = 5;
		}
	}
	else {
		memcpy(modes + MODES_COUNT, &(new_mode), sizeof(new_mode));
		index = MODES_COUNT;
		MODES_COUNT++;
	}
	char ind_str[1];
	sprintf(ind_str, "%u", index);
	transmit_uart_nl(&interrupt_status, ind_str, sizeof(ind_str));
	remaining_timeouts_input = false;
}

void set_active_mode(uint8_t mode_number) {
	if (mode_number < MODES_COUNT && mode_number >= 0) {
		//чтобы менять переменные
		pred_flag = flag;
		flag = false;
		pred_led_numbers[current_mode] = current_led_number;
		current_mode = mode_number;
		current_delay = modes[current_mode].states[pred_led_numbers[current_mode]].timeout;
		current_led_number = pred_led_numbers[current_mode];

		printf("current_mode - %d\n", current_mode);
		printf("current_delay - %d\n", current_delay);
		printf("current_led_number - %d\n", current_led_number);
		flag = pred_flag;
	}
	else {
		transmit_uart_nl(&interrupt_status, incorrect_mode_number_msg, sizeof(incorrect_mode_number_msg));
	}
}

void handle_command_line() {
	//чтобы после переключения из прерываний в по опросу он не зависал в ресиве пока все не допереключали
	pred_flag = flag;
	flag = false;
	printf("handle command\n");
	char command[buf_receive.head + 1];
	buf_pop(&buf_receive, command);
	command[sizeof(command) - 1] = '\0';
	printf("%s\n", command);

	if (string_equals("set interrupts on", command)) {
		enable_interrupt(&interrupt_status);
		transmit_uart(&interrupt_status, "\n", 1);
		transmit_uart_nl(&interrupt_status, interrupt_enabled_msg, sizeof(interrupt_enabled_msg));
	}
	else if (string_equals("set interrupts off", command)) {
		disable_interrupt(&interrupt_status);
		transmit_uart(&interrupt_status, "\n", 1);
		transmit_uart_nl(&interrupt_status, interrupt_disabled_msg, sizeof(interrupt_disabled_msg));
	}
	else if (starts_with("set ", command)) {
		transmit_uart(&interrupt_status, "\n", 1);
		const char* const mode_idx_str = command + 4;
		uint32_t mode_idx;
		if ((sscanf(mode_idx_str, "%lu", &mode_idx) == 1)) {
			set_active_mode(mode_idx);
		}
		else {
			transmit_uart_nl(&interrupt_status, incorrect_mode_number_msg, sizeof(incorrect_mode_number_msg));
		}
	}
	else if (starts_with("new ", command)) {
		transmit_uart(&interrupt_status, "\n", 1);
		handle_new_command(command);
	}
	else if (remaining_timeouts_input) {
		transmit_uart(&interrupt_status, "\n", 1);
		handle_new_command_timeout(command);
	}
	else {
		transmit_uart(&interrupt_status, "\n", 1);
		transmit_uart_nl(&interrupt_status, incorrect_command_msg, sizeof(incorrect_command_msg));
	}
	flag = pred_flag;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	pred_flag = flag;
		flag = false;
	if (el != '\r') {
		buf_push(&buf_receive, &el);
		transmit_uart(&interrupt_status, &el, 1);
		flag = pred_flag;
	}
	else {
		handle_command_line();
		flag = true;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	pred_flag = flag;
		flag = false;
  char buf[1024];
  if (buf_pop(&buf_transmit, buf)) {
	 printf("%d\n", strlen(buf));
	 HAL_UART_Transmit_IT(&huart6, buf, strlen(buf));
	 flag = pred_flag;
  } else {
    transmit_busy = false;
    flag = true;
  }

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  initialise_monitor_handles();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  disable_interrupt(&interrupt_status);
  buf_init(&buf_transmit);
  buf_init(&buf_receive);

  start_time = HAL_GetTick();
  current_delay = modes[0].states[0].timeout;
  pred_led_numbers[0] = modes[0].light_states_count - 1;
  pred_led_numbers[1] = modes[1].light_states_count - 1;
  pred_led_numbers[2] = modes[2].light_states_count - 1;
  pred_led_numbers[3] = modes[3].light_states_count - 1;
  pred_led_numbers[4] = 0;
  pred_led_numbers[5] = 0;
  pred_led_numbers[6] = 0;
  pred_led_numbers[7] = 0;

  /*
  if (interrupt_status.interrupt_enable) {
  	    transmit_uart_nl(&interrupt_status, interrupt_enabled_msg, sizeof(interrupt_disabled_msg));
  } else {
  	    transmit_uart_nl(&interrupt_status, interrupt_disabled_msg, sizeof(interrupt_enabled_msg));
  }
  */
  //HAL_UART_Transmit(&huart6, &buf_transmit, 6, 100);
  while (1)
  {
	  garland_modes();
	  if (flag == true) receive_uart(&interrupt_status);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
