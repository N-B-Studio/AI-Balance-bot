/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include <string.h>
#include <stdio.h>
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

/* USER CODE BEGIN PV */
// --- speed loop config ---
	float v_target = 0.0f;          // 目标线速度(可以先=0，后面接遥控/手动)
	float Kv_p     = 0.1f;          // 速度P增益 (自己慢慢调)
	float Kv_i     = 0.01f;          // 速度I增益 (先小一点)

	float v_integral      = 0.0f;
	float v_integral_limit = 5.0f;  // 防积分饱和

	float base_pitch       = -1.5f;   // keep your calibrated base
	float pitch_speed_limit = 6.0f;   // allow ±6° tilt from joystick
	float torque_limit      = 1.5f;   // give motors more torque headroom


	// 速度环频率：1kHz / 10 = 100 Hz
	#define VEL_LOOP_DIV 10
	uint8_t vel_loop_counter = 0;

	volatile float odrive_left_vel  = 0.0f;
	volatile float odrive_right_vel = 0.0f;
	volatile float odrive_left_pos  = 0.0f;
	volatile float odrive_right_pos = 0.0f;

	// ===== YAW 控制 =====
	float yaw               = 0.0f;      // 当前 yaw 角（积分出来的大概值）
	float Ky_p              = 0.0f;    // yaw 角度 P
	float Ky_d              = 0.02f;    // yaw 角速度 D
	float yaw_torque_limit  = 0.4f;      // yaw 能占用的最大力矩

	// 轮子 / 机器人几何参数（按你实际修改）
	#define WHEEL_RADIUS_M   0.06f   // 5cm 半径举例
	#define GEAR_RATIO       1.0f    // 直驱就填 1，减速箱就填减速比

	// 位置环
	float x_target      = 0.0f;     // 希望机器人停在 x=0
	float Kx_p          = 0.0f;     // 位置 P 增益（先 0.5~2 之间试）
	float Kx_i          = 0.0f;     // 位置 I（先关掉，调好再开）
	float x_integral    = 0.0f;
	float x_int_limit   = 0.5f;     // 积分限幅（米）

	// 记录上电时的初始平均位置
	float x0_turns = 0.0f;          // 以“轮子转数”为原点
	uint8_t pos_initialized = 0;    // 是否初始化过

	// 全局：
	volatile float odrive_left_pos_turns  = 0.0f;
	volatile float odrive_right_pos_turns = 0.0f;
	volatile float odrive_left_vel_turns  = 0.0f;
	volatile float odrive_right_vel_turns = 0.0f;

	// === RC command from Pi (forward/back & left/right) ===
	volatile float rc_fb = 0.0f;   // -1.0 .. +1.0 (forward/back)
	volatile float rc_lr = 0.0f;   // -1.0 .. +1.0 (left/right turn)

	static uint8_t uart10_rx_byte = 0;

	// UART10 simple frame parser state
	typedef enum {
	    RC_STATE_WAIT_AA = 0,
	    RC_STATE_WAIT_55,
	    RC_STATE_FB,
	    RC_STATE_LR,
	    RC_STATE_CHK
	} RC_State_t;

	static RC_State_t rc_state = RC_STATE_WAIT_AA;
	static int8_t rc_fb_int = 0;
	static int8_t rc_lr_int = 0;
	static uint8_t rc_checksum = 0;
	static uint8_t rc_sum = 0;
	// debug:
	volatile uint32_t rc_byte_count      = 0;   // total bytes seen on USART1
	volatile uint32_t rc_frame_ok_count  = 0;   // frames with correct checksum
	volatile int8_t   rc_last_fb_int     = 0;
	volatile int8_t   rc_last_lr_int     = 0;

	volatile uint8_t  rc_last_byte       = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "bsp_fdcan.h"

// ===== Debug: loop frequency & UART print =====
char uart_buf[128];
uint32_t loop_counter  = 0;
uint32_t last_print_ms = 0;

// ===== Mini ODrive CAN constants =====
#define ODRIVE_LEFT_NODE_ID   2   // CAN1
#define ODRIVE_RIGHT_NODE_ID  3   // CAN2

#define CMD_SET_AXIS_STATE       0x07
#define CMD_SET_CONTROLLER_MODE  0x0B
#define CMD_SET_INPUT_TORQUE     0x0E
#define CMD_GET_ENCODER_ESTIMATES  0x09

#define AXIS_STATE_IDLE          1
#define AXIS_STATE_CLOSED_LOOP   8

#define CONTROL_MODE_TORQUE      1
#define INPUT_MODE_PASSTHROUGH   1

// 1 kHz loop
volatile uint8_t imu_loop_flag = 0;
volatile float   pitch = 0.0f;
volatile float   roll  = 0.0f;

#define RAD2DEG 57.2957795f
const float dt = 0.001f;    // 1 kHz

// ===== Balance controller gains (tune these) =====
float Kp = 0.004f;            // proportional on angle (deg)
float Kd = 0.02f;           // derivative on rate (deg/s)
float Ki = 0.008f;            // you can start with 0

float pitch_target = 0.0f;
float integral_term = 0.0f;
float integral_limit = 10.0f;

// ===== CAN helper =====
static inline uint16_t odrive_build_id(uint8_t node_id, uint8_t cmd_id)
{
    return ((uint16_t)node_id << 5) | (cmd_id & 0x1F);
}

void odrive_set_axis_state(FDCAN_HandleTypeDef *hcan, uint8_t node_id, uint32_t state)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(state & 0xFF);
    data[1] = (uint8_t)((state >> 8) & 0xFF);
    data[2] = (uint8_t)((state >> 16) & 0xFF);
    data[3] = (uint8_t)((state >> 24) & 0xFF);

    uint16_t id = odrive_build_id(node_id, CMD_SET_AXIS_STATE);
    fdcanx_send_data(hcan, id, data, 4);
}

void odrive_set_controller_mode(FDCAN_HandleTypeDef *hcan,
                                uint8_t node_id,
                                uint32_t control_mode,
                                uint32_t input_mode)
{
    uint8_t data[8];

    data[0] = (uint8_t)(control_mode & 0xFF);
    data[1] = (uint8_t)((control_mode >> 8) & 0xFF);
    data[2] = (uint8_t)((control_mode >> 16) & 0xFF);
    data[3] = (uint8_t)((control_mode >> 24) & 0xFF);

    data[4] = (uint8_t)(input_mode & 0xFF);
    data[5] = (uint8_t)((input_mode >> 8) & 0xFF);
    data[6] = (uint8_t)((input_mode >> 16) & 0xFF);
    data[7] = (uint8_t)((input_mode >> 24) & 0xFF);

    uint16_t id = odrive_build_id(node_id, CMD_SET_CONTROLLER_MODE);
    fdcanx_send_data(hcan, id, data, 8);
}

void odrive_set_input_torque(FDCAN_HandleTypeDef *hcan, uint8_t node_id, float torque)
{
    uint8_t data[8] = {0};

    union {
        float f;
        uint8_t b[4];
    } u;
    u.f = torque;

    data[0] = u.b[0];
    data[1] = u.b[1];
    data[2] = u.b[2];
    data[3] = u.b[3];

    uint16_t id = odrive_build_id(node_id, CMD_SET_INPUT_TORQUE);
    fdcanx_send_data(hcan, id, data, 4);
}


typedef struct {
    float pos_turns;   // 位置，turns
    float vel_turns_s; // 速度，turns/s
} OdriveState;

OdriveState odrive_get_state(FDCAN_HandleTypeDef *hcan, uint8_t node_id)
{
    OdriveState st = {0.0f, 0.0f};

    uint16_t req_id = odrive_build_id(node_id, CMD_GET_ENCODER_ESTIMATES);

    uint8_t tx_data[8] = {0};
    fdcanx_send_data(hcan, req_id, tx_data, 0);  // 请求

    uint32_t start = HAL_GetTick();
    uint16_t rec_id;
    uint8_t rx_buf[8];

    while ((HAL_GetTick() - start) < 2)   // 最多等 2ms
    {
        uint8_t len = fdcanx_receive(hcan, &rec_id, rx_buf);
        if (len > 0 && rec_id == req_id)
        {
            union { float f; uint8_t b[4]; } u_pos, u_vel;
            u_pos.b[0] = rx_buf[0];
            u_pos.b[1] = rx_buf[1];
            u_pos.b[2] = rx_buf[2];
            u_pos.b[3] = rx_buf[3];

            u_vel.b[0] = rx_buf[4];
            u_vel.b[1] = rx_buf[5];
            u_vel.b[2] = rx_buf[6];
            u_vel.b[3] = rx_buf[7];

            st.pos_turns   = u_pos.f;
            st.vel_turns_s = u_vel.f;
            return st;
        }
    }

    return st; // 没收到就全 0
}

// === RC UART frame parser: 0xAA 0x55 fb_int lr_int checksum ===
static void RC_UART_ProcessByte(uint8_t b)
{
    switch (rc_state)
    {
    case RC_STATE_WAIT_AA:
        if (b == 0xAA) {
            rc_state = RC_STATE_WAIT_55;
            rc_sum = b;
        }
        break;

    case RC_STATE_WAIT_55:
        if (b == 0x55) {
            rc_state = RC_STATE_FB;
            rc_sum += b;
        } else {
            rc_state = RC_STATE_WAIT_AA;
        }
        break;

    case RC_STATE_FB:
        rc_fb_int = (int8_t)b;
        rc_sum += b;
        rc_state = RC_STATE_LR;
        break;

    case RC_STATE_LR:
        rc_lr_int = (int8_t)b;
        rc_sum += b;
        rc_state = RC_STATE_CHK;
        break;

    case RC_STATE_CHK:
        rc_checksum = b;
        if ((rc_sum & 0xFF) == rc_checksum)
        {
            rc_fb = (float)rc_fb_int / 100.0f;
            rc_lr = (float)rc_lr_int / 100.0f;

            rc_frame_ok_count++;
            rc_last_fb_int = rc_fb_int;
            rc_last_lr_int = rc_lr_int;
        }
        rc_state = RC_STATE_WAIT_AA;
        break;

    default:
        rc_state = RC_STATE_WAIT_AA;
        break;
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
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_USART10_UART_Init();
  /* USER CODE BEGIN 2 */

  // Start UART10 RX interrupt for RC frames from Pi
  HAL_UART_Receive_IT(&huart10, &uart10_rx_byte, 1);
  // GPIO, FDCAN, USART, TIM, SPI already in MX_*_Init();

    // Enable motor power (adjust pins if needed)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_Delay(20);

    // Init CANs (this calls HAL_FDCAN_Start etc.)
    bsp_can_init();
    HAL_Delay(200);

    // Init IMU
    uint8_t imu_err = BMI088_init();
    // (optional) print ok/err via UART here

    // Start TIM2 1kHz interrupt
    HAL_TIM_Base_Start_IT(&htim2);

    // Put both ODrives into CLOSED_LOOP torque mode
    odrive_set_axis_state(&hfdcan1, ODRIVE_LEFT_NODE_ID,  AXIS_STATE_CLOSED_LOOP);
    HAL_Delay(100);
    odrive_set_controller_mode(&hfdcan1, ODRIVE_LEFT_NODE_ID, CONTROL_MODE_TORQUE, INPUT_MODE_PASSTHROUGH);
    HAL_Delay(100);

    odrive_set_axis_state(&hfdcan2, ODRIVE_RIGHT_NODE_ID, AXIS_STATE_CLOSED_LOOP);
    HAL_Delay(100);
    odrive_set_controller_mode(&hfdcan2, ODRIVE_RIGHT_NODE_ID, CONTROL_MODE_TORQUE, INPUT_MODE_PASSTHROUGH);
    HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    float gyro[3];
    float accel[3];
    float temp = 0.0f;

    float prev_error = 0.0f;

    // === Init debug timing ===
    last_print_ms = HAL_GetTick();
    // add this:
    pitch_target = base_pitch;
  while (1)
  {
	  if (!imu_loop_flag)
	           continue;
	       imu_loop_flag = 0;

	       // 1) Read IMU
	       BMI088_read(gyro, accel, &temp);

	       // 2) Complementary filter to update pitch / roll
	       // (reuse your old code; sketch shown here)

	       const float alpha = 0.98f;
	       float ax = accel[0];
	       float ay = accel[1];
	       float az = accel[2];
	       float gx = gyro[0];   // deg/s
	       float gy = gyro[1];   // deg/s
	       float gz = gyro[2];   // deg/s  <-- YAW

	       float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
	       float roll_acc  = atan2f( ay, az ) * RAD2DEG;

	       // assuming gy is pitch rate
	       pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * pitch_acc;
	       roll  = alpha * (roll  + gx * dt) + (1.0f - alpha) * roll_acc;
	       yaw += gz * dt;   // yaw 单位：deg，大概朝向，慢慢会漂

	       // ==================== vel + pos loop ===========================
	       vel_loop_counter++;
	       if (vel_loop_counter >= VEL_LOOP_DIV)
	       {
	           vel_loop_counter = 0;

	           // 1) 读取 ODrive 状态（保留，将来用位置/速度）
	           OdriveState st_left  = odrive_get_state(&hfdcan1, ODRIVE_LEFT_NODE_ID);
	           OdriveState st_right = odrive_get_state(&hfdcan2, ODRIVE_RIGHT_NODE_ID);

	           odrive_left_pos_turns  = st_left.pos_turns;
	           odrive_right_pos_turns = st_right.pos_turns;
	           odrive_left_vel_turns  = st_left.vel_turns_s;
	           odrive_right_vel_turns = st_right.vel_turns_s;

	           if (!pos_initialized)
	           {
	               x0_turns = 0.5f * (odrive_left_pos_turns + odrive_right_pos_turns);
	               pos_initialized = 1;
	           }

	           // === RC 前后：直接映射到俯仰角偏移 ===
	           float pitch_offset = rc_fb * pitch_speed_limit;   // rc_fb ∈ [-1, 1]

	           pitch_target = base_pitch - pitch_offset;
	       }



	       // =================== 姿态（pitch）PID =======================
	       float error = pitch_target - pitch;     // target = 0 deg

	       integral_term += error * dt;
	       if (integral_term >  integral_limit) integral_term =  integral_limit;
	       if (integral_term < -integral_limit) integral_term = -integral_limit;

	       float derivative = -gy;   // gy>0 前倾时，derivative 为负来“刹车”

	       float torque_cmd = Kp * error + Kd * derivative + Ki * integral_term;

	       // 限幅
	       if (torque_cmd >  torque_limit) torque_cmd =  torque_limit;
	       if (torque_cmd < -torque_limit) torque_cmd = -torque_limit;


	       // =================== YAW PID（差分力矩） =======================
	       // rc_lr ∈ [-1, +1]  →   yaw_torque ∈ [-yaw_torque_limit, +yaw_torque_limit]

	       float yaw_torque = rc_lr * 0.1 * yaw_torque_limit;  // 主控制：遥控器

	       // 可选：用陀螺仪做一点阻尼，防止发散（类似 D）
	       float yaw_damping = Ky_d * gz;   // gz > 0 转向时，产生反向力矩
	       yaw_torque -= yaw_damping;

	       // 限幅
	       if (yaw_torque >  yaw_torque_limit) yaw_torque =  yaw_torque_limit;
	       if (yaw_torque < -yaw_torque_limit) yaw_torque = -yaw_torque_limit;

	       // =================== 合成左右轮力矩 =======================
	       // pitch 用共模, yaw 用差模
	       float left_torque  = torque_cmd + yaw_torque;
	       float right_torque = torque_cmd - yaw_torque;

	       // （如果发现 yaw 方向反了，左右加减号对调即可）

	       odrive_set_input_torque(&hfdcan1, ODRIVE_LEFT_NODE_ID,  -left_torque);
	       odrive_set_input_torque(&hfdcan2, ODRIVE_RIGHT_NODE_ID, right_torque);

	       prev_error = error;

	       // ===== DEBUG: measure loop frequency & print pitch =====
			 loop_counter++;
			 uint32_t now = HAL_GetTick();   // ms since boot

			 // print every 100 ms (10 Hz) so UART doesn't destroy timing
			 if ((now - last_print_ms) >= 100)
			 {
				 float elapsed_ms = (float)(now - last_print_ms);
				 float loop_hz    = (loop_counter * 1000.0f) / elapsed_ms;

				 int n = snprintf(uart_buf, sizeof(uart_buf),
				                  "[H725] pitch=%.2f pt=%.2f fb=%.2f lr=%.2f fb_int=%d frames=%lu loop=%.1fHz\r\n",
				                  pitch, pitch_target, rc_fb, rc_lr,
				                  (int)rc_fb_int,
				                  rc_frame_ok_count,
				                  loop_hz);

				 HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, n, 10);

				 loop_counter  = 0;
				 last_print_ms = now;
			 }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        imu_loop_flag = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART10)
    {
        rc_byte_count++;
        rc_last_byte = uart10_rx_byte;

        RC_UART_ProcessByte(uart10_rx_byte);

        // re-arm RX on UART10 (IMPORTANT: huart10, NOT huart1)
        HAL_UART_Receive_IT(&huart10, &uart10_rx_byte, 1);
    }
}


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
#ifdef USE_FULL_ASSERT
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
