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
 * 本文件内含BMI270陀螺仪的使用示例
 * 本文件内含无刷电桥的启动初始化的使用示例
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dodo_BMI270.h" //陀螺仪驱动
#include "multiplexer.h" //多路复用器驱动，用于读取光电管读数
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 陀螺仪调试打印开关。置 1 会引入约 5.4 kB 的浮点 printf 代码，
   MDK-Lite 的 32 kB 链接上限会因此报 L6047U 而链接失败。
   平时保持 0；要串口看数据时改成 1（并注意镜像大小）。 */
#define GYRO_DEBUG 0
#define IF_start_Brushless_motor 0 //是否启动无刷电机，1为启动，0为不启动
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_tim1_up;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//负压启动相关宏
//负压启动流程：900us脉宽50Hzpwm信号输出2秒后，改为持续2000us脉宽50Hzpwm信号输出
#define ESC_PULSE_SLOTS       200U       /* 20 ms at the 10 kHz TIM1 update rate */
#define ESC_START_PULSE_SLOTS 9U         /* 0.9 ms */
#define ESC_RUN_PULSE_SLOTS   20U        /* 2.0 ms */
#define ESC_START_PERIODS     100U       /* 100 x 20 ms = 2 s */

static uint32_t esc_dma_table[ESC_PULSE_SLOTS];
static uint32_t esc_high_slots;    /* high time currently stored in the table */
static volatile uint32_t esc_period_count;

/* Fill a whole 20 ms frame.  Only used once, before the DMA is started. */
static void ESC_BuildDmaTable(uint32_t high_slots)
{
  uint32_t i;

  for (i = 0U; i < ESC_PULSE_SLOTS; ++i)
  {
    esc_dma_table[i] = (i < high_slots) ? GPIO_PIN_11 : ((uint32_t)GPIO_PIN_11 << 16U);
  }
  esc_high_slots = high_slots;
}

/* Rewrite only the slots whose level really changes, in increasing index
   order: the circular DMA consumes the table in that same order, and slot 9
   is not transferred before ~0.9 ms after this interrupt, so the frame that
   is already running is never disturbed. */
static void ESC_SetPulseWidth(uint32_t high_slots)
{
  uint32_t first;
  uint32_t last;
  uint32_t i;

  if (high_slots == esc_high_slots)
  {
    return;
  }

  first = (high_slots < esc_high_slots) ? high_slots : esc_high_slots;
  last  = (high_slots > esc_high_slots) ? high_slots : esc_high_slots;

  for (i = first; i < last; ++i)
  {
    esc_dma_table[i] = (i < high_slots) ? GPIO_PIN_11 : ((uint32_t)GPIO_PIN_11 << 16U);
  }
  esc_high_slots = high_slots;
}
static void ESC_fullPulseWidth(void){
  uint32_t i;
  for (i = 0U; i < ESC_PULSE_SLOTS; ++i)
  {
    esc_dma_table[i] = GPIO_PIN_11;
  }
  esc_high_slots = ESC_PULSE_SLOTS;
}
/* One interrupt per 20 ms frame: DMA transfer complete in circular mode.
   The 50 Hz waveform itself is produced by TIM1 + DMA in hardware; this
   handler only counts frames and switches 0.9 ms -> 2.0 ms after 2 s. */
static void ESC_DmaTransferComplete(DMA_HandleTypeDef *hdma)
{
  if (esc_period_count < ESC_START_PERIODS)
  {
    ++esc_period_count;
    if (esc_period_count == ESC_START_PERIODS)
    {
      /* The frame that has just been completed was the last 0.9 ms frame:
         100 x 20 ms = 2 s have elapsed, 2.0 ms is now held forever. */
      if (!IF_start_Brushless_motor)
        ESC_SetPulseWidth(10U);
      else
        ESC_SetPulseWidth(ESC_RUN_PULSE_SLOTS);
      /* Nothing left to time, so stop the 50 Hz interrupt. */
      __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC);
    }
  }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ==================== 陀螺仪全局量 ====================
 * 必须先声明：按键清零、主循环、陀螺仪函数都要用。
 */
#define GYRO_SIGN   (-1)          /* 实测：顺时针(右转)角速度为负，乘 -1 让「右转为正」 */

float gyro_x, gyro_y, gyro_z;     /* °/s，z 轴已乘 GYRO_SIGN */
float accel_x, accel_y, accel_z;  /* g，暂未使用 */
float gyro_z_f     = 0.0f;        /* °/s，一阶低通后的角速度（给 D 项和积分用） */
float gyro_z_bias = 0.0f;         /* °/s，上电静止标定的零偏 */
float gyro_angle  = 0.0f;         /* °，z 轴积分得到的偏航角 */
static uint32_t gyro_t_ms;        /* 上一次积分的时间戳 */

/* 静止死区：(2σ)^2，标定时顺便算出。|ω-bias| 小于 2σ 视为静止，不积分，
 * 用来抑制"车不动但角度自己往上爬"。只比较平方，省掉 sqrtf。 */
float gyro_gate2  = 0.0f;
static float gyro_last_g = 0.0f;  /* 上一帧的 (ω-bias)，梯形积分用 */


/* 函数原型：ENC_Debug_KeyReset() 里要调用 Gyro_ResetAngle()，必须先声明 */
void Gyro_CalibBias(void);
void Gyro_ResetAngle(void);
void Gyro_Update(void);
void Gyro_Debug_Print(uint16_t period_ms);



/* ========== 电机方向极性：只改这两个宏 ======================================= */
#define L_DIR_FWD   GPIO_PIN_SET     /* 左轮"前进"时 L_DIR(PB15) 该给的电平 */
#define R_DIR_FWD   GPIO_PIN_RESET   /* 右轮"前进"时 R_DIR(PA10) 该给的电平 */
#define PWM_MAX     6500             /* 上限留余量，别顶到 7199 */
#define DEAD_L      300     /* 左轮死区 250 + 余量 */
#define DEAD_R      600     /* 右轮死区 500 + 余量 */

/* 正数 = 前进，负数 = 后退 */
static void Motor_Set(int16_t left, int16_t right)
{  
	/* 死区补偿：非零输出至少给到死区，否则电机不转、控制环却以为已经用力 */
  if (left  != 0 && left  > -DEAD_L && left  < DEAD_L)  left  = (left  > 0) ?  DEAD_L : -DEAD_L;
  if (right != 0 && right > -DEAD_R && right < DEAD_R)  right = (right > 0) ?  DEAD_R : -DEAD_R;
  /* ---- 左轮：TIM1_CH1 / PA8 ---- */
  if (left >= 0) {
    HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin, L_DIR_FWD);
    TIM1->CCR1 = (left > PWM_MAX) ? PWM_MAX : (uint32_t)left;
  } else {
    HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin,
                      (L_DIR_FWD == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    left = -left;
    TIM1->CCR1 = (left > PWM_MAX) ? PWM_MAX : (uint32_t)left;
  }

  /* ---- 右轮：TIM1_CH2 / PA9 ---- */
  if (right >= 0) {
    HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin, R_DIR_FWD);
    TIM1->CCR2 = (right > PWM_MAX) ? PWM_MAX : (uint32_t)right;
  } else {
    HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin,
                      (R_DIR_FWD == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    right = -right;
    TIM1->CCR2 = (right > PWM_MAX) ? PWM_MAX : (uint32_t)right;
  }
}


/* ==================== 编码器测试模块=========================== */
float ENC_FACTOR_L = 0.001918f;   /* 8 次推 1 m：100 / 52149 */
float ENC_FACTOR_R = 0.001951f;   /* 8 次推 1 m：100 / 51260 */
#define ENC_L_SIGN   (+1)   /* 手推前进时左轮计数应为正；是负的改 -1 */
#define ENC_R_SIGN   (-1)   /* 右轮同理 */
#define ENC_KEY_ACTIVE_LOW  1   /* PB14 按下为低电平；相反改 0 */
#define ENC_DT      0.02f
#define ENC_LPF_A   0.7f

static int16_t last3, last4;
static int32_t tot3, tot4;
static float   dist_cms, speed_cms;
static uint8_t div20;
static uint8_t  key_prev = 0U;
static uint32_t key_t0;

void ENC_Reset(void)
{
  last3 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  last4 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  tot3 = 0; tot4 = 0;
  dist_cms = 0.0f; speed_cms = 0.0f; div20 = 0;
}

void ENC_Init(void)
{
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  last3 = 0; last4 = 0;
  tot3 = 0; tot4 = 0;
  dist_cms = 0.0f; speed_cms = 0.0f; div20 = 0;
}

int32_t ENC_CountL(void) { return (int32_t)ENC_L_SIGN * tot3; }
int32_t ENC_CountR(void) { return (int32_t)ENC_R_SIGN * tot4; }

float ENC_Distance(void)
{
  return 0.5f * ((float)(ENC_L_SIGN * tot3) * ENC_FACTOR_L
               + (float)(ENC_R_SIGN * tot4) * ENC_FACTOR_R);
}


int32_t ENC_DistL_mm(void) { return (int32_t)((float)(ENC_L_SIGN * tot3) * ENC_FACTOR_L * 10.0f); }


int32_t ENC_DistR_mm(void) { return (int32_t)((float)(ENC_R_SIGN * tot4) * ENC_FACTOR_R * 10.0f); }


int32_t ENC_Dist_mm(void)  { return (int32_t)(ENC_Distance() * 10.0f); }


/* TIM2 更新中断里调用，1 ms 一次 */
void ENC_ISR_1ms(void)
{
  int16_t now3 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  int16_t now4 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

  tot3 += (int16_t)(now3 - last3);   /* 先差分再转 int16，回绕自动正确 */
  tot4 += (int16_t)(now4 - last4);
  last3 = now3;
  last4 = now4;

  if (++div20 >= 20) {              /* 每 20 ms 算一次速度 */
    float d = ENC_Distance();
    float v = (d - dist_cms) / ENC_DT;
    dist_cms  = d;
    speed_cms = ENC_LPF_A * speed_cms + (1.0f - ENC_LPF_A) * v;
    div20 = 0;
  }
}

/* 串口输出：ENC,左计数,右计数,里程mm,速度mm/s  （纯整数，不引 %f） */
void ENC_Debug_Print(uint16_t period_ms)
{
  static uint32_t t0;
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - t0) < period_ms) return;
  t0 = now;

  printf("ENC,%ld,%ld,%ld,%d\r\n",
         (long)ENC_CountL(),
         (long)ENC_CountR(),
         (long)(ENC_Distance() * 10.0f),
         (int)(speed_cms * 10.0f));
}

/* 按发车键 PB14 清零，省得反复烧录 */
void ENC_Debug_KeyReset(void)
{
  uint8_t  level   = (uint8_t)HAL_GPIO_ReadPin(GPIO_RUN_GPIO_Port, GPIO_RUN_Pin);
  uint8_t  pressed = ENC_KEY_ACTIVE_LOW ? (uint8_t)(level == 0U) : (uint8_t)(level != 0U);
  uint32_t t = HAL_GetTick();

  if (pressed && !key_prev && (uint32_t)(t - key_t0) > 30U) {
    key_t0 = t;
    ENC_Reset();
		Gyro_ResetAngle();          /* 按键同时清角度 */
    printf("ENC,RESET\r\n");
  }
  key_prev = pressed;
}

/* ==================== 直行：航向 PID + 里程闭环 ==================== */
#define STRAIGHT_BASE      800    /* 巡航 PWM */
#define STRAIGHT_DECEL_MM   200    /* 最后 20 cm 减速 */
#define STRAIGHT_DECEL_PWM  700   /* 减速终点，比 DEAD_R 高 */
#define STRAIGHT_PID_MAX    250    /* PID 修正量限幅（不含偏置） */
#define STRAIGHT_TURN_MAX   550    /* 最终差速限幅：1300-550=750 > DEAD_R */
#define STRAIGHT_TIMEOUT  8000U    /* 8 s 兜底 */

/* 调机模式：非 0 时【忽略整个 PID】，直接输出这个固定差速。
 * 用途：确认「正差速到底让车往左还是往右」，把符号问题彻底钉死。
 *   设 +150 跑一次、设 -150 再跑一次，两次方向必然相反。
 * 测完必须改回 0。 */
#define STRAIGHT_TEST_TURN   0

/* 右轮比左轮慢约 18%（同 PWM 下），车因此持续左偏
 * 这是【恒定扰动】，用固定偏置直接抵掉，比等积分慢慢长上来快得多。
 * 符号：turn 为负 → uR 变大 → 右轮快 → 车往右转。
 * 调法：跑一次看车偏哪边 —— 还往左偏就再减 50，变成右偏就往回收 50。 */
#define STRAIGHT_TURN_BIAS  (-39)

static float sp_kp = 45.0f;        /* 角度 → PWM */
static float sp_ki = 3.0f;         /* 积分，消残余恒定偏航（原 0.8 太慢） */
static float sp_kd = 4.0f;         /* 角速度阻尼 */
static float sp_i;                 /* 积分累加器 */
static float sp_turn;              /* 当前差速量，打印出来便于调 BIAS */

int32_t Drive_Straight_PID(int32_t target_mm)
{
  int32_t  start = ENC_Dist_mm();
  int32_t  rem   = target_mm;
  uint32_t t0    = HAL_GetTick();
  uint32_t t_ctl = t0;
  uint32_t t_prt = t0;

  sp_i = 0.0f;
  Gyro_ResetAngle();

  for (;;) {
    uint32_t now = HAL_GetTick();

    /* 本函数阻塞主循环，陀螺仪必须在这里读 */
    dodo_BMI270_get_data();
    gyro_z = GYRO_SIGN * BMI270_gyro_transition(BMI270_gyro_z);
    Gyro_Update();

    if ((uint32_t)(now - t0) > STRAIGHT_TIMEOUT) break;

    if ((uint32_t)(now - t_ctl) >= 10U) {          /* 10 ms 控制节拍 */
      float dt = (now - t_ctl) / 1000.0f;
      float e, turn;
      int16_t base, uL, uR;

      t_ctl = now;
      rem = target_mm - (ENC_Dist_mm() - start);
      if (rem <= 0) break;

      /* 距离 → 基础 PWM */
      if (rem > STRAIGHT_DECEL_MM)
        base = STRAIGHT_BASE;
      else
        base = (int16_t)(STRAIGHT_DECEL_PWM
             + (STRAIGHT_BASE - STRAIGHT_DECEL_PWM) * rem / STRAIGHT_DECEL_MM);

#if (STRAIGHT_TEST_TURN != 0)
      /* ---- 调机模式：忽略 PID，直接给固定差速 ----
       * 用来确认符号：设 +150 和 -150 各跑一次，车必然往相反方向转。 */
      (void)e; (void)dt;
      turn = STRAIGHT_TEST_TURN;
#else
      /* 航向 PID，目标 0° */
      e = -gyro_angle;
      sp_i += e * dt;
      if (sp_i >  100.0f) sp_i =  100.0f;      /* 3.0×100 = 300 PWM，足够覆盖不对称 */
      if (sp_i < -100.0f) sp_i = -100.0f;

      /* 先用滤波后的角速度做阻尼，避免振动噪声灌进 D 项 */
      turn = sp_kp * e + sp_ki * sp_i + sp_kd * (-gyro_z_f);
      if (turn >  STRAIGHT_PID_MAX) turn =  STRAIGHT_PID_MAX;
      if (turn < -STRAIGHT_PID_MAX) turn = -STRAIGHT_PID_MAX;

      /* 叠加固定偏置，抵掉右轮偏慢的恒定扰动 */
      turn += STRAIGHT_TURN_BIAS;
#endif
      if (turn >  STRAIGHT_TURN_MAX) turn =  STRAIGHT_TURN_MAX;
      if (turn < -STRAIGHT_TURN_MAX) turn = -STRAIGHT_TURN_MAX;
      sp_turn = turn;                          /* 供打印观察 */

      uL = (int16_t)(base + turn);             /* turn>0 = 往右修：左轮快 */
      uR = (int16_t)(base - turn);
      if (uL < 0) uL = 0;
      if (uR < 0) uR = 0;

      Motor_Set(uL, uR);
    }

    if ((uint32_t)(now - t_prt) >= 100U) {          /* 调参用打印 */
      t_prt = now;
      printf("V,%ld,%d,%d,%d\r\n", (long)rem,
             (int)(gyro_angle * 10.0f), (int)(gyro_z * 10.0f), (int)sp_turn);
    }
  }

  Motor_Set(0, 0);
  printf("DIST,%ld\r\n", (long)(ENC_Dist_mm() - start));
  return ENC_Dist_mm() - start;
}
/* ============ 光电管调试：12 路位图 + 加权偏差 ============ */
uint16_t mux_v;

void Sensor_Debug_Print(uint16_t period_ms)
{
  static uint32_t t0;
  uint32_t now = HAL_GetTick();
  /* 与循迹使用的非线性权重一致（×10 便于整数打印）：
   * 边缘重、中心轻 —— 直道更稳、弯道更狠 */
  static const int w[12] = {-70,-50,-35,-20,-10,-2, 2,10,20,35,50,70};
  char s[13];
  int sum10 = 0, n = 0;

  if ((uint32_t)(now - t0) < period_ms) return;
  t0 = now;

  MUX_get_value(&mux_v);

  for (int i = 0; i < 12; i++) {
    int b = MUX_GET_CHANNEL(mux_v, i);
    s[i] = b ? '1' : '0';
    if (b) { sum10 += w[i]; n++; }
  }
  s[12] = '\0';
  if (n) sum10 /= n; else sum10 = 0;   /* 丢线时先打 0，正式代码再保持上次 */

  printf("S,%s,%d\r\n", s, sum10);
}


/* ============ 陀螺仪：零偏标定 + 角度积分 ============ */
/* 车静止时调用一次，阻塞约 0.4 s。标定时须关闭负压风扇，否则振动污染零偏 */
void Gyro_CalibBias(void)
{
  float sum    = 0.0f;
  float sum_sq = 0.0f;
  float g;

  for (int i = 0; i < 200; i++) {
    dodo_BMI270_get_data();
    g = GYRO_SIGN * BMI270_gyro_transition(BMI270_gyro_z);  /* 与后面同符号 */
    sum    += g;
    sum_sq += g * g;
    HAL_Delay(2);
  }

  gyro_z_bias = sum / 200.0f;

  /* 方差 = E(x²) - (E(x))²，死区取 (2σ)²，约 95% 置信 */
  {
    float var = (sum_sq / 200.0f) - (gyro_z_bias * gyro_z_bias);
    if (var < 0.0f) var = 0.0f;
    gyro_gate2 = 4.0f * var;
  }

  gyro_t_ms    = HAL_GetTick();
  gyro_last_g  = 0.0f;

  printf("GB,%d,SD,%d\r\n",
         (int)(gyro_z_bias * 10.0f),
         (int)(gyro_gate2 * 100.0f));   /* gate2×100 只用于观察量级 */
}

/* 角度清零：拍 4×90° 视频、或每段重新计时时用 */
void Gyro_ResetAngle(void)
{
  gyro_angle  = 0.0f;
  gyro_t_ms   = HAL_GetTick();
  gyro_last_g = 0.0f;
}

/* 主循环里 gyro_z 更新之后调用，用实际时间差积分 */
void Gyro_Update(void)
{
  uint32_t now = HAL_GetTick();
  float dt = (now - gyro_t_ms) / 1000.0f;
  float g;

  gyro_t_ms = now;

  /* 一阶低通：滤掉电机/齿轮箱振动（实测噪声可达 ±300 °/s） */
  gyro_z_f = 0.8f * gyro_z_f + 0.2f * gyro_z;

  g = gyro_z_f - gyro_z_bias;

  /* 静止死区：小于 2σ 直接当 0，避免静止积分漂移 */
  if (g * g < gyro_gate2) g = 0.0f;

  if (dt > 0.0f && dt < 0.2f) {
    /* 梯形积分：比欧拉法少一阶截断误差 */
    gyro_angle += 0.5f * (g + gyro_last_g) * dt;
  }

  gyro_last_g = g;
}

/* G,<z轴角速度×10>,<偏航角×10> */
void Gyro_Debug_Print(uint16_t period_ms)
{
  static uint32_t t0;
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - t0) < period_ms) return;
  t0 = now;

  printf("G,%d,%d\r\n", (int)(gyro_z * 10.0f), (int)(gyro_angle * 10.0f));
}

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

/* ==================================================================
 * 视频②：原地旋转 4×90°，每段停约 1 s
 *
 * 相比旧版修了三个根因：
 *   1. 断输出后持续采样 —— 旧的 HAL_Delay 期间滑行角度全被漏掉
 *   2. 绝对目标 + 自动学习滑行量 —— 单段误差不会累积
 *   3. 陀螺比例校准 GYRO_SCALE 在这里真正生效
 * ================================================================== */
#define GYRO_SCALE       0.960f   /* 比例校准：陀螺读数偏高 4.2% */
#define SPIN_SEG_DEG      89.0f   /* 每段目标角（物理） */
#define SPIN_SEGS            4
#define SPIN_U             650    /* 对称输出（右轮死区 600 + 余量） */
#define SPIN_LEAD_INIT    14.0f   /* 首段提前量，第 2 段起自动学习 */
#define SPIN_LEAD_MIN      4.0f
#define SPIN_LEAD_MAX     25.0f
#define SPIN_SETTLE_MS   600U     /* 断输出后边采样边滑行的时长 */
#define SPIN_STOP_MS    1000U     /* 每段静止时间（视频要求约 1 s） */
#define SPIN_TMO_MS     4000U     /* 单段兜底超时 */

/* 一次完整的采样：读 → 换算 → 积分 */
static void Spin_Sample(void)
{
  dodo_BMI270_get_data();
  gyro_z = GYRO_SCALE * GYRO_SIGN * BMI270_gyro_transition(BMI270_gyro_z);
  Gyro_Update();
}

void Spin_90x4(void)
{
  int   seg;
  float target  = 0.0f;
  float lead    = SPIN_LEAD_INIT;
  float cut_ang = 0.0f;

  Motor_Set(0, 0);
  Gyro_ResetAngle();
  Spin_Sample();                       /* 建立积分起点 */
  printf("SPIN,START\r\n");

  for (seg = 0; seg < SPIN_SEGS; seg++)
  {
    uint32_t t0 = HAL_GetTick();

    target += SPIN_SEG_DEG;            /* 绝对目标：位置自校正 */

    /* ---- 1) 转到 (目标 − 提前量) ---- */
    while ((target - gyro_angle) >= lead)
    {
      Spin_Sample();
      Motor_Set(SPIN_U, -SPIN_U);
      if ((uint32_t)(HAL_GetTick() - t0) > SPIN_TMO_MS) break;
    }

    cut_ang = gyro_angle;              /* 断电瞬间的角度 */

    /* ---- 2) 断输出，边等边采样，把滑行量积进来 ---- */
    Motor_Set(0, 0);
    {
      uint32_t ts = HAL_GetTick();
      while ((uint32_t)(HAL_GetTick() - ts) < SPIN_SETTLE_MS) 
      {
        Spin_Sample();
      }
    }

    /* ---- 3) 学习本段真实滑行量，给下一段用 ---- */
    if (seg + 1 < SPIN_SEGS)
    {
      float coast = gyro_angle - cut_ang;
      if (coast < SPIN_LEAD_MIN) coast = SPIN_LEAD_MIN;
      if (coast > SPIN_LEAD_MAX) coast = SPIN_LEAD_MAX;
      lead = coast;
    }

    printf("SEG%d,%d,L%d\r\n", seg + 1, (int)(gyro_angle * 10.0f), (int)(lead * 10.0f));

    /* ---- 4) 停够 1 s（这段时间故意不采样，避免把漂移积进去） ---- */
    {
      uint32_t ts = HAL_GetTick();
      while ((uint32_t)(HAL_GetTick() - ts) < SPIN_STOP_MS)
      {
      }
    }
  }

  Motor_Set(0, 0);
  printf("SPIN,DONE,%d\r\n", (int)(gyro_angle * 10.0f));
}

/* ==================================================================
 *        循迹（视频①）—— 方法三：转向环 PD  →  速度环 PI
 * ==================================================================
 *
 *   ┌──────────────┐  err      ┌────────────┐   R      ┌──────────┐
 *   │ 12 路光电管   │ ────────► │ 转向环 PD   │ ───────► │ 左右分配  │
 *   │ Line_Read()  │  (±7°)    │ Line_Turn()│  (mm/s)  │          │
 *   └──────────────┘           └────────────┘          └────┬─────┘
 *                                                           │ vL_t, vR_t
 *   ┌──────────────┐  vL,vR    ┌────────────┐  uL,uR   ┌────▼─────┐
 *   │ 编码器测速    │ ────────► │ 每轮速度 PI │ ───────► │Motor_Set │
 *   │ Speed_Update │  (mm/s)   │ Inc_PID()  │  (PWM)   └──────────┘
 *   └──────────────┘           └────────────┘
 *
 *   方法三的两条核心公式（教程原文）：
 *       L_target = Target_speed - R
 *       R_target = Target_speed + R
 *
 *   不依赖陀螺仪。跑通之后再考虑加 GKD 项（方法二）。
 *   触发：RUN_MODE = 2，按 PB14 开跑，跑够 FOLLOW_LAPS 圈自动停。
 * ================================================================== */

/* ---- 运行模式：0 = 直行 1 m，1 = 原地转 4×90°，2 = 循迹两圈 ---- */
#define RUN_MODE            2

/* ================= 一、赛道路线（决定跑多远） ================= */
#define FOLLOW_LAP_MM       7190     /* 单圈里程 mm：由日志实测 10068-2879=7189 得出，非估值 */
#define FOLLOW_LAPS            2     /* 跑几圈 */
#define FOLLOW_TIMEOUT_MS 120000U     /* 兜底：两圈约 60~70 s 才跑完，120 s 留余量 */

/* ================= 二、速度调度（决定跑多快） =================
 * 偏差越大越慢：给转向留反应时间，防止冲进弯道出不来。 */
#define FOLLOW_V_FAST       300.0f   /* 正中时目标速度 mm/s（直道） */
#define FOLLOW_V_SLOW       180.0f   /* 偏差最大(±7°)时目标速度 mm/s（弯道） */
#define FOLLOW_RAMP_MS        300U   /* 起步升速时间 ms：0 → 目标速度线性上升 */
#define FOLLOW_V_MIN      (-120.0f) /* 允许内轮反转：突破死区地板（内轮正转最低约85mm/s），直角弯靠它 */

/* ================= 三、转向环 PD（方法三本体） =================
 * 输入 err（单位：°，范围 ±7），输出 R（单位：mm/s 的差速量）。
 * R > 0 → L_target 变小、R_target 变大（具体往哪边转由 TURN_SIGN 决定）。 */
#define FOLLOW_KP            150.0f   /* 比例：每偏 1°，给 60 mm/s 差速
                                       * 调大 → 回正快，但会画龙
                                       * 调小 → 迟钝，冲出去才修 */
#define FOLLOW_KD             18.0f   /* 微分：每 10 ms 偏差变化 1°，给 8 mm/s
                                       * 注意：这里是对"每帧"求差，等效微分增益 = 8/0.01 = 800
                                       * 调大 → 抗画龙，但可能发飘 */
#define FOLLOW_KI             0.8f   /* 积分：长期偏在同一边时慢慢补
                                       * 设 0 = 纯 PD（教程原版方法三）
                                       * 留 0.8 是因为本车存在稳态横向偏差 */
#define FOLLOW_I_MAX         60.0f   /* 积分限幅（mm/s）：防止停车/卡住时积分爆掉 */
#define FOLLOW_TURN_MAX     350.0f   /* R 的限幅（mm/s）：急弯转向能力上限
                                       * 调大 → 弯里更猛，但内侧轮可能停转 */
#define FOLLOW_TURN_SIGN    (-1)     /* err>0 时必须右转（由 Align_Hold 能收敛反推）；若车越修越偏改回 +1 */

/* 传感器零点：车正对白线时 err 的读数（单位：°）。
 * 若静止正对线时 err 不是 0，把这个值填进来，否则控制器会一直以为"偏了"。 */
#define FOLLOW_ERR_ZERO       0.0f

/* 恒定偏置：err=0 时额外给的差速（mm/s），用来压掉整车跑偏（打滑/地面/车架）。
 * 0 = 关闭。每次改 ±20 试，按速度等比缩放。 */
#define FOLLOW_TURN_BIAS     0.0f

/* ================= 四、每轮速度 PI（方法三的执行层） =================
 * 输入目标速度 mm/s，输出 PWM。左右各一个实例，互不影响。 */
#define FOLLOW_KP_SPD         1.2f   /* 比例：误差 1 mm/s → 增量 1.2 PWM */
#define FOLLOW_KI_SPD         0.4f  /* 积分：误差 1 mm/s → 每帧再 +0.5 PWM
                                       * 这个项负责吃掉右轮慢 18% 的硬件差异 */
#define FOLLOW_PWM_MAX     3000.0f   /* 输出限幅（PWM）：别把电机顶满 */
#define FOLLOW_PWM_START    900.0f   /* 起步初始 PWM：不用从 0 爬坡 */

/* ================= 五、保护 ================= */
#define FOLLOW_LOST_MS       400U    /* 连续丢线多久就停车（含降速找回的时间） */
#define FOLLOW_DT            0.01f   /* 控制周期 10 ms，用于积分 */

/* ==================================================================
 *   六、环岛（视频① 每圈必须进岛、绕一圈、出岛）
 *
 *   ┌──────────┐  连续 60 ms 像环岛   ┌──────────┐
 *   │ 正常 PD  │ ───────────────────► │  岛内    │
 *   └──────────┘                      └────┬─────┘
 *        ▲      冷却 1.5 s                 │ 累计偏航 ≥ 330°
 *        └─────────────────────────────────┘
 * ================================================================== */
#define ISL_MODE            2        /* 0=关  1=只识别不动作(干跑)  2=启用 */

#define ISL_TURN_DIR    (+1.0f)      /* -1=顺时针（turn<0 右转）；逆时针改 +1 */

/* --- 岛内转向 --- */
#define ISL_BIAS          120.0f     /* 偏置基准 mm/s（角速度环的初值） */
#define ISL_W_DPS          85.0f     /* 目标角速度 °/s：60 → 绕一圈约 6 s */
#define ISL_KW              1.5f     /* 角速度误差 → 附加偏置；0 = 关闭闭环 */
#define ISL_BIAS_MAX      300.0f     /* 偏置上限，防止失控 */
#define ISL_PD_SCALE        0.60f    /* 岛内 PD 权重；0 = 纯开环画圈 */
#define ISL_V             150.0f     /* 岛内目标速度 mm/s */

/* --- 出岛 --- */
#define ISL_EXIT_DEG      340.0f     /* 累计偏航多少度算绕完 */
#define ISL_MAX_MS      12000U       /* 岛内超时兜底 */

/* --- 进岛 --- */
#define ISL_N_WIDE           8U      /* ① 白路数 ≥ 此值 = 宽区 */
#define ISL_FORK             1       /* ② 启用"两端白+中间蓝"分叉判别 */
#define ISL_CONFIRM_MS      20U      /* 连续成立多久才认账 */
#define ISL_COOL_MS       1500U      /* 出岛冷却，防出口重复触发 */
#define ISL_WIN_LO_MM     2500       /* 距离窗下限 mm（单圈内） */
#define ISL_WIN_HI_MM     3300       /* 上限；0 = 关闭距离窗 */

/* --- 岛内丢线 --- */
#define ISL_LOST_MS       3000U      /* 直道 400 ms 在环道会误杀 */

/* --- 出岛过渡 --- */
#define ISL_OUT_MS         1000U     /* 出岛后这段时间内丢线窗口沿用 ISL_LOST_MS，
                                      * 给车时间从岔口重新抓到白线 */

#if (RUN_MODE == 2)

/* 非线性权重：边缘重、中心轻，直道更稳、弯道更狠 */
static const float LINE_W[12] = {
  -7.0f, -5.0f, -3.5f, -2.0f, -1.0f, -0.2f,
   0.2f,  1.0f,  2.0f,  3.5f,  5.0f,  7.0f
};

/* 增量式速度 PI */
typedef struct {
  float kp, ki, max;
  float out;    /* 累加的 PWM */
  float e1;     /* 上一次误差 */
} IncPID_t;

static IncPID_t  pidL, pidR;
static int32_t   spd_lastL, spd_lastR;
static uint32_t  spd_t_ms;

/* 12 路原始位图（ch0 = bit11 … ch11 = bit0）。
 * 环岛判别要看"具体哪几路是白的"，不能只看个数，所以把它存下来。 */
static uint16_t line_raw = 0x0000U;

/* 读 12 路，err 输出加权平均（±7），返回有效路数 */
uint8_t Line_Read(float *err)
{
  uint16_t v;
  float sum = 0.0f;
  uint8_t n = 0;

  MUX_get_value(&v);
	line_raw = v; 

  for (int i = 0; i < 12; i++) {
    if (MUX_GET_CHANNEL(v, i)) { sum += LINE_W[i]; n++; }
  }

  if (n) *err = sum / (float)n;
  return n;
}

static void Speed_PID_Init(void)
{
  pidL.kp = pidR.kp = FOLLOW_KP_SPD;
  pidL.ki = pidR.ki = FOLLOW_KI_SPD;
  pidL.max = pidR.max = FOLLOW_PWM_MAX;
  pidL.out = pidR.out = FOLLOW_PWM_START;
  pidL.e1  = pidR.e1  = 0.0f;
  spd_lastL = ENC_CountL();
  spd_lastR = ENC_CountR();
  spd_t_ms  = HAL_GetTick();
}

/* 用计数差算两轮实际速度（mm/s），顺便推进时间基准 */
static void Speed_Update(float *vL, float *vR)
{
  uint32_t now = HAL_GetTick();
  float dt = (now - spd_t_ms) / 1000.0f;
  int32_t cL, cR;

  if (dt < 0.005f) dt = 0.005f;
  spd_t_ms = now;

  cL = ENC_CountL();
  cR = ENC_CountR();

  *vL = (float)(cL - spd_lastL) * ENC_FACTOR_L * 10.0f / dt;
  *vR = (float)(cR - spd_lastR) * ENC_FACTOR_R * 10.0f / dt;

  spd_lastL = cL;
  spd_lastR = cR;
}

/* 增量式 PI：out += Kp*(e-e1) + Ki*e，输出累加并限幅 */
static float Inc_PID(IncPID_t *p, float target, float actual)
{
  float e = target - actual;

  p->out += p->kp * (e - p->e1) + p->ki * e;

  if (p->out >  p->max) p->out =  p->max;
  if (p->out < -p->max) p->out = -p->max;

  p->e1 = e;
  return p->out;
}

/* ============ 转向环（方法三的本体：PD + 可选 I） ============
 * 输入：e  = 当前偏差（°），de = 本帧与上帧的差（°）
 * 输出：R  = 差速量（mm/s），已限幅 ±FOLLOW_TURN_MAX
 *
 *   R = KP·e  +  KI·∫e dt  +  KD·de
 *       ↑比例     ↑积分        ↑微分
 */
static float line_i = 0.0f;          /* 积分累加器（°·s） */

static void Line_Reset(void)
{
  line_i = 0.0f;
}

static float Line_Turn(float e, float de)
{
  float r;

  /* 积分项：KI = 0 时整段不起作用（纯 PD，即教程原版方法三） */
  line_i += e * FOLLOW_DT;
  if (line_i >  FOLLOW_I_MAX) line_i =  FOLLOW_I_MAX;
  if (line_i < -FOLLOW_I_MAX) line_i = -FOLLOW_I_MAX;

  r = FOLLOW_KP * e + FOLLOW_KI * line_i + FOLLOW_KD * de;

  if (r >  FOLLOW_TURN_MAX) r =  FOLLOW_TURN_MAX;
  if (r < -FOLLOW_TURN_MAX) r = -FOLLOW_TURN_MAX;

  return r;
}

/* ================== 环岛：状态 ================== */
#define ISL_IDLE    0
#define ISL_IN      1

static uint8_t  isl_state = ISL_IDLE;
static uint8_t  isl_cfm   = 0U;      /* 连续识别标志 */
static uint32_t isl_t_cfm = 0U;      /* 连续识别起点 */
static uint32_t isl_t_in  = 0U;      /* 进岛时刻 */
static uint32_t isl_t_out = 0U;      /* 出岛时刻（冷却起点） */
static float    isl_yaw0  = 0.0f;    /* 进岛瞬间偏航角 */
static float    isl_yaw   = 0.0f;    /* 岛内已转角度（绝对值） */
static uint8_t  isl_cnt   = 0U;      /* 绕岛次数，看日志用 */

static void Island_Reset(void)
{
  isl_state = ISL_IDLE;
  isl_cfm   = 0U;
  isl_cnt   = 0U;
  isl_yaw   = 0.0f;
  isl_yaw0  = 0.0f;
  isl_t_in  = 0U;
  isl_t_out = 0U;
  isl_t_cfm = 0U;
}

/* ============ 判据：这一帧像不像环岛入口 ============ */
static uint8_t Island_Pattern(uint8_t n)
{
  uint8_t ed_l, ed_r, mid_blue, i;

  if (n >= ISL_N_WIDE) return 1U;            /* ① 宽区 */

#if (ISL_FORK)
  /* ② 分叉：最左和最右都有白，中间几路全蓝 */
  ed_l = (MUX_GET_CHANNEL(line_raw, 0) || MUX_GET_CHANNEL(line_raw, 1)
       || MUX_GET_CHANNEL(line_raw, 2))  ? 1U : 0U;
  ed_r = (MUX_GET_CHANNEL(line_raw, 9) || MUX_GET_CHANNEL(line_raw, 10)
       || MUX_GET_CHANNEL(line_raw, 11)) ? 1U : 0U;

  mid_blue = 1U;
  for (i = 4U; i <= 7U; i++)
    if (MUX_GET_CHANNEL(line_raw, i)) { mid_blue = 0U; break; }

  if (ed_l && ed_r && mid_blue) return 1U;
#else
  (void)ed_l; (void)ed_r; (void)mid_blue; (void)i;
#endif
  return 0U;
}

/* ==== 跑车期间自己采陀螺仪 ====
 * Follow_Line_Run 阻塞主循环，主循环里的 Gyro_Update() 跑车期间不执行，
 * gyro_angle 会一直停在 0 —— 出岛判据永远等不到 330°。必须在这里补采。 */
static void Gyro_Poll_1ms(uint32_t now)
{
  static uint32_t t_gy = 0U;

  if ((uint32_t)(now - t_gy) < 1U) return;
  t_gy = now;

  dodo_BMI270_get_data();
  gyro_z = GYRO_SIGN * BMI270_gyro_transition(BMI270_gyro_z);
  Gyro_Update();
}

/* ============ 距离窗：这一圈内是否允许进岛 ============
 * ISL_WIN_HI_MM = 0 表示关闭（任何位置都可能进岛）。
 * 打开后，只有里程落在 [LO, HI] 内才武装，普通直角弯就不会误触发。 */
static uint8_t Island_Armed(int32_t dist)
{
#if (ISL_WIN_HI_MM > 0)
  int32_t lap = (FOLLOW_LAP_MM > 0) ? (dist % FOLLOW_LAP_MM) : dist;
  return (uint8_t)(lap >= (int32_t)ISL_WIN_LO_MM
                && lap <= (int32_t)ISL_WIN_HI_MM);
#else
  (void)dist;
  return 1U;
#endif
}

/* ============ 状态机：每 10 ms 一次 ============
 * 返回 1 = 本帧按"岛内"处理，0 = 正常循迹 */
static uint8_t Island_Step(uint8_t n, uint32_t now, int32_t dist)
{
  uint8_t armed = Island_Armed(dist);

  /* ---------- 已在岛内：只判出岛 ---------- */
  if (isl_state == ISL_IN) {
    isl_yaw = fabsf(gyro_angle - isl_yaw0);

    if (isl_yaw >= ISL_EXIT_DEG) {                      /* 绕完了 */
      isl_state = ISL_IDLE;
      isl_t_out = now;
      isl_cnt++;
      printf("ISL,OUT,%ld,%d,%ld\r\n", (long)dist,
             (int)(isl_yaw * 10.0f), (long)(uint32_t)(now - isl_t_in));
      Line_Reset();                                     /* 清岛内涨起来的积分 */
      return 0U;
    }

    if ((uint32_t)(now - isl_t_in) > ISL_MAX_MS) {      /* 兜底：没转够也放出来 */
      isl_state = ISL_IDLE;
      isl_t_out = now;
      printf("ISL,TMO,%ld,%d\r\n", (long)dist, (int)(isl_yaw * 10.0f));
      Line_Reset();
      return 0U;
    }
    return 1U;
  }

  /* ---------- 出岛冷却 ---------- */
  if (isl_t_out != 0U && (uint32_t)(now - isl_t_out) < ISL_COOL_MS) {
    isl_cfm = 0U;
    return 0U;
  }

  /* ---------- 判进岛 ---------- */
  if (armed && Island_Pattern(n)) {
    if (!isl_cfm) { isl_cfm = 1U; isl_t_cfm = now; }
    else if ((uint32_t)(now - isl_t_cfm) >= ISL_CONFIRM_MS) {
      isl_cfm   = 0U;
      isl_t_out = now;          /* 干跑模式靠它去重 */
      isl_yaw0  = gyro_angle;
      isl_yaw   = 0.0f;

#if (ISL_MODE == 2)
      printf("ISL,IN,%ld,%d,%04X\r\n", (long)dist, (int)n, (unsigned)line_raw);
      isl_state = ISL_IN;
      isl_t_in  = now;
      Line_Reset();
      return 1U;
#else
      printf("ISL,DRY,%ld,%d,%04X\r\n", (long)dist, (int)n, (unsigned)line_raw);
      return 0U;                /* 干跑：只打印，不改变控制 */
#endif
    }
  } else {
    isl_cfm = 0U;
  }
  return 0U;
}

/* ============ 岛内偏置（幅值，方向由 ISL_TURN_DIR 定） ============
 * 偏置 = ISL_BIAS + ISL_KW × (目标角速度 − 陀螺实测角速度)
 * 实测转太快自动收、太慢自动加 → 调的是"绕一圈几秒"，不是差速数。 */
static float Island_Bias(void)
{
  float b = ISL_BIAS + ISL_KW * (ISL_W_DPS - fabsf(gyro_z_f));

  if (b < 0.0f)         b = 0.0f;
  if (b > ISL_BIAS_MAX) b = ISL_BIAS_MAX;
  return b;
}

void Follow_Line_Run(void)
{
  const int32_t goal_mm = (int32_t)FOLLOW_LAP_MM * FOLLOW_LAPS;
  int32_t  s0 = ENC_Dist_mm();
  uint32_t t0 = HAL_GetTick();
  uint32_t t_ctl = t0, t_prt = t0, t_lost = t0;
  float    err = 0.0f, last_err = 0.0f;
  float    e = 0.0f, last_e = 0.0f;
  float    vL = 0.0f, vR = 0.0f, v_tgt, turn,turn_pd, ae;
  int16_t  uL, uR;
  uint8_t  n, isl_now = 0U;
  uint8_t  isl_was = 0U;              /* 上一帧是否在岛内（抓出岛那一刻） */
  uint32_t t_grace = 0U;              /* 出岛时刻，出岛过渡窗口起点 */

  Speed_PID_Init();
  Line_Reset();                       /* 清转向环积分，避免上次残留 */
  Island_Reset();
	printf("FOLLOW,START\r\n");

  for (;;) {
    uint32_t now = HAL_GetTick();

		Gyro_Poll_1ms(now);
		
    if ((uint32_t)(now - t0) > FOLLOW_TIMEOUT_MS) { printf("TIMEOUT\r\n"); break; }

    if ((uint32_t)(now - t_ctl) >= 10U) {
      t_ctl = now;

      /* ---- 1) 读线 ---- */
      n = Line_Read(&err);

			isl_now = Island_Step(n, now, ENC_Dist_mm() - s0);
			
      /* ---- 出岛过渡 ----
       * 出岛那一帧必须重置丢线计时：岛内 t_lost 可能已经好几秒没更新
       * （环道里大段 n=0），直接沿用会拿旧计时比对，出岛第一帧就 LOST
       * 停车 —— 实测就是这样，330° 正常出岛后立刻停机，整轮白跑。
       * 另外出岛后 ISL_OUT_MS 内把丢线窗口放宽，给车时间重新抓线。 */
      if (isl_was && !isl_now) { t_grace = now; t_lost = now; }
      isl_was = isl_now;

      if (n == 0) {
        uint8_t wide = (isl_now || (t_grace != 0U
                       && (uint32_t)(now - t_grace) < ISL_OUT_MS)) ? 1U : 0U;

        err = last_err;
        if ((uint32_t)(now - t_lost) > (wide ? ISL_LOST_MS : FOLLOW_LOST_MS)) {
          printf("LOST\r\n"); break;
        }
      } else {
        t_lost   = now;
        last_err = err;                   /* 供丢线时保持 */
      }

      /* ---- 2) 误差零点补偿 ---- */
      e  = err - FOLLOW_ERR_ZERO;
      ae = (e > 0.0f) ? e : -e;
      if (ae > 7.0f) ae = 7.0f;

      /* ---- 3) 速度调度：偏得越多越慢 ---- */
      if (isl_now) {
        v_tgt = ISL_V;                       /* 岛内定速 */
      } else {
        v_tgt = FOLLOW_V_FAST - (FOLLOW_V_FAST - FOLLOW_V_SLOW) * (ae / 7.0f);

        /* 进岛预备减速：距离窗内先把速度压到岛内速度。
         * 否则 300 mm/s 冲进 T 口，等速度环降到 ISL_V 时已经冲过岔口了。 */
        if (Island_Armed(ENC_Dist_mm() - s0) && v_tgt > ISL_V)
          v_tgt = ISL_V;
      }

      /* 起步线性升速：给两轮速度环时间把左右差异拉平 */
      {
        uint32_t t_go = (uint32_t)(now - t0);
        if (t_go < FOLLOW_RAMP_MS)
          v_tgt *= (float)t_go / (float)FOLLOW_RAMP_MS;
      }

      /* ---- 4) 转向环 ---- */
      turn_pd = FOLLOW_TURN_SIGN * Line_Turn(e, e - last_e);
      if (isl_now) {
        turn = ISL_TURN_DIR * Island_Bias() + ISL_PD_SCALE * turn_pd;
      } else {
        turn = turn_pd;
      }
      turn += FOLLOW_TURN_BIAS * (v_tgt / FOLLOW_V_FAST);
      last_e = e;

      /* ---- 5) 方法三：分配到左右轮目标速度 ----
       *   L_target = V - R
       *   R_target = V + R
       * 下限钳到 FOLLOW_V_MIN，避免急弯时内侧轮目标为负（反转） */
      {
        float tL = v_tgt - turn;
        float tR = v_tgt + turn;

        if (tL < FOLLOW_V_MIN) tL = FOLLOW_V_MIN;
        if (tR < FOLLOW_V_MIN) tR = FOLLOW_V_MIN;

        /* 测速始终更新：保持 spd_t_ms/计数连续，堵转保护也要用 vL/vR */
        Speed_Update(&vL, &vR);

        uL = (int16_t)Inc_PID(&pidL, tL, vL);
        uR = (int16_t)Inc_PID(&pidR, tR, vR);
      }

      Motor_Set(uL, uR);

      /* ---- 堵转保护：两个轮子都不转但还在要求速度 = 顶住东西了 ----
       * 岛内低目标速度下速度环容易顶死（PI 一个拧到 +3000、一个拧到 −3000），
       * 没有这段会一直对顶，伤驱动板。 */
      {
        static uint16_t stall_ms = 0U;
        if (vL < 20.0f && vR < 20.0f && v_tgt > 100.0f) stall_ms += 10U;
        else                                            stall_ms  = 0U;
        if (stall_ms > 300U) { printf("STALL\r\n"); break; }
      }

      /* ---- 5) 圈数判定 ---- */
      if ((ENC_Dist_mm() - s0) >= goal_mm) { printf("LAPS,DONE\r\n"); break; }
    }

    if ((uint32_t)(now - t_prt) >= 100U) {
      t_prt = now;
      if (isl_now) {
        /* 岛内专用行：I,里程mm, 累计偏航×10, 角速度×10, uL, uR, 白路数
         * 累计偏航应单调增大，接近 ISL_EXIT_DEG 时出岛；
         * 角速度应稳定在 ±ISL_W_DPS 附近，若长期接近 0 说明车没真正转起来
         * （常见原因：tL/tR 落进电机死区，被 DEAD_L/DEAD_R 地板带偏）。 */
        printf("I,%ld,%d,%d,%d,%d,%d\r\n",
               (long)(ENC_Dist_mm() - s0),
               (int)(isl_yaw * 10.0f),
               (int)(gyro_z_f * 10.0f),
               (int)uL,
               (int)uR,
               (int)n);
      } else {
        /* F,里程mm, err×10, 目标速度×10, vL×10, vR×10, uL, uR
         * 重点看 vL 与 vR：两者差 >30 说明速度环没平衡，车必然画弧 */
        printf("F,%ld,%d,%d,%d,%d,%d,%d\r\n",
               (long)(ENC_Dist_mm() - s0),
               (int)(err * 10.0f),
               (int)(v_tgt * 10.0f),
               (int)(vL * 10.0f),
               (int)(vR * 10.0f),
               (int)uL,
               (int)uR);
      }
    }
  }

  Motor_Set(0, 0);
  printf("FOLLOW,STOP,%ld\r\n", (long)(ENC_Dist_mm() - s0));
}

/* ==================================================================
 *                 待发姿态回正（上电即生效）
 * ==================================================================
 * 上电后电机使能，但不前进：只做"原地小角度对正"。
 * 按发车键后进入循迹；循迹结束（停车）会自动回到这个状态。
 *
 * ALIGN_MODE = 0 → 关闭（上电后电机保持 0）
 *              1 → 按白线对正（光电管）：车头对到线中心
 *              2 → 按陀螺仪保持航向：车头对到上电时的方向（0°）
 *
 * 实现是"两点式"而不是 PID：偏差超过死区就用固定 PWM 反向转一下，
 * 进了死区就停。简单、可预测，不需要调参。
 */
#define ALIGN_MODE          1       /* 0=关 1=按线 2=按陀螺仪 */
#define ALIGN_DEAD_DEG      1.0f    /* 死区：|偏差| 小于这个值就不动（°） */
#define ALIGN_PWM         500.0f    /* 回正时的 PWM 幅值（要明显高于死区 600） */
#define ALIGN_PIVOT_HOLD   60U      /* 每次转多久 ms，然后停 40 ms 再看 */
#define ALIGN_SIGN         (+1)     /* 回正方向反了就改 -1 */

#if (ALIGN_MODE != 0)
void Align_Hold(void)
{
  static uint32_t t_state = 0;
  static uint8_t  moving  = 0;
  uint32_t now = HAL_GetTick();
  float    err = 0.0f, cmd;
  uint8_t  n;

  /* 状态机：转 60 ms → 停 40 ms → 重新测量，避免连续修正过冲 */
  if (moving) {
    if ((uint32_t)(now - t_state) < ALIGN_PIVOT_HOLD) return;
    Motor_Set(0, 0);
    moving  = 0;
    t_state = now;
    return;
  }
  if ((uint32_t)(now - t_state) < 40U) return;
  t_state = now;

  /* 取当前偏差，统一成"正 = 需要往右转" */
#if (ALIGN_MODE == 1)
  n = Line_Read(&err);
  if (n == 0) { Motor_Set(0, 0); return; }         /* 没看到线就不乱动 */
  cmd = ALIGN_SIGN * (err - FOLLOW_ERR_ZERO);
#else
  cmd = ALIGN_SIGN * (-gyro_angle);                /* 目标是 0° */
#endif

  if (cmd > ALIGN_DEAD_DEG) {
    Motor_Set((int16_t)ALIGN_PWM, (int16_t)(-ALIGN_PWM));   /* 原地右转 */
    moving  = 1;
    t_state = now;
  } else if (cmd < -ALIGN_DEAD_DEG) {
    Motor_Set((int16_t)(-ALIGN_PWM), (int16_t)ALIGN_PWM);   /* 原地左转 */
    moving  = 1;
    t_state = now;
  } else {
    Motor_Set(0, 0);                                        /* 已经正了 */
  }
}
#endif

#endif /* RUN_MODE == 2 */

/* USER CODE END 0 */
void Init_brushless_motor(void)
{
  /* PA11 idles low: the first high level then also lasts exactly 0.9 ms,
     because the DMA writes the first slot 0.1 ms after TIM1 is started. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  ESC_BuildDmaTable(ESC_START_PULSE_SLOTS);
  esc_period_count = 0U;
  hdma_tim1_up.XferCpltCallback = ESC_DmaTransferComplete;
  if (HAL_DMA_Start_IT(&hdma_tim1_up,(uint32_t)esc_dma_table,(uint32_t)&GPIOA->BSRR,ESC_PULSE_SLOTS) != HAL_OK)
    Error_Handler();
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
 if (IF_start_Brushless_motor)
   while (esc_period_count < ESC_START_PERIODS)
      HAL_Delay(200);//等待无刷电桥负压启动完成
  /* Disabled fan: return immediately; DMA completes the existing
     0.9 ms -> 1.0 ms idle-signal sequence in the background. */
}
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  Init_brushless_motor(); //等待无刷电调负压启动完成

  /* USER CODE BEGIN 2 */
  int8_t imu = dodo_BMI270_init();       /* 初始化陀螺仪，0 = 成功 */
  printf("IMU,%d\r\n", (int)imu);
  Motor_Set(0, 0);                       /* 标定期间轮子不转 */
  Gyro_CalibBias();                      /* 采零偏：调用时车必须静止 */
  ENC_Init();                            /* 启动编码器并清零 */
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim2);    // 1 ms 时基，ENC_ISR_1ms() 在里面跑
  printf("ENC,READY\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		 /* ---- 按 PB14 发车：走 1 m ---- */
   static uint8_t key_prev2 = 1;
    uint8_t key_now = (uint8_t)HAL_GPIO_ReadPin(GPIO_RUN_GPIO_Port, GPIO_RUN_Pin);

    if (key_prev2 && !key_now) {
      HAL_Delay(300);                 /* 等手离开 */
      ENC_Reset();                    /* 里程清零 */
			Gyro_ResetAngle();  
      printf("GO\r\n");
#if (RUN_MODE == 0)
      Drive_Straight_PID(742);        /* 直行 1 m（视频③） */
#elif (RUN_MODE == 1)
      Spin_90x4();                    /* 原地转 4×90°（视频②） */
#else
      Follow_Line_Run();              /* 循迹跑两圈（视频①） */
#endif
      printf("STOP\r\n");
    }
    key_prev2 = key_now;

    dodo_BMI270_get_data();                                       /* 读传感器 */
    gyro_x = BMI270_gyro_transition(BMI270_gyro_x);
    gyro_y = BMI270_gyro_transition(BMI270_gyro_y);
    gyro_z = GYRO_SIGN * BMI270_gyro_transition(BMI270_gyro_z);   /* 统一为「右转为正」 */
    Gyro_Update();                                                /* 积分角度 */

#if (ALIGN_MODE != 0)
    Align_Hold();      /* 待发姿态回正：按键之前一直生效，按键后进入循迹 */
#endif
    accel_x = BMI270_acc_transition(BMI270_accel_x); // 将原始加速度计数据转换为物理值，单位为g，一般不需要使用此数据
		

		accel_y = BMI270_acc_transition(BMI270_accel_y);
    accel_z = BMI270_acc_transition(BMI270_accel_z);
    if (GYRO_DEBUG)
      printf("%f,%f,%f,%f,%f,%f\r\n",gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z);

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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | R_DIR_Pin | GPIO_PIN_11 | MUX_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L_DIR_Pin | MUX_1_Pin | MUX_2_Pin | MUX_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_RUN_Pin */
  GPIO_InitStruct.Pin = GPIO_RUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_RUN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L_DIR_Pin */
  GPIO_InitStruct.Pin = L_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R_DIR_Pin PA11 */
  GPIO_InitStruct.Pin = R_DIR_Pin | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_READ_Pin */
  GPIO_InitStruct.Pin = MUX_READ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MUX_READ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_0_Pin */
  GPIO_InitStruct.Pin = MUX_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(MUX_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_1_Pin MUX_2_Pin MUX_3_Pin */
  GPIO_InitStruct.Pin = MUX_1_Pin | MUX_2_Pin | MUX_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    ENC_ISR_1ms();
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
