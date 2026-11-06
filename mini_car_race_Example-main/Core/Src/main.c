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
 *        循迹（视频①）—— 重构版
 * ==================================================================
 *
 *  ① 传感器   Line_Read()      12 路 → err(±7)、路数 n、位图 raw
 *  ② 误差整形 Line_Shape()     宽区/丢线时"保持上一次有效偏差"，避免假居中
 *  ③ 转向     Steer_PD()       P + I + D → R(mm/s)，限幅
 *  ④ 分配     L = V - R，R = V + R
 *  ⑤ 执行     Inc_PID()        每轮速度 PI → PWM
 *  ⑥ 状态机   Follow_Line_Run() FOLLOW / ISLAND / RECOVER
 *
 *  触发：RUN_MODE = 2，按 PB14 开跑；**跑动中再按一次 PB14 立即停车**。
 *
 *  设计要点：
 *   - 进岛只看图案（最左、最右两路是否同时为白），不用编码器里程
 *   - 宽区（T 口）时 12 路加权平均会趋近 0，此时不用当前 err，沿用上次有效值
 *   - 岛内用"固定偏置 + 角速度环"，按累计偏航出岛
 *   - 丢线不立刻停：低速朝最后见线方向找，找不回才停
 * ================================================================== */

/* ---- 运行模式：0=直行 1m，1=原地转 4×90°，2=循迹两圈 ---- */
#define RUN_MODE            2

/* ==================== 一、赛道与总时限 ==================== */
#define FOLLOW_LAP_MM       7190     /* 单圈里程 mm（日志实测推算） */
#define FOLLOW_LAPS            2
#define FOLLOW_TIMEOUT_MS 120000U    /* 总超时 120 s */

/* ==================== 二、速度 ==================== */
#define FOLLOW_V_FAST       300.0f   /* 正中时 mm/s */
#define FOLLOW_V_SLOW       180.0f   /* 偏差最大(±7°)时 mm/s */
#define FOLLOW_RAMP_MS        300U   /* 起步升速时间 ms */
#define FOLLOW_V_MIN      (-120.0f)  /* 单轮目标下限（负=允许反转） */

/* ==================== 三、转向环 PD(+I) ==================== */
#define FOLLOW_KP           150.0f   /* mm/s per ° */
#define FOLLOW_KI             0.8f   /* mm/s per °·s */
#define FOLLOW_KD            18.0f   /* mm/s per °（每 10 ms 差分，等效 ×100） */
#define FOLLOW_I_MAX         60.0f   /* 积分限幅 mm/s */
#define FOLLOW_TURN_MAX     350.0f   /* 差速限幅 mm/s */
#define FOLLOW_TURN_SIGN    (-1)     /* 方向开关，已实测 */
#define FOLLOW_ERR_ZERO       0.0f   /* 传感器零点 ° */
#define FOLLOW_TURN_BIAS      0.0f   /* 整车跑偏补偿 mm/s */

/* ==================== 四、每轮速度 PI ==================== */
#define FOLLOW_KP_SPD         1.2f   /* PWM per mm/s */
#define FOLLOW_KI_SPD         0.4f   /* PWM per mm/s 每帧 */
#define FOLLOW_PWM_MAX     3000.0f
#define FOLLOW_PWM_START    900.0f
#define FOLLOW_DT            0.01f   /* 控制周期 s */

/* ==================== 五、宽区（T 口）处理 ==================== */
#define WIDE_NEAR_N           5U     /* 路数 ≥ 此值：略减速（预备） */
#define ISL_NEAR_V        200.0f   /* 预备减速的目标值：只压到 200，不压到岛内 150 */
#define WIDE_HOLD_N           6U     /* 两端都白且路数 ≥ 此值 → 判为 T 口，冻结偏差 */

/* ==================== 六、环岛（T 口直行锁定） ====================
 * 路线（已确认）：T 口不停、不转，直着穿过去；到方框拐角才由 PD 左转。
 *
 * 为什么不能只看"两端都白"：200 mm 的方框里传感器排约 150 mm，
 * 拐角处会同时看到两条边，同样"两端都白"，和真 T 口一模一样。
 * 误判后车被锁定直行 -> 不转弯 -> 冲进方框内部（走错方向/绕圈出不来）。
 *
 * 实测位图（同一辆车）：
 *   真 T 口    0EFF(11 路) / 0FFF(12 路)
 *   方框拐角   0FF8(9 路) / 0FFC(10 路) / 03FF(10 路)
 * 所以用「白路数 >= 11」判 T 口，拐角最多 10 路，分得开。 */
#define ISL_ENABLE            1      /* 0=关闭 1=启用 */
#define ISL_T_N              11U     /* T 口：白路数下限 */
#define ISL_N_WIDE            8U     /* 保留：宽区下限 */
#define ISL_CONFIRM_MS       20U     /* 图案连续成立多久才认账 */
#define ISL_CROSS_MM         60      /* 锁定直行距离 mm（横线 12 mm + 余量） */
#define ISL_CROSS_MIN_MS     80U     /* 锁定最短时间 */
#define ISL_CROSS_V        200.0f    /* 过 T 口速度 mm/s */
#define ISL_COOL_MS         800U     /* 穿过后冷却，防重复触发 */
#define ISL_OUT_MS         1000U     /* 穿过后丢线宽限 */
#define ISL_LOST_MS        1500U     /* T 口内/刚穿过的丢线窗口（比 400 ms 宽） */

/* ==================== 七、丢线找线 ==================== */
#define LOST_MS           400U       /* 丢线超过此值 → 找线 */
#define RECOVER_MS       1500U       /* 找线总时长，超过才停 */
#define RECOVER_V         120.0f     /* 找线速度 mm/s */
#define RECOVER_TURN      260.0f     /* 找线时的转向量 mm/s */

/* ==================== 八、保护 ==================== */
#define STALL_MS          300U       /* 两轮都不转多久算堵转 */

#if (RUN_MODE == 2)

/* ==================== 传感器层 ==================== */

/* 非线性权重：边缘重、中心轻 */
static const float LINE_W[12] = {
  -7.0f, -5.0f, -3.5f, -2.0f, -1.0f, -0.2f,
   0.2f,  1.0f,  2.0f,  3.5f,  5.0f,  7.0f
};

static uint16_t line_raw = 0x0000U;   /* 12 路位图：ch0=bit11 … ch11=bit0 */

/* 读 12 路：err 输出加权平均（±7），返回有效路数 */
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

/* ==================== 误差整形 ====================
 * 返回本帧用于控制的偏差。
 *
 * 只在"假居中"时才冻结，不能一刀切按路数：
 *   普通 90° 弯 0FF8 / 03FF 是"半边白"，加权平均仍有明确方向（±5~6），
 *   照常使用；否则会把正常转弯也冻住，车冲出去。
 *   T 口是"两端都白"，左右对称，平均趋近 0 —— 这时才沿用上次有效值。
 *   丢线（n == 0）同样沿用，让车保持原来的转向趋势。
 */
static float Line_Shape(uint8_t n, float err, float *keep)
{
  uint8_t end_l = MUX_GET_CHANNEL(line_raw, 0)  ? 1U : 0U;   /* 最左 */
  uint8_t end_r = MUX_GET_CHANNEL(line_raw, 11) ? 1U : 0U;   /* 最右 */

  if (n == 0U) return *keep;                          /* 丢线 */
  if (end_l && end_r && n >= WIDE_HOLD_N) return *keep;  /* T 口：平均不可信 */

  *keep = err;
  return err;
}

/* ==================== 转向环 ==================== */
static float line_i = 0.0f;

static void Steer_Reset(void)
{
  line_i = 0.0f;
}

static float Steer_PD(float e, float de)
{
  float r;

  line_i += e * FOLLOW_DT;
  if (line_i >  FOLLOW_I_MAX) line_i =  FOLLOW_I_MAX;
  if (line_i < -FOLLOW_I_MAX) line_i = -FOLLOW_I_MAX;

  r = FOLLOW_KP * e + FOLLOW_KI * line_i + FOLLOW_KD * de;

  if (r >  FOLLOW_TURN_MAX) r =  FOLLOW_TURN_MAX;
  if (r < -FOLLOW_TURN_MAX) r = -FOLLOW_TURN_MAX;

  return r;
}

/* ==================== 每轮速度 PI ==================== */
typedef struct {
  float kp, ki, max;
  float out;
  float e1;
} IncPID_t;

static IncPID_t pidL, pidR;
static int32_t  spd_lastL, spd_lastR;
static uint32_t spd_t_ms;

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

static float Inc_PID(IncPID_t *p, float target, float actual)
{
  float e = target - actual;

  p->out += p->kp * (e - p->e1) + p->ki * e;

  if (p->out >  p->max) p->out =  p->max;
  if (p->out < -p->max) p->out = -p->max;

  p->e1 = e;
  return p->out;
}

/* ==================== 陀螺仪补采 ==================== */
static void Gyro_Poll_1ms(uint32_t now)
{
  static uint32_t t_gy = 0U;

  if ((uint32_t)(now - t_gy) < 1U) return;
  t_gy = now;

  dodo_BMI270_get_data();
  gyro_z = GYRO_SIGN * BMI270_gyro_transition(BMI270_gyro_z);
  Gyro_Update();
}

/* ==================== 环岛 ==================== */
/* ==================== 环岛（T 口直行锁定） ====================
 * 三种状态：
 *   ISL_NONE  正常循迹
 *   ISL_CROSS 正穿过 T 口：锁定转向（保持进 T 前的转向量），低速直行
 *   ISL_RET   穿过后冷却，期间不识别，防重复触发
 */
#define ISL_NONE     0U
#define ISL_CROSS    1U
#define ISL_RET      2U

static uint8_t  isl_state = ISL_NONE;
static uint8_t  isl_cfm   = 0U;
static uint32_t isl_t_cfm = 0U;
static uint32_t isl_t_state = 0U;
static int32_t  isl_d0    = 0;      /* 进入 T 口时的里程 */
static float    isl_turn_hold = 0.0f; /* 锁定期间使用的转向量 */

static void Island_Reset(void)
{
  isl_state    = ISL_NONE;
  isl_cfm      = 0U;
  isl_t_cfm    = 0U;
  isl_t_state  = 0U;
  isl_d0       = 0;
  isl_turn_hold = 0.0f;
}

/* 判据：这一帧像不像 T 口
 *   方框拐角   0FF8(9) / 0FFC(10) / 03FF(10) —— 最多 10 路
 *   真 T 口    0EFF(11) / 0FFF(12)          —— 11 路以上
 * 只用路数，不用"两端都有白"（拐角也会两端白）。 */
static uint8_t Island_Pattern(uint8_t n)
{
  return (uint8_t)(n >= ISL_T_N);
}

/* 每 10 ms 一次。
 * 入参 turn_now：本帧 PD 算出的转向量，进入 T 口时用它做锁定值。
 * 返回 1 = 本帧处于"穿过 T 口"状态（调用方应按此给速度/转向）。 */
static uint8_t Island_Step(uint8_t n, uint32_t now, int32_t dist, float turn_now)
{
  /* ---- 正在穿 T 口 ---- */
  if (isl_state == ISL_CROSS) {
    uint8_t done = 0U;

    /* 走够距离 且 图案已经散开（不再是整排白）→ 穿完了 */
    if ((dist - isl_d0) >= ISL_CROSS_MM
        && (uint32_t)(now - isl_t_state) >= ISL_CROSS_MIN_MS
        && n <= 5U) {
      done = 1U;
    }
    /* 兜底：走太久也放出来 */
    if ((uint32_t)(now - isl_t_state) > 1500U) done = 1U;

    if (done) {
      isl_state   = ISL_RET;
      isl_t_state = now;
      printf("ISL,OUT,%ld\r\n", (long)(dist - isl_d0));
      Steer_Reset();
      return 0U;
    }
    return 1U;
  }

  /* ---- 冷却 ---- */
  if (isl_state == ISL_RET) {
    if ((uint32_t)(now - isl_t_state) < ISL_COOL_MS) return 0U;
    isl_state = ISL_NONE;
  }

  /* ---- 判进 T 口（多帧确认） ---- */
  if (Island_Pattern(n)) {
    if (!isl_cfm) { isl_cfm = 1U; isl_t_cfm = now; }
    else if ((uint32_t)(now - isl_t_cfm) >= ISL_CONFIRM_MS) {
      isl_cfm       = 0U;
      isl_state     = ISL_CROSS;
      isl_t_state   = now;
      isl_d0        = dist;
      isl_turn_hold = turn_now;          /* 锁定进 T 前的转向 */
      printf("ISL,IN,%d,%04X,%d\r\n", (int)n, (unsigned)line_raw,
             (int)(turn_now));
      return 1U;
    }
  } else {
    isl_cfm = 0U;
  }

  return 0U;
}

/* 处于 T 口时应使用的转向量：锁定为 0（直行）。
 * 不再沿用"进 T 前的转向量"——那会把进 T 瞬间的修正量一直放大，
 * 表现为出 T 后方向跑偏。 */
static float Island_Turn(void)
{
  return 0.0f;
}

/* ==================== PB14 按键（跑动中急停） ==================== */
/* 返回 1 = 本帧检测到一次按下（下降沿） */
static uint8_t Key_Pressed(void)
{
  static uint8_t  prev = 1U;
  static uint32_t t_last = 0U;
  uint8_t now_key = (uint8_t)HAL_GPIO_ReadPin(GPIO_RUN_GPIO_Port, GPIO_RUN_Pin);
  uint8_t fired = 0U;

  if (prev && !now_key && (uint32_t)(HAL_GetTick() - t_last) > 250U) {
    t_last = HAL_GetTick();
    fired = 1U;
  }
  prev = now_key;
  return fired;
}

/* ==================== 主状态机 ==================== */
#define FS_FOLLOW    0U
#define FS_ISLAND    1U
#define FS_RECOVER   2U

void Follow_Line_Run(void)
{
  const int32_t goal_mm = (int32_t)FOLLOW_LAP_MM * FOLLOW_LAPS;
  int32_t  s0 = ENC_Dist_mm();
  uint32_t t0 = HAL_GetTick();
  uint32_t t_ctl = t0, t_prt = t0, t_lost = t0, t_rcv = 0U;
  uint32_t t_grace = 0U;

  float    err_raw = 0.0f, err_keep = 0.0f, e = 0.0f, last_e = 0.0f;
  float    vL = 0.0f, vR = 0.0f, v_tgt = 0.0f, turn = 0.0f;
  float    err_dir = 0.0f;
  int16_t  uL = 0, uR = 0;
  uint8_t  n = 0U, isl_now = 0U, isl_was = 0U;
  uint16_t stall_ms = 0U;
  float    turn_prev = 0.0f;   /* 上一帧转向量，T 口锁定时用 */

  Speed_PID_Init();
  Steer_Reset();
  Island_Reset();
  printf("FOLLOW,START\r\n");

  for (;;) {
    uint32_t now = HAL_GetTick();

    Gyro_Poll_1ms(now);

    if ((uint32_t)(now - t0) > FOLLOW_TIMEOUT_MS) { printf("TIMEOUT\r\n"); break; }

    /* 跑动中再按 PB14 → 立即停车（前 500 ms 不响应，避免发车那一下误触发） */
    if ((uint32_t)(now - t0) > 500U && Key_Pressed()) { printf("ABORT\r\n"); break; }

    if ((uint32_t)(now - t_ctl) >= 10U) {
      t_ctl = now;

      /* ---------- 1) 读线 ---------- */
      n = Line_Read(&err_raw);

      isl_now = ISL_ENABLE ? Island_Step(n, now, ENC_Dist_mm() - s0, turn_prev) : 0U;

      /* 出岛那一帧重置丢线计时：岛内可能几秒没更新过 */
      if (isl_was && !isl_now) { t_grace = now; t_lost = now; }
      isl_was = isl_now;

      /* ---------- 2) 丢线处理 ---------- */
      if (n == 0U) {
        uint8_t wide = (isl_now || (t_grace != 0U
                     && (uint32_t)(now - t_grace) < ISL_OUT_MS)) ? 1U : 0U;

        if ((uint32_t)(now - t_lost) > (wide ? ISL_LOST_MS : LOST_MS)) {
          if (t_rcv == 0U) {          /* 第一次进入找线 */
            t_rcv = now;
          } else if ((uint32_t)(now - t_rcv) > RECOVER_MS) {
            printf("LOST\r\n");
            break;
          }
        }
      } else {
        t_lost = now;
        t_rcv  = 0U;
        if (err_raw >  0.3f) err_dir =  1.0f;   /* 记住最后见线方向 */
        if (err_raw < -0.3f) err_dir = -1.0f;
      }

      /* ---------- 3) 误差整形 + 速度调度 ---------- */
      e = Line_Shape(n, err_raw, &err_keep) - FOLLOW_ERR_ZERO;

      if (t_rcv != 0U) {
        /* 找线：低速、朝最后方向打满，不走 PD */
        v_tgt = RECOVER_V;
        turn  = FOLLOW_TURN_SIGN * err_dir * RECOVER_TURN;
      } else if (isl_now) {
        /* 穿过 T 口：转向锁定（保持进 T 前的转向量），低速直行
         * T 口左右对称，12 路平均会趋近 0，此时 PD 方向不可信 */
        v_tgt = ISL_CROSS_V;
        turn  = Island_Turn();
      } else {
        float ae = (e > 0.0f) ? e : -e;
        if (ae > 7.0f) ae = 7.0f;

        v_tgt = FOLLOW_V_FAST - (FOLLOW_V_FAST - FOLLOW_V_SLOW) * (ae / 7.0f);

        /* 接近宽区先减速（看路数，不看里程） */
        if (n >= WIDE_NEAR_N && v_tgt > ISL_NEAR_V) v_tgt = ISL_NEAR_V;

        turn = FOLLOW_TURN_SIGN * Steer_PD(e, e - last_e)
             + FOLLOW_TURN_BIAS * (v_tgt / FOLLOW_V_FAST);
        if (turn >  FOLLOW_TURN_MAX) turn =  FOLLOW_TURN_MAX;
        if (turn < -FOLLOW_TURN_MAX) turn = -FOLLOW_TURN_MAX;
      }
      last_e    = e;
      turn_prev = turn;

      /* ---------- 4) 起步升速 ---------- */
      {
        uint32_t t_go = (uint32_t)(now - t0);
        if (t_go < FOLLOW_RAMP_MS && t_rcv == 0U && !isl_now)
          v_tgt *= (float)t_go / (float)FOLLOW_RAMP_MS;
      }

      /* ---------- 5) 分配到左右轮 + 每轮速度环 ---------- */
      {
        float tL = v_tgt - turn;
        float tR = v_tgt + turn;

        if (tL < FOLLOW_V_MIN) tL = FOLLOW_V_MIN;
        if (tR < FOLLOW_V_MIN) tR = FOLLOW_V_MIN;

        Speed_Update(&vL, &vR);
        uL = (int16_t)Inc_PID(&pidL, tL, vL);
        uR = (int16_t)Inc_PID(&pidR, tR, vR);
      }

      Motor_Set(uL, uR);

      /* ---------- 6) 堵转保护 ---------- */
      if (vL < 20.0f && vR < 20.0f && v_tgt > 100.0f) stall_ms += 10U;
      else                                           stall_ms  = 0U;
      if (stall_ms > STALL_MS) { printf("STALL\r\n"); break; }

      /* ---------- 7) 圈数判定 ---------- */
      if ((ENC_Dist_mm() - s0) >= goal_mm) { printf("LAPS,DONE\r\n"); break; }
    }

    /* ---------- 打印（100 ms） ---------- */
    if ((uint32_t)(now - t_prt) >= 100U) {
      t_prt = now;
      /* F,状态,里程mm,err×10,目标×10,vL×10,vR×10,uL,uR,路数,位图 */
      printf("F,%d,%ld,%d,%d,%d,%d,%d,%d,%d,%04X\r\n",
             (int)(t_rcv ? FS_RECOVER : (isl_now ? FS_ISLAND : FS_FOLLOW)),
             (long)(ENC_Dist_mm() - s0),
             (int)(e * 10.0f),
             (int)(v_tgt * 10.0f),
             (int)(vL * 10.0f),
             (int)(vR * 10.0f),
             (int)uL, (int)uR, (int)n, (unsigned)line_raw);
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
