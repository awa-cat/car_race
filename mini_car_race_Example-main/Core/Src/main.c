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
#include "string.h"   /* memset */
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
#define GYRO_MAX_DPS   900.0f   /* 超过这个角速度视为坏帧，不积分 */
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

  /* ---------------- 标定合理性检查 ----------------
   * 上电时车只要在动，零偏和方差就会爆表，偏航角积分整段失效。
   * 环岛的出岛判据是"累计偏航 330°"，陀螺坏了就永远等不到，
   * 只能靠 ISL_SES_MAX_MS 的 20 s 超时强制结束 —— 表现为"绕不出来"，
   * 而且会白跑一整趟。与其如此，不如在这里直接卡住逼人重新上电。
   *
   * 正常值（1.txt 里成功的几次）：
   *   GB,1,SD,43    → 零偏 0.1 °/s，死区 0.66 °/s
   *   GB,2,SD,610   → 零偏 0.2 °/s，死区 2.5  °/s
   * 已观测到的坏值：
   *   GB,-180,SD,114323   → 零偏 -18.0 °/s，死区 34  °/s
   *   GB, 423,SD,656482   → 零偏 +42.3 °/s，死区 81  °/s
   *   GB,-818,SD,3181225  → 零偏 -81.8 °/s，死区 178 °/s
   *
   * 判据取 4 倍余量：|零偏| > 5 °/s 或 死区 > 10 °/s 就判定失败。 */
  {
    float dz = sqrtf(gyro_gate2);          /* 积分死区，°/s */
    float ab = fabsf(gyro_z_bias);         /* 零偏绝对值，°/s */

    if (ab > 5.0f || dz > 10.0f) {
      Motor_Set(0, 0);
      for (;;) {                           /* 卡死，不允许带着坏标定跑车 */
        printf("CAL_BAD,%d,%d\r\n", (int)(ab * 10.0f), (int)(dz * 10.0f));
        printf("CAL_BAD,PUT_CAR_STILL_AND_RESET\r\n");
        HAL_Delay(2000);
      }
    }
  }
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

  /* 尖峰保护：BMI270 量程 ±2000 °/s，这台车正常转弯最多约 350 °/s。
   * SPI 偶发坏帧会读出接近满量程的值，一帧就能把角度积分甩出去几十度，
   * 直接导致"刚进环岛就判出岛"。超过 GYRO_MAX_DPS 的帧整帧丢弃。 */
  if (g > GYRO_MAX_DPS || g < -GYRO_MAX_DPS) {
    gyro_last_g = 0.0f;
    return;
  }

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
#define FOLLOW_LAP_MM      20580     /* 单圈里程 mm（实测，两根竖条的中点）。
                                      * 依据：11/18 15:43 那趟门在 20349 / 20767，
                                      *       中点是 20558；11/18 15:19 那趟约 20880。
                                      * 用途有两个：
                                      *   1) 竖条门的"里程窗口"中心（±BAR_LAP_TOL_MM）
                                      *   2) FOLLOW_LAPS 圈全都没认出竖条时的里程兜底 */
#define FOLLOW_LAPS            1     /* 【先只跑一圈，验证一圈能完整跑完再改回 2】
                                      * 之前一直是 2，变量太多：格2/八边形/格8/门/第二圈
                                      * 任何一处出问题都混在一起看不出因果。
                                      * =1 时：第一圈过门即停（GATE,STOP 用"第一根竖条"分支），
                                      * 目标只有一个——一圈所有格走完 + 停在发车区。
                                      * 跑通后改回 2 再验第二圈。 */
#define FOLLOW_TIMEOUT_MS 240000U    /* 总超时 240 s。两圈约 2×20450/250 ≈ 164 s，
                                      * 加上环岛减速和找线，240 s 有余量。
                                      * 原 120 s 不够（后面那版就是被 120 s 截断的）。 */

/* ==================== 二、速度 ====================
 * 【提速只动这两个数，其余保护速度全部保持原样】
 *
 * 为什么敢只换这两个：
 *   循迹速度的实际来源是  v_tgt = V_FAST - (V_FAST - V_SLOW) * (|e| / 7)
 *   所以 V_FAST 只决定"线在正中时多快"，也就是直道速度；
 *   一旦偏离，速度沿这条直线掉到 V_SLOW。
 *
 *   而所有"该慢的地方"都是独立的硬限幅，不受这两个数影响：
 *     n>=WIDE_NEAR_N → ISL_NEAR_V(150)        宽区/方框直角前减速
 *     急弯预判       → CORNER_V(120)          |e|>3.2 且变化快
 *     过 T 口        → ISL_CROSS_V(150)       锁直行穿口
 *     岛内           → Island_Cfg()->v_island  每格自己配（290/300/150）
 *     找线/断路      → RECOVER_V(160) / GAP_V(250)
 *   也就是说提速只作用在"本来就该全速跑"的直道上，
 *   进弯、进岛、过 T 口、丢线恢复的速度一个都没变。
 *
 * 为什么 +10% 而不是更多：
 *   - 急弯预判靠"|e| 每帧增长量"，速度越高增长越快，
 *     所以提速反而让预判更早触发，这一点是安全的；
 *   - 但丢线容忍 LOST_MS(130 ms) 是按时间算的，
 *     300 mm/s 下等于 39 mm，330 下是 43 mm，略微变长；
 *     再快就要同步收紧 LOST_MS，那是另一处改动，先不碰。
 *   - 差速限幅 FOLLOW_TURN_MAX(450) 不变，speed 越高转向半径越大，
 *     所以先只 +10%，跑一趟看弯道是否还收得住。 */
/* ==================== 二、速度（提速第 2 步：320/190 -> 335/200）====================
 * 循迹速度的唯一来源是
 *     v_tgt = V_FAST - (V_FAST - V_SLOW) * (|e| / 7)
 * 只动这两个端点，其余一切减速出口保持原值 —— 这就是"保证功能完整"的含义：
 *   接近宽区 ISL_NEAR_V(150) / 急弯预判 CORNER_V(120)    不动
 *   过 T 口 ISL_CROSS_V(150) / 岛内 v_island(200~300)    不动
 *   找线 RECOVER_V(160) / 断路 GAP_V(250)               不动
 *   差速限幅 FOLLOW_TURN_MAX(450)、转向 PD/KD            不动
 * 所以提速只作用在"线在正中间、不宽、不弯"的直线段上。
 *
 * 【第 1 步 300->320 已跑过，这是第 2 步 320->335，+4.7%】
 * 幅度故意比第 1 步（+6.7%）小：这一趟同时还改了八边形的岛内速度
 * （v_island 300->200）和里程兜底（2200->1400），两个变量并行，
 * 幅度必须保守，否则失败时分不清是谁的锅。
 *
 * ⚠️ 已知风险：**最后那段波浪形走的就是 FOLLOW_V_FAST**（普通循迹），
 *    而波浪段一直是薄弱环节（历史上出现过"波浪形超出赛道"）。
 *    如果这趟在波浪段失败，先怀疑速度：把 FAST 退回 320 单独再验一趟，
 *    不要同时去改波浪相关的其他参数。
 *
 * 【LOST_MS 暂时不动】130 ms 在 320 下是 41.6 mm、335 下 43.6 mm，
 * 差 2 mm，对丢线判定没有实质影响；超过 350 才需要同步收紧。 */
#define FOLLOW_V_FAST       335.0f   /* 正中时 mm/s（原 320，+4.7%） */
#define FOLLOW_V_SLOW       200.0f   /* 偏差最大(±7°)时 mm/s（原 190，+5.3%）
                                      * 跟着 FAST 一起抬，保证减速直线斜率不塌 */
#define FOLLOW_RAMP_MS        300U   /* 起步升速时间 ms */
#define FOLLOW_V_MIN      (-200.0f)  /* 单轮目标下限（负=允许反转） */

/* ==================== 三、转向环 PD(+I) ====================
 * 按官方教程的"方法二（双 PD）"组织：
 *   转向输出 = KP*e + KP2*e*|e| + KD*de + KI*∫e − GKD*gyro_z
 *   其中 KP2 只在大偏差时额外加力，GKD 用陀螺仪抑制摆动。
 * 两项默认 0，等于原来的纯 PD 行为；调参时按教程顺序打开：
 *   ① 调 KP 到"能回正但会摆" ② 加 GKD 消除摆动（会变慢）
 *   ③ 加 KP2 把速度补回来（又快又不摆）
 */
#define FOLLOW_KP           150.0f   /* mm/s per ° */
#define FOLLOW_KI             0.8f   /* mm/s per °·s */
#define FOLLOW_KD            18.0f   /* mm/s per °（每 10 ms 差分，等效 ×100） */
#define FOLLOW_KP2            40.0f   /* 非线性项：e*|e| 系数（大偏差加力） */
#define FOLLOW_GKD            3.0f   /* 陀螺仪阻尼：gyro_z 系数（抑制摆动） */
#define FOLLOW_I_MAX         60.0f   /* 积分限幅 mm/s */
#define FOLLOW_TURN_MAX     450.0f   /* 差速限幅 mm/s */
#define FOLLOW_TURN_SIGN    (-1)     /* 方向开关，已实测 */
#define FOLLOW_ERR_ZERO       0.0f   /* 传感器零点 ° */
#define FOLLOW_TURN_BIAS      0.0f   /* 整车跑偏补偿 mm/s */

/* ==================== 四、每轮速度 PI ==================== */
#define FOLLOW_KP_SPD         1.2f   /* PWM per mm/s */
#define FOLLOW_KI_SPD         0.4f   /* PWM per mm/s 每帧 */
#define FOLLOW_PWM_MAX     4200.0f
#define FOLLOW_PWM_START    900.0f
#define FOLLOW_DT            0.01f   /* 控制周期 s */

/* ==================== 五、宽区（T 口）处理 ==================== */
#define WIDE_NEAR_N           6U     /* 路数 ≥ 此值：略减速（预备）
                                      * 原来 5 太低：普通直线 n=3、略粗的线 n=4~5，
                                      * 于是直道上也会突然减速一下。抬到 8 后：
                                      *   普通直线(3) / 粗线(4~5) → 不减速
                                      *   环岛入口(6~8)           → 只在最宽处减速
                                      *   T 口(11~12)             → 减速
                                      * 副作用是好的：一旦减速就说明 n≥8，早已超过
                                      * 进岛门槛 ISL_FORK_N(6)，"减速"变成进岛的强信号。 */
#define ISL_NEAR_V        150.0f     /* 接近宽区（含方框直角）先压到 150，给转向留时间
                                      * 原来 180 偏快，直角弯来不及转
                                      * 真正的 T 口穿越由 ISL_CROSS_V(150) 负责，不受这里影响 */
#define CORNER_V          120.0f     /* 急弯预判触发时的目标速度 mm/s */
#define WIDE_REL_MS         150U     /* 宽区减速的解除条件：n 跌回 3 路后还要持续这么久
                                      * 普通细线的 n 就是 3（位图 00E0/0070），所以解除阈值
                                      * 必须放到 3；再配 300 ms 是为了入口处 n 在 3~8 之间
                                      * 逐帧跳时不来回开关 */
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
#define ISL_CROSS_V        150.0f    /* 过 T 口速度 mm/s（慢一点，进方框第一个直角更从容） */
#define ISL_COOL_MS         350U     /* 穿过后冷却，防重复触发 */
#define ISL_OUT_MS         1000U     /* 穿过后丢线宽限 */
#define ISL_LOST_MS        1500U     /* T 口内/刚穿过的丢线窗口（比 400 ms 宽） */

/* ---- 环岛会话：给六边形这种"浅角分叉进入"的环岛用 ----
 * 入口不是 T（凑不到 11 路白），而是两条线并在一起、路数比平时多。
 * 进入后正常循迹绕行，靠累计偏航 + 回到单线判出口。 */
#define ISL_FORK_N            6U     /* 分叉判据：白路数 >= 此值
                                      * 1.txt 实测：成功的 8 次进岛 n 全部 >= 6；
                                      * 失败的 2 次都是 n=5 的假分叉（00F8 中间粗线、
                                      * 0C0E），进去 4 s 内就 STALL 撞停。
                                      * 所以 5 不够，抬到 6。 */
#define ISL_FORK_MM          60      /* 分叉要持续走过这么多 mm 才算（普通弯只有 30~50 mm）
                                      * 2.txt 实测：降到 30 后，在普通弯的粗线段
                                      * （03F8/03DE/079C，n=7~8 但两端都不白）误判进岛，
                                      * 之后 T 口处理被禁用，2183 mm 后撞 T 口横线 STALL。
                                      * 1.txt 里 8 次成功进岛全部是在 60 下发生的，恢复 60。 */
#define ISL_HEX_V         300.0f     /* 六边形岛内固定速度：六边形转弯不剧烈，不需要减速 */
#define ISL_EXIT_CROSS_MM  170       /* 出环时锁定直行穿过岔口的距离 mm */
#define ISL_EXIT_DEG      340.0f     /* 出岛偏航阈值（纯偏航判据，不再依赖分叉图案）
                                      * 实测分叉出现的角度窗口：330.5 ~ 347.2（当前读数单位）
                                      * 取正中 340。注意循迹路径的 gyro 没乘 GYRO_SCALE，
                                      * 读数是物理角的 1/0.96 倍，所以 340 对应物理约 326°。 */
#define ISL_EXIT_MIN_MM     600      /* 出岛里程下限护栏：进岛后至少走这么远才允许出。
                                      * 防止 yaw 偏大时在环道前半段误出 —— 那时还没到岔口，
                                      * 一旦进 ISL_HEXOUT 锁定直行 170 mm 会撞岛壁。 */

/* ---- 假进岛自检 ----
 * 真环岛：850 mm 里转约 340°（实测 337.2 / 340.8 / 341.3）
 * 假进岛：850 mm 里只转 50~107°（实测 51.6 / 50.5 / 107.1 / 0.0）
 * 两者差一个数量级，用 300 mm 处 40° 这条线就能分开。 */
#define ISL_FAKE_MM        300       /* 进岛后走这么远开始自检 */
#define ISL_FAKE_DEG        25.0f    /* 偏航还不到这个值 → 判为假进岛。
                                      *
                                      * 【40 -> 25】八边形连续四次都卡在这条线上：
                                      *   17:43 ISL,FABORT,301,390  → 39.0°
                                      *   17:45 ISL,FABORT,303,300  → 30.0°
                                      *   18:19 ISL,FABORT,302,379  → 37.9°
                                      *   （更早还有一次 301,390）
                                      * 阈值 40，四次全部落在 30~39 之间，一次都没到。
                                      * 八边形周长约 2000 mm，300 mm 只走了 15%，
                                      * 本来就不该要求它转 40° —— 这是拿"六边形/八边形
                                      * 头 300 mm 的平均速率"去要求一个起步更慢的形状。
                                      *
                                      * 为什么可以降到 25：
                                      *   真绕环岛  850 mm 转 337 / 341 / 341°
                                      *   假进岛    850 mm 只转 50~107°，而且头 300 mm 更少
                                      *              （从直线上穿过去时偏航 0~10°）
                                      * 25 仍然能把"从直线穿过去"（0~10°）挡掉，
                                      * 同时放过 30~39° 的正常起步。 */
#define ISL_OVER_DEG      380.0f     /* 超过这个偏航强制结束，防绕圈
                                      * 紧到 380：一旦错过了出口岔口，也别绕太远才停 */
#define ISL_SES_MAX_MS   20000U      /* 会话最长 20 s */
#define ISL_RET_MM          260      /* 会话结束后，至少再走这么远才允许重新进岛（新赛道环岛更密） */
#define ISL_PHASE_MS      30000U     /* 单阶段超时：这么久没等到图案就强制切换 */
/* 六边形格的"放弃里程"**默认值**（配合计划格起点里程 isl_d_phase）：
 * 这一格是六边形、但走够这么多 mm 还没进岛，就强制 Island_Advance 到下一格。
 * 每一格可以在 isl_cfg 的 skip_mm 里单独覆盖（0 = 用这个默认值）。
 *
 * 【为什么改成按格配置】第 2 格（SMALLHEX）从来没成功过，每趟都是
 * ISL,FABORT 后被这里跳过，但要吃掉 1300~1600 mm，而且"吐出来"的位置每趟不同：
 *   16:48 那趟  FABORT,303,21   HEXOVER 1530
 *   17:03 第一趟 FABORT,301,56   HEXOVER 1523
 *   17:03 第二趟 FABORT,425,399  HEXOVER 1574
 * 偏航 2.1° -> 5.6° -> 39.9°，出口位置漂几百毫米。
 * 它交棒的早晚决定了下一格 SQ3 从哪里开始找 T 口：
 * 17:03 第一趟第 3 格在 9317 开启，而 SQ3 第一个 T 口在 9335 —— 只差 18 mm，
 * 抢到就对上，抢不到就把 SQ3 的两个 T 口配错，后面每个方形都错位一格；
 * 那趟最后在 10544 撞进死路分支、满舵恢复顶墙 STALL（"入岛变转弯"）。
 *
 * 所以给第 2 格单独设 skip_mm = 500（在 ~8300 就交棒，距 SQ3 第一个 T 口
 * 还有 1000 mm 余量）。而八边形入口实测只在本格起点后 400 mm，
 * 全局改成 500 会把它误杀，所以八边形 / 第 8 格保持 1200。 */
#define ISL_HEX_MAX_MM    1200       /* 默认值；按格覆盖见 isl_cfg 的 skip_mm */

/* ==================== 大环岛：里程强制进/出 ====================
 * 分叉图案判据实测不可靠（n 在 4~8 之间跳、位图重复），漏判就整圈不认，
 * 一旦漏判就要再绕一整圈。改为**里程强制**：以"第二个 T 口"为基准点，
 * 走够固定里程就无条件进岛；进岛后走够固定里程就无条件出岛。
 * 图案判据保留，只作为"更早触发"的补充。
 *
 * 基准：第二个 T 口的触发里程。
 * 实测 T口2 → 大环岛真入口：
 *   1586 / 1591 / 1690 / 1691 / 1700 / 1705   → 均值 1660，散布 ±60 */
/* 里程强制进岛（旧赛道专用，第二阶段赛道已停用）：
 * 原来按"第二个 T 口之后 1660 mm 就是大环岛入口"写死，只对旧赛道有效。
 * 新赛道一圈 10 个环岛，同一个里程值不可能都适用，置大值等于停用，
 * 进岛完全靠图案判据 + 计划表。 */
#define ISL_ENTRY_MM      1660       /* 队友版：第二个 T 口之后走这么远强制进岛 */

/* 进岛后走够这么远强制出环（兜底，防止图案判据失灵时绕不停）：
 * 六边形周长约 800 mm，八边形约 1200 mm，取 1600 留足余量。 */
#define ISL_EXIT_MM        850       /* 队友版：进岛后走这么远强制出岛 */

/* ==================== 七、丢线找线 / 断线盲走 ==================== */
/* 丢线容忍时间分两种场景 —— 一个值无法同时满足：
 *   普通赛道（直角弯）：要快，130 ms 就开始打角，否则冲出去；
 *   分叉候选已出现（isl_cfm=1）：要慢，等图案稳定再判断，
 *     否则偶尔丢一两帧就被"找线"拐离环岛入口，进不去岛。 */
#define LOST_MS           130U       /* 普通赛道：丢线 130 ms 就打角（约 38 mm） */
#define LOST_ISL_MS       500U       /* 分叉候选期：放宽到 500 ms，别急着打角 */
#define RECOVER_MS       1200U       /* 找线/盲走总时长：3 s 会拐出 700 mm 的大圈 */
#define ISL_RECOVER_MS   3000U       /* 环岛会话内找线的时长（比普通赛道长）。
                                      * 【为什么必须分开】11/19 17:00 八边形实测：
                                      *   18299 进岛，偏航一路涨到 +352°（exit_deg 要 340°）
                                      *   18767 丢线 → 盲走（Island_Step 被冻结，
                                      *                          偏航/里程出口都判不了）
                                      *   18881 找回线（偏航只 +188°，还没到 340）
                                      *   19089 再丢线 → 盲走 → 19282 LOST 停机
                                      * 第二段盲走只给了 RECOVER_MS(1200 ms)，
                                      * 160 mm/s 下约 192 mm，还没等偏航走完/线回来就 LOST。
                                      * 环岛内本来就是"绕一圈"的运动，允许更长的寻找时间；
                                      * 而普通赛道的直角弯必须短（1200 ms），拖长会拐大圈冲出去，
                                      * 所以这两个值不能共用一个。 */
#define RECOVER_V         160.0f     /* 找线速度：慢一点转弯半径更小 */
#define RECOVER_TURN      220.0f     /* 找线时的转向量：直角弯需要更狠 */

/* 断线盲走：赛道上有 650 mm 的"断路口区域"没有线，
 * 丢线后不能朝最后方向打满（那会在缺口里原地打转），
 * 而要锁住航向直行冲过去。 */
#define GAP_MM           1500        /* 断路：斜穿 693 + 接线余量（不用于直角弯） */
#define GAP_MS           4000U       /* 断路专用超时。必须和 RECOVER_MS 分开：
                                      * RECOVER_MS(1200) 是给直角弯丢线用的，
                                      * 提到 4 s 会让直角弯跑飞时多冲好几秒。 */
#define GAP_TURN_DEG     24.0f       /* 断路斜穿角，右偏为正（20 偏小，实测接上了但角度不足，加到 24）。
                                      * 实测几何：水平 650 mm、右侧接收线低 240 mm，
                                      * atan(240/650) = 20.3°。
                                      * 保持原航向（0°）会从目标白线上方 240 mm 飞过。
                                      * 现场按 ±5° 一格调。 */
#define GAP_CONFIRM_N     3U         /* 接线确认：连续这么多帧见到线才交还 PD，
                                      * 防止扫到反光/杂物就提前结束盲走 */
#define GAP_MIN_THIN_MM   150        /* 断路判定前置条件：丢线之前必须已经连续走了
                                      * 这么长的"细线"（n<=4）。清零门槛见下面的
                                      * thin_mm 统计处：n>=5 才算宽白。
                                      *
                                      * 【为什么】原来只靠丢线那一帧的 |err|>1.6 区分
                                      * "断路"和"弯道丢线"，实测两次都判 0，分不出来：
                                      *   真断路 11687：err +0.2  <- 该判 0
                                      *   误判   20391：err -0.2  <- 也该判 1，却判了 0
                                      * 后果：八边形只转了 181° 就被 850 mm 兜底踢出来，
                                      * 出口方向全错，200 mm 后丢线被当成断路，
                                      * 以 250 mm/s 直着盲走 823 mm，冲出赛道 STALL。
                                      *
                                      * 真正的区别在丢线之前那 100~200 mm：
                                      *   真断路前  0038(n=3)→0070(n=3)→0070→0070  连续 200 mm 细线
                                      *   误判那一次 0FFF(12)→003F(6)→007E(6)→007C(5)→00FE(7)
                                      *              全是宽白，细线只维持了 17 mm
                                      * 断路在赛道上是"干净直道中间断一截"，前面不会有宽白；
                                      * 出岛/岔口/拐角丢线前面一定有宽白。 */
#define GAP_V             250.0f     /* 盲走速度 mm/s */
#define GAP_KP              2.5f     /* 航向保持：航向差 1 度 -> 2.5 mm/s 差速 */
#define GAP_TURN_MAX      130.0f     /* 航向保持差速限幅 */

/* ==================== 七点五、发车/终点门（"工"字竖条） ====================
 * 目的：跑完设定圈数时，靠光电管认出发车区那两根竖条，**停在发车区里**，
 *       不用再去猜单圈里程。
 *
 * 为什么不能只用里程：FOLLOW_LAP_MM 是"车实际走过的路"，环岛绕进去还是
 * ISL,FABORT 跳过，单圈就差几百毫米；换一次环岛代码就得重新标定。
 * 为什么不能只认光电管：发车区竖条和方形 T 口在 12 路上长得一模一样
 * （整排全白 + 前后细线），单看位图分不开。
 *
 * 所以两个判据一起用：
 *   ① 里程窗口：只在本圈里程接近 FOLLOW_LAP_MM 时才把整排白当"门候选"
 *   ② 成对：发车区是两根竖条、间距约 400 mm；方形 T 口只有一根
 * 两个条件同时满足才算过门。
 *
 * ---- 竖条指纹 ----
 *   整排白(n>=11) + 前面 >=120 mm 干净细线 + 后面连续 3 帧回到细线
 *   宽白持续超过 15 帧(150 ms) -> 判为 T 口/宽区，不是竖条 */
#define BAR_FULL_N        11U        /* 整排白：n >= 11（门的第一根） */
#define BAR_FULL_N2       10U        /* 门的第二根：放宽到 n >= 10。
                                      * 实测 20648 那根是 0FFA，ch2 掉了只有 10 路，
                                      * 用 11 会让整圈的门永远配不成对。 */
#define BAR_THIN_MM      120.0f      /* 竖条前必须有这么长的干净细线 */
#define BAR_ERR_MAX        3.0f      /* |err| 小于此值：正着过，不是斜切。
                                      * 【2.0 -> 3.0】17:14 那趟跑完第一圈但第二圈
                                      * 没进大套小，根因就是发车线两根竖条都没认出来。
                                      * 那趟窗口内(20080~21080)只有两处整排白：
                                      *   20133  0FFF n=12  err=+1.0   <- err 合格却没 BAR,HIT
                                      *   20516  0FFF n=12  err=-2.6   <- 被 |err|<2.0 拒掉
                                      * 两根都没认成门 -> 没有 LAP,1 -> lap_start_mm 仍是 0
                                      * -> 第二圈进第 1 格时 dist-lap_start=25259
                                      * -> BS,MISS，整个大套小被跳过。
                                      * 车过发车区本来就会带一点偏（这次 -2.6°），2.0 太严；
                                      * 真正的"斜切"是 err 5 以上，3.0 仍能挡住。
                                      * 方形 T 口不靠这个条件排除（靠 BAR_THIN_MM 和里程窗口），
                                      * 所以放宽不会增加误判。 */
#define BAR_CONFIRM_N      3U        /* 之后连续这么多帧回到细线才确认"薄" */
#define BAR_WIDE_MAX      15U        /* 宽白最多持续这么多帧（150 ms） */
#define BAR_DUP_MM        80         /* 两根竖条不可能这么近，用于去重 */

/* ---- 成对判据（中间圈认圈用）----
 * 发车区是"工"字两根竖条，实测相距 409 / 415 / 418 / 419 mm
 * （17:31 那趟 SQ,CLOSE 打出来的就是这两根）。
 * 上下限放到 300~550 容纳斜穿和打滑。
 * 注意和方形的 SQ_MIN_GAP_MM(450) 互相配合：415 < 450，
 * 所以方形格不会把发车区这两根当成自己的一对 T 口。 */
#define PAIR_MIN_MM       300
#define PAIR_MAX_MM       550

/* ---- 里程窗口 ----
 * 中心 = FOLLOW_LAP_MM（本圈里程），容差 ±500 mm。
 * 实测发车门位置：11/18 两趟分别在 20349~20767 和 ~20880，跨度约 500，
 * 所以窗口不能太窄；但也不能像第一版 ±1800 那样宽 —— 那样途中方形的 T 口
 * 会掉进窗口，被判成门。
 * 窗口外一律不理（BAR,OUT），只有进窗口的整排白才有资格当"发车门"。 */
#define BAR_LAP_TOL_MM    500

/* ---- 停车点 ----
 * 现场实测（11/18 15:43）：第一根 20349、第二根 20767，停 20767 时
 * 车头超出发车区 7~8 cm，要求再提前 10 cm。
 * 所以改成"认到窗口内第一根竖条就定停车点"，停在这根之后
 * STOP_FROM_FIRST_MM 处 —— 落在两根竖条之间。
 *
 * 代价：不再要求"成对"，所以窗口内若出现方形 T 口会被误当成门。
 * 用收紧的里程窗口(±500)来防；实跑时如果车停在半路而不是发车区，
 * 就是撞上了这个，届时把 BAR_LAP_TOL_MM 再调小。 */
#define STOP_FROM_FIRST_MM  110        /* 认到竖条之后这么多 mm 停。
                                      * 280 -> 110：11/18 16:13 那趟现场反馈"停车偏近，
                                      * 再多走 10 cm"。那次实际是里程兜底停在 20580，
                                      * 而竖条在 20569 —— 相当于只走了 11 mm，
                                      * 所以基准从"竖条 + 280"改成"竖条 + 110"。 */

/* 里程兜底的额外余量。
 * 【必须给】11/18 16:13：竖条在 20569，而 goal_mm(=FOLLOW_LAP_MM×FOLLOW_LAPS)
 * 正好是 20580 —— 里程兜底抢在竖条门确认之前 11 mm 就把车停了，
 * BAR,HIT 需要的"3 帧回到细线"（约 30~50 mm）根本来不及。
 * 加 600 mm 余量后，门总是先有机会；只有在门完全认不出来时才用里程兜底。 */
#define MILEAGE_STOP_MARGIN_MM  600

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

  /* 方法二（双 PD）：
   *   KP*e        基础回正
   *   KP2*e*|e|   大偏差时额外加力（小偏差几乎不参与，不干扰直线）
   *   KD*de       提前量
   *   KI*∫e       消除稳态偏移
   *   -GKD*gyro_z 陀螺仪阻尼：gyro_z>0 表示正在右转，
   *               减去后让 turn 往左，抵消正在发生的摆动
   * KP2 / GKD 默认 0，行为与原来的纯 PD 完全一致。 */
  r = FOLLOW_KP             * e
    + FOLLOW_KP2            * e * fabsf(e)
    + FOLLOW_KI             * line_i
    + FOLLOW_KD             * de
    - FOLLOW_GKD            * gyro_z;

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

/* ==================== 环岛 ====================
 * 第二阶段赛道一圈有 10 个环岛，顺序固定（见 isl_plan）。
 * 两种环岛的图案在光电管上会重叠，靠图案分不干净，
 * 所以用"计划表 + 指针"：当前该处理哪一个由 plan_idx 决定，
 * 处理完就往后挪一格，跑满 10 个回到开头。
 *
 * 计划表按赛道顺序（来源：relate/赛道特殊元素按顺序列举.docx）：
 *   1 大六边形（套小六边形）
 *   2 小六边形
 *   3 六边形
 *   4 方形   5 方形   6 方形   7 方形
 *   -- 断路口区域（650 mm 无线，由盲走逻辑处理，不占计划格）--
 *   8 多边形（八边形 45°）
 *   9 六边形
 *  10 方形
 *
 * 状态：
 *   ISL_NONE     正常循迹
 *   ISL_CROSS    正穿过 T 口：锁定直行（方形环岛）
 *   ISL_SESSION  六边形/多边形会话：正常循迹绕行
 *   ISL_HEXOUT   出环：锁定直行穿过岔口
 *   ISL_RET      冷却
 */

#define ISL_NONE     0U
#define ISL_CROSS    1U
#define ISL_RET      2U
#define ISL_SESSION  3U
#define ISL_HEXOUT   5U     /* 出环：锁定直行穿过岔口 */
#define ISL_HIN      6U     /* 进岛：先锁定直行穿过交叉点，再交给 PD */

/* 计划表里的元素类型 */
#define PLAN_HEX     0U     /* 六边形/多边形：分叉进、绕行、出环 */
#define PLAN_SQ      1U     /* 方形：T 口直行进入 + 直行穿出 */

#define ISL_PLAN_N    9U

/* 顺序 = 赛道上环岛的实际出现顺序（现场确认）：
 *   大套小 → 1 个六边形 → 4 个正方形 → 30° 六边形 → 八边形 → 六边形 → 正方形
 *
 * 之前第 3 格写的是六边形（HEX3），但赛道上大套小之后只有 **一个** 六边形，
 * 紧接着就是正方形。于是车走完六边形后 plan_idx=2 以为还在等六边形：
 *   - 正方形的 T 口是 n=12，被 Island_Fork 的 `n > 9U` 直接拒掉
 *   - T 口判据 Island_Pattern 只在 PLAN_SQ 的格子才被调用
 * 结果正方形完全不可见，车以 290~300 mm/s 直插 90° 直角冲出赛道
 * （日志 8147 mm 的 0FFF 就是那个被无视的 T 口）。
 * 所以把 30° 六边形挪到 4 个正方形之后，方形段整体前移一格。 */
static const uint8_t isl_plan[ISL_PLAN_N] = {
  PLAN_HEX,   /*  1 大套小（专用流程） */
  PLAN_HEX,   /*  2 六边形（顶点进入·逆时针） */
  PLAN_SQ,    /*  3 正方形① */
  PLAN_SQ,    /*  4 正方形② */
  PLAN_SQ,    /*  5 正方形③ */
  PLAN_SQ,    /*  6 正方形④ */
  PLAN_HEX,   /*  7 多边形（八边形） */
  PLAN_HEX,   /*  8 六边形 */
  PLAN_SQ     /*  9 方形 */
};

static uint8_t  isl_state = ISL_NONE;
static uint8_t  isl_cfm   = 0U;
static uint8_t  plan_idx  = 0U;         /* 当前该处理计划表里的第几个（0 基） */
static uint8_t  sq_events = 0U;         /* 方形 T 口计数：每个方形需要 2 次（进+出） */
static int32_t  sq_d0     = 0;          /* 方形第一个 T 口的里程（算两次间隔用） */
static uint32_t isl_t_cfm = 0U;
static uint32_t isl_t_state = 0U;
static uint32_t isl_t_phase = 0U;       /* 计划格起始时刻 */
static int32_t  isl_d0    = 0;          /* 进入 T 口时的里程 */
static float    isl_yaw0  = 0.0f;       /* 会话起始航向 */
static float    isl_err_off = 0.0f;     /* 切向进岛的误差偏移（等价补线），出岛清零 */
static int32_t  isl_block_dist = 0;     /* 在这之前禁止进入任何环岛（大套小刚结束用） */
static int32_t  isl_ret_dist = 0;       /* 上次会话结束时的里程 */
static uint8_t  isl_out_arm  = 0U;      /* 出岛武装：偏航到 ISL_EXIT_DEG 后置 1，
                                         * 之后任意一帧匹配分叉即出岛 */
static int32_t  isl_d_t2     = 0;       /* 第二个 T 口的里程，强制进岛的基准 */
static int32_t  isl_d_fin    = 0;       /* 本次会话进岛的里程，强制出岛的基准 */
static int32_t  isl_d_phase  = 0;       /* 当前这一计划格开始时的里程（里程兜底用） */
static uint8_t  isl_plan_last = 0xFFU;  /* 上一帧的 plan_idx，用来发现"换格了" */
/* 本格是否已经用过"里程强制进岛"（force_mm）。
 *
 * 【必须有】没有它的话：强制进岛 -> 300 mm 判假 FABORT -> 350 ms 冷却
 * -> 里程还在阈值后面 -> 立刻重新强制进岛 -> 再 FABORT …… 无限重进。
 * 每进一次都重置 isl_yaw0，偏航永远累不起来，车被反复拽进同一个假环岛。
 * 11/19 16:38 那趟的实证：
 *   [18249] ISL,FIN_M,OCT7,560  -> ISL,FABORT,691,-1438
 *   [19047] ISL,FIN_M,OCT7,1158 -> ISL,FABORT,300,-51
 *   [19532] ISL,FIN_M,OCT7,1557 -> 之后 GAP / LOST
 * 三次强制进岛的里程正好相差 560 / 1158 / 1557，就是同一个条件反复命中。
 * 换格时必须清零（Island_Advance / Island_Reset / Lap_Reset 里都要清）。 */
static uint8_t  isl_force_used = 0U;
static uint8_t  rsy_cfm      = 0U;      /* 计划表自校正：T 口持续判据 */
static uint32_t t_rsy        = 0U;

/* ---- 发车/终点门（竖条）状态：全部由 Island_Reset() 清零 ---- */
static uint8_t  bar_cand  = 0U;   /* 1 = 刚看到整排白，等它"变薄"确认 */
static uint8_t  bar_wide  = 0U;   /* 已经在整排白上待了几帧 */
static uint8_t  bar_ok    = 0U;   /* 之后连续细线帧数 */
static int32_t  bar_mm    = 0;    /* 候选出现那一刻的里程（比确认时刻准约 30 mm） */
static uint8_t  bar_hit   = 0U;   /* 1 = 刚确认一根有效竖条，等 Lap_Gate() 消费 */
static int32_t  bar_hit_mm = 0;   /* 这根竖条的里程 */
static int32_t  pair_mm   = 0;    /* 门的第一根里程（0 = 还没有），成对判门用 */
static int32_t  last_bar_mm = -1000;  /* 上一根竖条里程，用于去重 */

/* ---- 圈数 / 停车 ---- */
static int32_t  lap_start_mm = 0;  /* 本圈起点里程：发车=0，每过门更新 */
static uint8_t  lap_cnt   = 0U;    /* 已确认过的圈数 */
static int32_t  stop_mm   = 0;     /* 0 = 不停；非 0 = 走到这个里程就停 */
static uint8_t  lap_wrap_pending = 0U;  /* 1 = 计划表刚回绕到第 1 格，等 Island_Step
                                         * 拿到 dist 后补做换圈重置（见 Island_Advance） */

/* 大套小的阶段编号（Island_InSession 等函数要提前用到） */
#define BS_IDLE    0U
#define BS_BIG     1U     /* 右转 240°，走大环 4 条边 */
#define BS_MID     2U     /* 直行一小段，过渡到小环 */
#define BS_SMALL   3U     /* 右转 360°，走小环 6 条边 */
#define BS_TAIL    4U     /* 直行，走最后 2 条边 */
#define BS_EXT     6U     /* 出岛前右转 30 度 */
#define BS_OUT     5U
/* ==================== 大套小专用流程的状态 ====================
 * 全部收进一个结构体，复位时直接 memset。
 * 教训：之前是 11 个散落的静态变量，每次新增都会忘记加进 Island_Reset()，
 * 结果"第一次发车正常，之后每次都错"（残留的 err_off 让车全程歪着走）。 */
typedef struct {
  uint8_t  state;      /* BS_IDLE / BS_BIG / BS_MID / BS_SMALL / BS_TAIL / BS_EXT / BS_OUT */
  float    yaw0;       /* 进岛时的航向基准 */
  float    yaw1;       /* 出岛转向段的航向基准 */
  float    yaw2;       /* 进入小环时的航向基准 */
  float    turn;       /* 叠加给跟随环的转向偏置 */
  float    err_off;    /* 误差目标偏移（等价补线） */
  int32_t  j_dist;     /* 最近一次看到交叉点的里程 */
  int32_t  d0;         /* 本段起点的里程 */
  int32_t  d0_track;   /* 轨迹打印的上一个里程点 */
  int32_t  tr;         /* 进度打印的里程累加 */
  uint32_t t0;         /* 进入 BS_OUT 的时刻 */
} BigSmall_t;

static BigSmall_t bs;   /* 由 Island_Reset() 里 memset 清零 */

/* ==================== 每个环岛一组独立参数 ====================
 * 一圈 10 个环岛，尺寸/角度都不同（六边形 200~300、八边形 250、方框 250×350），
 * 用同一套阈值必然有的进不去、有的出不来。
 * 所以每个环岛在这里占一行，各自带自己的进岛门槛、出岛角度、锁直行距离、岛内速度。
 * 状态机（plan_idx）决定当前用哪一行，调参时只改对应那一行。 */
#define ISLK_LOOP    0U     /* 需要绕行的（六边形 / 八边形），出岛用 |偏航| */
#define ISLK_BSS     2U     /* 大六边形套小六边形：走专用流程 Isl_BigSmall() */
#define ISLK_CROSS   1U     /* 直行穿越的（方形 T 口） */
#define ISLK_HEXV    3U     /* 顶点进入的六边形：主线穿过一个顶点，
                             * 逆时针绕一圈，再从同一顶点直着出来（图 1：225°/60°）
                             * 注意：原来写的是"车直着进去"，以为画线自己会把车顺进环。
                             * 实测否掉了这个假设 —— 主线从顶点直穿，六边形的两条边
                             * 以 ±60° 岔出去，没有任何一段线会把车拐进去。
                             * 不给打角时车就一路顺主线直行：296 mm 只转了 20.4°，
                             * PD 的 err 在 7878→7945 之间从 -3.5 塌到 -0.3，两轮拉平，
                             * 最后撞上六边形的下一条边（8058 01FF n=9）后全盲冲出赛道。
                             * 所以 entry_deg 必须给 ≈ 顶点标注角（60°），不能是 0。 */
#define ISLK_HEXT    4U     /* 切向进入的六边形：主线与一条边成 30°，要打角才能进去，
                             * 顺时针绕一圈，从同一点出来（图 2：30°） */

typedef struct {
  const char *name;          /* 日志里显示的名字 */
  uint8_t  kind;             /* ISLK_LOOP / ISLK_CROSS / ISLK_BSS / ISLK_HEXV / ISLK_HEXT */
  uint8_t  fork_n;           /* 进岛：白路数下限 */
  uint16_t fork_mm;          /* 进岛：图案要持续的距离 */
  uint16_t exit_deg;         /* 出岛：累计偏航阈值（度） */
  uint16_t exit_fall_mm;     /* 出岛：兜底里程（走够就强制出） */
  uint16_t out_cross_mm;     /* 出环：锁定直行穿过岔口的距离 */
  uint16_t v_island;         /* 岛内目标速度 mm/s */
  int8_t   dir;              /* 绕行方向：+1 顺时针（右转），-1 逆时针（左转），
                              * 0 = 用 |偏航|（老行为，不分方向） */
  uint16_t entry_deg;        /* 进岛后前多少度内给误差偏移；0 = 不打角，顺线进 */
  uint16_t in_cross_mm;      /* 进岛时先锁直行穿过交叉点的距离；0 = 不锁 */
  uint16_t force_mm;         /* 里程强制进岛：本格开始后走够这么远就无条件进岛，
                              * 不看图案。参考第一阶段 3b580d1（跑完两圈）的做法：
                              * 六边形入口的位图实测不可靠，同一条赛道两趟分别
                              * 读成 0FFE(n=11) 和 003F(n=6)，只靠图案必然漏判。
                              * 0 = 不用里程兜底。 */
  uint16_t skip_mm;          /* 本格放弃里程：走够这么远还没进岛就 Island_Advance
                              * 到下一格（只看"压根没进岛"的情况，正在绕行时不生效）。
                              * 0 = 用默认 ISL_HEX_MAX_MM。
                              * 按格配置的原因：第 2 格（SMALLHEX）从来没成功过，
                              * 需要压到 500 让它早点交棒；但八边形入口实测在
                              * 本格起点后只有 400 mm，用 500 会误伤，必须分开给。 */
} IslandCfg;

/* ==================== 八边形（第 7 格）必须打角才能进去 ====================
 * 11/18 16:12 那趟抓到的证据 —— 八边形一直没真的绕上去过：
 *
 *   18451  n=3  0070  err= +3    干净细线
 *   18476  n=9  0FF8  err=-17    ★ 分叉出现，分支在【左】
 *   18495  n=3  0E00  err=-51    线跑到最左边
 *   18511  n=5  07C0  err=-23
 *   18545  n=4  03C0  err=-16    又回到中间
 *   18636  n=10 03FF  err=+12    完全回正中
 *
 * 原来的配置是 dir=0 / entry_deg=0 —— 进了"环岛会话"但没有任何打角偏置，
 * PD 看到分叉散开、加权重心被两条分支平均回中间（err 从 -51 回到 +12），
 * 于是**顺着主线直接走过去**，根本没拐进八边形。
 *
 * 决定性证据是偏航：这段 1715 mm 里累计偏航只变了 +161°，
 * 而且在 -543~-356 之间来回摆 ±80°：
 *   18523 -548.9° / 18968 -492.2° / 19386 -361.8° / 19984 -436.4° / 20226 -377.9°
 * 真绕八边形（周长约 2000 mm）应该是单调 +360°。
 * 所以 18476 那次是**假进岛**，靠 1700 mm 里程兜底被踢出来（SEX_F,1700），
 * 330 mm 后撞上发车线 20569 —— 第 8 格（六边形）、第 9 格（方形）一次都没轮到。
 *
 * 改法和当初修第 2 格（SMALLHEX entry_deg 0->60）完全一样：
 *   dir        = -1   逆时针（左转），因为 18476 的分支在左边
 *   entry_deg  = 60   进岛后前 60° 给 -ISL_ENTRY_OFF 偏置，把车掰进环
 *   exit_fall_mm 1700 -> 2200
 *     真绕一圈约 2000 mm（周长），1700 会在还差 100 多毫米时又把车踢出来。
 *     2200 让 ISL_OVER_DEG(380°) 先兜底，不会绕不停。
 *
 * ---- 17:09 那趟证明上面这套还不够，于是又改两处 ----
 * 该趟日志：18203 n=9 0FF8（真岔口）→ 18250 n=5 03E0 → ISL,FIN,OCT7,18251
 *           → ISL,FABORT,300,-529
 *
 * ① fork_n 5 -> 9
 *    原来 fork_n=5，任何 5 路白的瞬间都算"分叉入口"。
 *    真岔口是 n=9（0FF8，17:09 的 18203 / 16:12 的 18476 两趟都是这个），
 *    而车实际是在 18250 的 03E0（n=5、err 只有 -13 的一条普通略宽线）上进岛的。
 *    提到 9 之后只有 0FF8 能触发。
 *
 * ② in_cross_mm 0 -> 80
 *    进岛后先锁直行穿过岔口，再交给 PD。
 *    原来 0 = 进岛立刻给 PD，而岔口处两条分支平均，加权重心趋近 0，
 *    PD 的方向不可信 —— 这就是"进岛时左转、300 mm 后净往右 52.9°"的由来。
 *
 * 注意两个副作用（跑一趟要盯着）：
 *   - ISL_HIN 的提前退出条件是 "走了 >=10 mm 且 n < ISL_T_N(11)"。
 *     八边形的岔口是 n=9，一离开岔口 n 就掉到 4，所以实际只锁约 10 mm，
 *     到不了 in_cross_mm 的 80 mm。若仍判假进岛，下一步就是把那个提前退出
 *     阈值改成按格的 fork_n。
 *   - in_cross_mm != 0 会让"顶点进入型"那条分支也对本格生效
 *     （条件是 Island_Pattern，即 n>=11）。本格附近若出现 n=12 的 T 口，
 *     会从那条路提前进岛。 */
#define ISL_ENTRY_OFF   2.5f   /* 切向进岛的误差偏移量（等价摄像头补线） */

/* ---- 方形两次 T 口的合理间隔 ----
 * 小于下限 = 同一个口被数两次；大于上限 = 已经走到下一个元素了。
 *
 * 【上限从 1300 放到 1800】11/18 两趟日志把全部间距列出来后，有两簇很清楚的分布：
 *   647 ~ 971      12 次   真的一对
 *   1629 ~ 1689     6 次   真的一对  <- 1300 把它们全误拒了
 *   2015 ~ 2510     4 次   跨到下一个元素了
 * 1300 把中间那簇砍掉，后果很具体（15:53 那趟）：
 *   SQ,FIRST,13257
 *   SQ,FAR,14886,1629      <- 误拒
 *   SQ,FAR,16598,1652      <- 误拒
 *   SQ,PAIR,16598,17245,647
 * 一格方形吞掉四根 T 口的路（3988 mm），计划表一路落后两格：
 * 跑到 20580 里程上限时还停在第 6 格，八边形 / 六边形 / 第九格都没轮到。
 * 1800 正好卡在 1689 和 2015 之间，两簇分开。 */
#define SQ_MIN_GAP_MM     450
#define SQ_MAX_GAP_MM    1800

static const IslandCfg isl_cfg[ISL_PLAN_N] = {
  /* 名字        类型       进岛n 进岛mm 出岛° 兜底mm 出环mm 岛内速度  方向 进岛打角° */
  /* 【OCT7 的 exit_fall_mm 850 -> 1700】11/18 16:00 那趟实测：
   *   进岛 17779，ISL,SEX_F,850,1872 —— 走满 850 mm 兜底强退时偏航只有 187.2°，
   *   而 exit_deg 要 340°。换算成速率 187.2/850 = 0.220 °/mm，
   *   绕满 340° 需要 340/0.220 ≈ 1545 mm —— 850 只给了一半。
   * 后果：车在八边形走到一半就被扔出来，车头方向全错；出岛后 250 mm 全盲，
   *   偏航在 103 mm 里摆了 138.6°（原地打转），然后打转中撞到 03E0(n=5)
   *   被当成分叉"进岛"，300 mm 只转 89.3° 立刻 ISL,FABORT，
   *   里程强制再进一次后 19896 丢线 LOST —— 表现就是"第 8 格六边形进不去"。
   * 1700 > 1545，留给偏航判据先触发（ISL_EXIT_MIN_MM=900 不会挡）。
   * 预期日志：ISL,SEX,3400,~1545（走正常出岛，不再是 SEX_F）。 */
  /* ==================== 八边形 / 六边形 进岛判据的根治 ====================
   * 17:31 那五趟把根因钉死了：**不是识别不到，是 skip_mm 恰好卡在入口上**。
   *
   *   第 7 格起点（= SQ6 一对结束）   实测 17697 / 17829 / 17920
   *   ISL,HEXOVER 发生的本格里程      实测 1202 / 1201 / 1200
   *   真岔口 0FF8(n=9) 的本格里程     实测 1207（第 4 趟 19127 − 17920）
   *
   * 也就是说 skip_mm=1200 在真岔口【前 7 mm】就把这一格判死了。
   * 放弃之后计划表立刻推进到第 8 格，而第 8 格的 fork_n=5 会接受
   * 03E0(n=5) 这种普通宽线 —— 于是 80 mm 后就在错的位置"进岛"，
   * 493 mm 只转 14.6° 被判假。看起来像"六边形进不去"，实际是
   * "八边形被提前 7 mm 跳过 + 六边形在普通宽线上乱进"。
   *
   * 三处一起改（缺一不可）：
   *   OCT7 skip_mm   1200 -> 1800   走得到 1207 那个真岔口，留 600 mm 余量
   *   OCT7 fork_mm     60 -> 30     真岔口很窄（100 ms 采样 = 33 mm），60 抓不住
   *   HEX8 fork_n       5 -> 9      不再被 03E0(n=5) 骗进，只认 0FF8(n=9)
   *   HEX8 fork_mm     60 -> 30     同上
   *
   * 改完这一格就是"里程窗口 + 路口识别"的双重判据：
   *   下限 格子一开始就允许认岔口，但只认 n>=9（普通宽线过不了）
   *   入口 fork_mm=30，真岔口一出现就抓住
   *   上限 skip_mm=1800，走够还没认到就放弃，不会挂死 */
  /* ==================== 八边形 / 六边形 进岛判据的根治 ====================
   * 17:31 那五趟把根因钉死了：**不是识别不到，是 skip_mm 恰好卡在入口上**。
   *
   *   第 7 格起点（= SQ6 一对结束）   实测 17697 / 17829 / 17920
   *   ISL,HEXOVER 发生的本格里程      实测 1202 / 1201 / 1200
   *   真岔口 0FF8(n=9) 的本格里程     实测 1207（第 4 趟 19127 − 17920）
   *
   * 也就是说 skip_mm=1200 在真岔口【前 7 mm】就把这一格判死了。
   * 放弃之后计划表立刻推进到第 8 格，而第 8 格的 fork_n=5 会接受
   * 03E0(n=5) 这种普通宽线 —— 于是 80 mm 后就在错的位置"进岛"，
   * 493 mm 只转 14.6° 被判假。看起来像"六边形进不去"，实际是
   * "八边形被提前 7 mm 跳过 + 六边形在普通宽线上乱进"。
   *
   * 三处一起改（缺一不可）：
   *   OCT7 skip_mm   1200 -> 1800   走得到 1207 那个真岔口，留 600 mm 余量
   *   OCT7 fork_mm     60 -> 30     真岔口很窄（100 ms 采样 = 33 mm），60 抓不住
   *   HEX8 fork_n       5 -> 9      不再被 03E0(n=5) 骗进，只认 0FF8(n=9)
   *   HEX8 fork_mm     60 -> 30     同上
   *
   * 改完这一格就是"里程窗口 + 路口识别"的双重判据：
   *   下限 格子一开始就允许认岔口，但只认 n>=9（普通宽线过不了）
   *   入口 fork_mm=30，真岔口一出现就抓住
   *   上限 skip_mm=1800，走够还没认到就放弃，不会挂死
   *
   * ⚠️ 教训（17:55 踩的坑）：本版本的 IslandCfg **没有 fake_mm 字段**，
   *   字段顺序是 ... in_cross_mm, force_mm, skip_mm。
   *   假进岛自检用的是全局常量 ISL_FAKE_MM(300) / ISL_FAKE_DEG(40)，不能按格配。
   *   曾经把 600 写进 force_mm 的位置，结果八边形在 602 mm 处被强制进岛三次、
   *   三次都判假（ISL,FIN_M,OCT7,602 / 1021 / 1424）。改参数前先数清楚字段。 */
  { "BIGHEX",  ISLK_BSS,    5U,   60,   700,  4200,   180,   290,   0,    0,    0,    0,    0 },  /*  1 大套小（专用） */
  { "SMALLHEX",ISLK_HEXV,   5U,   60,   340,   850,   170,   300,  +1,   60,   80,    0,  500 },  /*  2 顶点进入：dir -1 -> +1 */
  { "SQ3",     ISLK_CROSS, 11U,   20,     0,     0,    60,   150,   0,    0,    0,    0,    0 },  /*  3 正方形① */
  { "SQ4",     ISLK_CROSS, 11U,   20,     0,     0,    60,   150,   0,    0,    0,    0,    0 },  /*  4 正方形② */
  { "SQ5",     ISLK_CROSS, 11U,   20,     0,     0,    60,   150,   0,    0,    0,    0,    0 },  /*  5 正方形③ */
  { "SQ6",     ISLK_CROSS, 11U,   20,     0,     0,    60,   150,   0,    0,    0,    0,    0 },  /*  6 正方形④ */
  /* 【八边形 OCT7 治本：dir -1 -> +1，force_mm 0 -> 560】
   * 11/19 16:30 那趟实测：
   *   [16792] PLAN,7/9,HEX,SQOK        格 7 起点
   *   [17321] ISL,FIN,OCT7,9,03FE      进岛成功（图案没问题，n=9）
   *   [17628] ISL,FABORT,301,-387      进岛 301 mm 就判假退出
   *
   * 判假退出（FABORT）的判据是全局 ISL_FAKE_MM(300) + ISL_FAKE_DEG(25)：
   *   run >= 300 且 yaw < 25  → 判为假进岛
   * 而 yaw = dir * (gyro_angle - yaw0)，dir=-1 → 实测 yaw = -38.7，
   * 说明车实际是**右转 38.7°**（dy=+38.7）。dir 取 -1 时 yaw 永远是负数，
   * 再怎么转也到不了 +25 → 必然 FABORT。
   * 改成 dir=+1 后 yaw=+38.7 > 25，能通过自检，继续绕到 exit_deg(340)。
   *
   * force_mm=560：按里程强制进岛，不看图案。
   * 本版 IslandCfg 没有 fake_mm 字段（见 1506 行注释），所以这里只加里程入口。
   * 实测"格起点 → 八边形入口"的距离：
   *   11/19 16:30  16792 -> 17321  = 529
   *   11/18 16:13  ~17874 -> 18525 = 651
   * 取 560 居中；散布 ±60 mm 由 in_cross_mm(80) 的锁直行吃掉。
   *
   * 为什么这两个一起改：只改 dir，入口仍靠图案（有时读 0FFE/0FFF，
   * 有时 n=5），漏一次就整格跳过；只改 force_mm，dir 还是反的，
   * 强制进岛后照样在 300 mm 处 FABORT。两个必须同时给。 */
  /* 【11/19 17:00 再调两处：v_island 300->200、exit_fall 2200->1400】
   * 上一趟八边形里的实测（格7起点 17731，进岛 18299）：
   *   18318~18767  bm 003E->00E0->03C0->0380->0780->00F0->00E0->0038
   *                err +4.8 -> +0.8 -> -0.3 -> +2.5 -> +1.0
   *   线在传感器上左右扫得很快，300 mm/s 下 PD 跟不住 -> 18767 丢线
   *   -> 盲走（Island_Step 被冻结，偏航/里程出口都判不了）-> 19089 再丢
   *   -> 19282 LOST 停机，run 只有 983 mm。
   * 进岛到停车累计偏航 +352°，说明"绕是绕到了"，只是没机会判出口。
   *
   * v_island 300 -> 200 -> 300（回退）
   *   【降速 200 是错的】17:06 那趟降到 200 后：
   *     进岛 18475 → 兜底出岛 19876，1401 mm 只转了 128.9°（0.092°/mm），
   *     八边形会话白占 1.4 米，把格 8、格 9 的位置整段吞掉。
   *   而 300 时是 983 mm / +352°（0.36°/mm，真在绕）。速度越低，PD 越容易
   *   "跟着当前这条线直着走"，反而不切线进岛。所以退回 300。
   *
   * exit_fall 2200 -> 1400：保留。实测绕一圈约 983 mm，1400 留 40% 余量。
   *
   * 真正让 17:00 那趟死掉的是"环岛内找线超时 1200 ms 太短"，
   * 已在全局新增 ISL_RECOVER_MS(3000) 专门给环岛会话用，见 RECOVER_MS 处注释。 */
  { "OCT7",    ISLK_LOOP,   9U,   30,   340,  1400,   170,   300,  +1,   60,   80,  560, 1800 },  /*  7 八边形：速度退回 300、兜底 1400 */
  /* 【HEX8 的 fork_n 9 -> 5（最后一个六边形进不去）】
   * 11/19 16:18 那趟跑到格 8，被跳过：
   *   PLAN,8/9,HEX,HEXOVER
   *   ISL,HEXOVER,1202,HEX8,3,0070
   * 同段位图（格 8 区间 21400~22900）：
   *   21701 n=9 01FF
   *   21729 n=5 003E    <- 典型六边形分叉
   *   21874 n=5 03E0    <- 典型分叉
   * 分叉实际只有 5~6 路，而 fork_n 被抬到 9，永远匹配不上。
   * 对照 11/18 16:00 那趟（当时 fork_n=5）：
   *   ISL,FIN,HEX8,5,03E0,19306   <- n=5 就进去了
   * 所以把 fork_n 还原成 5。
   * 风险：中途 n=5 的宽白可能被误判进岛；但有 skip_mm(1200) 兜底，
   * 误进后会被 ISL,HEXOVER 推过去，代价可控。 */
  { "HEX8",    ISLK_HEXT,   5U,   30,   340,  2000,   170,   300,  +1,  120,    0,    0, 1200 },  /*  8 六边形：fork_n 9 -> 5 */
  { "SQ9",     ISLK_CROSS, 11U,   20,     0,     0,    60,   150,   0,    0,    0,    0,    0 }   /*  9 方形 */
};

static const IslandCfg *Island_Cfg(void)
{
  return &isl_cfg[plan_idx % ISL_PLAN_N];
}

/* 当前计划格是不是"方形" */
static uint8_t Island_IsSquare(void)
{
  return (uint8_t)(isl_plan[plan_idx % ISL_PLAN_N] == PLAN_SQ);
}

static void Island_Advance(uint32_t now, const char *why)
{
  plan_idx = (uint8_t)((plan_idx + 1U) % ISL_PLAN_N);
  isl_t_phase = now;
  isl_cfm     = 0U;
  /* 【必须清】方形计数属于"当前格"，不能带到下一格。
   * 某格只穿了一次就被跳过（HEXOVER / 超时 / RESYNC）时，
   * 残留的计数会让下一个方形格只穿一次就 SQOK —— 连跳两格，日志里看不出来。 */
  sq_events   = 0U;
  sq_d0       = 0;
  isl_force_used = 0U;        /* 换格：里程强制进岛重新允许一次 */
  printf("PLAN,%d/%d,%s,%s\r\n", (int)plan_idx + 1, (int)ISL_PLAN_N,
         Island_IsSquare() ? "SQ" : "HEX", why);

  /* ================= 计划表回绕 = 换圈：做和门完全一样的重置 =================
   * 这是"第二圈 = 第一圈"的另一半保证。
   *
   * 之前只有门会重置（Lap_Gate 里 Lap_Reset + lap_start_mm）。
   * 门一旦漏判，计划表仍然会按 %9 回绕到第 1 格，但 lap_start_mm 还是 0，
   * 于是第二圈进第 1 格时 dist - lap_start_mm = 25259，
   * 远超大套小的窗口 (2500~4200) -> BS,MISS，整格被跳过。
   * 17:16 / 17:31 那两趟第一圈就是这个死法。
   *
   * 真里程要等 Island_Step 里拿到 dist 才能填，所以先挂标志，
   * 由 Island_Step 的"换格检测"那一处消费。 */
  if (plan_idx == 0U) lap_wrap_pending = 1U;
}

static void Island_Reset(void)
{
  isl_state    = ISL_NONE;
  isl_cfm      = 0U;
  plan_idx     = 0U;
  sq_events    = 0U;
  isl_t_cfm    = 0U;
  isl_t_state  = 0U;
  isl_t_phase  = HAL_GetTick();   /* 计划格超时从发车开始计时 */
  isl_d0       = 0;
  isl_yaw0     = 0.0f;
  isl_err_off  = 0.0f;
  isl_block_dist = 0;
  isl_ret_dist = 0;
  isl_out_arm  = 0U;
  isl_d_t2     = 0;
  isl_d_fin    = 0;
  isl_d_phase  = 0;
  isl_plan_last = 0xFFU;      /* 强制第一帧 Island_Step 重新记录格起点里程 */
  isl_force_used = 0U;        /* 里程强制进岛：发车时允许一次 */
  /* 发车/终点门 + 圈数：发车时全部清零，本圈起点里程 = 0 */
  bar_cand     = 0U;
  bar_wide     = 0U;
  bar_ok       = 0U;
  bar_mm       = 0;
  bar_hit      = 0U;
  bar_hit_mm   = 0;
  pair_mm      = 0;
  last_bar_mm  = -1000;
  lap_start_mm = 0;
  lap_cnt      = 0U;
  stop_mm      = 0;
  lap_wrap_pending = 0U;      /* 发车时清掉回绕标志，避免上一趟残留 */
  /* 大套小状态整体清零：一个 memset 覆盖所有字段，
   * 以后再加字段也不可能漏（之前就是漏了 err_off 导致"第一次对、之后全错"）。 */
  memset(&bs, 0, sizeof(bs));

  printf("PLAN,1/%d,%s,START\r\n", (int)ISL_PLAN_N,
         Island_IsSquare() ? "SQ" : "HEX");

  /* ==================== 配置指纹 ====================
   * 每趟发车打一行关键参数，用来确认"烧进去的到底是哪一版"。
   *
   * 起因：已经三次因为烧了旧 hex 而误分析现场数据
   *   （11/15 一次、11/19 两次：日志里 ISL,FIN_M,OCT7,600 明明是上一版的 bug，
   *    但源码已经是 force_mm=0，白分析一整轮）。
   * 以后只要看这一行就知道固件对不对：
   *   CFG,OCT7,<fork_n>,<fork_mm>,<force_mm>,<skip_mm>,<dir>
   *   CFG,HEX8,<fork_n>,<fork_mm>,<force_mm>,<skip_mm>,<dir>
   * 这一版应该是 CFG,OCT7,9,30,560,1800,1 和 CFG,HEX8,5,30,0,1200,1。
   * （dir 是八边形治好 FABORT 的关键，必须能从日志核对） */
  printf("CFG,OCT7,%u,%u,%u,%u,%d\r\n",
         (unsigned)isl_cfg[6].fork_n, (unsigned)isl_cfg[6].fork_mm,
         (unsigned)isl_cfg[6].force_mm, (unsigned)isl_cfg[6].skip_mm,
         (int)isl_cfg[6].dir);
  printf("CFG,HEX8,%u,%u,%u,%u,%d\r\n",
         (unsigned)isl_cfg[7].fork_n, (unsigned)isl_cfg[7].fork_mm,
         (unsigned)isl_cfg[7].force_mm, (unsigned)isl_cfg[7].skip_mm,
         (int)isl_cfg[7].dir);
  printf("CFG,V,%d,%d,%d\r\n", (int)FOLLOW_V_FAST,
         (int)FOLLOW_V_SLOW, (int)ISL_FAKE_DEG);
  /* 门判据指纹：确认"第二根竖条放宽到 n>=10"这一版有没有真的烧进去 */
  printf("CFG,BAR,%u,%u,%u\r\n", (unsigned)BAR_FULL_N,
         (unsigned)BAR_FULL_N2, (unsigned)BAR_LAP_TOL_MM);
}

/* ==================== 换圈复位 ====================
 * 过门时调用，把"按格计"的状态全部清成上电状态，保证"第二圈 = 第一圈"。
 *
 * 教训（后续版本的提交里踩过）：bs.d0 / isl_d_phase 这类"本格起点里程"
 * 如果不在换圈时重设，第二圈进第 1 格时 dist - 起点 会是一个巨大的值，
 * 大套小的位置窗口(2500~4200)立刻判超界，整格被 BS,MISS 跳过。
 *
 * 不动这三个跨圈的量：lap_start_mm / lap_cnt / stop_mm。 */
static void Lap_Reset(int32_t dist)
{
  memset(&bs, 0, sizeof(bs));       /* 大套小：state / d0 / j_dist / yaw* 全清 */

  isl_state      = ISL_NONE;
  isl_cfm        = 0U;
  isl_out_arm    = 0U;
  isl_d_t2       = 0;
  isl_d_fin      = 0;
  isl_d0         = 0;
  isl_err_off    = 0.0f;
  isl_block_dist = 0;
  isl_ret_dist   = 0;
  sq_events      = 0U;
  isl_force_used = 0U;              /* 换圈：里程强制进岛重新允许一次 */
  rsy_cfm        = 0U;
  bar_cand       = 0U;
  bar_ok         = 0U;
  bar_wide       = 0U;
  bar_hit        = 0U;
  pair_mm        = 0;

  /* 计划表回到第 1 格；isl_d_phase 重设成"这一圈格 1 的起点"，
   * 于是大套小的位置窗口 (dist - lap_start_mm) 每圈都一样。 */
  plan_idx      = 0U;
  isl_d_phase   = dist;
  isl_plan_last = 0xFFU;            /* 让 Island_Step 下一帧重新记录格起点 */

  /* 【必须重设】计划格超时从"这一格开始"计时。
   * 不重设的话，第 10 格（门）的 isl_t_phase 会一直留到第二圈，
   * 第二圈一开始 (now - isl_t_phase) 就可能已经超过 ISL_PHASE_MS(30 s)，
   * 于是第 1 格还没跑就被 SKIP 掉。 */
  isl_t_phase   = HAL_GetTick();
  isl_t_state   = HAL_GetTick();
  isl_t_cfm     = 0U;

  printf("PLAN,1/%d,%s,LAPRESET\r\n", (int)ISL_PLAN_N,
         Island_IsSquare() ? "SQ" : "HEX");
}

/* ==================== 发车/终点门判定 ====================
 * 每 10 ms 调一次（在 Island_Step 之后）。
 * 事件来源：Follow_Line_Run 里的竖条检测块，确认后置 bar_hit=1 并写 bar_hit_mm。
 *
 * 判据：里程窗口 + 成对。单独一根（方形 T 口）不认。 */
static void Lap_Gate(int32_t dist)
{
  int32_t d_cell = dist - lap_start_mm;    /* 本圈已走里程 */

  /* ---- ① 去重：太近的两根当同一根 ---- */
  if (bar_hit) {
    int32_t since = bar_hit_mm - last_bar_mm;

    if (since < (int32_t)BAR_DUP_MM) {
      printf("BAR,DUP,%ld,%ld\r\n", (long)bar_hit_mm, (long)since);
      bar_hit = 0U;
      return;
    }

    /* ---- ② 里程窗口：本圈里程离 FOLLOW_LAP_MM 太远就不当门 ----
     * 这是区分"发车区竖条"和"途中方形 T 口"的第一道闸。
     * 窗口外仍然打印，方便调参时看真实位置。 */
    if (d_cell < (int32_t)FOLLOW_LAP_MM - (int32_t)BAR_LAP_TOL_MM
        || d_cell > (int32_t)FOLLOW_LAP_MM + (int32_t)BAR_LAP_TOL_MM) {
      printf("BAR,OUT,%ld,%ld\r\n", (long)bar_hit_mm, (long)d_cell);
      bar_hit = 0U;
      return;
    }

    last_bar_mm = bar_hit_mm;
    bar_hit     = 0U;

    /* ================= 分层：圈数用"成对"，停车用"第一根" =================
     * 单根整排白在光电管上和方形 T 口完全同形，分不开 —— 这就是门反复漏判的根源。
     * 发车区唯一独有的物理特征是**两根竖条相距约 415 mm**：
     *   17:31 那趟日志里 SQ,CLOSE 反复打出 409 / 418 / 419，就是这两根。
     * 而方形格要求两次 T 口间隔 >= SQ_MIN_GAP_MM(450)，所以 415 那根
     * 不会被方形格吃掉 —— 两套判据互相保护。
     *
     *   中间圈（决定 lap_cnt / 状态重置）：必须成对，间距 300~550 mm
     *       单根永远不认圈 -> 不会把方形 T 口当门，也不会被某格反吃掉
     *   最后一圈（决定停车点）：认到第一根就定 stop_mm
     *       成对要等第二根才确认，那时车已经在第二根上，来不及停进两根之间
     * 两个需求分开满足，不用再互相让步。 */
    printf("GATE,BAR,%ld,%ld,%ld\r\n", (long)bar_hit_mm, (long)d_cell,
           (long)(pair_mm ? (bar_hit_mm - pair_mm) : 0));

    /* ---- 最后一圈：第一根就定点停车 ---- */
    if (lap_cnt + 1U >= (uint8_t)FOLLOW_LAPS) {
      lap_cnt++;
      stop_mm = bar_hit_mm + (int32_t)STOP_FROM_FIRST_MM;
      printf("GATE,STOP,%d,%ld,%ld\r\n", (int)lap_cnt,
             (long)bar_hit_mm, (long)stop_mm);
      pair_mm = 0;
      return;
    }

    /* ---- 中间圈：必须凑成一对才认圈 ---- */
    if (pair_mm == 0) {
      pair_mm = bar_hit_mm;
      printf("GATE,FIRST,%ld,%ld\r\n", (long)bar_hit_mm, (long)d_cell);
      return;
    }

    {
      int32_t gap = bar_hit_mm - pair_mm;

      if (gap < (int32_t)PAIR_MIN_MM || gap > (int32_t)PAIR_MAX_MM) {
        /* 间距不对：这根当新的第一根，重新等 */
        printf("GATE,NOPAIR,%ld,%ld,%ld\r\n",
               (long)pair_mm, (long)bar_hit_mm, (long)gap);
        pair_mm = bar_hit_mm;
        return;
      }

      /* ---- 成对确认 = 真的过门 ---- */
      printf("GATE,PAIR,%ld,%ld,%ld\r\n",
             (long)pair_mm, (long)bar_hit_mm, (long)gap);
      pair_mm = 0;
      lap_cnt++;
      lap_start_mm = bar_hit_mm;
      Lap_Reset(dist);
      printf("LAP,%d,%ld,%ld\r\n", (int)lap_cnt,
             (long)bar_hit_mm, (long)(dist - bar_hit_mm));
    }
  }
}

/* 是否正处在环岛内（供速度调度使用）。
 * 大套小的 BS_* 状态也必须算"岛内"，否则它会掉进普通循迹的速度调度，
 * 被"宽区减速"（WIDE_NEAR_N=6，而岛内 n 常年 6~8）反复压到 150，
 * 表现为车在岛里一顿一顿、速度在 150 和 290 之间来回跳。 */
static uint8_t Island_InSession(void)
{
  if (isl_state == ISL_SESSION) return 1U;

  if (bs.state == BS_BIG || bs.state == BS_MID
      || bs.state == BS_SMALL || bs.state == BS_TAIL) return 1U;

  return 0U;
}

/* 是否正处在"必须走完"的环岛流程里。
 * 用于圈数判定：里程到了也不能停，否则会把环岛截断在半个。
 * ISL_RET（冷却）不算——那时环岛已经处理完了。 */
static uint8_t Island_Busy(void)
{
  return (uint8_t)(isl_state == ISL_CROSS
                || isl_state == ISL_SESSION
                || isl_state == ISL_HEXOUT);
}

/* 判据：这一帧像不像 T 口
 *   方框拐角   0FF8(9) / 0FFC(10) / 03FF(10) —— 最多 10 路
 *   真 T 口    0EFF(11) / 0FFF(12)          —— 11 路以上
 * 只用路数，不用"两端都有白"（拐角也会两端白）。 */
static uint8_t Island_Pattern(uint8_t n)
{
  /* T 口阈值必须用固定的 ISL_T_N(11)，不能跟着当前环岛的 fork_n 走。
   * 之前误写成 Island_Cfg()->fork_n，结果计划表里绕行格 fork_n=6，
   * 于是 n=6~7 的普通图案就被当成 T 口，开局连续误触发多次 PLAN,RESYNC。 */
  return (uint8_t)(n >= ISL_T_N);
}

/* 分叉判据：只看"白路数"。
 *
 * 曾经加过"最左与最右同时有白"的形状条件，结果 12.txt 里 4 次经过六边形
 * 一次都没触发 —— 入口处位图实际是这样跳的（100 ms 采样）：
 *   079E(左) -> 02FC(右) -> 00F8(右) -> 03E0(右) -> 07B8(右) -> 0E9E(左) -> 0E07(两端)
 * "两端同时白"只占一帧，配合"持续 60 mm"就永远满足不了。
 * 而 10.txt 里只按 n>=5 的那版是 4/4 全部识别成功。
 * 所以这里只保留路数判据，形状条件去掉。 */
static uint8_t Island_Fork(uint8_t n)
{
  /* 与队友那版（3b580d1，能完整跑完两圈）保持一致：只看白路数。
   * 之前加过"必须有两个白段（有断口）"，实测新赛道六边形入口的位图
   * 经常是一整段，加上这条之后一次都进不去，已去掉。 */
  /* 注意：这里不能加"n 上限"。
   * 曾经为了排除大套小入口处的十字路口，加过 if (n > 9U) return 0;
   * 结果把六边形的正常入口也一起拒掉了 —— 37.txt 里第 8 格的入口
   * 读成 0FFE(n=11) / 0FFF(n=12)，被上限拒收后紧接着被 PLAN,RESYNC
   * 当成"计划表错位"直接跳过。
   * 大套小那个误判现在由 Isl_BigSmall 里的里程窗口(BS_ENTRY_MIN_MM/MAX_MM)负责，
   * 不需要在这里做形状过滤。
   * 本函数只被非 BSS 的环岛用到（大套小走自己的 n>=fork_n 判断）。 */
  return (uint8_t)(n >= Island_Cfg()->fork_n);
}

/* ==================== 大六边形套小六边形（专用流程） ====================
 * 现场照片：大六边形与小六边形在一个顶点处交叉（X 形）。
 * 正确走法 = 绕大环 → 过交叉点 → 绕小环 → 回主赛道，累计约 720 度。
 *
 * 通用的"绕一圈 + 锁直行出岛"在这里走不通：第一圈结束时车正好停在
 * 两个环的交叉点上，锁直行会直接切到另一条边上，等于放弃第二个环。
 * 而交叉点处 12 路的平均偏差趋近 0，PD 也会想直行，所以必须加转向偏置顶住。
 *
 * 这个函数只服务计划表第 1 格，其余六边形仍走通用流程，互不影响。
 */

/* 按现场确认的走法：入口右转进岛 → 转 240°（4 条边）→ 直行进小环
 *                    → 转 360°（6 条边）→ 直行 → 走 2 条边出岛
 * 合计右转约 600°。方向为右转，所以偏置取负。 */
#define BS_TURN1_DEG   235.0f   /* 第一段：大环 4 条边（已不用于切段，仅作参考） */
#define BS_SMALL_DEG   335.0f   /* 小环单独转这么多度（6 条边约 360）
                                 * 注意方向：大环顺时针（右转），小环逆时针（左转），
                                 * 所以两个角度是相减关系，不能累加到 590。 */

/* ---- 双判据：切段不只看角度，还要"最近看到过交叉点图案" ----
 * 交叉点是赛道上唯一可靠的路标：直边上 n=3，两线交汇时 n 跳到 6~8。
 * 角度只当粗筛（防止图案误判），真正的切换点由图案决定，
 * 这样就不再依赖"每次转弯半径都一样"这个不成立的假设。 */
/* 开源方案（电磁组 / 全向组）的共同做法：只在进环那一下强制打角，
 * 进环后立刻切回正常循迹。全向组代码里明确写着"陀螺仪 >60 度入环补线结束"。
 * 我们之前整段 240 度一直打角，车被拧着走，才会切内角、画小圆。 */
/* 光电管等价于摄像头的补线：不改转向，改车认为的居中位置。
 * 偏置只有 95，被 PD(150/度) 抵消后车只偏 0.63 个单位就停住，
 * 所以在岔口它仍然顺着直的那条走。改成偏移目标点后，
 * PD 会主动把车开到新目标上并稳住，才能真正切到另一条分支。 */
#define BS_ERR_OFF         2.5f  /* 目标偏移量（单位同 err，满量程 7） */
#define BS_ENTRY_DEG      130.0f /* 每段开头这么多度内强制打角。
                                  * 现场确认车不切内角（贴线走），说明循迹没问题，
                                  * 问题在岔口选错了分支：PD 倾向走直的那条，
                                  * 只有偏置能把它掰进环。70 度在岔口处不够，
                                  * 加到 130 度（约 400 mm），确保整车进入环内再撒手。 */
#define BS_FAKE_MM        250    /* 假进岛自检：进岛后走够这么远 */
#define BS_FAKE_DEG        35.0f /* 偏航还不到这个值 -> 判为假进岛，退回等入口 */
#define BS_JUNC_MM       150    /* 最近 150 mm 内出现过 n>=fork_n 就算在交叉点 */
#define BS_BIG_MIN_DEG 190.0f   /* 大环：至少转这么多，才允许按图案切段 */
#define BS_BIG_MAX_DEG 300.0f   /* 大环：到这么多度还没遇到交叉点就强制切段 */
#define BS_SML_MIN_DEG 290.0f   /* 小环：至少转这么多，才允许按图案切段 */
#define BS_SML_MAX_DEG 430.0f   /* 小环：到这么多度强制切段 */

#define BS_BIAS        (-95.0f) /* 右转偏置（turn<0 = 右转），顶住交叉点别让 PD 直行 */
#define BS_MID_MM        600    /* 大环→小环之间的直行过渡距离 */
#define BS_TAIL_MM       600    /* 小环出来后的强制直行段（约 2 条边 x 300） */
#define BS_EXT_DEG        40.0f /* 出岛前右转角度（现场确认：右转 30 度再直行） */
#define BS_OUT_MM        300    /* 最后锁直行距离：转完 40 度后 600 太长，会顶到环外侧 */
#define BS_MAX_MM       4600    /* 兜底：整段走满这么远强制结束 */
#define BS_TRACE_MM      400    /* 每走这么远打一行进度 */

/* ==================== 大套小入口的位置闸门 ====================
 * 实测赛道：发车线 0~70 mm、波浪弯约 800 mm 起、大套小入口约 3430 mm。
 * 入口判据（n >= fork_n）只是"够宽"，它分不清"赛道变宽/波浪弯/普通弯道"和"真分叉"：
 *   发车线 00F0(4 路白)  -> fork_n 放到 4 时 64 mm 就进岛（实测，整套流程在发车区跑飞）
 *   波浪弯 边缘位图 4~5 路 -> 同样会骗过判据
 * 所以先用里程把"不可能有入口"的区间整段排除。窗口外连候选都不许累积，
 * 免得候选在窗口前形成、窗口一开就立刻触发（那样位置仍然是错的）。
 * 这两个数按赛道实测值改：MIN 取波浪弯之后、真入口之前；MAX 取真入口之后一点。 */
#define BS_ENTRY_MIN_MM   2500  /* 只在这之后才认大套小入口 */
#define BS_ENTRY_MAX_MM   4200  /* 过了这个里程还没进，放弃这一格，别影响后面的环岛 */

static uint8_t Isl_BigSmall(uint8_t n, uint32_t now, int32_t dist)
{
  const IslandCfg *c = Island_Cfg();
  float yaw = fabsf(gyro_angle - bs.yaw0);

  /* ---- 全程轨迹打印（每 100 mm 一行）----
   * 只要进过一次岛，之后不管在哪个段都打，直到 BS_OUT 结束。
   * 格式：BS,TRK,<偏航×10>,<进岛后里程>,<路数>,<状态>
   * 用它一次跑完就能画出"偏航-里程"曲线，从而把 240/600 度
   * 到底对应多少毫米算清楚，不用再盲猜。 */
  if (bs.state != BS_IDLE && bs.state != BS_OUT) {
    if ((dist - bs.d0_track) >= 100) {
      bs.d0_track = dist;
      printf("BS,TRK,%d,%ld,%d,%d\r\n",
             (int)(yaw * 10.0f), (long)(dist - bs.d0), (int)n, (int)bs.state);
    }
  }

  switch (bs.state) {
  case BS_IDLE:
    /* 等入口分叉图案。
     * 23.txt 教训：车在入口处 n 在 5~8 之间跳，只要有一次掉到门槛以下
     * 就把已经累积的距离清零，结果 1.3 m 处始终认不出来，
     * 车直行撞进环内侧边线堵转。
     * 所以中断条件不要用"一帧掉下去就清零"，而是给一个宽限：
     * 只有连续 300 mm 都没再看到图案，才认为这次候选作废。 */

    /* ---- 位置闸门 ----
     * 窗口外直接清候选并返回：不判、也不累积。
     * 不清候选的话，波浪弯里形成的候选会一直留着，
     * 等里程一进窗口就立刻 BS,IN —— 位置还是错的，只是错得晚一点。
     *
     * 【必须用"本圈里程"而不是绝对里程】
     * 原来写的是 dist，第一圈没问题（入口在 2500~4200），
     * 但第二圈时 dist 已经两万多，一进第 1 格就 dist > 4200 -> BS,MISS，
     * 整个大套小被跳过，后面每个环岛都错位。
     * 改成 dist - lap_start_mm，每圈的窗口完全一样。 */
    {
      int32_t d_lap = dist - lap_start_mm;

      if (d_lap > (int32_t)BS_ENTRY_MAX_MM) {
        /* 窗口过了还没进：放弃这一格。
         * 不能只是 return —— plan_idx 会永久卡在 0（BIGHEX），
         * 于是之后每一个环岛都走 Isl_BigSmall 的入口判据，全部不被处理。 */
        printf("BS,MISS,%ld,%d,%04X\r\n", (long)d_lap, (int)n, (unsigned)line_raw);
        isl_cfm = 0U;
        Island_Advance(now, "BSMISS");
        return 0U;
      }
      if (d_lap < (int32_t)BS_ENTRY_MIN_MM) {
        isl_cfm = 0U;                   /* 窗口前：允许清候选，但不许形成新的 */
        return 0U;
      }
    }

    if (n >= c->fork_n) {
      if (!isl_cfm) {
        isl_cfm = 1U; isl_t_cfm = now; isl_d0 = dist;
        /* 诊断：第一次看到入口图案时打一行，用来确认"候选到底有没有形成"。
         * 如果车跑过入口却一行 BS,CAND 都没有，说明门槛太高；
         * 有 CAND 却没有 BS,IN，说明距离/时间条件没凑够。 */
        printf("BS,CAND,%d,%04X,%ld\r\n", (int)n, (unsigned)line_raw, (long)dist);
      }
      else {
        isl_t_cfm = now;                                  /* 持续刷新"最后看到"时刻 */
        if ((dist - isl_d0) >= (int32_t)c->fork_mm) {      /* 累计走过足够距离 */
          isl_cfm   = 0U;
          bs.state  = BS_BIG;
          bs.yaw0   = gyro_angle;
          bs.d0     = dist;
          bs.d0_track = dist;
          bs.turn   = BS_BIAS;
          isl_d_fin = dist;
          printf("BS,IN,%d,%04X,%ld\r\n", (int)n, (unsigned)line_raw, (long)dist);
          Steer_Reset();
        }
      }
    } else if (isl_cfm && (uint32_t)(now - isl_t_cfm) > 250U) {
      /* 超过 250 ms 没再看到图案，才算候选失败 */
      isl_cfm = 0U;
    }
    return 0U;

  case BS_BIG:
    /* 第一段：走大环。
     * 只在开头 BS_ENTRY_DEG 度内强制打角（负责拐进环），之后交给正常 PD。
     * 依据两份开源方案：
     *   电磁组：固定中线差值强制打角一段时间，进环后切回正常循迹
     *   全向组：陀螺仪 >60 度入环补线结束
     * 整段常开会把车拧成半径约 172 mm 的小圆（实测 3.0 mm/度，
     * 而边长 300 的六边形应是 5.0 mm/度）。 */
    if (yaw < BS_ENTRY_DEG) { bs.turn = 0.0f; bs.err_off =  BS_ERR_OFF; }
    else                    { bs.turn = 0.0f; bs.err_off =  0.0f; }
    if (n >= c->fork_n) bs.j_dist = dist;      /* 记下最近一次看到交叉点的位置 */

    if ((dist - bs.d0) >= (int32_t)(bs.tr + BS_TRACE_MM)) {
      bs.tr += BS_TRACE_MM;
      printf("BS,TR,BIG,%d,%ld,%d\r\n", (int)(yaw * 10.0f), (long)(dist - bs.d0), (int)n);
    }
    /* 双判据：转够最小角度 + 最近见过交叉点；或到了上限强制切段 */
    {
      uint8_t junc = (uint8_t)(bs.j_dist != 0
                     && (dist - bs.j_dist) <= (int32_t)BS_JUNC_MM);
      if ((yaw >= BS_BIG_MIN_DEG && junc) || yaw >= BS_BIG_MAX_DEG) {
        printf("BS,BIGOK,%d,%ld,%s\r\n", (int)(yaw * 10.0f), (long)(dist - bs.d0),
               (yaw >= BS_BIG_MAX_DEG) ? "F" : "J");
        bs.state = BS_MID;                     /* 到交叉点 → 直行过渡到小环 */
        bs.d0    = dist;
        bs.turn  = 0.0f;
        bs.tr    = 0;
        bs.j_dist = 0;
        Steer_Reset();
        return 0U;
      }
    }
    if ((dist - bs.d0) > (int32_t)BS_MAX_MM) {
      printf("BS,SOVR,%d,%ld\r\n", (int)(yaw * 10.0f), (long)(dist - bs.d0));
      bs.state = BS_OUT;
      bs.d0    = dist;
      bs.t0    = now;
      bs.turn  = 0.0f;
      bs.tr    = 0;
    }
    return 0U;

  case BS_MID:
    /* 第二段：直行过渡（不偏置，让 PD 跟着直线进小环） */
    bs.turn = 0.0f;
    if ((dist - bs.d0) >= (int32_t)BS_MID_MM) {
      printf("BS,MIDOK,%ld,%d\r\n", (long)(dist - bs.d0), (int)n);
      bs.state = BS_SMALL;
      bs.yaw2  = gyro_angle;    /* 小环从这里开始算，方向和偏置都要反过来 */
      bs.turn  = -BS_BIAS;      /* 小环绕的是逆时针 -> 左转 -> 偏置取正 */
      bs.tr    = 0;
    }
    return 0U;

  case BS_SMALL:
    /* 第三段：小环 6 条边。
     * 大环顺时针（右转，偏置负），小环逆时针（左转，偏置要取正），
     * 所以这里用 -BS_BIAS，并且出段条件是"相对 bs.yaw2 转够了 355 度"，
     * 不能用"相对进岛航向累计 590 度"——那样等于让车接着同方向绕。 */
    if (fabsf(gyro_angle - bs.yaw2) < BS_ENTRY_DEG) { bs.turn = 0.0f; bs.err_off = -BS_ERR_OFF; }
    else                                             { bs.turn = 0.0f; bs.err_off = 0.0f; }
    if (n >= c->fork_n) bs.j_dist = dist;
    if ((dist - bs.d0) >= (int32_t)(bs.tr + BS_TRACE_MM)) {
      bs.tr += BS_TRACE_MM;
      printf("BS,TR,SMALL,%d,%ld,%d\r\n",
             (int)(fabsf(gyro_angle - bs.yaw2) * 10.0f), (long)(dist - bs.d0), (int)n);
    }
    {
      float sy = fabsf(gyro_angle - bs.yaw2);       /* 小环自身转过的角度 */
      uint8_t junc = (uint8_t)(bs.j_dist != 0
                     && (dist - bs.j_dist) <= (int32_t)BS_JUNC_MM);
      if ((sy >= BS_SML_MIN_DEG && junc) || sy >= BS_SML_MAX_DEG) {
        printf("BS,ALLOK,%d,%ld,%s\r\n", (int)(sy * 10.0f), (long)(dist - bs.d0),
               (sy >= BS_SML_MAX_DEG) ? "F" : "J");
        bs.state  = BS_TAIL;
        bs.d0     = dist;
        bs.turn   = 0.0f;
        bs.tr     = 0;
        bs.j_dist = 0;
        Steer_Reset();
        return 0U;
      }
    }
    if ((dist - bs.d0) > (int32_t)BS_MAX_MM) {
      printf("BS,SOVR,%d,%ld\r\n",
             (int)(fabsf(gyro_angle - bs.yaw2) * 10.0f), (long)(dist - bs.d0));
      bs.state = BS_OUT;
      bs.d0    = dist;
      bs.t0    = now;
      bs.turn  = 0.0f;
      bs.tr    = 0;
    }
    return 0U;

  case BS_TAIL:
    /* 第四段：直行走最后两条边 */
    bs.turn = 0.0f;
    if ((dist - bs.d0) >= (int32_t)BS_TAIL_MM) {
      printf("BS,TAILOK,%ld,%d\r\n", (long)(dist - bs.d0), (int)n);
      bs.state = BS_EXT;
      bs.yaw1  = gyro_angle;
      bs.turn  = BS_BIAS;
      bs.tr    = 0;
    }
    return 0U;

  case BS_EXT:
    /* 第五段：右转 30 度。现场确认：不转这 30 度，车会顺着环边又拐回岛里 */
    bs.turn = BS_BIAS;
    if (fabsf(gyro_angle - bs.yaw1) >= BS_EXT_DEG) {
      printf("BS,EXTOK,%d,%ld,%d\r\n",
             (int)(fabsf(gyro_angle - bs.yaw1) * 10.0f), (long)(dist - bs.d0), (int)n);
      bs.state = BS_OUT;
      bs.d0    = dist;
      bs.t0    = now;
      bs.turn  = 0.0f;
      Steer_Reset();
    }
    return 0U;

  case BS_OUT:
    /* 出岛收尾段。两件事：
     * ① 距离上限 BS_OUT_MM 到点就交还 PD；
     * ② 本段期间交给 PD 循迹（return 0U），不再锁直行。
     *
     * 【为什么不再锁直行】三趟日志（11:32 / 11:40 / 11:52）死法完全一样：
     *   ~7125 车在大套小出口走得很正（00E0 居中，err -0.3）
     *   ~7300 赛道急左弯，线从 0FE0 一路滑到 0F00 / 0780 / 03C0（最左边缘）
     *   ~7387 整排全蓝 n=0，开始全盲
     *   7387~7609 BS_OUT 锁直行 222 mm @150 —— 赛道在拐弯，车被强行推直
     *   7609~7798 判成"弯道丢线"，满舵 220 @160 原地打转，vL -48 / vR 488
     *   7815 才撞上 03FE，7822 撞上 07FF —— 车是"横着"撞到入口的
     * 所以后面 `entry_deg` 怎么调都没用：进岛瞬间的航向本来就是随机的。
     * 出岛那几十毫米里线还在（只要不硬掰），让 PD 跟着拐才不会把自己推下赛道。
     *
     * err_off 在这里必须清零：BS_SMALL 阶段留下的 ±BS_ERR_OFF(2.5) 会一直
     * 挂到 BS_OUT 退出才清，锁直行时看不出来（不走 PD 分支），
     * 一旦改走 PD 就会变成整段 2.5 个单位的固定偏置。 */
    bs.err_off = 0.0f;
    /* 超时用本段自己的起点时间 bs.t0，不能用 isl_t_state：
     * 那是会话开始时刻，早就超过 3000 ms，一进来就立刻退出，
     * 表现为 BS,OUT,3（只走 3 mm）就出岛了。 */
    if ((dist - bs.d0) >= (int32_t)BS_OUT_MM
        || (uint32_t)(now - bs.t0) > 3000U) {
      printf("BS,OUT,%ld,%d\r\n", (long)(dist - bs.d0), (int)n);
      bs.state       = BS_IDLE;
      bs.turn        = 0.0f;
      bs.err_off     = 0.0f;
      isl_state      = ISL_RET;
      isl_t_state    = now;
      isl_ret_dist   = dist;
      /* 331.txt：嵌套环岛一结束，第 2 格立刻在出口交叉点上"进岛"
       * （位图 0F80 看起来就是个分叉），随后 ISL,FABORT 或直接跑飞。
       * 这里加一道硬闸门：出门后一段时间内禁止任何环岛开始。
       *
       * 距离从 500 缩到 150：BS_OUT 本身已经锁直行 300 mm 穿过出口岔口了，
       * 再叠 500 mm 等于出岛后 800 mm 内不认环岛。而实测大套小出岛
       * （约 7781）到六边形顶点入口（约 8006，位图 07FF n=11）只有约 225 mm，
       * 正好被这段封锁整段盖掉 —— 六边形永远进不去，车直冲到 8091 丢线撞墙。
       * 150 mm 仍能盖住出口岔口那段（n=6~10 的残影都在出岛后 0~40 mm 内），
       * 又放得开 225 mm 处的真入口。 */
      isl_block_dist = dist + 150;
      sq_events    = 0U;
      Island_Advance(now, "BSDONE");
      Steer_Reset();
      return 0U;
    }
    return 0U;                               /* 交给 PD 循迹（不再锁直行） */

  default:
    bs.state = BS_IDLE;
    return 0U;
  }
}

/* 每 10 ms 一次。
 * 返回 1 = 本帧按"穿过 T 口"处理（直行锁定）。
 * 会话（六边形）期间返回 0，由普通 PD 循迹。 */
static uint8_t Island_Step(uint8_t n, uint32_t now, int32_t dist, float turn_now)
{
  (void)turn_now;

  /* 换格了就记下这一格的起点里程。两套兜底（时间 ISL_PHASE_MS / 里程 skip_mm）
   * 都从这一格里程算起，和 Island_Advance 里的 isl_t_phase 是同一个时刻。 */
  if (isl_plan_last != plan_idx) {
    isl_plan_last = plan_idx;
    isl_d_phase   = dist;

    /* ---- 计划表回绕到第 1 格 = 换圈 ----
     * 门漏判时这是唯一的兜底：把"本圈起点"和所有按格计的状态一起重置，
     * 保证第二圈和第一圈从同一个状态出发（用户的原话：走一圈应该重置得一样）。
     * 门正常认出时不会走到这里 —— Lap_Gate 已经把 plan_idx 拨回 0 并重置过了，
     * 所以不会重复计数 lap_cnt。 */
    if (lap_wrap_pending) {
      lap_wrap_pending = 0U;
      lap_cnt++;
      /* 【本圈起点用"里程推算"，不要用当前里程】
       * 计划表回绕发生在"第 9 格做完"的时候，它比真实发车门晚了多少
       * 取决于路径长短，完全没有保证。11/19 16:18 那趟：
       *   真实发车门  20767（第二根竖条）
       *   回绕触发点  26281（第 9 格在错位置才完成）
       *   用 dist 当起点 -> 偏差 5514 mm -> 第二圈大套小窗口跑到错位置
       *   -> 假大套小罩住第二次断路 -> 断路被当弯道满舵 -> STALL
       * 门漏判时我们唯一能信的是"一圈大约 FOLLOW_LAP_MM"，
       * 用 lap_start += FOLLOW_LAP_MM 推算，误差只剩几百毫米。 */
      lap_start_mm += (int32_t)FOLLOW_LAP_MM;
      Lap_Reset(dist);              /* 清 bs / isl_* / sq_* / bar_*，plan_idx=0 */
      printf("LAP,W,%d,%ld,%ld\r\n", (int)lap_cnt,
             (long)dist, (long)lap_start_mm);
    }
  }

  /* 大套小专用流程：整格交给 Isl_BigSmall，不经过下面的通用状态机 */
  if (Island_Cfg()->kind == ISLK_BSS) {
    return Isl_BigSmall(n, now, dist);
  }

  /* ---- 正在穿 T 口 ---- */
  if (isl_state == ISL_CROSS) {
    uint8_t done = 0U;

    if ((dist - isl_d0) >= ISL_CROSS_MM
        && (uint32_t)(now - isl_t_state) >= ISL_CROSS_MIN_MS
        && n <= 5U) done = 1U;
    if ((uint32_t)(now - isl_t_state) > 1500U) done = 1U;

    if (done) {
      isl_state   = ISL_RET;
      isl_t_state = now;
      printf("ISL,OUT,%ld\r\n", (long)(dist - isl_d0));
      Steer_Reset();
      /* 方形：每个方形需要穿过 2 次 T 口（进 + 出），凑齐才前进一格。
       *
       * 【必须校验两次 T 口的间隔】11/18 15:39 那趟实测四个方形的间隔是
       *   166 / 2015 / 2502 / 2378 mm，正常只有 675~1030。
       * 166  = 同一个口被数了两次（ISL,OUT 后 171 mm 又触发一次）；
       * 2000+ = 已经走到下一个元素了，把远处的 T 口当成了本格的第 2 个。
       * 结果是计划表一路错位：车跑到 20902 mm 才只走到第 7 格（八边形），
       * 根本没回到发车区，单圈里程比正常那趟多了 4000 mm。
       * 所以两次 T 口之间必须落在合理窗口里，超出的当"新的第一个口"。 */
      if (Island_IsSquare()) {
        if (sq_events == 0U) {
          sq_events = 1U;
          sq_d0     = dist;
          printf("SQ,FIRST,%ld,%ld\r\n", (long)dist, (long)(dist - isl_d_phase));
        } else {
          int32_t gap = dist - sq_d0;

          if (gap < (int32_t)SQ_MIN_GAP_MM) {
            /* 太近：同一个口被数了两次，保持"第一个口"不变 */
            printf("SQ,CLOSE,%ld,%ld,%ld\r\n", (long)dist, (long)gap,
                   (long)(dist - isl_d_phase));
          } else if (gap > (int32_t)SQ_MAX_GAP_MM) {
            /* 太远：已经走到下一个元素了，这根当新的第一个口 */
            printf("SQ,FAR,%ld,%ld,%ld\r\n", (long)dist, (long)gap,
                   (long)(dist - isl_d_phase));
            sq_d0 = dist;
          } else {
            printf("SQ,PAIR,%ld,%ld,%ld,%ld\r\n", (long)sq_d0, (long)dist,
                   (long)gap, (long)(dist - isl_d_phase));
            sq_events = 0U;
            sq_d0     = 0;
            isl_d_t2  = dist;            /* 强制进岛的里程基准 = 第二个 T 口 */
            Island_Advance(now, "SQOK");
          }
        }
      }
      return 0U;
    }
    return 1U;
  }

  /* ---- 环岛会话：正常循迹；绕满一圈后直接把控制权交还 PD ----
   * 不走"航向回正"：实测那样做会让车在环内原地打转（左右轮一正一反），
   * 转完车头方向对了但位置还在环里，PD 又把它拉回环线，表现为绕不出去。
   *
   * 出岛判据的"治本"改法：**角度到位就武装，之后任意一帧看到分叉都算**。
   * 原来写成 "yaw>=阈值 && 同一帧看到分叉"，两个问题：
   *   1) 阈值比实际出口角大 1° 就完全不匹配，得再绕一整圈；
   *      而 ISL_OVER_DEG 会在半路强制结束，等于"绕一圈出不来"。
   *   2) 出口那几十毫米里只要 n 有一次掉到 5，那一帧就废了，
   *      下一次机会要再等一整圈。
   * 实测 ISL,SEX 的出口偏航在 330.5~347.2 之间漂 17°，用绝对阈值注定临界。
   * 现在：yaw 一过 ISL_EXIT_DEG 就置 out_arm，之后偏航继续涨的同时
   * 只要有任意一帧匹配到分叉就立刻出岛，不再依赖"恰好那一帧"。 */
  if (isl_state == ISL_SESSION) {
    /* 绕行方向：
     *   dir = 0  老行为，用 |偏航|（不分方向）
     *   dir = +1 顺时针（右转），偏航取正
     *   dir = -1 逆时针（左转），偏航取反 —— 这样"绕对方向"时数值单调增大，
     *            出岛阈值仍然是正的 340，不必给每行写负数。 */
    float   dy  = gyro_angle - isl_yaw0;
    float   yaw = (Island_Cfg()->dir == 0) ? fabsf(dy)
                                           : ((float)Island_Cfg()->dir * dy);
    int32_t run = dist - isl_d_fin;              /* 进岛后已走的里程 */

    /* 切向进岛：只在开头 entry_deg 度内给误差偏移，把车掰进环。
     * 顶点进岛的 entry_deg = 0：车顺着线自然进去，不需要打角。 */
    if (Island_Cfg()->entry_deg != 0U
        && yaw < (float)Island_Cfg()->entry_deg) {
      isl_err_off = (Island_Cfg()->dir > 0) ? ISL_ENTRY_OFF : -ISL_ENTRY_OFF;
    } else {
      isl_err_off = 0.0f;
    }

    /* ---- 假进岛自检 ----
     * 日志实测：真环岛在 850 mm 里转约 340°（真样本 337.2 / 340.8 / 341.3），
     * 而假进岛只转 50~107°（假样本 51.6 / 50.5 / 107.1 / 0.0）。
     * 所以走够 ISL_FAKE_MM(300) 偏航还不到 ISL_FAKE_DEG(40) 就判为假，
     * 立刻退出会话。不做这步的话，会话会罩在一段直线上，
     * Island_Step 返回 0 => T 口处理被禁用 => 下一个真 T 口直接撞上去 STALL。
     * 退出后阶段不动（仍是 HEX），图案判据还能在后面重新认出真环岛。 */
    if (run >= (int32_t)ISL_FAKE_MM && yaw < ISL_FAKE_DEG) {
      printf("ISL,FABORT,%ld,%d\r\n", (long)run, (int)(yaw * 10.0f));
      isl_out_arm  = 0U;
      isl_state    = ISL_RET;
      isl_t_state  = now;
      isl_ret_dist = dist;
      Steer_Reset();
      return 0U;
    }

    if (yaw >= (float)Island_Cfg()->exit_deg) isl_out_arm = 1U;   /* 偏航到阈值就锁定，抗陀螺抖动 */

    /* ---- 纯偏航出岛 ----
     * 不再看 Island_Fork(n)。分叉图案判据实测不稳定（n 在 4~8 之间跳），
     * 漏判一帧就要再等一整圈，是"绕一圈出不来"的根源。
     * 这里只看两个量：累计偏航 >= ISL_EXIT_DEG，且进岛后已走够 ISL_EXIT_MIN_MM。
     * 后者是护栏，防止 yaw 偏大时在环道前半段误出（那时进 ISL_HEXOUT 会撞岛壁）。 */
    if (isl_out_arm && run >= (int32_t)ISL_EXIT_MIN_MM) {
      printf("ISL,SEX,%d,%ld\r\n", (int)(yaw * 10.0f), (long)run);
      isl_out_arm = 0U;  isl_err_off = 0.0f;
      isl_state   = ISL_HEXOUT;
      isl_t_state = now;
      isl_d0      = dist;
      Steer_Reset();
      return 1U;
    }

    /* ---- 里程强制出岛：进岛后走够 ISL_EXIT_MM 无条件出 ----
     * 偏航判据失效（陀螺漂/打滑）时靠它兜底，保证"一定出得去"。
     * 出口流程和正常出岛完全一样（进 ISL_HEXOUT 锁定直行穿过岔口）。 */
    if (isl_d_fin != 0 && Island_Cfg()->exit_fall_mm != 0U
        && run >= (int32_t)Island_Cfg()->exit_fall_mm) {
      printf("ISL,SEX_F,%ld,%d\r\n", (long)run, (int)(yaw * 10.0f));
      isl_out_arm = 0U;  isl_err_off = 0.0f;
      isl_state   = ISL_HEXOUT;
      isl_t_state = now;
      isl_d0      = dist;
      Steer_Reset();
      return 1U;
    }

    if (yaw >= ISL_OVER_DEG || (uint32_t)(now - isl_t_state) > ISL_SES_MAX_MS) {
      printf("ISL,SOVR,%d\r\n", (int)(yaw * 10.0f));
      isl_out_arm  = 0U;  isl_err_off = 0.0f;
      isl_state    = ISL_RET;
      isl_t_state  = now;
      isl_ret_dist = dist;
      sq_events    = 0U;
      Island_Advance(now, "HEXDONE");
      Steer_Reset();
    }
    return 0U;
  }

  /* ---- 进岛：先锁定直行穿过交叉点 ----
   * 交叉点处加权重心被两条分支平均成斜向，PD 会带着车斜穿过去。
   * 这几十毫米内不理会 err，按车头方向直着冲过交叉点，过了再交给 PD。
   *
   * 但"顶点进入型"六边形的顶点很窄：实测只有约 19 mm
   * （8036 n=11 bm=0FFE → 8055 n=5 bm=0F80 → 8072 n=0），
   * 而 in_cross_mm 写的是 80 mm。多锁的那 60 mm 里线已经从左边溜走，
   * HINOK 之后 PD 抬头一看 n=0，只能直冲撞墙（STALL）。
   * 所以加一条"走够 10 mm 且宽白已经消失"就交还 PD，
   * in_cross_mm 只当上限兜底用。10 mm 是防抖动：进岛那一帧 n 偶然掉一下不算穿完。 */
  if (isl_state == ISL_HIN) {
    if (((dist - isl_d0) >= 10 && n < ISL_T_N)
        || (dist - isl_d0) >= (int32_t)Island_Cfg()->in_cross_mm
        || (uint32_t)(now - isl_t_state) > 1200U) {
      printf("ISL,HINOK,%ld,%d\r\n", (long)(dist - isl_d0), (int)n);
      isl_state   = ISL_SESSION;
      isl_t_state = now;
      isl_yaw0    = gyro_angle;      /* 绕行角从穿过交叉点之后开始算 */
      isl_d_fin   = dist;
      Steer_Reset();
      return 0U;
    }
    return 1U;                       /* 直行锁定 */
  }

  /* ---- 出环：锁定直行穿过岔口，防止 PD 又沿环走 ---- */
  if (isl_state == ISL_HEXOUT) {
    /* 超时只做保护，不能比"走完 ISL_EXIT_CROSS_MM"更早触发：
     * 速度 150 时走 220 mm 要 1.47 s，原来 1200 ms 会在 180 mm 就截断，
     * 导致车没穿过岔口就交还 PD -> 立刻丢线 -> LOST。 */
    if ((dist - isl_d0) >= (int32_t)Island_Cfg()->out_cross_mm
        || (uint32_t)(now - isl_t_state) > 3000U) {
      printf("ISL,SOUT,%s,%ld,%d\r\n", Island_Cfg()->name, (long)(dist - isl_d0), (int)n);
      isl_state    = ISL_RET;
      isl_t_state  = now;
      isl_ret_dist = dist;
      sq_events    = 0U;
      Island_Advance(now, "HEXDONE");
      Steer_Reset();
      return 0U;
    }
    return 1U;                       /* 直行锁定 */
  }

  /* ---- 冷却 ----
   * 冷却的本意是防止"刚出岛的六边形分叉"被重复触发（分叉判据 5<=n<=9）。
   * 直角环岛走的是 T 口判据（n>=11），图案完全不同、不会误触发，
   * 所以方形格不该被这段冷却卡住。否则六边形出岛后 350ms 内，
   * 方形的 T 口整段被无视（方形分支在冷却之后才执行），
   * 车低速冲过 T 口、到 90° 直角时飞出赛道（现场：直接走方形没问题，
   * 六边形之后接方形就减速飞出 —— 根因就是这段冷却）。 */
  if (isl_state == ISL_RET) {
    if (!Island_IsSquare() && (uint32_t)(now - isl_t_state) < ISL_COOL_MS) return 0U;
    isl_state = ISL_NONE;
  }

  /* ---- 计划格超时兜底：太久没等到该格的图案，强制前进一格 ---- */
  if (isl_t_phase != 0U && (uint32_t)(now - isl_t_phase) > ISL_PHASE_MS) {
    if (Island_IsSquare()) {
      sq_events = 0U;
      isl_d_t2  = dist;                /* 跳过方框时也要立基准，否则强制进岛失效 */
    }
    Island_Advance(now, "SKIP");
    return 0U;
  }

  /* ---- 六边形格的【里程】兜底：走够 ISL_HEX_MAX_MM 还没进岛就换下一格 ----
   * ISL_PHASE_MS 那条是 30 s 才生效，而这段路只有十几秒，等于没有兜底。
   * 六边形入口的图案实测极不稳：同一条赛道，两趟分别读成 0FFE(n=11) 和 003F(n=6)，
   * 判据一旦漏掉，plan_idx 就永远停在这一格 —— 后面每个环岛都会拿错图案去处理，
   * 表现就是"到了直角环岛反而飞出去"（方形格只认 T 口，六边形格只认分叉，两种都不对）。
   * 所以这里用里程强制换格：只要这一格是六边形、而且一直没进岛（isl_state 还是 ISL_NONE），
   * 走够 ISL_HEX_MAX_MM 就 Island_Advance 到下一格（方形），把直角状态机解锁出来。
   * 注意判 isl_state == ISL_NONE：正在绕行的六边形不受这条影响，
   * 只有"压根没进去"的情况才会被跳过。 */
  {
    int32_t skip_mm = Island_Cfg()->skip_mm ? (int32_t)Island_Cfg()->skip_mm
                                            : (int32_t)ISL_HEX_MAX_MM;
    if (!Island_IsSquare() && isl_state == ISL_NONE
        && (dist - isl_d_phase) >= skip_mm) {
      printf("ISL,HEXOVER,%ld,%s,%d,%04X\r\n",
             (long)(dist - isl_d_phase), Island_Cfg()->name,
             (int)n, (unsigned)line_raw);
      isl_cfm        = 0U;
      isl_block_dist = 0;
      isl_out_arm    = 0U;
      isl_err_off    = 0.0f;
      Island_Advance(now, "HEXOVER");
      return 0U;
    }
  }

  /* ================= 方形格：只认 T 口 ================= */
  if (Island_IsSquare()) {
    if (Island_Pattern(n)) {
      if (!isl_cfm) { isl_cfm = 1U; isl_t_cfm = now; }
      else if ((uint32_t)(now - isl_t_cfm) >= ISL_CONFIRM_MS) {
        isl_cfm     = 0U;
        isl_state   = ISL_CROSS;
        isl_t_state = now;
        isl_d0      = dist;
        /* 诊断：多打两列，用来判定"这个 T 口到底是本格的入口还是别的东西"
         *   %ld  本格已走里程 (dist - isl_d_phase)
         *   %d   这是本格的第一个 T 口(sq_events=0) 还是第二个(1)
         * 17:16 那趟的问题就是无法区分：三次跑动里同一格 SQ6 的入口
         * 分别在 16944 / 17132 / 16330，最后一次早了 600 mm，
         * 但只看绝对里程分不清是"入口漂了"还是"咬到了别的宽白"。 */
        printf("ISL,IN,%s,%d,%04X,%ld,%ld,%d\r\n", Island_Cfg()->name,
               (int)n, (unsigned)line_raw, (long)dist,
               (long)(dist - isl_d_phase), (int)sq_events);
        return 1U;
      }
    } else {
      isl_cfm = 0U;
    }
    return 0U;
  }

  /* ================= 六边形阶段 =================
   * 两条路：① 分叉图案（早、但不稳）  ② 里程强制（稳、作为兜底） */

  /* ---- 禁止进岛闸门（大套小刚结束时用） ----
   * 331.txt：嵌套环岛一出门，第 2 格就在出口交叉点上误判进岛。 */
  if (isl_block_dist != 0) {
    if (dist < isl_block_dist) return 0U;
    isl_block_dist = 0;
  }

  /* ---- ① 里程强制进岛（参考第一阶段 3b580d1，那版能跑完两圈） ----
   * 第一阶段的六边形入口根本不看图案：
   *   if (isl_d_t2 != 0 && (dist - isl_d_t2) >= ISL_ENTRY_MM) 直接进会话。
   * 理由是位图不可靠——六边形入口同一条赛道两趟可能分别读成
   * 0FFE(n=11) 和 003F(n=6)，靠图案必然漏判。
   * 这里用"本格起点 + force_mm"做基准，因为第 8 格前面是八边形不是方形，
   * 没有"第二个 T 口"可以当参照。 */
  if (Island_Cfg()->force_mm != 0U && !isl_force_used
      && (dist - isl_d_phase) >= (int32_t)Island_Cfg()->force_mm) {
    isl_force_used = 1U;         /* 本格只许强制进岛一次，防止 FABORT 后原地反复重进 */
    printf("ISL,FIN_M,%s,%ld,%d,%04X\r\n", Island_Cfg()->name,
           (long)(dist - isl_d_phase), (int)n, (unsigned)line_raw);
    isl_cfm     = 0U;
    isl_t_state = now;
    isl_yaw0    = gyro_angle;
    isl_out_arm = 0U;
    isl_d_fin   = dist;
    isl_d_t2    = 0;
    if (Island_Cfg()->in_cross_mm != 0U) {
      isl_state = ISL_HIN;
      isl_d0    = dist;
    } else {
      isl_state = ISL_SESSION;
    }
    Steer_Reset();
    return 0U;
  }

  /* ---- 顶点进入型六边形：入口是"穿过顶点"的宽白（n >= 11） ----
   * 这种六边形的入口和方形 T 口长得一模一样（都是整排几乎全白，实测位图 07FF n=11，
   * 持续约 50 mm）。之前六边形只靠 Island_Fork(5<=n<=9) 认"分叉"，
   * 于是这个 n=11 的顶点入口被 if (n > 9U) 拒掉，六边形永远进不去，
   * 车直冲过去撞外侧墙（现场：大套小出岛后那个六边形就是这种情况）。
   * 方形格在上一段已经 return 了，所以这里只会命中"非方形且 in_cross_mm!=0"的格，
   * 也就是 SMALLHEX；切向六边形（HEX7）和老行为环岛 in_cross_mm==0 不受影响。 */
  if (Island_Cfg()->in_cross_mm != 0U && Island_Pattern(n)) {
    if (!isl_cfm) { isl_cfm = 1U; isl_t_cfm = now; }
    else if ((uint32_t)(now - isl_t_cfm) >= ISL_CONFIRM_MS) {
      isl_cfm     = 0U;
      isl_state   = ISL_HIN;
      isl_t_state = now;
      isl_yaw0    = gyro_angle;
      isl_out_arm = 0U;
      isl_d_fin   = dist;
      isl_d_t2    = 0;
      isl_d0      = dist;
      printf("ISL,FIN,%s,%d,%04X,%ld\r\n", Island_Cfg()->name, (int)n, (unsigned)line_raw, (long)dist);
      printf("ISL,HIN,%ld\r\n", (long)dist);
      Steer_Reset();
      return 1U;                        /* 本帧起锁直行穿顶点 */
    }
    return 0U;
  }

  /* ---- ② 里程强制进岛：第二个 T 口之后走够 ISL_ENTRY_MM 就无条件进 ----
   * 这个判据不看位图，所以漏判不可能发生。实测散布只有 ±60 mm。 */
  if (isl_d_t2 != 0 && (dist - isl_d_t2) >= (int32_t)ISL_ENTRY_MM) {
    printf("ISL,FIN_F,%ld,%d,%04X\r\n", (long)(dist - isl_d_t2),
           (int)n, (unsigned)line_raw);
    isl_cfm     = 0U;
    isl_state   = ISL_SESSION;
    isl_t_state = now;
    isl_yaw0    = gyro_angle;
    isl_out_arm = 0U;
    isl_d_fin   = dist;
    isl_d_t2    = 0;                    /* 用掉就清，防止下次误触发 */
    Steer_Reset();
    return 0U;
  }

  /* ---- ① 分叉图案进岛（更早，能提前于强制点触发） ---- */
  if (Island_Fork(n)) {
    /* 刚出过岛就先别急着再进 */
    if (isl_ret_dist != 0 && (dist - isl_ret_dist) < (int32_t)ISL_RET_MM) {
      isl_cfm = 0U;
      return 0U;
    }
    if (!isl_cfm) { isl_cfm = 1U; isl_t_cfm = now; isl_d0 = dist; }
    else {
      isl_t_cfm = now;                 /* 刷新"最后看到图案"的时刻 */
      if ((dist - isl_d0) >= (int32_t)Island_Cfg()->fork_mm) {
        isl_cfm     = 0U;
        isl_t_state = now;
        isl_yaw0    = gyro_angle;
        isl_out_arm = 0U;                 /* 新会话：出岛武装清零 */
        isl_d_fin   = dist;
        isl_d_t2    = 0;
        printf("ISL,FIN,%s,%d,%04X,%ld\r\n", Island_Cfg()->name, (int)n, (unsigned)line_raw, (long)dist);
        Steer_Reset();
        /* 进岛先锁直行穿过交叉点（顶点进入型）：
         * 交叉点处加权重心会被两条分支平均成斜向，PD 会带着车斜穿过去。
         * 这几十毫米内不理会 err，按车头方向直着冲过交叉点，过了再交给 PD。 */
        if (Island_Cfg()->in_cross_mm != 0U) {
          isl_state = ISL_HIN;
          isl_d0    = dist;
          printf("ISL,HIN,%ld\r\n", (long)dist);
        } else {
          isl_state = ISL_SESSION;
        }
        return 1U;                        /* 本帧起直行锁定 */
      }
    }
  } else if (isl_cfm && (uint32_t)(now - isl_t_cfm) > 400U) {
    /* 宽限从 250 提到 400 ms。
     * 实测真分叉两次出现相隔可达 83 mm ≈ 286 ms（8661 n=5 bm=038C →
     * 8744 n=5 bm=0383），250 ms 会在两次之间把候选作废，永远进不去；
     * 400 ms 能保住候选，又不至于把 8146 那种 n=10 交叉口的瞬时上升沿也接上。 */
    isl_cfm = 0U;
  }

  /* ---- 计划表自校正 ----
   * 本格应该是六边形，却看到"持续的 T 口"（150 ms 以上，分叉的瞬时白闪到不了）
   * → 说明前面漏掉了一格，指针错位了，强制前进一格。
   * 不做这一步的话，指针一旦落后，后面每个元素都会用错图案去处理：
   * 方形被当成环岛绕着转，六边形被当成方形直接穿过去。 */
  /* 正在处理某个环岛时不做自校正。
   * 37.txt：第 8 格刚被推进计划表 0.6 s，就在它的入口宽白（0FFF n=12）上
   * 被 RESYNC 当成"指针错位"直接跳过。入口那一帧本来就是宽白，
   * 用它来判断"计划表落后"必然误伤。 */
  if (isl_state != ISL_NONE) {
    rsy_cfm = 0U;
  } else if (Island_Pattern(n)) {
    if (!rsy_cfm) { rsy_cfm = 1U; t_rsy = now; }
    else if ((uint32_t)(now - t_rsy) >= 150U) {
      rsy_cfm = 0U;
      printf("PLAN,RESYNC,%d,T,%d,%04X,%ld\r\n", (int)plan_idx + 1,
             (int)n, (unsigned)line_raw, (long)dist);
      Island_Advance(now, "RESYNC");
      return 0U;
    }
  } else {
    rsy_cfm = 0U;
  }

  return 0U;
}

/* 大套小流程需要叠加给跟随环的转向偏置（BS_EXT 出岛段用），其余恒为 0。 */
static float Island_Turn(void)
{
  return bs.turn;
}

/* 大套小流程的误差目标偏移（等价摄像头的补线），其余恒为 0。
 * 进环靠它把车掰到另一条分支上：PD 会主动开到新目标并稳住，
 * 不像固定偏置那样被 PD 抵消掉（偏置 95 只能让车偏 0.63 个单位）。 */
static float Island_ErrOffset(void)
{
  /* 大套小（bs.err_off）+ 通用环岛切向进岛（isl_err_off），两者不会同时非零 */
  return bs.err_off + isl_err_off;
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
  uint8_t  near_wide = 0U;     /* 宽区减速的迟滞状态，见下方速度调度 */
  uint32_t t_wide = 0U;        /* 最后一次 n>=WIDE_NEAR_N 的时刻 */
  uint16_t stall_ms = 0U;
  float    turn_prev = 0.0f;   /* 上一帧转向量，T 口锁定时用 */
  float    gap_yaw = 0.0f;     /* 断线盲走：最后一次看到线时的航向 */
  uint8_t  gap_corner = 0U;    /* 1 = 这次丢线是弯道造成的（不是断路区） */
  uint8_t  gap_ok_n = 0U;      /* 接线确认计数：连续见到线的帧数 */
  float    yaw_ok  = 0.0f;     /* 每一帧见到线就更新，盲走用它当基准 */
  int32_t  gap_d0 = 0;         /* 断线盲走：丢线瞬间的里程 */
  float    thin_mm = 0.0f;     /* 最近连续"细线"(n=1~3)走了多少 mm，断路判定用 */
  float    thin_before = 0.0f; /* 本帧清零之前的细线累计（竖条判据用） */
  int32_t  thin_d_prev = 0;    /* 上一帧的里程（相对 s0），算 thin_mm 的增量 */

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

      /* 细线里程统计：n>=5（宽白/分叉/拐角）清零，n=1~4 累加。
       * n=0（丢线）既不加也不清 —— 130 ms 的丢线判定窗口里车还在走，
       * 那段的细线里程应该保留下来，用来判断"丢线前是不是在干净直道上"。
       *
       * 【门槛为什么是 5 而不是 4】正常白线会读成 2~4 路：
       *   00F0(4) 权重 -1,-0.2,+0.2,+1 -> 中心 0，是标准居中细线，只是画得粗
       *   0038(3) 003C(4) 0070(3) 0060(2) 都正常
       * 第一版写成 n>=4 清零，结果真断路口那条线正好是 00F0(4)：
       *   11606 00F0(4) 清零 -> 11638 00F0(4) 清零 -> 11668 00E0(3) 只剩 29
       *   -> 11717 GAP,START,...,1,...,10 判成弯道 -> 1200 ms 后 LOST 停车
       * 宽白特征（0FFF=12 / 003F=6 / 007E=6 / 007C=5 / 00FE=7）都在 5 路以上，
       * 所以门槛放 5 能把"粗一点的正常线"和"真正的宽白"分开：
       *   真断路(11717) 细线累计 378 mm；真断路(11687) 334 mm；假断路(20391) 31 mm。 */
      {
        int32_t d_now = ENC_Dist_mm() - s0;
        thin_before = thin_mm;                 /* 竖条判据要用"清零前"的值 */
        if (n >= 5U)       thin_mm = 0.0f;
        else if (n >= 1U)  thin_mm += (float)(d_now - thin_d_prev);
        thin_d_prev = d_now;
      }

      /* ---------- 1b) 发车/终点门（"工"字竖条）检测 ----------
       * 指纹：整排白(n>=11) + 前面 >=120 mm 干净细线 + 后面连续 3 帧回到细线。
       *   方形 T 口前面是一串宽白（thin_mm 早被清 0），所以过不了 BAR_THIN_MM；
       *   宽区/T 口本身能持续上百毫米，过不了 BAR_WIDE_MAX。
       * 这里只负责"报告"一根竖条的事实，判"是不是门"交给 Lap_Gate()。 */
      {
        int32_t d_now = ENC_Dist_mm() - s0;
        /* 门槛分两档：第一根用 n>=11，第二根放宽到 n>=10。
         *
         * 【为什么第二根必须放宽】11/19 16:18 那趟实测发车区两根竖条：
         *   20227  n=12  bm=0FFF   第一根，正常
         *   20648  n=10  bm=0FFA   第二根，ch2 掉了，只有 10 路
         * 间距 421 mm，完全符合 PAIR 300~550，但 n=10 < BAR_FULL_N(11)
         * 直接被判掉 -> 没有 GATE,PAIR -> 第一圈没计上 -> 兜底 LAP,W 把
         * 本圈起点记成 26281（真实门 20767），偏差 5514 mm ->
         * 第二圈大套小窗口跑到错位置 -> 假大套小罩住第二次断路 ->
         * 断路被当成弯道满舵 -> STALL。整条链的源头就是这 1 路。
         *
         * 只放宽"第二根"（pair_mm 非零时），第一根仍要 11 路：
         * 方框拐角实测 0FF8(9) / 0FFC(10) / 03FF(10) 都只有 9~10 路，
         * 拿 10 路当第一根会把拐角当门；但当已经有一根待配对时，
         * 再出现 10 路且间距 300~550，几乎不可能是巧合。 */
        uint8_t need_n = (pair_mm != 0) ? (uint8_t)BAR_FULL_N2 : (uint8_t)BAR_FULL_N;

        if (bar_cand == 0U) {
          if (n >= need_n && thin_before >= BAR_THIN_MM
              && fabsf(err_raw) < BAR_ERR_MAX && t_rcv == 0U) {
            bar_cand = 1U;
            bar_wide = 1U;
            bar_ok   = 0U;
            bar_mm   = d_now;
          }
        } else if (n >= need_n) {
          /* 还在竖条上：竖条约 20 mm，300 mm/s 下要待 6~7 个控制周期，
           * 所以"宽白持续"不能当失败；只有持续太久（>150 mm）才判为 T 口。 */
          if (bar_wide < 250U) bar_wide++;
          if (bar_wide > BAR_WIDE_MAX) {
            printf("BAR,WIDE,%ld,%d\r\n", (long)bar_mm, (int)bar_wide);
            bar_cand = 0U;
          }
        } else if (n >= 1U && n <= 4U) {       /* 回到细线（n=0 丢线不算） */
          if (bar_ok < 10U) bar_ok++;
          if (bar_ok >= BAR_CONFIRM_N) {
            printf("BAR,HIT,%ld,%ld\r\n", (long)bar_mm, (long)(d_now - bar_mm));
            bar_cand   = 0U;
            bar_hit    = 1U;
            bar_hit_mm = bar_mm;               /* 用竖条出现那一刻，比确认时刻准 */
          }
        } else {
          /* n = 5~10：竖条的边缘过渡，既不算宽也不算薄，保持候选 */
        }
      }

      /* 盲走（断路/找线）期间冻结环岛状态机：
       * 断路有 650 mm，盲走时里程照涨，Island_Step 里的 ISL_HEX_MAX_MM(400)
       * 会在半路打 ISL,HEXOVER 把计划表推走一格，之后所有环岛都对不上。 */
      isl_now = (ISL_ENABLE && t_rcv == 0U)
                  ? Island_Step(n, now, ENC_Dist_mm() - s0, turn_prev) : 0U;

      /* 发车/终点门：消费竖条事件，判"是不是过门"/"该不该停"。
       * 放在 Island_Step 之后：换圈时它会重置 plan_idx，下一帧才生效。 */
      Lap_Gate(ENC_Dist_mm() - s0);

      /* 出岛那一帧重置丢线计时：岛内可能几秒没更新过 */
      if (isl_was && !isl_now) { t_grace = now; t_lost = now; }
      isl_was = isl_now;

      /* ---------- 2) 丢线处理 ---------- */
      if (n == 0U) {
        uint8_t wide = (isl_now || (t_grace != 0U
                     && (uint32_t)(now - t_grace) < ISL_OUT_MS)) ? 1U : 0U;

        /* 分叉候选已出现时放宽丢线容忍，避免把车拐离环岛入口 */
        uint32_t lm = (isl_cfm != 0U) ? LOST_ISL_MS : LOST_MS;

        if ((uint32_t)(now - t_lost) > (wide ? ISL_LOST_MS : lm)) {
          if (t_rcv == 0U) {          /* 第一次进入盲走：记住当前航向与里程 */
            t_rcv     = now;
            gap_ok_n  = 0U;
            gap_d0    = ENC_Dist_mm() - s0;
            /* 区分"断路区"和"直角弯丢线"：
             *   断路区：进缺口前车正对着走，|err| 很小  -> 锁航向直行
             *   直角弯：拐不过去才丢线，丢线前 |err| 已很大 -> 朝最后方向转
             * 判错的后果：断路区该直行却去转弯（撞墙），
             *            直角弯该转弯却直行（直接冲出赛道）。
             *
             * 28.txt 教训：同一个位置一次判 0、一次判 1（|err| 刚好在 2.5 附近），
             * 所以门槛降到 1.6；而且只要处在环岛流程里（BS_*），
             * 一律按弯道处理 —— 断路区在赛道别处，不会在环岛内部。 */
            gap_corner = (fabsf(err_keep) > 1.6f) ? 1U : 0U;
            /* 断路的前置条件：丢线之前必须已经在细线上走够 GAP_MIN_THIN_MM。
             * 出岛/岔口/拐角丢线前面一定是一串宽白，thin_mm 会被清零，
             * 于是这类丢线一律按弯道处理，不会拿 24° 的斜穿角去盲走 800 mm。 */
            if (thin_mm < (float)GAP_MIN_THIN_MM) gap_corner = 1U;
            if (bs.state == BS_BIG || bs.state == BS_MID
                || bs.state == BS_SMALL || bs.state == BS_TAIL) {
              gap_corner = 1U;
            }

            /* ---- 环岛会话内丢线：恢复方向跟随环岛的绕行方向 ----
             * 弯道恢复用的是 err_dir（丢线前最后看到线在左还是右）：
             *     turn = FOLLOW_TURN_SIGN * err_dir * RECOVER_TURN
             * 在环岛里这是**极其危险**的：11/19 16:52 第 6 趟八边形实测
             *   18820 丢线 err=-1.0            yaw -4284
             *   18963 恢复模式中               yaw -5635
             *   19027 退出恢复（转了 209.7°）  yaw -6381
             * 进岛后明明右转了 78.8°，只因丢线前线略偏左（err=-1.0），
             * 恢复模式就按"往左找线"打了满舵，把车掉了 210° 往回开。
             * 环岛是有固定绕行方向的，丢线就该朝那个方向继续找，
             * 而不是朝"上一条线在哪边"回头——那是往回走。
             * dir=+1（右转）-> err_dir=+1 -> turn=-220 -> 右转 220，
             * 沿环岛继续绕；dir=-1 同理。 */
            if (Island_InSession() && Island_Cfg()->dir != 0) {
              err_dir = (float)Island_Cfg()->dir;
            }

            /* ---- 断路斜穿的目标航向 ----
             * 断路两端有位置错位：水平 650 mm、右侧接收线低 240 mm，
             * 需要向右偏 20.3°；保持原航向会从目标白线上方 240 mm 飞过。
             * 直角弯丢线（gap_corner=1）不加这个偏转，那是"拐回原线"，
             * 目标就是原来的航向。 */
            gap_yaw = gap_corner ? yaw_ok : (yaw_ok + GAP_TURN_DEG);

            printf("GAP,START,%ld,%d,T%d,%d\r\n",
                   (long)gap_d0, (int)gap_corner, (int)(gap_yaw * 10.0f),
                   (int)thin_mm);
          } else if ((gap_corner
                        ? ((uint32_t)(now - t_rcv)
                             > (Island_InSession() ? ISL_RECOVER_MS : RECOVER_MS))
                        : ((uint32_t)(now - t_rcv) > GAP_MS))
                     || (ENC_Dist_mm() - s0 - gap_d0) > (int32_t)GAP_MM) {
            printf("LOST\r\n");
            break;
          }
        }
      } else {
        t_lost = now;
        if (t_rcv != 0U) {
          /* 接线确认：连续 GAP_CONFIRM_N 帧见到线才结束盲走。
           * 只认一帧的话，扫到反光、锥桶边、或目标白线之外的东西就会
           * 提前交还 PD，车还在缺口里，接着又丢线，来回抽搐。 */
          if (gap_ok_n < 250U) gap_ok_n++;
          if (gap_ok_n >= GAP_CONFIRM_N) {
            int32_t glen = ENC_Dist_mm() - s0 - gap_d0;

            printf("GAP,DONE,%ld\r\n", (long)glen);
            t_rcv    = 0U;
            gap_ok_n = 0U;
            /* 盲走期间环岛逻辑是冻结的（见 Island_Step 调用处的 t_rcv 判断），
             * 但里程照涨。这段距离不该算进"这一格还没进岛"的预算里：
             * 35.txt 里绕完八边形出岛丢了一次线（仅 153 mm），
             * 下一格 HEX8 的 ISL_HEX_MAX_MM(400) 预算就被吃掉一大半，
             * 车还没走到六边形入口就在 401 mm 处被 HEXOVER 跳过。
             * 把盲走里程补回格起点，等于"盲走不计时"。 */
            if (glen > 0) isl_d_phase += glen;
          }
        } else {
          gap_ok_n = 0U;
        }
        yaw_ok = gyro_angle;                    /* 记住最后一次见到线时的航向 */
        if (err_raw >  0.3f) err_dir =  1.0f;   /* 记住最后见线方向 */
        if (err_raw < -0.3f) err_dir = -1.0f;
      }

      /* ---------- 3) 误差整形 + 速度调度 ---------- */
      e = Line_Shape(n, err_raw, &err_keep) - FOLLOW_ERR_ZERO + Island_ErrOffset();

      if (t_rcv != 0U && gap_corner) {
        /* 弯道丢线：朝最后见线方向打满，把车拽回来。
         * 直角弯转弯时线会整条离开传感器排，此时若锁航向直行必然冲出赛道。 */
        v_tgt = RECOVER_V;
        turn  = FOLLOW_TURN_SIGN * err_dir * RECOVER_TURN;
      } else if (t_rcv != 0U) {
        /* 断线盲走：锁住丢线瞬间的航向直行，冲过"断线路段"。
         * 之前是"朝最后见线方向打满"，在 650 mm 没有线的缺口里会原地打转。 */
        float d = gap_yaw - gyro_angle;
        float t = -GAP_KP * d;

        if (t >  GAP_TURN_MAX) t =  GAP_TURN_MAX;
        if (t < -GAP_TURN_MAX) t = -GAP_TURN_MAX;

        v_tgt = GAP_V;
        turn  = t;
      } else if (isl_now) {
        /* 穿过 T 口：转向锁定（保持进 T 前的转向量），低速直行
         * T 口左右对称，12 路平均会趋近 0，此时 PD 方向不可信 */
        v_tgt = ISL_CROSS_V;
        turn  = Island_Turn();
      } else {
        float ae = (e > 0.0f) ? e : -e;
        if (ae > 7.0f) ae = 7.0f;

        if (Island_InSession()) {
          /* 六边形岛内：固定速度。
           * 岛内两条线并排，路数 n 一直在 4~6 之间跳；
           * 原来用 n>=5 触发预备减速，会让目标速度在 294 和 200 之间来回跳，
           * 车就跑得一窜一窜的。 */
          v_tgt = (float)Island_Cfg()->v_island;
        } else {
          v_tgt = FOLLOW_V_FAST - (FOLLOW_V_FAST - FOLLOW_V_SLOW) * (ae / 7.0f);

          /* 接近宽区先减速（看路数，不看里程）。
           *
           * 迟滞：n>=WIDE_NEAR_N(8) 进入；跌回 n<=3 **并持续 WIDE_REL_MS** 才解除。
           * 普通细线的 n 就是 3（位图 00E0/0070），所以解除阈值必须放到 3 ——
           * 曾经写成 n<=2，结果普通直线永远满足不了，整段被压在 ISL_NEAR_V。
           * 再加 300 ms 时间条件，是为了入口处 n 在 3~8 之间逐帧跳时
           * （2.txt: 3483:3 / 3493:3 / 3507:4 / 3523:3 / 3552:5）不来回开关。 */
          if (n >= WIDE_NEAR_N) {
            near_wide = 1U;
            t_wide    = now;
          } else if (near_wide && n <= 3U
                     && (uint32_t)(now - t_wide) >= WIDE_REL_MS) {
            near_wide = 0U;
          }

          if (near_wide && v_tgt > ISL_NEAR_V) v_tgt = ISL_NEAR_V;

          /* 急弯预判：偏差"又大又变快"说明曲率很大。
           * 光电管看不到前方，但 |e| 的增长速率本身就是提前量：
           * 直角弯进弯前 100~200 mm 内 |e| 会从 1 快速涨到 5 以上。
           * 这时立刻压到 CORNER_V，比"等 |e| 到 7 才降速"早得多 ——
           * 之前直角弯冲出去，就是因为降速发生时车已经到弯上了。 */
          {
            float de_abs = fabsf(e - last_e);
            if (fabsf(e) > 3.2f && de_abs > 0.55f && v_tgt > CORNER_V) {
              v_tgt = CORNER_V;
            }
          }
        }

        turn = FOLLOW_TURN_SIGN * Steer_PD(e, e - last_e)
             + FOLLOW_TURN_BIAS * (v_tgt / FOLLOW_V_FAST)
             + Island_Turn();      /* 大套小交叉点的强制转向偏置，其余恒 0 */
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
      /* 必须取绝对值：内侧轮允许反转（FOLLOW_V_MIN = -120），
       * 旧写法 vL<20 && vR<20 会把反转的 -120 也算成没转，
       * 于是转弯 / 进岛 / 发车打方向时会随机误报 STALL。 */
      if (fabsf(vL) < 20.0f && fabsf(vR) < 20.0f && v_tgt > 100.0f) stall_ms += 10U;
      else                                           stall_ms  = 0U;
      if (stall_ms > STALL_MS) { printf("STALL\r\n"); break; }

      /* ---------- 7) 停车判定 ----------
       * 优先用竖条门：GATE,STOP 设了 stop_mm（第一根竖条 + STOP_FROM_FIRST_MM），
       * 走到那个里程就停 —— 位置由光电管决定，不看里程漂移多少。
       * 里程（goal_mm + MILEAGE_STOP_MARGIN_MM）只在"一直没认出竖条"时兜底。
       * 那个 600 mm 余量是必须的：11/18 16:13 竖条在 20569、goal_mm 正好 20580，
       * 里程兜底抢在门确认前 11 mm 就把车停了。
       *
       * 【这里故意不看 Island_Busy()】11/18 15:43 那趟：第二根竖条被第 5 格
       * 的方形 T 口判据抢走，车进了 ISL_CROSS；停车判据因为 Island_Busy()=1
       * 一直等着，等穿完 T 口再停，多走了 60 mm + 80 ms + 滑行 ≈ 7~8 cm。
       * 到了发车区就该停，不必等"穿完 T 口"。 */
      if (stop_mm != 0) {
        int32_t d_now = ENC_Dist_mm() - s0;

        if (d_now >= stop_mm) {
          printf("LAPS,DONE,BAR,%ld\r\n", (long)d_now);
          break;
        }
        /* 冲过停车点 300 mm 还没停下来：强制停，别压过发车区 */
        if (d_now >= stop_mm + (int32_t)300) {
          printf("LAPS,OVER,%ld,%ld\r\n", (long)d_now, (long)stop_mm);
          break;
        }
      } else if ((ENC_Dist_mm() - s0) >= goal_mm + (int32_t)MILEAGE_STOP_MARGIN_MM
                 && !Island_Busy()) {
        printf("LAPS,DONE\r\n"); break;
      }
    }

    /* ---------- 打印（100 ms） ---------- */
    if ((uint32_t)(now - t_prt) >= 100U) {
      t_prt = now;
      /* F,状态,里程mm,err×10,目标×10,vL×10,vR×10,uL,uR,路数,位图,偏航×10
       * 最后一列是累计偏航角（gyro_angle）：环岛里应该单调增长，
       * 出岛判据靠它，调参时先看这一列涨不涨。 */
      printf("F,%d,%ld,%d,%d,%d,%d,%d,%d,%d,%04X,%d\r\n",
             (int)(t_rcv ? FS_RECOVER : (isl_now ? FS_ISLAND : FS_FOLLOW)),
             (long)(ENC_Dist_mm() - s0),
             (int)(e * 10.0f),
             (int)(v_tgt * 10.0f),
             (int)(vL * 10.0f),
             (int)(vR * 10.0f),
             (int)uL, (int)uR, (int)n, (unsigned)line_raw,
             (int)(gyro_angle * 10.0f));
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
