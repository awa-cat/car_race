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
#define FOLLOW_LAP_MM      60000     /* ★ 临时值：只为测单圈里程，不是真实单圈长度。
                                      * 设得足够大，goal_mm = 60000*2 = 120 m，
                                      * 保证不会在测量途中被 LAPS,DONE 截停。
                                      *
                                      * 【为什么必须改】之前写 12000，goal_mm = 24000。
                                      * 而第二阶段一圈至少 12~20 m（37.txt 里八边形
                                      * 就到了 17.7 m），第二圈刚出发就会被截停，
                                      * 永远量不到完整一圈。
                                      *
                                      * 【量到之后怎么填】
                                      *   1) 跑到出现 PLAN,1/9 那一刻，记 F 行里程（单圈近似值）
                                      *   2) 更准的是：两次 BS,IN 的里程相减 = 精确单圈
                                      *   3) 把这里改回实测值，FOLLOW_LAPS 保持 2 */
#define FOLLOW_LAPS            2
#define FOLLOW_TIMEOUT_MS 120000U    /* 总超时 120 s */

/* ==================== 二、速度 ==================== */
#define FOLLOW_V_FAST       300.0f   /* 正中时 mm/s */
#define FOLLOW_V_SLOW       180.0f   /* 偏差最大(±7°)时 mm/s */
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
#define ISL_FAKE_DEG        40.0f    /* 偏航还不到这个值 → 判为假进岛 */
#define ISL_OVER_DEG      380.0f     /* 超过这个偏航强制结束，防绕圈
                                      * 紧到 380：一旦错过了出口岔口，也别绕太远才停 */
#define ISL_SES_MAX_MS   20000U      /* 会话最长 20 s */
#define ISL_RET_MM          260      /* 会话结束后，至少再走这么远才允许重新进岛（新赛道环岛更密） */
#define ISL_PHASE_MS      30000U     /* 单阶段超时：这么久没等到图案就强制切换 */
/* 六边形格的里程兜底（配合计划格起点里程 isl_d_phase）：
 * 这一格是六边形、但走够这么多 mm 还没进岛，就强制 Island_Advance 到下一格（方形）。
 *
 * 标定依据（实测里程，以"这一格开始"为 0）：
 *   约 450 mm  六边形入口出现（同一条赛道两趟分别读成 0FFE n=11 / 003F n=6）
 *   约 520 mm  车丢线 STALL（2026-11-15 两趟都是这个位置）
 * 取 400 mm 是为了在入口出现之前就把计划推进到方形格，
 * 让方形的 T 口判据（n>=11）去接那个 0FFE，而不是继续拿六边形的分叉判据去空等。 */
#define ISL_HEX_MAX_MM    1200       /* 六边形格：走这么远还没进岛就换下一格。
                                      * 原来 400 太短：35.txt 里八边形出岛后丢线 153 mm，
                                      * 加上正常行驶，401 mm 处就把 HEX8 跳过了，
                                      * 而六边形入口还在更远处。 */

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
/* ==================== 发车/终点门（"工"字竖条） ====================
 * 赛道：车在两根竖线之间、朝左发车。竖条横穿主线，宽约 20 mm。
 *
 * 实测指纹（37.txt 发车后 208 mm 处）：
 *   mm=180 n=3 00E0 -> mm=208 n=12 0FFF（一帧）-> mm=226 n=3 00E0
 * 和方形 T 口完全不同：T 口前是一串宽白（003F/007E/007C/00FE），
 * 竖条前一定是干净细线，所以用 thin_mm 就能分开。
 *
 * 用法：
 *   第 1 圈出发穿左根(约 208 mm) -> 距上一根太近，不计圈
 *   每圈结束先穿右根 -> 距上一根 ≈ 单圈 -> 计一圈
 *               再穿左根(+400) -> 太近，忽略
 *   第 LAPS 圈的右根确认后 -> GATE_HALF_MM 处刹车停住（在两根之间） */
#define BAR_FULL_N       11U     /* 整排白：n >= 11 */
#define BAR_THIN_MM     120.0f   /* 竖条前必须有这么长的干净细线（排除 T 口） */
#define BAR_ERR_MAX       2.0f   /* |err| 小于此值：正着过，不是斜切 */
#define BAR_CONFIRM_N     3U     /* 之后连续这么多帧回到细线，才确认"薄" */
#define BAR_WIDE_MAX     15U         /* 宽白最多持续这么多帧（150 ms ≈ 150 mm）。
                                      * 竖条宽约 20 mm，300 mm/s 下要占 6~7 个控制周期，
                                      * 所以不能一帧就判成 T 口；真正的 T 口会持续更久。 */
#define BAR_GAP_MIN_MM   8000    /* 距上一根小于此值 -> 忽略（发车那根 / 门的第二根） */
#define GATE_WIDTH_MM     400    /* 两根竖条间距（实测：BAR 打印的第三个数） */
#define GATE_HALF_MM      200    /* = 门宽/2，停车点 */
#define BRAKE_MM          180    /* 提前这么多开始刹车 */
#define BRAKE_V         150.0f   /* 刹车起点速度 */
#define MIN_CREEP_V      60.0f   /* 最低蠕行速度，别半路停死 */

#define GAP_V             250.0f     /* 盲走速度 mm/s */
#define GAP_KP              2.5f     /* 航向保持：航向差 1 度 -> 2.5 mm/s 差速 */
#define GAP_TURN_MAX      130.0f     /* 航向保持差速限幅 */

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
static uint32_t isl_t_cfm = 0U;
static uint32_t isl_t_state = 0U;
static uint32_t isl_t_phase = 0U;       /* 计划格起始时刻 */
static int32_t  isl_d0    = 0;          /* 进入 T 口时的里程 */
static float    isl_yaw0  = 0.0f;       /* 会话起始航向 */
static float    isl_err_off = 0.0f;     /* 切向进岛的误差偏移（等价补线），出岛清零 */
static int32_t  isl_block_dist = 0;     /* 在这之前禁止进入任何环岛（大套小刚结束用） */
static int32_t  lap_d0 = 0;             /* 本圈起点里程：发车=0，每过发车/终点门更新。
                                         * 环岛的位置窗口（BS_ENTRY_MIN/MAX_MM）用它做基准，
                                         * 这样第二圈和第一圈的行为完全一致。 */

/* ---- 发车/终点门（竖条）状态：全部由 Island_Reset() 清零 ---- */
static uint8_t  bar_cand  = 0U;   /* 1 = 刚看到整排白，等它"变薄"确认 */
static uint8_t  bar_ok    = 0U;   /* 确认计数：之后连续细线帧数 */
static uint8_t  bar_wide  = 0U;   /* 已经在竖条（整排白）上待了几帧 */
static int32_t  bar_mm    = 0;    /* 候选出现时的里程 */
static int32_t  last_bar  = 0;    /* 上一根"有效竖条"的里程 */
static uint32_t t_bar     = 0U;   /* 上一次竖条时刻（调试用） */
static uint8_t  lap_cnt   = 0U;   /* 已完成圈数 */
static int32_t  stop_mm   = 0;    /* 0=不停；非 0=走到这个里程就停（两根竖条之间） */
static int32_t  isl_ret_dist = 0;       /* 上次会话结束时的里程 */
static uint8_t  isl_out_arm  = 0U;      /* 出岛武装：偏航到 ISL_EXIT_DEG 后置 1，
                                         * 之后任意一帧匹配分叉即出岛 */
static int32_t  isl_d_t2     = 0;       /* 第二个 T 口的里程，强制进岛的基准 */
static int32_t  isl_d_fin    = 0;       /* 本次会话进岛的里程，强制出岛的基准 */
static int32_t  isl_d_phase  = 0;       /* 当前这一计划格开始时的里程（里程兜底用） */
static uint8_t  isl_plan_last = 0xFFU;  /* 上一帧的 plan_idx，用来发现"换格了" */
static uint8_t  isl_force_used = 0U;    /* 本格已经用过一次"里程强制进岛"。
                                         * 没有它的话：强制进岛 -> FABORT -> 350 ms 冷却
                                         * -> 里程还在阈值后面 -> 立刻重进，无限循环，
                                         * 每进一次都把 yaw0 重置，偏航永远累不起来，
                                         * 表现就是"进不去 / 在入口反复抽"。 */
static uint8_t  rsy_cfm      = 0U;      /* 计划表自校正：T 口持续判据 */
static uint32_t t_rsy        = 0U;
/* 大套小的阶段编号（Island_InSession 等函数要提前用到） */
#define BS_IDLE    0U
#define BS_BIG     1U     /* 右转 240°，走大环 4 条边 */
#define BS_MID     2U     /* 直行一小段，过渡到小环 */
#define BS_SMALL   3U     /* 右转 360°，走小环 6 条边 */
#define BS_TAIL    4U     /* 直行，走最后 2 条边 */
#define BS_EXT     6U     /* 出岛前右转 30 度 */
#define BS_OUT     5U
/* ==================== 第 8 格：30° 切向六边形（专用流程） ====================
 * 现场确认的走法（和第一阶段那个六边形一样）：
 *   进岛：向右转 30° -> 强制直行一小段 -> 交给 PD 绕环
 *   出岛：绕完一圈回到切点 -> 向右转 30° -> 强制直行一小段 -> 出岛
 * 为什么必须"先转 30°"：主线与环的第一条边差 30°，
 * 不打角的话 PD 会顺着主线直行过去（实测 296 mm 只转 20°）。
 * 为什么转完还要"强制直行"：刚转完 30° 时车头正对着环的边，
 * 这一段交给 PD 反而会被主线残余拉回去，先锁航向把车"送进"边里。 */
#define H8_TURN_DEG      45.0f   /* 进/出岛各转多少度（右转），现场实测改 30 -> 45 */
#define H8_TURN_TOL       5.0f   /* 转到离目标这么近就算到位 */
#define H8_TURN_KP        6.0f   /* 转向增益：航向差 1 度 -> 6 mm/s 差速 */
#define H8_TURN_MAX     300.0f   /* 转向量限幅 */
#define H8_HOLD_KP        4.0f   /* 直行段保持航向的增益 */
#define H8_IN_MM        300      /* 进岛转完后强制直行多远，现场实测改 120 -> 300 */
#define H8_OUT_MM       300      /* 出岛转完后强制直行多远，现场实测改 150 -> 300 */
#define H8_TRACE_DEG    330.0f   /* 绕环一圈的偏航（环内累加） */
#define H8_MAX_MM      1600      /* 绕环里程兜底（一圈约 1200） */
#define H8_SES_MAX_MS  20000U    /* 整段超时 */
#define H8_MISS_MM      1500    /* 本格走这么远还没进岛 -> 放弃这一格，换下一格。
                                  * 没有它：第 8 格会一直挂在等入口状态，
                                  * 直到下一个宽白出现（38.txt 一直挂到第二圈的
                                  * 大套小入口 23700 才误进，套在别的环岛上执行）。 */
#define H8_MIN_TRACE_DEG 150.0f  /* 绕环至少要转这么多度，否则认为"根本没绕起来"：
                                  * 不做出岛转向、也不强制直行，直接交还 PD。
                                  * 38.txt 证据：TRACE 段偏航在 -150 和 -366 之间来回摆
                                  * （不是单调绕环），实际是车在 Z 字弯里；
                                  * 再右转 40 度 + 强制直行 300 mm 就把车推下赛道。 */
#define H8_ABORT_ERR     4.0f    /* 强制直行期间 |err| 超过这个值 -> 立刻交还 PD，
                                  * 避免"线已经拐走了还在锁航向直行" */

#define H8_IDLE      0U
#define H8_TURN_IN   1U
#define H8_STR_IN    2U
#define H8_TRACE     3U
#define H8_TURN_OUT  4U
#define H8_STR_OUT   5U

typedef struct {
  uint8_t  state;
  float    yaw_t;      /* 当前转向段的起点航向 */
  float    yaw_hold;   /* 直行段要保持的航向 */
  float    yaw_fin;    /* 开始绕环时的航向（算绕环角度用） */
  float    turn;       /* 本帧强制转向量 */
  int32_t  d0;         /* 本段起点里程 */
  int32_t  d_fin;      /* 进岛里程 */
  uint32_t t0;         /* 本次进岛时刻 */
} Hex30_t;

static Hex30_t h8;     /* 由 Island_Reset() 里 memset 清零 */

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
#define ISLK_H30     5U     /* 第 8 格：30 度切向六边形，走专用流程 Isl_Hex30() */
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
} IslandCfg;

#define ISL_ENTRY_OFF   2.5f   /* 切向进岛的误差偏移量（等价摄像头补线） */

static const IslandCfg isl_cfg[ISL_PLAN_N] = {
  /* 名字        类型       进岛n 进岛mm 出岛° 兜底mm 出环mm 岛内速度  方向 进岛打角° */
  { "BIGHEX",  ISLK_BSS,    5U,   60,   700,  4200,   180,   290,   0,    0,    0,    0 },  /*  1 大套小（专用） */
  { "SMALLHEX",ISLK_HEXV,   5U,   60,   340,   850,   170,   300,  -1,   60,   80,    0 },  /*  2 顶点进入·逆时针 */
  { "SQ3",     ISLK_CROSS, 11U,   20,     0,     0,    60,   150,   0,    0,    0,    0 },  /*  3 正方形① */
  { "SQ4",     ISLK_CROSS, 11U,   20,     0,     0,    60,   150,   0,    0,    0,    0 },  /*  4 正方形② */
  { "SQ5",     ISLK_CROSS, 11U,   20,     0,     0,    60,   150,   0,    0,    0,    0 },  /*  5 正方形③ */
  { "SQ6",     ISLK_CROSS, 11U,   20,     0,     0,    60,   150,   0,    0,    0,    0 },  /*  6 正方形④ */
  { "OCT7",    ISLK_LOOP,   5U,   60,   340,  1800,   170,   300,   0,    0,    0,    0 },  /*  7 八边形：兜底 850->1800，一圈约 1600，850 会在半路出岛 */
  { "HEX8",    ISLK_H30,    5U,   60,   330,  1300,   170,   180,   0,    0,    0,    0 },  /*  8 30°切向六边形（专用：进/出各右转30°+直行） */
  { "SQ9",     ISLK_CROSS, 11U,   20,     0,     0,    60,   150,   0,    0,    0,    0 }   /*  9 方形 */
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
  printf("PLAN,%d/%d,%s,%s\r\n", (int)plan_idx + 1, (int)ISL_PLAN_N,
         Island_IsSquare() ? "SQ" : "HEX", why);
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
  isl_force_used = 0U;
  /* 发车/终点门状态：每次发车必须清零，
   * 否则第二次发车会带着上次的圈数/停车点直接乱掉。 */
  bar_cand      = 0U;
  bar_ok        = 0U;
  bar_mm        = 0;
  last_bar      = 0;
  t_bar         = 0U;
  lap_cnt       = 0U;
  lap_d0        = 0;
  stop_mm       = 0;
  /* 大套小状态整体清零：一个 memset 覆盖所有字段，
   * 以后再加字段也不可能漏（之前就是漏了 err_off 导致"第一次对、之后全错"）。 */
  memset(&bs, 0, sizeof(bs));
  memset(&h8, 0, sizeof(h8));      /* 第 8 格 30 度六边形的状态 */

  printf("PLAN,1/%d,%s,START\r\n", (int)ISL_PLAN_N,
         Island_IsSquare() ? "SQ" : "HEX");
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

  /* 第 8 格 30° 六边形：进岛转向/直行/绕环/出岛各阶段都算"岛内"，
   * 否则速度会被宽区减速反复打断（沿边时 n 常在 5~8）。 */
  if (h8.state != H8_IDLE) return 1U;

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
     * 等里程一进窗口就立刻 BS,IN —— 位置还是错的，只是错得晚一点。 */
    /* 【关键】窗口用"本圈相对里程" d_lap，不能用 dist。
     * 原来直接比 dist（从发车算起的绝对里程）：
     *   第一圈大套小在 2.6 m -> 落在 2500~4200 窗口内 OK
     *   第二圈大套小在 23.7 m -> 永远在窗口外 -> BS,MISS，永远进不去。
     * lap_d0 在每次确认发车/终点竖条时更新，所以每圈的窗口都一样。 */
    /* 用"本圈相对里程" (dist - lap_d0)：第二圈和第一圈的窗口完全一致 */

    if ((dist - lap_d0) > (int32_t)BS_ENTRY_MAX_MM) {
      /* 窗口过了还没进：放弃这一格。
       * 不能只是 return —— plan_idx 会永久卡在 0（BIGHEX），
       * 于是之后每一个环岛都走 Isl_BigSmall 的入口判据，全部不被处理。 */
      printf("BS,MISS,%ld,%d,%04X\r\n", (long)(dist - lap_d0), (int)n, (unsigned)line_raw);
      isl_cfm = 0U;
      Island_Advance(now, "BSMISS");
      return 0U;
    }
    if ((dist - lap_d0) < (int32_t)BS_ENTRY_MIN_MM) {
      isl_cfm = 0U;                     /* 窗口前：允许清候选，但不许形成新的 */
      return 0U;
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
/* ==================== 第 8 格：30° 切向六边形（专用） ====================
 * 返回 1 = 本帧由环岛接管转向（turn = Island_Turn()），PD 不参与；
 * 返回 0 = 交还 PD 巡线（绕环阶段）。
 *
 * 符号约定（全工程统一）：
 *   gyro_angle 正 = 右转；turn 正 = 左转、负 = 右转。
 *   所以"向右转"时 turn 必须给负值。 */
static uint8_t Isl_Hex30(uint8_t n, uint32_t now, int32_t dist)
{
  const IslandCfg *c = Island_Cfg();
  float dy;

  if (h8.state != H8_IDLE && (uint32_t)(now - h8.t0) > H8_SES_MAX_MS) {
    printf("H8,TIMEOUT,%d\r\n", (int)h8.state);
    h8.state = H8_IDLE;
    h8.turn  = 0.0f;
    Island_Advance(now, "H8TO");
    return 0U;
  }

  switch (h8.state) {

  case H8_IDLE:
    /* ---- 里程兜底：本格走够这么远还没进岛，说明这一格压根没等到入口 ---- */
    if (isl_d_phase != 0 && (dist - isl_d_phase) >= (int32_t)H8_MISS_MM) {
      printf("H8,MISS,%ld,%d,%04X\r\n",
             (long)(dist - isl_d_phase), (int)n, (unsigned)line_raw);
      h8.state     = H8_IDLE;
      h8.turn      = 0.0f;
      isl_ret_dist = dist;
      Island_Advance(now, "H8MISS");
      return 0U;
    }

    if (n >= c->fork_n) {
      if (h8.d0 == 0) {
        h8.d0 = dist;
      } else if ((dist - h8.d0) >= (int32_t)c->fork_mm) {
        h8.t0    = now;
        h8.d_fin = dist;
        h8.yaw_t = gyro_angle;
        h8.turn  = 0.0f;
        h8.state = H8_TURN_IN;
        h8.d0    = dist;
        printf("H8,IN,%d,%04X,%ld\r\n", (int)n, (unsigned)line_raw, (long)dist);
        return 1U;
      }
    } else {
      h8.d0 = 0;
    }
    return 0U;

  case H8_TURN_IN:
    dy = gyro_angle - h8.yaw_t;
    if (dy >= (H8_TURN_DEG - H8_TURN_TOL)) {
      h8.yaw_hold = gyro_angle;
      h8.d0       = dist;
      h8.state    = H8_STR_IN;
      h8.turn     = 0.0f;
      printf("H8,TURNIN,%d\r\n", (int)(dy * 10.0f));
      return 1U;
    }
    h8.turn = -H8_TURN_KP * (H8_TURN_DEG - dy);
    if (h8.turn < -H8_TURN_MAX) h8.turn = -H8_TURN_MAX;
    return 1U;

  case H8_STR_IN:
    /* 强制直行要被现实否决：线没了、或偏差已经很大，
     * 说明"应该直行"的前提不成立，立刻交还 PD。 */
    if (n == 0U) {                        /* 线没了 -> 直行前提不成立 */
      printf("H8,ABORTIN,%d,%ld\r\n", (int)n, (long)(dist - h8.d0));
      h8.yaw_fin = gyro_angle;
      h8.state   = H8_TRACE;
      h8.turn    = 0.0f;
      return 0U;
    }
    h8.turn = H8_HOLD_KP * (gyro_angle - h8.yaw_hold);
    if (h8.turn >  H8_TURN_MAX) h8.turn =  H8_TURN_MAX;
    if (h8.turn < -H8_TURN_MAX) h8.turn = -H8_TURN_MAX;
    if ((dist - h8.d0) >= (int32_t)H8_IN_MM) {
      h8.yaw_fin = gyro_angle;
      h8.state   = H8_TRACE;
      h8.turn    = 0.0f;
      printf("H8,TRACE,%ld,%d\r\n", (long)(dist - h8.d_fin), (int)n);
      return 0U;
    }
    return 1U;

  case H8_TRACE:
    dy = gyro_angle - h8.yaw_fin;
    if (dy >= c->exit_deg
        || (dist - h8.d_fin) >= (int32_t)H8_MAX_MM) {
      /* 压根没绕起来（偏航远不到一圈）-> 入口认错了。
       * 这时再做"右转 45° + 强制直行 300 mm"只会把车推下赛道
       * （38.txt：TRACE 段偏航在 -150/-366 之间摆，实际车在 Z 字弯里）。
       * 直接结束这一格、交还 PD，让车继续正常巡线。 */
      if (dy < H8_MIN_TRACE_DEG) {
        printf("H8,NOCIRCLE,%d,%ld\r\n",
               (int)(dy * 10.0f), (long)(dist - h8.d_fin));
        h8.state     = H8_IDLE;
        h8.turn      = 0.0f;
        isl_ret_dist = dist;
        Island_Advance(now, "H8NC");
        return 0U;
      }
      h8.yaw_t = gyro_angle;
      h8.d0    = dist;
      h8.state = H8_TURN_OUT;
      h8.turn  = 0.0f;
      printf("H8,OUT,%d,%ld\r\n", (int)(dy * 10.0f), (long)(dist - h8.d_fin));
      return 1U;
    }
    return 0U;

  case H8_TURN_OUT:
    dy = gyro_angle - h8.yaw_t;
    if (dy >= (H8_TURN_DEG - H8_TURN_TOL)) {
      h8.yaw_hold = gyro_angle;
      h8.d0       = dist;
      h8.state    = H8_STR_OUT;
      h8.turn     = 0.0f;
      printf("H8,TURNOUT,%d\r\n", (int)(dy * 10.0f));
      return 1U;
    }
    h8.turn = -H8_TURN_KP * (H8_TURN_DEG - dy);
    if (h8.turn < -H8_TURN_MAX) h8.turn = -H8_TURN_MAX;
    return 1U;

  case H8_STR_OUT:
    /* 同上：线没了或偏差过大，就不该继续锁航向直行 */
    if (n == 0U) {                        /* 线没了 -> 直行前提不成立 */
      printf("H8,ABORTOUT,%d,%ld\r\n", (int)n, (long)(dist - h8.d0));
      h8.state     = H8_IDLE;
      h8.turn      = 0.0f;
      isl_ret_dist = dist;
      Island_Advance(now, "H8AB");
      return 0U;
    }
    h8.turn = H8_HOLD_KP * (gyro_angle - h8.yaw_hold);
    if (h8.turn >  H8_TURN_MAX) h8.turn =  H8_TURN_MAX;
    if (h8.turn < -H8_TURN_MAX) h8.turn = -H8_TURN_MAX;
    if ((dist - h8.d0) >= (int32_t)H8_OUT_MM) {
      printf("H8,DONE,%ld,%d\r\n", (long)(dist - h8.d_fin), (int)n);
      h8.state     = H8_IDLE;
      h8.turn      = 0.0f;
      isl_ret_dist = dist;
      Island_Advance(now, "H8DONE");
      return 0U;
    }
    return 1U;

  default:
    h8.state = H8_IDLE;
    h8.turn  = 0.0f;
    return 0U;
  }
}

static uint8_t Island_Step(uint8_t n, uint32_t now, int32_t dist, float turn_now)
{
  (void)turn_now;

  /* 换格了就记下这一格的起点里程。两套兜底（时间 ISL_PHASE_MS / 里程 ISL_HEX_MAX_MM）
   * 都从这一格里程算起，和 Island_Advance 里的 isl_t_phase 是同一个时刻。 */
  if (isl_plan_last != plan_idx) {
    isl_plan_last  = plan_idx;
    isl_d_phase    = dist;
    isl_force_used = 0U;       /* 换格了：新的一格可以再用一次里程强制进岛 */
  }

  /* 大套小专用流程：整格交给 Isl_BigSmall，不经过下面的通用状态机 */
  if (Island_Cfg()->kind == ISLK_BSS) {
    return Isl_BigSmall(n, now, dist);
  }

  /* 第 8 格：30° 切向六边形专用流程 */
  if (Island_Cfg()->kind == ISLK_H30) {
    return Isl_Hex30(n, now, dist);
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
      /* 方形：每个方形需要穿过 2 次 T 口（进 + 出），凑齐才前进一格 */
      if (Island_IsSquare()) {
        if (sq_events < 250U) sq_events++;
        if (sq_events >= 2U) {
          sq_events = 0U;
          isl_d_t2  = dist;              /* 强制进岛的里程基准 = 第二个 T 口 */
          Island_Advance(now, "SQOK");
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
  if (!Island_IsSquare() && isl_state == ISL_NONE
      && (dist - isl_d_phase) >= (int32_t)ISL_HEX_MAX_MM) {
    printf("ISL,HEXOVER,%ld,%d,%04X\r\n",
           (long)(dist - isl_d_phase), (int)n, (unsigned)line_raw);
    isl_cfm        = 0U;
    isl_block_dist = 0;
    isl_out_arm    = 0U;
    isl_err_off    = 0.0f;
    Island_Advance(now, "HEXOVER");
    return 0U;
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
        printf("ISL,IN,%s,%d,%04X,%ld\r\n", Island_Cfg()->name, (int)n, (unsigned)line_raw, (long)dist);
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
  if (Island_Cfg()->force_mm != 0U
      && !isl_force_used               /* 本格只强制一次：否则 FABORT -> 350 ms 冷却
                                        * -> 里程仍在阈值后 -> 立刻重进，无限循环，
                                        * 每次都把 yaw0 重置，偏航永远累不起来 */
      && isl_state == ISL_NONE         /* 冷却/会话/出岛期间一律不重进 */
      && (dist - isl_d_phase) >= (int32_t)Island_Cfg()->force_mm) {
    isl_force_used = 1U;
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
  /* 大套小的转向偏置 + 第 8 格 30° 六边形的强制转向 */
  return bs.turn + h8.turn;
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
  uint8_t  is_bar   = 0U;      /* 1 = 本帧刚确认一根发车/终点竖条 */
  float    thin_before = 0.0f; /* 本帧清零之前的细线累计（竖条判据用） */
  float    yaw_ok  = 0.0f;     /* 每一帧见到线就更新，盲走用它当基准 */
  int32_t  gap_d0 = 0;         /* 断线盲走：丢线瞬间的里程 */
  float    thin_mm = 0.0f;     /* 最近连续"细线"(n=1~3)走了多少 mm，断路判定用 */
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
        thin_before = thin_mm;             /* 先存"清零前"的值，竖条判据要用它 */
        if (n >= 5U)       thin_mm = 0.0f;
        else if (n >= 1U)  thin_mm += (float)(d_now - thin_d_prev);
        thin_d_prev = d_now;
      }

      /* ---------- 1b) 发车/终点门（"工"字竖条） ----------
       * 指纹：单帧整排白（0FFF, n=12），前面是 >=120 mm 干净细线，
       * 后面连续几帧回到细线（说明它"薄"）。
       * 方形 T 口前面是一串宽白，thin_mm 早被清 0，所以不会误认。 */
      {
        int32_t d_now = ENC_Dist_mm() - s0;

        is_bar = 0U;
        if (bar_cand == 0U) {
          if (n >= BAR_FULL_N && thin_before >= BAR_THIN_MM
              && fabsf(err_raw) < BAR_ERR_MAX && !Island_Busy()) {
            bar_cand = 1U;
            bar_wide = 1U;   /* 记"已经在竖条上待了几帧" */
            bar_ok   = 0U;
            bar_mm   = d_now;
          }
        } else if (n >= BAR_FULL_N) {
          /* 还在竖条上：竖条宽约 20 mm，300 mm/s 下要待 6~7 个控制周期，
           * 所以"宽白持续"绝不能当失败 —— 之前这里把候选清掉，
           * 导致 bar_cand 每帧置位/清零来回抖，永远凑不满 3 帧细线，
           * 一根竖条都认不出来（39.txt：20757/21172 条件全满足却没打印 BAR）。 */
          if (bar_wide < 250U) bar_wide++;
          if (bar_wide > BAR_WIDE_MAX) bar_cand = 0U;   /* 太厚 = T 口 */
        } else if (n <= 4U) {                    /* 回到细线 */
          if (bar_ok < 10U) bar_ok++;
          if (bar_ok >= BAR_CONFIRM_N) {         /* 确认是"薄"的竖条 */
            bar_cand = 0U;
            is_bar   = 1U;
            /* 距上一根太近 -> 发车那根，或同一道门的第二根，都忽略 */
            if ((d_now - last_bar) >= (int32_t)BAR_GAP_MIN_MM) {
              printf("BAR,%ld,%d,%ld\r\n",
                     (long)d_now, (int)last_bar, (long)(d_now - last_bar));
              last_bar = d_now;
              if (lap_cnt < 250U) lap_cnt++;
              /* 最后一圈的那根（门的右根）确认 -> GATE_HALF_MM 后停在两根之间 */
              if (lap_cnt >= (uint8_t)FOLLOW_LAPS && stop_mm == 0) {
                stop_mm = d_now + (int32_t)GATE_HALF_MM;
                printf("GATE,STOP,%ld,%ld\r\n", (long)d_now, (long)stop_mm);
              }
            }
          }
        } else {
          /* n = 5~10：竖条的边缘过渡，既不算宽也不算薄，保持候选 */
        }
        if (is_bar) t_bar = now;
      }

      /* 盲走（断路/找线）期间冻结环岛状态机：
       * 断路有 650 mm，盲走时里程照涨，Island_Step 里的 ISL_HEX_MAX_MM(400)
       * 会在半路打 ISL,HEXOVER 把计划表推走一格，之后所有环岛都对不上。 */
      isl_now = (ISL_ENABLE && t_rcv == 0U)
                  ? Island_Step(n, now, ENC_Dist_mm() - s0, turn_prev) : 0U;

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
                        ? ((uint32_t)(now - t_rcv) > RECOVER_MS)
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
          /* 竖条（发车/终点门）不算宽区：整排白只是"横穿一条竖线"，
           * 不是弯道，不能减速 —— 发车区要直接直行通过。
           * bar_cand 也要算进去：整排白那一帧 is_bar 还是 0（要等确认），
           * 但那时已经会把 near_wide 置 1，会白减一次速。 */
          if (is_bar || bar_cand) {
            near_wide = 0U;
          } else if (n >= WIDE_NEAR_N) {
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

      /* ---------- 3b) 终点前刹车 ----------
       * stop_mm 由发车/终点门（竖条）确认后设定，是"两根竖条中间"。
       * 必须在到达之前减速：Motor_Set(0,0) 是立即断电，
       * 300 mm/s 会滑行 50~100 mm，满速切电会冲过左根。 */
      if (stop_mm != 0 && t_rcv == 0U) {
        int32_t left_mm = stop_mm - (ENC_Dist_mm() - s0);
        if (left_mm <= (int32_t)BRAKE_MM) {
          float vb = BRAKE_V;
          if (left_mm > 0) {
            vb = BRAKE_V * ((float)left_mm / (float)BRAKE_MM);
          } else {
            vb = 0.0f;
          }
          if (vb < MIN_CREEP_V && left_mm > 0) vb = MIN_CREEP_V;
          if (v_tgt > vb) v_tgt = vb;
        }
      }

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

      /* ---------- 7) 圈数判定 ----------
       * 里程到了但还在环岛流程里时不能停 —— 视频①要求每圈都把环岛走完。
       * 1.txt 第二圈实例：12876 进大环岛，13601 就被 LAPS,DONE 截断，
       * 环岛只走了 725 mm 就断在半路。
       * 而且两圈的大环岛位置本身差 1008 mm（第一圈 5100、第二圈 12876），
       * 靠调 FOLLOW_LAP_MM 保证不了，只能靠状态判断。 */
      /* ---- 竖条门：精确停车（停在两根竖条之间） ---- */
      if (stop_mm != 0 && (ENC_Dist_mm() - s0) >= stop_mm && !Island_Busy()) {
        printf("LAPS,DONE,BAR,%ld\r\n", (long)(ENC_Dist_mm() - s0));
        break;
      }
      /* 兜底：冲过停车点一个门宽就立刻停，别压到左根 */
      if (stop_mm != 0
          && (ENC_Dist_mm() - s0) >= stop_mm + (int32_t)GATE_WIDTH_MM) {
        printf("LAPS,OVER,%ld\r\n", (long)(ENC_Dist_mm() - s0));
        break;
      }
      /* 纯里程兜底：竖条一直没认出来时才生效 */
      if (stop_mm == 0 && (ENC_Dist_mm() - s0) >= goal_mm && !Island_Busy()) {
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
