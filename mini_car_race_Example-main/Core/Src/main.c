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
/* 负压风扇（无刷 + 电调，PA11 出 50 Hz PWM）
 *   1 = 允许起负压；0 = 整场都不转（PA11 只维持 1.0 ms 零油门信号）
 *
 * 【11/24 改成"发车才起、停车即关"】上电只把 PA11 拉到 1.0 ms 停用信号，
 * 风扇不转；按键发车时才走电调解锁序列（0.9 ms 保持 2 s -> 2.0 ms 全速），
 * Follow_Line_Run() 一返回立刻回 1.0 ms 关风扇。这样：
 *   · 待发对正时地面没有被吸住，Align_Hold 的原地转向还能动；
 *   · 不在场上时风扇不转，不耗电、不吸灰、不吵；
 *   · 代价是按键后约 2.4 s 才真正起步（要等电调解锁窗口）。
 *
 * 【两条硬约束】
 *   ① 陀螺零偏必须在风扇静止时标（振动污染）：上电流程里 Gyro_CalibBias()
 *      在风扇启动之前；风扇全速之后的那一刻再补一次 Gyro_RebaseBiasFan()
 *      把"带风扇"的直流偏置换上去，否则长距离积分会漂。
 *   ② 角度清零必须放在风扇全速之后（起转振动会污染积分），顺序不能换。
 *
 * 【当前状态：11/24 18:4x 已开启】按键发车时起的完整时序：
 *   按键 → (300 ms 等手离开) → FAN,START
 *        → 电调认 0.9 ms 低油门 **2 s**（硬件协议，删不掉）→ FAN,ON（2.0 ms 全速）
 *        → FAN,GB,<零偏×10>,<σ×10>,<是否采用>（0.3 s 采样，σ>3 会自动重采一次）
 *        → GO（此时才清陀螺角度和里程）→ 跑 → STOP → FAN,OFF
 *   也就是按键到起步约 **2.3 s**（重采的话 2.6 s），这期间车要扶稳、别推。
 * 关闭（宏=0）：上电只打 FAN,IDLE，按键即走，没有这 2 s。 */
#define IF_start_Brushless_motor 1 //是否启动无刷电机，1为启动，0为不启动

/* ==================== 软件盲盒单独调试 ====================
 * 1 = 按 PB14 发车键**直接执行盲盒**：不起负压、不跑两圈，
 *     清一次陀螺角度和里程后就走“停稳 → 自旋 180° → 直行到 T 口”。
 *     用来单独调自旋角度、直行速度、T 口判据，不用每次跑完两圈。
 * 0 = 正常比赛模式（循迹两圈，停车后由发车区状态机接盲盒）。
 * ⚠ 比赛/跑两圈前务必改回 0，否则发车键不会跑赛道。 */
#define BB_TEST_MODE        0       /* 【11/26】盲盒已单独调通（116.txt：自旋准、T 口 100mm、不再倒冲），
                                     * 改回 0 继续跑两圈赛道 */
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
#define ESC_START_PULSE_SLOTS 9U         /* 0.9 ms：电调解锁窗口里的低油门 */
#define ESC_RUN_PULSE_SLOTS   20U        /* 2.0 ms：负压全速 */
#define ESC_STOP_PULSE_SLOTS  10U        /* 1.0 ms：零油门，风扇停但信号还在（电调不会掉解锁） */
#define ESC_START_PERIODS     100U       /* 100 x 20 ms = 2 s 解锁窗口 */
#define FAN_START_TMO_MS    3000U        /* 起转兜底超时：DMA 中断没跑起来时不要死等 */

static uint32_t esc_dma_table[ESC_PULSE_SLOTS];
static uint32_t esc_high_slots;    /* high time currently stored in the table */
static volatile uint32_t esc_period_count;
static volatile uint8_t  esc_arming;   /* 1 = 正在跑 0.9 ms -> 全速 的解锁序列 */
static volatile uint8_t  fan_on;       /* 1 = 风扇应当是全速的 */

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
   handler only counts frames of the ESC arming window (0.9 ms for 2 s)
   and then switches to full speed or back to zero throttle.
   只在解锁序列期间开中断：序列走完就关（Fan_Start 里会重新打开）。 */
static void ESC_DmaTransferComplete(DMA_HandleTypeDef *hdma)
{
  if (!esc_arming)
  {
    __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC);
    return;
  }

  ++esc_period_count;
  if (esc_period_count >= ESC_START_PERIODS)
  {
    /* 100 x 20 ms = 2 s 解锁窗口走完：按 fan_on 决定继续全速还是回零油门。
     * 注意中途被 Fan_Stop() 打断的情况由 Fan_Stop 自己回写脉宽，这里不再管。 */
    esc_arming = 0U;
    ESC_SetPulseWidth(fan_on ? ESC_RUN_PULSE_SLOTS : ESC_STOP_PULSE_SLOTS);
    __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC);
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
/* 停车后的“软件盲盒”：在 Follow_Line_Run 末尾调用，函数定义在后面；
 * 里面要关负压，Fan_Stop 也定义在后面。 */
void Fan_Stop(void);
static void BlindBox_Run(int32_t d_stop);
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

    if ((uint32_t)(now - t_ctl) >= 5U) {          /* 同 CTL_MS，定义都在下面 */
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

/* ============ 开负压之后重测零偏（只在 IF_start_Brushless_motor=1 时调用）============
 * 【为什么必须重测】Gyro_CalibBias() 是在风扇静止时采的，得到的是"干净"零偏；
 * 但跑车时风扇一直在转，风扇/机架振动会被陀螺整流成一份直流偏置叠加进来。
 * 偏航角是唯一用来判出岛的积分量（阈值 345°），几 °/s 的残余就会在 20 s 里
 * 积出上百度的假偏航，直接导致环岛提前出岛 / 拐错方向。
 * 所以：风扇全速之后，在同样的静止条件下再采一次，把 gyro_z_bias 换成"带风扇"那一版。
 *
 * 【判据：宁可不用，也不能采错】
 * 关键事实（1.txt 那一趟）：`FAN,ON` 只是软件打印，风扇/电调实际有没有转，
 * 日志看不出来。那趟的 `FAN,VIB,3,40`（σ=4.0）既可能是风扇振动，也可能是
 * 采样时手还扶着车 —— 两种情况的 σ 一样。
 * 所以这里不做"噪声大就放宽门槛"的事：偏置只差 0.3 °/s 顶多让偏航慢漂，
 * 而把"车正在转"误当成偏置会直接毁掉整场（几十 °/s 的假偏航）。
 * 做法：σ <= FAN_GB_SD_MAX(3 °/s) 才采用；否则【重采一次】，仍不行就保留
 * 上电静标那一版并打 FAN,VIB。多发 0.3 s 换一次可信测量，很划算。
 *
 * 死区 gyro_gate2 不动：它是集成前的死区，噪声积分出的随机游走只有 ~0.5°，
 * 远小于 345° 门限的误差量级；真正要命的是偏置。
 * 判读：FAN,GB,<零偏×10>,<σ×10>；FAN,VIB 出现 = 两次采样都不干净，
 *       跑车用的是上电静标偏置（没有风扇补偿），此时别指望 yaw 判据很准。 */
#define FAN_GB_SD_MAX        3.0f    /* 采用偏置的噪声上限（°/s） */

void Gyro_RebaseBiasFan(void)
{
  const int N = 300;                   /* 300 × 1 ms ≈ 0.3 s / 次 */
  float mean = 0.0f, sd = 9.9f;
  uint8_t ok = 0U;

  for (uint8_t try_n = 0U; try_n < 2U && !ok; try_n++) {
    float sum = 0.0f, sum_sq = 0.0f, var;

    for (int i = 0; i < N; i++) {
      float g;

      dodo_BMI270_get_data();
      g       = GYRO_SIGN * BMI270_gyro_transition(BMI270_gyro_z);
      sum    += g;
      sum_sq += g * g;
      HAL_Delay(1);
    }

    mean = sum / (float)N;
    var  = (sum_sq / (float)N) - mean * mean;
    if (var < 0.0f) var = 0.0f;
    sd   = sqrtf(var);

    if (sd <= FAN_GB_SD_MAX) {
      gyro_z_bias = mean;              /* 采样干净：换成"带风扇"的零偏 */
      ok = 1U;
    }
  }

  if (!ok) {
    printf("FAN,VIB,%d,%d\r\n", (int)(mean * 10.0f), (int)(sd * 10.0f));
  }

  gyro_t_ms   = HAL_GetTick();
  gyro_last_g = 0.0f;
  printf("FAN,GB,%d,%d,%d\r\n", (int)(gyro_z_bias * 10.0f), (int)(sd * 10.0f), (int)ok);
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

/* ==================================================================
 *   【11/19 22:00 提速 ×3 -> 11/19 23:40 回落到 800】直道 350 -> 800 mm/s
 * ==================================================================
 * 提速分四类量纲处理，混着改就是之前"改一个坏三个"的来源：
 *
 *   ① 速度量纲（mm/s）→ 按比例放大（350→1050 是 ×3；现在是 350→800 = ×2.29）
 *      目标速度、转向环输出(KP/KI/KP2/I_MAX)、差速限幅、单轮下限、
 *      各段限速(宽区/急弯/T口/找线/盲走)、岛内速度、大套小偏置
 *      理由：同样的偏差要产生同样的【曲率】。差速与速度的关系是
 *            κ = 2·turn /(L·v)，v 放大多少倍 turn 也要放大多少倍，
 *            否则同样的 err 只能拐出几分之一的弯 —— 表现就是"直道飞快、
 *            进弯直接冲出去"。
 *
 *   ② 时间量纲（ms / 帧数）→ 按同比例缩小
 *      （350→1050 时 ÷3；现在 350→800 时 ÷2.29，即上一版的 ×1.31）
 *      丢线容忍、找线时长、各类冷却与超时、宽区解除、门"宽白"帧数
 *      理由：这些阈值当初都是按"走多远"定的。速度放大后同样的毫秒数
 *            代表更多里程，不缩就会"晚好几倍距离才反应"。
 *
 *   ③ 距离(mm) / 角度(°) 量纲 → 一律不动
 *      环岛几何(fork_mm / exit_deg / out_cross_mm / entry_deg / entry_off)、
 *      门指纹(BAR_THIN_MM / PAIR_* / BAR_LAP_TOL_MM)、里程窗口、
 *      FOLLOW_LAP_MM、GAP_MM、ISL_FAKE_MM
 *      理由：赛道没变，车走的还是那条路径。
 *
 *   ④ 车体/电机属性 → 不动
 *      DEAD_L/DEAD_R、FOLLOW_KP_SPD/KI_SPD、FOLLOW_PWM_START、RUN_MODE 0/1 的
 *      STRAIGHT_* / SPIN_*
 *      理由：死区是机械阻尼，"每 mm/s 要多少 PWM"是电机特性，
 *            跟目标速度无关；放大只会让起步窜车、速度环发散。
 *
 *   ⑤ 有意不动的两个增益：FOLLOW_KD / FOLLOW_GKD
 *      de 和 gyro_z 都是"每帧变化量"，速度放大时它们自己就放大，
 *      等效增益自动跟上；再乘一次就是过阻尼空转。
 *
 *   ★★ 为什么最后停在 800 而不是 1050 ★★
 *      控制周期 10 ms 是硬的，所以【采样步长】就是 速度 × 10 ms：
 *        350 mm/s → 3.5 mm/帧   12 mm 的 T 口横线跨 3.4 帧   → 稳（当初就是在这调通的）
 *        800 mm/s → 8.0 mm/帧   横线跨 1.5 帧                → 勉强
 *       1050 mm/s → 10.5 mm/帧  横线跨 1.1 帧                → 抽奖
 *      实际"整排全白"的那一瞬间比 12 mm 还短（车头略斜，12 路不同时压亮），
 *      只有 7~8 mm。于是漏采率 ≈ 1 − 窗口/步长：
 *        800  → 约 6%     方块整格成功率 ≈ 0.94² ≈ 88%
 *        1050 → 约 29%    方块整格成功率 ≈ 0.71² ≈ 50%
 *      56.txt 实测正好是后者：8 个方块只成功 5 个，其余被 2500 mm 兜底跳过。
 *      所以 1050 不是"跑不快"，是**采样到极限了**。
 *      真要跑到 1000 以上，必须把控制周期降到 5 ms（见 FOLLOW_DT 那一段）。
 *
 *      凡是 <20 mm 的图案判据（如 fork_mm）属于第 ③ 类，不随速度改；
 *      但提速后一帧只有 8 mm，它们的"抗噪帧数"会掉，调参时按帧数折算：
 *        16 mm ≈ 2 帧 @800   48 mm ≈ 6 帧 @800
 * ================================================================== */

/* ==================== 一、赛道与总时限 ==================== */
/* 【11/21 11:31 那趟：21380 是错的，已改回 20580 —— 实测发车线就在 20310】
 * 那一趟日志里有一条决定性的证据：
 *   20310  BAR,HIT,20310,36        ← 发车线竖条，指纹合格（后面 36 mm 内回到细线）
 *         然后 BAR,OUT              ← 被 21380±500=[20880,21880] 拒掉
 *   22881  LAPS,DONE               ← 于是只能靠里程兜底（21380+1500）停车
 * 也就是说真实发车线在 20310，落在原来的 [20080,21080] 里，而我按"短了 800mm"
 * 改成 21380 正好把它排除在窗口外了 —— 那次"在波浪形中间停下"不是里程基准偏小。
 * （那次真正的现象是：六边形 ISL,SEX,274.7°/691mm 正常出岛后，出锁 n=0 又继续
 *   右转了 171°（gyro -1953 -> -237），所以现场看起来"绕了两圈"。）
 * 结论：这个数不要凭感觉加减，必须用日志里的 BAR,HIT 里程来标。
 * 标定方法：看 `BAR,HIT,<里程>` 里落在发车区那两根，中点就是这个值。
 * 若下一趟出现 BAR,OUT 且 `本圈里程` 在 20310±700 以内 → 窗口偏窄，
 * 调 BAR_LAP_TOL_MM 500 -> 800，别动这个数。 */
#define FOLLOW_LAP_MM      23000     /* 单圈里程 mm（两根门竖条中点）——
                                      * 【11/24 20:4x 起它只是"第一圈的种子"】，
                                      * 成对成功一次之后窗口改用实测值 ±12.5%（见 Lap_Gate）。
                                      *
                                      * 【11/24 19:4x 23386 -> 22600 -> 23000：单圈里程本身会变】
                                      * 连续几趟实测"第一根/第二根竖条里程"：
                                      *   89.txt（无负压，900 速）      22794 / 23218   中点 23006
                                      *   90.txt s1（负压，门口偏线）    22814 / 23491   中点 ~23150（间距 677 那次）
                                      *   1.txt v2（负压）               21747 / 22173   中点 21960
                                      *   91.txt（负压）                 21683 / 22108   中点 21896
                                      *   93.txt（负压，1350 速）        25742 / 25748   中点 25745  ← 提速后 +17%
                                      * 结论：里程 = 轮子实际滚过的路程，抓地越好越短、速度越高越长，
                                      * 它就不是个几何常量。93.txt 那趟 +17% 直接把 25 根竖条全部挤出窗口。
                                      * 所以现在：这个常量只给第一圈开窗（±BAR_LAP_TOL_MM=2000，
                                      * 覆盖 21k~25k），从第二圈起用"上一圈实测值 ±12.5%"。
                                      * 标定方法：看 `LAP,<圈>,<第二根里程>,<实测单圈>,<下圈容差>`。 */
#define FOLLOW_LAPS            2     /* 跑两圈：
                                      * 两圈都使用同一对竖条门，都必须成对确认；
                                      * 第 1 圈成对后直行通过，Lap_Reset 进入第二圈；
                                      * 第 2 圈成对后设 stop_mm，停在两根竖条中点（可加 STOP_MID_OFFSET_MM），
                                      *   车主体落在发车区内。
                                      *   允许光电管离开发车区，车主体仍在两根竖条之间。 */
#define FOLLOW_TIMEOUT_MS 180000U    /* 【80 s -> 180 s（11/19 22:15）】
                                      * 实测：×3 之后单圈 21180 mm 只要 26 s（55.txt 两趟
                                      * 分别 26.1 / 26.2 s），所以 80 s 本来够用 ——
                                      * "提前停车"的真凶是里程兜底，不是这个超时。
                                      * 但环岛里多绕几圈、找线多试几次都可能吃掉几十秒，
                                      * 而超时一触发就是硬停车。宁可让它多跑，也别在赛道
                                      * 中间把它掐掉：留 180 s ≈ 7 倍余量。
                                      * （原注释：240 s 是两圈的量，2×20450/250 ≈ 164 s） */

/* ==================== 二、速度（350/210 -> 800/480，×2.29）====================
 * 循迹速度的唯一来源是
 *     v_tgt = V_FAST - (V_FAST - V_SLOW) * (|e| / 7)
 *
 * 【和前三步不同，这一步不再是"只动两个端点"】
 * 前三次提速（+10% 级）可以只改 V_FAST/V_SLOW，因为 10% 的偏差由
 * 转向环的余量消化得掉。×2 以上不行：转向输出是差速，同样的偏差要拐出
 * 同样的半径，turn 必须同比放大（κ = 2·turn/(L·v)），否则弯道半径成倍变大。
 * 所以所有"速度量纲"的出口一起放大，逐条对照见文件头那张规则表。
 *   接近宽区 ISL_NEAR_V    150 -> 340   （42% 的比例不变）
 *   急弯预判 CORNER_V      120 -> 270
 *   过 T 口 ISL_CROSS_V    150 -> 340
 *   岛内 v_island          150~300 -> 340~680
 *   找线 RECOVER_V         160 -> 360
 *   断路 GAP_V             250 -> 570
 *   差速限幅 TURN_MAX      450 -> 1020，单轮下限 -200 -> -460
 *   转向环 KP 150 -> 340、KP2 40 -> 90、KI 0.8 -> 1.8、I_MAX 60 -> 140
 *   不动的只有 KD / GKD（理由见各自那一行）
 *
 * 【提速历程：300 -> 320 -> 335 -> 350 -> 1050(已退) -> 800，累计 +129%】
 *   320/190：跑通了断路、4 个方形、大套小
 *   335/200：跑通了完整一圈 + 停在发车区（CFG,V,335,200,25 那趟）
 *   350/210：11/19 18:00 版本，是把这套 PD 调通的基线
 *   1050/630：11/19 22:00。实测只跑出"8 个方块成功 5 个"——不是 PD 不行，
 *            是 10 ms 采样 + 12 mm 横线撞上了采样极限（漏采率约 29%）。
 *   800/480：每帧 8 mm，漏采率降到约 6%，方块整格成功率从 ~50% 提到 ~88%。
 *
 * 【11/24 10:50 现场要求提到 900（每帧 9 mm）】
 *   采样步长从 8 mm 回到 9 mm，T 口（12 mm 宽）漏采率会从 ~6% 抬到 ~15~25%
 *   （1050 那次 10.5 mm/帧 实测漏采 29%）。所以这一版的验收重点是：
 *   每个 T 口是否还成对（SQ,FIRST + SQ,PAIR / T,IN + T,OUT），有没有某一格只剩一半。
 *   若漏采明显：① 退回 800/480 只用一行改动；② 或者上 5 ms 控制周期
 *   （900 mm/s × 5 ms = 4.5 mm/帧，比现在还细一半，那才是 900 的正确做法，
 *    但 5 ms 要同步把"帧数"判据翻倍：BAR_WIDE_MAX / BAR_CONFIRM_N / GAP_CONFIRM_N）。
 *
 * ⚠️ 800 之后最可能的失败模式，按概率排序：
 *    1) 偶发漏采 T 口 —— 8 mm/帧 仍略大于 7~8 mm 的"全白窗口"，
 *       大约每 16 个 T 口漏 1 个。查日志：某一格只有 SQ,FIRST 没有 SQ,PAIR，
 *       然后 ISL,HEXOVER,2500 跳过。对策：速度再降到 700（每帧 7 mm，漏采率≈0）。
 *    2) 弯道冲出去 —— 电机给不出 1020 的差速（外轮要 1820 mm/s）。
 *       查日志：uL/uR 贴 6500 且弯道里 vL/vR 差不到 800。
 *       对策不是降 KP，是降速度。
 *    3) 六边形进不去 —— 见格 2 / 格 8 那两行（11/19 23:40 已按"分叉实际 5~6 路"
 *       和"入口偏置太弱"两处改过）。
 *
 * ⚠️ 回退办法（先跑一趟看基线，别一次调一圈）：
 *     git stash            # 保留本版
 *     git checkout -- .    # 回到 HEAD 那版
 *   或者只退速度：把 FOLLOW_V_FAST/V_SLOW 改成 700/420，
 *   其余保持本版（其余参数是"比例量"，700 下仍然自洽，只是更保守）。 */
/* ==================== 【11/24 20:4x 提速 ×1.25：900 -> 1125 mm/s】====================
 * 来历：先试过 ×1.5（1350），93.txt 表明"不是电机不够，是里程判据被冲垮"：
 *   · PWM 贴 6500 的帧 0%，实测/目标 = 1.01  → 电机完全跟得上；
 *   · 但单圈里程从 21.9k 涨到 25.6k（+17%，打滑随速度上升），
 *     25 根竖条全部被里程窗口拒掉 → 门不成对 → 没有 Lap_Reset → 计划表卡死。
 * 所以这次只提到 ×1.25，并把里程窗口改成自适应（第一圈常量、之后按实测 ±12.5%）。
 *
 * 规则沿用 11/19 那次 ×2.29（就写在每个常量旁边的注释里）：
 *   · 速度类、与速度同量纲的增益（KP/KP2/KI/I_MAX/GAP_KP/GAP_TURN_MAX/CORNER_DE）  ×1.25
 *   · 差分/角速度类阻尼（FOLLOW_KD / FOLLOW_GKD / HEXT_ENTRY_KD）  **不动**
 *     —— de 与 gyro_z 自己随速度涨，等效增益自动放大
 *   · 时间类 *_MS ÷1.25；帧数类 ÷1.25（>=2）；空间类与硬件类不动
 * 想换别的速度系数：改完这个注释下面那批数就够，比例关系会自动保持；
 * 或者直接跑 tools 里的换算（_rescale.py 1.25），它按同一套规则重算全部参数。
 * ⚠️ 提速后必须重新看 `LAP` 行第 4 列（实测单圈）—— 它会随速度/负压漂。
 */
#define FOLLOW_V_FAST       1125.0f   /* 正中时 mm/s。【900 ×1.25，11/24 现场降速】
                                      * 800 = 350×2.29；900 是在 800 基础上再 ×1.125。
                                      * ⚠️ 每帧 5.6 mm（5 ms 节拍），T 口 12 mm 宽，
                                      *    采样余量只剩约 2 帧 —— 见上面提速历程里的验收重点。 */
#define FOLLOW_V_SLOW       675.0f   /* 偏差最大(±7°)时 mm/s。【540 ×1.25，同比例 0.6】
                                      * 保持 (FAST-SLOW)/7 的减速斜率不变：
                                      * (900-540)/7 = 51.4，(800-480)/7 = 45.7，
                                      * 再往前 350 时是 20 —— 同一条比例线。 */
#define FOLLOW_RAMP_MS        700U   /* 起步升速时间 ms。【300 ×2.33 = 700】
                                      * 保持升速【速率】不变：800/0.7 = 350/0.3
                                      * = 1.14 m/s²。若仍用 300 ms，起步加速度 2.7 m/s²，
                                      * 轮子打滑、编码器里程与真值脱钩，环岛判据全错 */
#define FOLLOW_V_MIN      (-575.0f)  /* 单轮目标下限（负=允许反转）。【-200 ×2.29 = -460】
                                      * 内轮反转比例保持 -460/800 = -200/350 = -0.575
                                      *
                                      * 【11/26 从 -1000 退回 -575：114.txt 掉头】
                                      * 那一版 RECOVER_TURN=1100 / V_MIN=-1000，第二圈出小六边形、
                                      * 过下一个直角后车连续几次丢线恢复都往左多转一点，
                                      * 航向从 -637° 一路漂到 -737°；34.6k 再丢线时 PD 用上一帧
                                      * 的 +2.7° 误差硬转，430 mm 内转了 190°，接回的是来时的线，
                                      * 于是“掉头往回跑”。这两个值先退回 50.7 s 那版。 */

/* ==================== 三、转向环 PD(+I) ====================
 * 按官方教程的"方法二（双 PD）"组织：
 *   转向输出 = KP*e + KP2*e*|e| + KD*de + KI*∫e − GKD*gyro_z
 *   其中 KP2 只在大偏差时额外加力，GKD 用陀螺仪抑制摆动。
 * 两项默认 0，等于原来的纯 PD 行为；调参时按教程顺序打开：
 *   ① 调 KP 到"能回正但会摆" ② 加 GKD 消除摆动（会变慢）
 *   ③ 加 KP2 把速度补回来（又快又不摆）
 */
#define FOLLOW_KP           425.0f   /* mm/s per °。【150 ×2.29 = 340】
                                      * 这是提速后最关键的一个数：turn 是差速(mm/s)，
                                      * 同样的 1° 偏差要让车拐出同样的半径，v ×3 则
                                      * turn 也必须 ×3，否则弯道半径变成 3 倍直接飞出去。
                                      * 若跑起来"直道 OK、弯道摆"→ 先降这个(400/350)。
                                      *
                                      * 【11/25 第四轮：425 -> 455 -> 回退 425】
                                      * 99d 实测 455：p90|e| 4.27 -> 4.20（几乎没动），
                                      * 横摆 0.32 -> 0.36、Δv p90 761 -> 775（都变差）→ 回退。
                                      * 原因想明白了：|e|>3 时 KP*e 已经超过 TURN_MAX(1275)，
                                      * 那 10% 的大偏差帧本来就在限幅上跑，P 再大也被削掉。
                                      * 所以真正缺的是"转向权限"（TURN_MAX），不是 P。 */
#define FOLLOW_KI             2.25f   /* mm/s per °·s。【0.8 ×2.29 = 1.8】
                                      * 跟 KP 同比例，保持 T_i = KP/KI = 187 s 不变，
                                      * 积分作用的"时间尺度"才不会因为提速而变味。 */
#define FOLLOW_KD            36.0f   /* mm/s per °（每 10 ms 差分，等效 ×100）。
                                      * 【×3 时刻意不动】de 是"每帧变化量"，
                                      * 速度 ×3 时 de 自己就 ×3，等效 KD 自动变成 54，
                                      * 正好是几何等效值；再乘 3 就过阻尼了。 */
/* 【11/25 112.5 -> 60：调参第一轮】
 * 用 94/95/96/97 四份日志反推转向环各项占比：P 37% / P2 31% / GKD 32% / D≈0，
 * 而差速被 TURN_MAX(1275) 限幅的时间占 **20~37%**。
 * 顶到限幅的正是这一项：112.5×e×|e| 在 e=3.7 时就是 1540（>1275），
 * 于是进"饱和 → 大幅回正 → 再偏"的极限环（e 每 137mm 翻一次符号）。
 * 先把它砍到 60（e=3.7 时 = 821，不再单靠它顶死），看饱和率与横摆频率是否下降。
 * 这也让它成为"上电默认值"：不用每次复位后重新下发 KP2=60。
 * 想回到原来的激进行为，串口发 KP2=112.5 即可（RAM 生效，掉电回 60）。 */
#define FOLLOW_KP2           112.5f   /* 非线性项：e*|e| 系数（大偏差加力）
                                      * 【11/25 14:55：80 -> 112.5，回退到"上负压可完赛第二版"(43aecb2)】
                                      * 现场反馈当前版本"几个地方会飞出赛道"。
                                      * 与可完赛版逐项对比后，正常循迹只差三个参数，这是其一：
                                      *   |e|=1.5 时 turn 由 891 降到 818、|e|=2 时 1275 降到 1170，
                                      *   弯道前段（|e|1~2）回正力弱了 30~105，车先往外飘；
                                      *   等 |e|≥2.5 才突然给到 1450（见 TURN_MAX），一慢一猛最易冲出弯外。
                                      * 下面这些是当时调 80 的依据，留档：
                                      * 【11/25 第三轮：60 -> 80】
                                      * 99a.txt（KIS=0.15）跑完两整圈、cost 1.138、转向饱和 0%，
                                      * 但 p90|e| 仍是 4.27（KP2=112.5 那轮是 3.71）——
                                      * 说明砍到 60 伤了大偏差的回正能力。
                                      * 速度环已经用 KIS=0.15 压住了轮速打摆（Δv p90 1151 -> 730），
                                      * 所以这里把 KP2 回补一半到 80（e=3.7 时 1095，接近但不再顶死 TURN_MAX）。 */
#define FOLLOW_GKD            3.0f   /* 陀螺仪阻尼：gyro_z 系数（抑制摆动）。
                                      * 【×3 时刻意不动】理由同 KD：环岛里同样的
                                      * 路径，gyro_z 读数随速度 ×3，等效阻尼自动 ×3。 */
#define FOLLOW_I_MAX        175.0f   /* 积分限幅 mm/s。【60 ×2.29 = 140】
                                      * 它和 turn 同量纲，不放大就等于积分永远出力不足。 */
#define FOLLOW_TURN_MAX    1275.0f   /* 差速限幅 mm/s。【450 ×2.29 = 1020】
                                      * 【11/25 14:55：1450 -> 1275，回退到可完赛版 43aecb2】
                                      * 1450 时满速(1125)内轮目标 -325、外轮 2575 mm/s，
                                      * 电机根本给不出，命令与实际脱节；与 KP2 回退配套。
                                      * 留档原注释：
                                      * 几何等效值。注意它在物理上可能给不出去：
                                      * v_tgt=800 时外轮要 800+1020 = 1820 mm/s，
                                      * 电机到不了，Inc_PID 会顶在 PWM_MAX。
                                      * 这是"高速时转向权限被电机上限吃掉"的物理限制，
                                      * 调小 TURN_MAX 只会让弯更大，不能解决它。 */
#define FOLLOW_TURN_SIGN    (-1)     /* 方向开关，已实测 */
#define FOLLOW_ERR_ZERO       0.0f   /* 传感器零点 ° */
#define FOLLOW_TURN_BIAS      0.0f   /* 整车跑偏补偿 mm/s（0 = 关闭） */

/* ==================== 四、每轮速度 PI ==================== */
#define FOLLOW_KP_SPD         1.2f   /* PWM per mm/s */
#define FOLLOW_KI_SPD        0.15f   /* PWM per mm/s 每帧。
                                      * 【11/25 15:05 又改回 0.15（本轮第二次）】
                                      * 昨天回退到 0.4 换来了"大套小交界处抖"，实测对比：
                                      *   KI=0.15：相邻帧 |Δv| 中位 331、p90  999（106.txt）
                                      *   KI=0.40：相邻帧 |Δv| 中位 501、p90 1481（本趟）
                                      * 具体表现（本趟大套小段）：vL/vR 在 -368 ~ +2170 之间来回，
                                      *   4657 帧 vL=-326/vR=2010、6187 vL=-211/vR=1709、
                                      *   7060 vL=-368/vR=2170 —— 一轮反转一轮空转，车在岛里"抖"。
                                      * 抖会顺着 ①小环串进大套小的绕行（本趟小环跑了 3595mm，
                                      * 且 yaw 来回摆 132→-89→-28→-260），也会拖坏六边形入口判据。
                                      * 结论：0.4 是"弯道飞出"与"轮速极限环"之间的错误妥协，回 0.15。
                                      * 原注释：
                                      * 【11/25 14:55：0.15 -> 0.4，回退到可完赛版 43aecb2】
                                      * ⚠️ 这一项当时降 0.15 是为了压"轮速极限环"（98.txt 相邻帧
                                      * |Δv| p90=1151）。回退到 0.4 后如果日志里 vL/vR 又开始在
                                      * 100~1700 之间来回跳、弯道里实际差速跟不上命令，
                                      * 说明极限环回来了 —— 那就单独把这项改回 0.15，不要连带别的。
                                      * 留档原注释：
                                      * 【11/25 0.4 -> 0.15】速度环是增量式 PI（200 Hz）：
                                      * out += KPS*Δe + KIS*e。KIS=0.4 相当于每 1 mm/s 误差
                                      * 每秒加 80 PWM，打滑时冲顶、恢复后过冲 —— 98.txt 里
                                      * 两轮实测速度在 100~1700 之间来回（相邻帧 |Δv| 中位 269、
                                      * p90 1151）就是这个极限环。降到 0.15 后每秒 30 PWM。 */
#define FOLLOW_PWM_MAX     6500.0f   /* 【4200 -> 6500，等于 PWM_MAX 硬上限】
                                      * 速度 ×3 后巡航就可能顶到 4000 以上，
                                      * 4200 会把速度环卡在"永远差一点"，表现是
                                      * vL/vR 到不了 v_tgt、车越跑越慢却查不出原因。
                                      * 若日志里 uL/uR 长时间贴在 6500 而 v 仍不达标，
                                      * 说明电机到极限了 —— 那不是参数问题，别再加。 */
#define FOLLOW_PWM_START    900.0f   /* 【×3 时刻意不动】它是"克服静摩擦起步需要多少
                                      * PWM"，是车体属性，不是速度的函数。
                                      * 乘 3 会让发车第一帧就窜出去（目标速度此刻还是 0）。 */
/* 发车前置（正向偏置）：GO 后给两轮额外 PWM，用来抵消静摩擦和
 * "初始目标速度接近 0"造成的加速迟滞。不改变巡航/转向 PID。
 *
 * 【11/27 改成"大起点 + 线性衰减"】原来恒定 +400 PWM 只给 50 ms：
 * 开负压时车身被吸在地上、静摩擦明显变大，50 ms 往往刚把车顶起来就收掉，
 * 之后立刻掉回 FOLLOW_PWM_START(900) 的巡航基数，看起来就是"起步肉"。
 * 现在：起步瞬间给满 START_PRELOAD_PWM，之后按时间线性衰减，到
 * START_PRELOAD_MS 归零。破静摩擦的力更大，又不会在 50 ms 处
 * 突然掉一截把车"顿"住；衰减段正好盖住 FOLLOW_RAMP_MS 的升速期。
 * 调参：还肉就加大 START_PRELOAD_PWM；若起步瞬间听到轮胎打滑、或者整趟
 * 里程明显偏短（编码器与真值脱钩，环岛判据会跟着错），就把它调小。 */
#define START_PRELOAD_MS    300U
#define START_PRELOAD_PWM   800.0f
#define CTL_MS                 5U    /* 控制周期 ms：900mm/s 下 4.5mm/帧（原 10ms/9mm）*/
#define FOLLOW_DT            0.005f   /* 控制周期 s */

/* ==================== 五、宽区（T 口）处理 ==================== */
#define WIDE_NEAR_N           6U     /* 路数 ≥ 此值：略减速（预备）
                                      * 原来 5 太低：普通直线 n=3、略粗的线 n=4~5，
                                      * 于是直道上也会突然减速一下。抬到 8 后：
                                      *   普通直线(3) / 粗线(4~5) → 不减速
                                      *   环岛入口(6~8)           → 只在最宽处减速
                                      *   T 口(11~12)             → 减速
                                      * 副作用是好的：一旦减速就说明 n≥8，早已超过
                                      * 进岛门槛 ISL_FORK_N(6)，"减速"变成进岛的强信号。 */
#define ISL_NEAR_V        750.0f     /* 接近宽区先压到 600（原 340），给转向留时间。
                                      * 【11/20 试验：340 -> 600】57.txt 里正常弯 n=6~7
                                      * 也进宽区，v_tgt 在 785 和 340 之间反复跳（2.3 倍），
                                      * 先把这个目标抬到 600（800 的 75%），减小阶跃；
                                      * WIDE_NEAR_N 等其它参数不动。
                                      * 真正的 T 口穿越仍由 ISL_CROSS_V(340) 负责，不受这里影响。 */
#define CORNER_V          337.5f     /* 急弯预判触发时的目标速度 mm/s。【120 ×2.29 = 270】 */

/* ==================== 第二格之后 → 第 8 格入口：整段提速（11/25）====================
 * 【为什么要单独提这一段】109.txt 实测（两圈几乎一致）：
 *   PLAN,3/3,HEX,HEXOVER -> HEXT,ENTRY   8.75 m / 9.45 s，平均 925 mm/s
 *   其中 46% 的帧目标就压在 FOLLOW_V_FAST(1125) 上，
 *   而电机 PWM 只用到 6500 的 10~20%（uL/uR 600~1300）—— 明显有余量。
 * 这一段里所有判据（格3~6 方形 T 口、格7 八边形、断路线段）靠的是
 * "T 口锁直行(ISL_CROSS_V) + 接近宽区减速(ISL_NEAR_V) + 急弯预判(CORNER_V)"，
 * 所以：
 *   · 巡航目标 fast/slow 按 FASTSEG_V_GAIN 整体放大；
 *   · 宽区减速只按 FASTSEG_V_NEAR_GAIN 跟一点点，保留 T 口识别裕量；
 *   · 锁直行 875 / 急弯 337.5 / 盲走 712.5 一律不动（这些本身是判据的一部分）。
 *
 * 生效窗口（见 Island_FastSeg）：plan_idx==2（已过第二格小六边形）且
 * hext_done==0（还没进第 8 格），且不在岛内/不在穿 T 口 ——
 * 正好是"第二格 → 离开八边形"这一段。
 * 调法：T 口漏判或丢线变多 -> GAIN 降到 1.08；觉得还能再快 -> 试 1.15~1.18。 */
#define FASTSEG_V_EN         1U
#define FASTSEG_V_GAIN       1.12f   /* 巡航目标 fast/slow 的倍数 */
#define FASTSEG_V_NEAR_GAIN  1.08f   /* 接近宽区减速 ISL_NEAR_V 的倍数 */

/* ---- 急弯预判的两个门限 ----
 * 判据：|e| > CORNER_E && |Δe| > CORNER_DE  → 目标速度压到 CORNER_V
 *
 * 原来这两个数是直接写在函数体里的 3.2f / 0.55f，属于"提速时最容易被漏掉"的
 * 一类参数 —— 它们没有名字，改速度时不会有人想起来检查。
 * 现在提出来，并且说清楚它们的量纲【不一样】：
 *   CORNER_E = 3.2f   err 单位，是"空间量"：车已经偏了 3.2 个单位。
 *                     **不随速度变**。
 *   CORNER_DE = 0.55f 是"每帧的变化量"，而 Δe/帧 = v × 10 ms × d(err)/ds，
 *                     所以它【正比于速度】。同样的弯，速度翻倍 de 就翻倍。
 *                     也就是说：这个数保持不变，速度快了之后急弯预判会更灵，
 *                     而不是更钝（方向上是安全的：宁可早减速）。
 * 11/19 23:40 降到 800 时特意**不改它**，理由：
 *   56.txt 实测（1050 mm/s）CORNER_V 一共触发 65 次，说明这个门限是有效的；
 *   降到 800 后逐帧 de 变成 0.76 倍，触发频率只会略降。若想回到 350 mm/s 当年
 *   的灵敏度，把它抬到 0.55×2.29 ≈ 1.26 —— 但那会让车更晚减速，不建议。
 *
 * 【11/20 61.txt】方框直角前 e 只到 ±2.1~2.6，3.2 根本不触发：
 *   10516 n=8 e=-2.1 v=600 -> 10550 丢线/GAP
 *   10795 n=7 e=-2.6 v=600 -> 10830 丢线/GAP
 *   11064 n=8 e=-2.1 v=600 -> 11105 丢线/GAP
 * 车以 600 进直角，线整排消失，GAP 用 360/500 满舵找回，再进下一个直角，
 * 表现就是"剧烈晃动"。所以 CORNER_E 降到 2.0：e=-2.1 就先压到 CORNER_V=270。 */
#define CORNER_E            2.0f
#define CORNER_DE           0.69f
#define WIDE_REL_MS          46U     /* 宽区减速的解除条件：n 跌回 3 路后还要持续这么久
                                      * 【150 ÷2.29 = 65 ms，同一段物理距离】
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
#define ISL_CONFIRM_MS        0U     /* T 口图案确认时间。【10 -> 0 = 单帧即认账】
                                      *
                                      * 【为什么必须改成 0（11/19 22:15 实测死因）】
                                      * T 口的那根横线只有 12 mm 宽，而控制周期 10 ms：
                                      *   350 mm/s  → 每帧 3.5 mm → 横线跨 3~4 帧，2 帧确认安全
                                      *   800 mm/s  → 每帧 8.0 mm → 横线跨 1.5 帧 ← 现在在这里
                                      *   1050 mm/s → 每帧 10.5 mm → 横线只跨 1 帧
                                      * 原来的判据是"第一帧置候选、第二帧再看时间差"，
                                      * 也就是至少要连续 2 帧看到 n>=11 —— 800 以上就不可能了。
                                      * 所以 2 帧确认必须换成单帧；剩下的漏采风险只能靠降速解决
                                      * （见文件头"为什么最后停在 800"那段）。
                                      * 结果就是一半 T 口漏检，方块格的两次 T 口配不上对：
                                      *   55.txt 实测 SQ3 的四个 T 口 15760 / 17682 / 19617 / 20477
                                      *   (15760,17682) gap=1922 -> SQ,FAR 被拒（>SQ_MAX_GAP_MM）
                                      *   (15760,19617) gap=1935 -> SQ,FAR 又被拒
                                      *   (19617,20477) gap=860  -> SQ,PAIR 才成功
                                      * 一个方块吃掉两个方块的赛道：计划表每格落后一格，
                                      * 4 个方块之后落后 4 格 → 车跑到八边形/第 8 格六边形时，
                                      * 计划表还停在方块格，于是"环岛被略过"或者"用错规则乱进岛"。
                                      *
                                      * 【单帧为什么可以接受】
                                      * 真正的鉴别力在阈值上，不在帧数上：
                                      *   普通细线 n=3，粗线 n=4~5，方框拐角 n=9~10，真 T 口 n=11~12
                                      * ISL_T_N(11) 已经能把拐角挡在外面，再加时间确认是重复保险。
                                      * 若以后出现"普通弯被误判成 T 口"，不要加大这个数（那会退回漏检），
                                      * 而是降速，或者把控制周期从 10 ms 提到 5 ms。 */
/* 锁直行距离：**统一由配置表的 out_cross_mm 给**（方形 60、八边形 170、六边形出岛 170）。
 * 原来还有一个全局 ISL_CROSS_MM(60) 和 ISL_CROSS_MIN_MS(27)，两个都删了：
 *   - ISL_CROSS_MM 和方形那一行的 out_cross_mm 是同一个数，等于两份真相；
 *   - ISL_CROSS_MIN_MS 在 1050 mm/s 下永远不成立（60 mm 要 57 ms > 27 ms），
 *     是一条"写了但从不生效"的判据，删掉少一个需要维护的量。 */
#define ISL_ENTER_LOCK_MM    10      /* 进岛穿交叉点至少锁这么多 mm（防"进岛那一帧 n 偶然掉一下"） */
#define ISL_CROSS_V        875.0f    /* 锁直行时的目标速度 mm/s。
                                      * 【11/20 试验：340 -> 700】T 口锁直行段原来压到
                                      * 340，和正常 785/800 之间落差太大；先抬到 700，
                                      * 60 mm 锁定段约 86 ms，仍够穿过 12 mm 横线。
                                      * 其它参数不动。 */
#define ISL_COOL_MS          39U     /* 锁直行结束后的冷却，防同一个岔口被重复触发。
                                      * 【117 -> 40 ms（11/19 23:00）】锁直行本身就是
                                      * 60~170 mm 的"不去看图案"，同一个岔口早就在身后了；
                                      * 冷却再留 117 ms（1050 mm/s 下 123 mm）会把
                                      * **紧跟在后面的下一格入口一起吃掉** ——
                                      * 54.txt 里格 8 的入口距上一次 SOUT 只有 102 mm，
                                      * 就是这么被跳过的。40 ms ≈ 42 mm，够抖两帧即可。 */
#define ISL_OUT_MS          313U     /* 穿过后丢线宽限。【1000 ÷2.29 = 440 ms】 */
#define ISL_LOST_MS         470U     /* T 口内/刚穿过的丢线窗口（比普通赛道宽）。【1500 ÷2.29 = 660 ms】 */

/* ---- 环岛会话：给六边形这种"浅角分叉进入"的环岛用 ----
 * 入口不是 T（凑不到 11 路白），而是两条线并在一起、路数比平时多。
 * 进入后正常循迹绕行，靠累计偏航 + 回到单线判出口。 */
#define ISL_FORK_N            6U     /* 分叉判据：白路数 >= 此值
                                      * 1.txt 实测：成功的 8 次进岛 n 全部 >= 6；
                                      * 失败的 2 次都是 n=5 的假分叉（00F8 中间粗线、
                                      * 0C0E），进去 4 s 内就 STALL 撞停。
                                      * 所以 5 不够，抬到 6。 */
#define ISL_FORK_MM          60      /* 【默认值，实际由每格的 fork_mm 覆盖】
                                      * 分叉要持续走过这么多 mm 才算（普通弯只有 30~50 mm）
                                      * 2.txt 实测：降到 30 后，在普通弯的粗线段
                                      * （03F8/03DE/079C，n=7~8 但两端都不白）误判进岛，
                                      * 之后 T 口处理被禁用，2183 mm 后撞 T 口横线 STALL。
                                      * 1.txt 里 8 次成功进岛全部是在 60 下发生的，恢复 60。 */
#define ISL_FORK_TMO_MS     285U     /* 分叉候选的"掉帧宽容窗口"：这么久没再看到图案
                                      * 才作废候选。原来这个 400 是直接写在函数体里的
                                      * 字面量，属于提速时最容易漏掉的一类，提出来。
                                      *
                                      * 【为什么这次不按速度缩】它是"容忍图案抖一下"的窗口，
                                      * 而当前故障是【漏进岛】不是误进岛：
                                      *   缩短 → 真分叉掉一帧就被丢弃 → 整格 1200 mm 被跳过；
                                      *   放长 → 最坏是误进一次，但 ISL_FAKE_MM(300) 的自检
                                      *          会在 300 mm 内把它踢出来，代价只有 1 秒。
                                      * 两个方向的代价不对等，所以保守地不动它。
                                      * 若以后出现"明明没有环岛却 ISL,FIN"，再把它缩到 200。 */
/* ==================== 【11/24 23:1x 顶点型环岛（第2格 SMALLHEX）的入口最小里程】====================
 * 96.txt 实测（D 版，第三次会话）：
 *   8756  BS,OUT                        ← 大套小出岛 = 第 2 格起点
 *   9020  ISL,FIN,SMALLHEX,12,0FFF,245  ← 格内 245mm 的"整排白"就判进岛
 *   9320  ISL,FABORT,303,9              ← 300mm 后只有 0.9°，判假退出
 *   9360  ISL,HEXOVER,535,SMALLHEX      ← 40mm 后整格跳过
 * 那块 0FFF 是【大套小出口那个十字】（出岛后 264mm，279mm 后还有第二块），
 * 车在那里本来就该直行（e≈0、偏航 +14°→-15° 净 0），`entry_off=4.0` 推不进一个不存在的环。
 * 而真正的候选入口在格内 760 / 1186 / 1818 mm —— 全在 604mm 的窗口之外，车根本没机会看到。
 *
 * 所以：格起点后这一小段内**不接受**顶点型进岛，先让车开过前一个格留下的十字。
 * 只有 in_cross_mm != 0 的格（当前就是 SMALLHEX）受这条限制，
 * 大套小、第 8 格六边形的入口逻辑完全不受影响。 */
#define ISL_ENTRY_MIN_MM     600     /* 格起点后至少走这么远才允许顶点型进岛
                                      *
                                      * 【11/25 400 -> 600】97.txt 第 1 趟实测：
                                      * 格内 474mm 处以 n=5 / 位图 0x007C 进了岛，
                                      * 850mm 只转了 96.9°（真六边形应 0.42°/mm）→ SEX_F 兜底。
                                      * 原因：Island_Fork() 只看"路数 >= fork_n"，没有形状判据，
                                      * 弯道变宽（5 路连续白）就能过。
                                      * 而历史日志里真正的候选入口都在 760 / 1186 / 1818mm，
                                      * 假入口都在 188~474mm —— 用 600 正好把两者分开。 */

/* ==================== 【11/24 23:2x 小环前馈 BS_SMALL_FF】====================
 * 【问题】MIDOK 时本来写了 `bs.turn = -BS_BIAS;`（= +215，左转前馈），
 * 但 BS_SMALL 的 case 体里每一帧都会把 bs.turn 写回 0 —— 前馈从第一帧就被清零了，
 * 小环实际是"纯 PD 追偏差"。
 * 825 mm/s 走 ~170 mm 半径的小环，需要的差速约 v×轮距/R ≈ 580 mm/s，
 * PD 只能靠"先偏出去再纠回来"产生它 —— 于是车在内外线之间来回横跳：
 *   95.txt 小环 892 mm / 295°（勉强），96.txt 小环 3387 mm 才攒到 290.7°。
 * 同一次会话里 575 mm/s 的第 8 格用同样的逻辑干净出岛（345.1°/895 mm），
 * 说明不是判据问题、也不是速度上限问题，而是【小环缺前馈】。
 *
 * 【做法】过了 BS_ENTRY_DEG(130°) 的入口段之后，给一个恒定左转前馈（正 = 左转，
 * 小环绕的是逆时针）。它只补"绕圈本身需要的差速"，横向纠偏仍交给 PD，
 * 因此不牺牲直道/岛内速度。先取 150（约为理论值 580 的 1/4）保守起步：
 *   · 太大 → 车贴内圈走，出岛角提前（<290° 就到了），表现为 BS,ALLOK 里程偏短；
 *   · 太小 → 还是靠 PD 硬追，横摆照旧。
 * 调法：看 BS,ALLOK 的里程，正常应该在 900~1400 mm；逾 1500 就加大到 250~350。 */
#define BS_SMALL_FF        0.0f  /* 【11/25 15:00：150 -> 0，等于撤掉小环前馈】
                                      * 现场反馈"大套小里绕小环岛两圈"。查 106.txt 两趟：
                                      *   好的一趟（run3）：小环 604->1514mm，yaw -75.6->-370.7 单调左转
                                      *   坏的一趟（run2）：小环 610->3120mm（正好两倍），
                                      *      yaw +52.5 -> -151.9 -> **-101.3 -> -63.7**（反向回摆）
                                      *      -> -102.6 -> -232.0（292° 才出段）
                                      * 坏的那趟 F 行出现 vL=0、vR=1502（一轮停一轮空转）= 在岛里顶住转圈。
                                      * 根因是这条前馈的**方向判据会中途翻号**：
                                      *   big = gyro_angle - bs.yaw0 本意是"大环绕向"，
                                      *   但小环自身是左转，会把 gyro 一路拉回到 yaw0 另一侧，
                                      *   于是 big 在段中途过零 -> bs.turn 从 +150 翻成 -150，
                                      *   车先被推紧缩半径、再被推往外，两头都不是环上的线。
                                      * 43aecb2（上负压可完赛第二版）这一段 bs.turn 恒为 0，没有前馈。
                                      * 所以先把它置 0 回到已验证行为。
                                      * 若以后还要前馈，正确做法是**在 BS_MIDOK 那一刻把符号锁存**，
                                      * 不要在整段里实时算 gyro-yaw0。 */
#define BS_SMALL_FF_OLD  150.0f  /* 原值留档（正 = 左转）*/

/* 【11/19 23:00 删掉两个"定义了但没人用"的宏】
 *   ISL_HEX_V(900)        —— 岛内速度一律由配置表 v_island 给，这个默认值没被读过
 *   ISL_EXIT_CROSS_MM(170)—— 出岛锁直行距离由配置表 out_cross_mm 给
 * 留着只会让人以为改它有用（这一类"两份真相"正是这次合并要消掉的东西）。 */
#define ISL_EXIT_DEG      340.0f     /* 出岛偏航阈值（纯偏航判据，不再依赖分叉图案）
                                      * 实测分叉出现的角度窗口：330.5 ~ 347.2（当前读数单位）
                                      * 取正中 340。注意循迹路径的 gyro 没乘 GYRO_SCALE，
                                      * 读数是物理角的 1/0.96 倍，所以 340 对应物理约 326°。 */
#define ISL_EXIT_MIN_MM     600      /* 出岛里程下限护栏：进岛后至少走这么远才允许出。
                                      * 防止 yaw 偏大时在环道前半段误出 —— 那时还没到岔口，
                                      * 一旦进锁直行（ISL_CROSS）170 mm 会撞岛壁。 */

/* ---- 假进岛自检 ----
 * 真环岛：850 mm 里转约 340°（实测 337.2 / 340.8 / 341.3）
 * 假进岛：850 mm 里只转 50~107°（实测 51.6 / 50.5 / 107.1 / 0.0）
 * 两者差一个数量级，用 300 mm 处 40° 这条线就能分开。 */
#define ISL_FAKE_MM        300       /* 进岛后走这么远开始自检 */
#define ISL_FAKE_DEG        25.0f    /* 偏航还不到这个值 → 判为假进岛。
                                      * （历史说明见下）
                                      *
                                      * 八边形/大六边形用 25° 是历史标定；
                                      * HEXT（第 8 格）改用更严的两段判据，见下面
                                      * ISL_HEXT_FAKE_*：入口大角打角 + 强制直行后，300 mm 处
                                      * 至少 30°、600 mm 处至少 120°，否则直接 FABORT。
                                      * 59.txt 的 1001 mm / 63.2° 就是 25° 太松、
                                      * 被 SEX_F 兜底后伪装成"跑完"的。 */
/* HEXT 专用假进岛判据（更严）：
 *   300 mm 处 yaw < 30°   -> FABORT（直穿/没拐进去）
 *   600 mm 处 yaw < 120°  -> FABORT（切内角，不是绕六边形）
 * 入口大角打角到 60°，随后强制直行 120 mm，PD 可能把过转的部分修回来，
 * 所以 300 mm 的门限取 30° 而不是 60°，避免误杀。
 * 这里只做兜底，不让 1000 mm / 63° 这种"直穿环岛"走到 SEX_F 再 Advances。 */
/* 【11/21 11:00：入口目标角 60 -> 35（按图纸），这条门限要同步 30 -> 15】
 * 否则 35° 的落点正好压在 30° 门限上：实测 19003 那次 302 mm 处是 26.6°、
 * 19161 那次 302 mm 处是 26.6°/29.1° —— 差不到 1° 就决定"进/不进"，
 * 现场表现就是"可能进岛可能不进岛"。降到 15° 后还有 20° 余量。 */

#define ISL_HEXT_FAKE_MM1   300
#define ISL_HEXT_FAKE_DEG1  10.0f
#define ISL_HEXT_FAKE_MM2   600
#define ISL_HEXT_FAKE_DEG2  10.0f
/* 【11/25 16:30 新增：进岛"承诺"闸门】
 * 上面两条只在 10° 上卡，太松 —— 假进岛后车在主线上也会一路转弯，
 * 300/600mm 处随便就攒到 30~90°，根本挡不住。
 * 真正能分开真假的是**进岛后前 200mm 有没有真的拐进环**：六边形入口
 * 要右拐 60°（打角 15° + PD 补 45°），真进岛 220mm 时累计偏航已经 77~93°；
 * 假进岛（只是从岔口旁边擦过去）会掉回 0 附近。四趟实测：
 *   16:10 FIN@17745  n=3 000E  +150:+61  +200:+93  +250:+105  +300:+116  (真)
 *   16:15 FIN@17549  n=3 0803  +150:+66  +200:+77  +250:+108  +300:+118  (真)
 *   16:26 FIN@17641  n=3 0380   -3       -4        +5        +32        (假)
 * 所以加一道"220mm 处必须偏航 >= 30°"。16:26 那趟就是漏了这一道：
 * 假进岛没被拦下，状态机带着"在环里"的假设跑了 1500mm，最后被里程兜底
 * ISL,SEX_F 触发，在直道上左打 71° 把车开出赛道（LOST 24159）。 */
#define ISL_HEXT_COMMIT_MM   220
#define ISL_HEXT_COMMIT_DEG  30.0f
/* 里程兜底出岛前的"角密度"自检（°/mm）。六边形环线实测 0.37~0.44，
 * 假进岛在主线上的 0.23 —— 低于这个值说明根本不在环上，此时禁止打角，
 * 直接认输换格（否则就是直道上拉 70°，车必飞）。 */
#define ISL_HEXT_EXIT_FALL_RATE 0.30f
#define ISL_OVER_DEG      420.0f     /* 超过这个偏航强制结束，防绕圈
                                      * 【380 -> 420（11/21）】HEX8 的出岛角按环线曲率
                                      * 实测标定为 360°（走满一圈回入口顶点），380 只比它
                                      * 高 20°，陀螺抖动或一圈略大就会先被 SOVR 截断，
                                      * 于是永远走不到顶点、车留在环线上继续绕（"绕两圈"）。
                                      * 420 给 360 留 60° 余量。BIGHEX/SMALLHEX 的
                                      * exit_deg 分别是 700/340，不受影响。 */
#define ISL_SES_MAX_MS    4765U      /* 会话最长。【20000 ÷3 = 6700 ms】一圈环岛
                                      * 在 600~900 mm/s 下只要 2~3 s，7 s 仍是兜底 */
#define ISL_RET_MM          120      /* 会话结束后，至少再走这么远才允许重新进岛。
                                      * 【260 -> 120】260 mm 在 1050 mm/s 下是 250 ms 的
                                      * "禁进窗口"，它会把紧跟在出岛后面的下一格入口一起吃掉。
                                      * 实证（54.txt，格 7 八边形出岛后接格 8 六边形）：
                                      *   ISL,SOUT,OCT7 在 19690 -> 格 8 起点 19703
                                      *   格 8 的真入口图案在 19792（n=11 07FF）/19802（n=6 00FC）
                                      *   距 SOUT 只有 102 / 112 mm < 260 -> Island_Fork 直接 return
                                      *   窗口到 19950 才开，那时车早已过去 -> ISL,FIN,HEX8 一次都没打
                                      * 120 mm 仍能挡住"刚出岛那个分叉"（ISL_COOL_MS 117 ms 也还在管），
                                      * 但放得过紧贴着的下一格入口。 */
#define ISL_PHASE_MS      8000U     /* 单阶段超时：这么久没等到图案就强制切换。
                                      * 【30000 ÷3 = 10000 ms】 */
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
 * 全局改成 500 会把它误杀，所以第 2 格单独 500、第 8 格单独 1500，
 * 其余格按需覆盖。 */
#define ISL_HEX_MAX_MM    1200       /* 默认值；按格覆盖见 isl_cfg 的 skip_mm */

/* 【11/20 重构】第 8 格不再用“本格起点 + 920~1400 mm”窗口。
 * 方形/八边形从计划表删除后没有八边形出口锚点；入口改为纯图案：
 *   n=5~8 且最右四路至少一路亮（见 Island_Step 的 HEXT 分支）。
 * 中间的 T 口由全局 T 口直行处理，不推进 plan_idx。 */

/* ==================== HEXT 入口/出口打角段 ====================
 * 入口：先大角度打角，再强制直行一段，之后交还 PD 绕岛内。
 * 出口：先小角度打角，再强制直行穿过岔口，然后 Island_Advance。
 * 岛内仍由 PD 循迹；entry_deg=0，不使用 err offset。 */
/* 【11/21 11:00 那趟：60° 打角让车头越过边线，边落到左侧】
 * 打角本身已经修好（turn 400 之后过冲从 35° 降到 11°，航向保持把它稳稳锁住）：
 *   19024 yaw +4.4 / 19048 +23.7 / 19070 +41.3 / 19090 +57.6
 *   19113 切直行 +70.9（过冲 11°）→ 19162 +63.7 → 19184 +61.5 → 19207 +60.6
 * 但转到位之后，车看到的那条线在【左边】：
 *   19252 n=7 0F07（两个白团）→ 19275 n=2 0C00 e=-6.0 → 19300 yaw 坠到 29.4°
 *   → 302 mm 处 FABORT(26.6°)；第二次尝试同样（19658 n=3 0700 e=-3.5）
 * 也就是 PD 按"线在左"往左打，把车拉回主线，六边形根本没绕。
 *
 * 原因：这个元素"主线与一条边成 30~35°"（图纸第 9 个元素标注 35°），
 * 打 60° 就多了 25°，车头越过那条边 -> 边落在阵列左侧 -> PD 反向。
 * 所以入口目标角按图纸取 35°。
 * ⚠️ 300 mm 处的自检门限（ISL_HEXT_FAKE_DEG1）必须同步从 30° 降到 15°，
 *   否则 35° 的落点又紧贴 30° 门限，会重演"有时进得去有时进不去"。
 * 600 mm / 120° 那条不用动：真绕起来 600 mm 应该有 180° 左右。 */
/* ===== 【11/27 11:50 入口整段回滚到"完赛版"】=====
 * 依据 git 里两版真完赛的提交（入口参数完全一致）：
 *   651d266「可以正常完赛，用时51秒」
 *   5cee503「成功完赛，时间为50.7秒」
 * 那两版是：TURN_DEG 45 / TURN_V 525 / TURN_MAX 500 /
 *           没有入口延迟、没有 ENTRY_PD_DEG（打角完就交 PD）。
 * 中间为了修"内切/锐角早打角"改过 38°、330、620、延迟 80、交 PD 要求 55°，
 * 结果反而出现"掉头/左转飞出去"。现在：
 *   ISL_HEXT_ENTRY_DELAY_MM = 0   （延迟段失效）
 *   ISL_HEXT_ENTRY_PD_DEG   = 45  （= 打角角，交 PD 时机与老版一致）
 * 其余代码路径保留，但行为等价于 5cee503 那一版。 */
#define ISL_HEXT_ENTRY_TURN_DEG     35.0f   /* 【11/28 09:55：40 -> 35】进岛强制右转角。
                                      * 130：40° 时第 1 圈 PD 48.4°、SEX 351/875 正常；
                                      * 第 2 圈入口干净但交 PD 后连续丢线，岛内满舵
                                      * 恢复甩成 SEXBAD 1501/244 —— 40° 没解决掉头。
                                      * 35° 更浅：入环角再小 5°，少切一点；风险是
                                      * 切不进去/从外侧出去（PD 后偏航 <45° 且丢线）。
                                      * 历史：15° 会被岔口主线拽回；45° 能进但 PD
                                      * 63.8°、内切丢线；38° 那版曾"掉头/左转飞出去"，
                                      * 11/50 回滚到 45；现在 40 -> 35 继续试。
                                      * 调法：还丢线/掉头 -> 30~32；明显进不去 -> 38~40。
                                      * 以下是 45/38 那轮的现场记录：
                                      * 【问题】现场反馈"最后一个六边形进岛右转太多，在环内找不到线"。
                                      * 日志实测（10:51:32 那趟）：
                                      *   HEXT,PD,17990,638   <- 交 PD 时偏航已 63.8°
                                      *   GAP,START,18034,1   <- 44mm 后丢线
                                      *   ISL,FABORT,249,294  <- 偏航掉到 29.4°，判假进岛
                                      * 六边形内角切线是 60°：45° 打角 + 过冲 18.8° = 63.8°，
                                      * 拐过了切线、压到环内侧，传感器看到线在左边就丢线了。
                                      * 减到 38° 后 PD 接手应在 55~57°，正好贴在切线内侧几度，
                                      * 留给 PD 把剩下的几度补进环。
                                      * 调法：HEXT,PD 的偏航仍 >60 -> 32~35；明显偏外/切不进去 -> 42。 */   /* 进岛强制右转角。【11/25 17:05：15 -> 45】
                                      * 【为什么改】15° 之后就把控制权交给 PD，而岔口处 PD 看到的是
                                      * 两条线的加权重心（+4~-4 乱跳），它会把车拽回主线 ——
                                      * 5 次进岛里 3 次成、2 次被拽回来。最典型 16:49 第 2 圈：
                                      *   HEXT,PD,39435,363   打角到 36.3°
                                      *   39450 vL 195 / vR 1108 -> 39481 vL -422 / vR 1713  右打到底
                                      *   39514 e=1.8 n=4     又看见线（其实是主线）
                                      *   ISL,FABORT,220,6    220mm 处偏航只剩 +6°
                                      * 真进岛的两趟（16:10/16:15）220mm 处偏航是 77~93°。
                                      * 六边形入口本来就要右转 60° 才算进环（第一段边），
                                      * 现在把强制段加到 45°，PD 只需再补 15°，岔口的模糊误差
                                      * 就压不倒它了。实测打角速率 0.26°/mm，45° 约 175mm，
                                      * 仍在 ISL_HEXT_ENTRY_TURN_MAX_MM(500) 以内。
                                      * 过头的表现：HEXT,PD 之后立刻 n=0 或 e 长时间满量程
                                      * -> 回调到 35。 */
#define ISL_HEXT_ENTRY_TURN_V      525.0f   /* 【11/27 11:35：525 -> 330】进岛打角段速度。
                                      * 【问题】现场反馈"最后六边形内切还是太多，找不到线然后掉头反走"。
                                      * 11:19 那趟实测：
                                      *   HEXT,STRAIGHT_START,40036,402  （打角 40.2°）
                                      *   e = −4.2 一直偏左           <- 横向已经切到环线内侧
                                      *   HEXT,PD,40099,595           <- 航向 59.5° 是对的（环切线 60°）
                                      *   GAP,START,40147 → ISL,FABORT,232,-12
                                      * 也就是"角度对、位置内"：PD 只能往左纠正，偏航一掉就被判假进岛。
                                      * 【为什么降速能减内切】°/mm = ω/v。差速(750)不变、速度从 525
                                      * 降到 330，同样的 55° 只走 ~85mm 而不是 ~150mm：
                                      * 圆弧半径从 ~156mm 降到 ~90mm，横向内缩 R(1-cos55°)
                                      * 从 ~67mm 降到 ~20mm —— 正好是 e 从 −4.2 回到 −1.5 左右。
                                      * 出岛枢轴当初也是靠这一条把甩出去的 50mm 压到 30mm 的。
                                      * 调法：仍内切(e < -3) -> 280；切不进去(e 一直为正) -> 380。 */
/* 入口打角段：v=420 / turn=600，右转 15° 后进入直行段。
 * dir=+1 时内右轮目标 = 420-600 = -180，允许反转，转角更快。 */
#define ISL_HEXT_ENTRY_TURN        750.0f
#define ISL_HEXT_ENTRY_TURN_MAX_MM 500   /* 【11/27 11:40】500 -> 620：前面的
                                      * ISL_HEXT_ENTRY_DELAY_MM(80) 也算在这段里程里，
                                      * 上限不跟着加就等于把打角配额吃掉 80mm。 */

/* 旧线触发参数，当前 15°+20mm 方案不使用。 */
#define ISL_HEXT_ENTRY_LINE_MIN_DEG    30.0f
#define ISL_HEXT_ENTRY_LINE_CFM        1U
#define ISL_HEXT_ENTRY_SETTLE_MM        120
#define ISL_HEXT_ENTRY_RECOVER_DEG      25.0f
#define ISL_HEXT_ENTRY_RECOVER_MAX_MM   180

/* 当前 HEX8 入口：向右打角 15°，然后直行 20mm，最后交 PD。
 * 直行只保留 20mm，尽快把控制权交回 PD；直行段仍用航向 P + 角速度阻尼，
 * 避免 turn=600 收尾的残余角速度造成内切。 */
#define ISL_HEXT_ENTRY_STRAIGHT_V  575.0f   /* 打角后的直行速度 */
#define ISL_HEXT_ENTRY_STRAIGHT_MM 60       /* 【11/25 17:35：20 -> 60】45° 强制打角后的
                                      * "锁航向"段加长到 60mm 再交 PD。
                                      * 【为什么】17:26 那趟两次进岛都是：
                                      *   HEXT,PD 时偏航 64.5 / 70.4（打角过冲到 64~70°）
                                      *   再走 ~100mm 后 ISL,FABORT,222 只剩 16.7 / 21.8
                                      * 也就是 PD 一接手就把车从 65° 拽回 17° ——
                                      * 车头已经越过环的切线(60°)、压到环内侧，
                                      * PD 看到线在左边就往回拽，这一趟就白跑了。
                                      * 这一段的 turn 是"航向 P 环 + 角速度阻尼"，
                                      * 会把过冲收敛回 ISL_HEXT_ENTRY_TURN_DEG(45°)，
                                      * 给 60mm 它才能真正把车稳住再交 PD。 */
/* 交 PD 之后的前 160mm 里如果还是丢线（n==0），**不要**用"上一次有效 err"。
 * 那个值来自入口那几帧（例如 0780 → 重心 -2.88，指向主线在左），
 * PD 会照着它往左打：run3 圈1 yaw 从 39° 一路掉到 -13°，直接横穿
 * 18051 的 0FFF 岔口，丢线 180mm 靠 GAP 才救回来，那一圈 SEX 拖到 1107mm
 * （其余三圈 887~944）。现在这段时间改成保持 20° 航向直行，等看见线再交 PD。 */
#define ISL_HEXT_PD_WAIT_MM      160
/* 交 PD 后的盲段不再"保持 20° 直行"，而是按**环线曲率**继续右转：
 *   ω = 0.4 °/mm × 460 mm/s = 184 °/s -> 差速 ≈ 200 mm/s（实测 0.88 °/s per mm/s）
 * 之前保持 20° 等于把直行又延长了 160mm，车直接冲出环岛（11/23 16:35 那趟
 * HEXT,PD 之后 yaw 从 35° 被拉回 12.8°，ISL,FABORT,333,128）。 */
#define HEXT_BLIND_TURN          250.0f
/* 打角结束后的角速度阻尼（phase2 直行段用）。
 * 【11/23 16:40 实测：打角 600 收尾"刹不住"，这就是内切的真身】
 *   17933 HEXT,STRAIGHT_START  yaw 24.8
 *   17959 (+26mm)              yaw 73.5   ← 26mm 内又转了 49°（残转 ~850°/s）
 *   18005 HEXT,PD              yaw 46.1   ← 本该 20°，多了 26°
 * turn=600 的 ω≈0.88×600≈530°/s，角度一到位就写 turn=0，
 * 车会带着 20~26° 余转冲进环里 —— 这 26° 就是"内切"。
 * 只靠 P 项（GAP_KP=5.7）刹不住，必须加 -KD·ω 这一项。 */
#define HEXT_ENTRY_KD            0.6f    /* mm/s per °/s：用陀螺角速度反向刹车 */

/* ==================== HEX8 盲绕圈（不依赖光电管） ====================
 * 【11/21 12:08 之前的六种做法全部失败，共同点是"转一个固定角度然后直走"】
 *   打角35+直行120 -> 110mm 后被主线吸回
 *   打角35+立刻交PD -> 同上
 *   打角70+直行300 -> 直接飞出赛道（几何必然：岔口在切线处，直走就走出了赛道）
 *   闭环打角(线回中即停) -> 循迹发飘
 *   出岛角 274/360 调整 -> 车根本没上去，调出岛角没意义
 * 六边形是**闭合环线**，进岛必须"持续按环的曲率转"（实测 0.26°/mm），
 * 直走一定出界。所以打角完成后改成**盲绕一圈**：固定速度 + 固定差速，
 * 转满 360° 由陀螺判定，全程不看光电管。
 *   需要的角速度：ω = v/r，r 取六边形内切半径约 187mm
 *     -> ω = 460/187 = 2.46 rad/s = 141 °/s
 *   差速：turn = ω × 轮距 / 2 = 2.46 × 0.15 / 2 = 0.184 m/s = 184 mm/s
 * 转满 360° 后 Island_Step 的 SEX 判据（exit_deg=360）自然触发，
 * 走的是原有的"出岛小角打角 + 80mm 直行"流程，不需要额外收尾。
 * 里程兜底 ISL_HEXT_ORBIT_MM：万一陀螺没到 360°（打滑/丢帧），
 * 走满这么多就交回 PD，不至于一直转。 */
#if 0  /* 整圈盲绕已停用：入口改为盲走 90° 后交岛内 PD */
#define ISL_HEXT_ORBIT_V       575.0f   /* 盲绕圈速度 mm/s */
/* 【11/21 12:12 那趟实测：turn=184 转得太快，比设计快 19%】
 * 进岛 18331（yaw0≈-495.8），yaw 采样：
 *   18602 +111.8°   19016 +261.5°   19430 +404.3°   19577 +452.9°
 *   -> 实测 0.366 °/mm = 168 °/s；设计 0.307 °/mm = 141 °/s
 * 由 ω = 2*turn/轮距 反推：有效轮距 ≈ 125 mm（不是配置里假设的 150 mm）。
 * 所以同样 turn 下车的圈比六边形小、更切内侧 —— 这是"多转 1/3 圈"的一部分。
 * 按 125 mm 折算要得到 0.28 °/mm（六边形环线实测 0.26~0.30）：
 *   ω = 0.28*460 = 129 °/s = 2.25 rad/s -> turn = ω*125/2 = 141 mm/s
 * 184 -> 141。 */
/* 【11/21 12:57 那趟：半径确实**大了** —— 141 让车在环线外侧 20~33mm 绕】
 * ⚠️ 上面引用的"环线实测 0.26~0.30 °/mm"是从**盲绕本身**量的，循环论证，
 *    不能当环线的真曲率。这一次改用"线上点"直接定标：
 *    1) 把整趟的 yaw+编码器做航位推算，把 n=0 的盲绕段(18080~18795)拟合成圆：
 *       R = 213mm（32 个点，拟合半径与 ds/dθ 一致，可信）
 *    2) 盲绕结束时车重新看到线的那 8 帧（18841~19093，单团窄线），
 *       用车位置 + 位图重心换算出**线上点**，这些点到盲绕圆心的距离只有
 *       179~208mm、均值 193mm —— 线在车的圈**内侧**
 *       => 车在环线外侧 ~20~33mm，就是"半径大了、没沿线走"。
 *    所以曲率要加：目标 R = 186~190mm -> turn = 141 × 213/188 ≈ 160。
 * 取 162（R ≈ 185mm）。这样打角段(180、v=420)和盲绕段(162、v=460)
 * 的曲率都落在 0.31 °/mm 附近，两段还是同一个圆。
 * 【11/21 14:37】现场反馈 155 的盲绕半径偏大，回到 162；
 * 【11/21 15:xx】再按现场要求把盲绕差速加到 250：R 约 120mm，曲率更紧。
 * 入口 180 暂不动；如果入口切内，再同步调入口差速。 */
#define ISL_HEXT_ORBIT_TURN    312.5f   /* 盲绕圈差速 mm/s（R≈120mm，曲率更紧） */
#define ISL_HEXT_ORBIT_MM      1400     /* 盲绕里程兜底：超过就交 PD */
#endif

/* ==================== 进岛前减速（只在最后一个六边形那一段） ====================
 * 【11/21 12:17 那趟：入口判据抓到的是普通宽线，不是六边形岔口】
 *   19065 ISL,FIN,HEX8,7,007F  -> 打角后 19137~19683 连续 550mm n=0，
 *   中间还判了 3 次 GAP —— 车根本不在环线上，321°/1110mm 是在空白区绕了一圈。
 * 把 8 次入口的位图全列出来（007F / 007E / 00FF / 00F8 / 01F8 / 00F2 / 0FE0 / 03E1），
 * **没有一次读到"两个白团中间有缺口"的真分叉**，全是一整条连续宽白。
 * 原因：800 mm/s 下每帧 8mm，岔口两条分支分开的那 20~40mm 只有 2~5 帧，
 * 12mm 间距的阵列在同一帧里把两条分支糊成一条 —— 信号本身糊了，改判据没用。
 * 400 mm/s 下每帧 4mm，同样一段有 5~10 帧，两条分支才分得开。
 * 所以只在 HEXT 格、距本格起点 8000mm 之后把速度压到 400：
 *   实测入口在"格3起点 + 9017~9849"，所以这段减速覆盖入口前 1000~1850mm；
 *   单圈多花 1.2~2.2 s（总 26 s 左右）。其它地方（方形/八边形/断路口/波浪形）不变。 */
/* 【11/21 12:25 已停用】进岛减速：改用"两团白"判据后不需要，宏保留不引用。 */
#if 0  /* 进岛减速已停用 */
#define ISL_HEXT_APPROACH_V    500.0f   /* 停用（保留定义，避免删注释时出错） */
#endif
/* 【11/21 12:21 那趟：8000 太早，限速段长达 1937mm】
 *   限速开始 ~17570（格3起点+8000），六边形入口 19507（格3起点+10384）
 *   -> 白白慢了近 2 米。改成 9500：只覆盖入口前约 880mm。
 * 调法（跑完看"首次 tgt=4000 的里程"减 "ISL,FIN,HEX8 的相对里程"）：
 *   想要 ~400mm 减速 -> 10000；想要 ~900mm -> 9500；想要 ~1400mm -> 9000 */
#if 0  /* 进岛减速已停用 */
#define ISL_HEXT_APPROACH_MM   9500     /* 停用 */
#endif

/* 【11/21 12:12 那趟：exit_deg=360 实际在 444° 才触发，滞后 84°】
 * 原因：盲绕期间车是 n==0，而 Island_Step 在丢线时被冻结
 *   （isl_now = (ISL_ENABLE && t_rcv == 0U) ? Island_Step(...) : 0U）
 * 所以 yaw 到 360° 那一帧判据根本没跑；等车重新看到线时 yaw 已经 444°，
 * 才立刻触发 SEX。这段滞后 ≈84°，而且几乎是系统性的（"什么时候重新见线"决定的）。
 * 所以出岛角按"360 - 84 ≈ 280"给，让实际出岛落在 360 附近：
 *   280 -> 触发时约 364° ≈ 一圈。
 * 调法（跑完看 ISL,SEX 实际值）：
 *   落在 355~375 -> 保持 280
 *   落在 400 以上 -> 减到 250
 *   落在 330 以下 -> 加到 300
 * 这个值直接写在 isl_cfg 的 HEX8 行 exit_deg 字段里（= 280），本处不留常量。 */

#define ISL_HEXT_EXIT_NEED_MAX      0.0f   /* 【11/25 20:20：60 -> 0，取消"补圆"右打角】
                                      * 110.txt 实测证明这 9° 右打是反的：
                                      *   SEX 18651 yaw 351.0（JUNC 已在 333.2、BLOB 在 322.0）
                                      *   -> 补圆右打 9.8°（EXIT,98,23）-> 3.7cm 后 n=0，线没了
                                      *   -> SETTLE 直行 80mm 全程 n=0（车走在环线与主线之间的缝里）
                                      *   -> +192mm 主线出现在最左通道 0800，为时已晚 -> LOST 停车
                                      * 原因：JUNC/BLOB 在 322~333° 就报出合并点，切线点约在 345~350°，
                                      * 所以 SEX 落在 351° 时车头已经对着主线；再右打 9° 反而是把
                                      * 车头压向环线内侧，等于把自己塞回环里。
                                      * 现在 need 恒为 0：SEX 后不打角，直接按当前航向锁直行
                                      * （见 SETTLE），环线右曲率会自己把线甩到右侧，主线从左侧进来。
                                      * 若某趟反过来变成"出岛偏左、错过主线"：把这行恢复成 20~30，
                                      * 让补圆只补一点点，别回到 60。
                                      * 以下是 17:00 的原记录（保留作历史）：
                                      * 出岛"补圆"右转角上限（度）。
                                      * 【11/25 17:00 出岛方向/角度的最终结论】
                                      * yaw 是从 ISL,FIN（车还在主线上）开始累计的右转量。
                                      * 一圈走完 yaw=360 时，车头自然回到主线方向 ——
                                      * 所以出岛该做的是"把这一圈补完"：
                                      *   在 yaw=Y 开火 -> 右转 (360-Y) 度 -> 航向已与主线平行
                                      * 实测四趟的 Y 与本应右转角：
                                      *   321.6 -> 38.4° | 333.8 -> 26.2° | 348.7 -> 11.3°
                                      *   356.6 -> 3.4°
                                      * 而代码一直给固定"左转 70°"，每次多转 30~85°，
                                      * 车头直接横过来。16:44 那趟的铁证：
                                      *   18551 HEXT,EXIT,-701,63,1   （左转 70.1°）
                                      *   18895 n=12 bm=0FFF 只有一帧  <- 阵列与线平行
                                      *   18941 起 n=0 到底          <- 车头与主线垂直，横扫过去
                                      * 0FFF 一次点亮 12 路，只有"传感器排和线平行"才会出现，
                                      * 即"车垂直于线"；正常循迹时线只占 2~4 路。
                                      * 现在按 Y 现算：右转 = clamp(360-Y, 0, 60)。
                                      * 触发越晚（Y 越大）转得越少，最后基本等于"直着带出去"。
                                      * 原固定目标角（已停用，仅留档）：
                                      * 【11/25 16:23：-55 -> -70。原因见下面"落点"一节】
                                      * 六边形外角 60°：环上绕完一圈车头还比主线偏 60°，
                                      * 所以几何上左转 60° 就"平行于主线"了 —— 这恰恰是最坏的值：
                                      * 平行 + 横向偏 40~55mm = 永远看不见线。
                                      * 打角时车在前进，圆弧的弦方向 = 初速方向左转 θ/2，
                                      * 端点相对交叉口的横向偏移 ≈ R(1-cosθ)，R≈110mm（350mm/s 实测）：
                                      *   θ=55°：偏 47mm，车头还差 5°（几乎平行）-> 16:15 那趟全盲
                                      *   θ=60°：偏 55mm，车头完全平行主线     -> 最坏
                                      *   θ=70°：偏 53mm，但车头已越过主线 10°  -> 会往线上切
                                      *   θ=75°：偏 51mm，越过 15°
                                      * 所以必须**多转 10~15°**，让车头指到主线另一侧，
                                      * 由 ISL_HEXT_EXIT_HOLD_MM 段把车带过去（见下面 phase3）。
                                      * 原来 -55 的取值依据（留档）：
                                      * 【11/25 15:20：15 -> -55，修"出不了六边形、绕在里面"]
                                      * 本趟日志（1.txt）出岛后 5 米内 yaw 从 -358° 一路掉到
                                      * -2099°（1741° ≈ 4.8 圈），而且每隔 ~1600mm 就重复
                                      * 一组 HEXT,FORK 簇 —— 车根本没离开环线。
                                      * 几何原因：六边形外角 60°，进场时向右转 60° 才进环；
                                      * 绕一圈回到 X 口时车头仍比主线偏 ~60°。
                                      * 原来这里给 +15（继续右转），"直行"其实是沿环线切线走
                                      * -> 又回到环上，就成了"绕两圈/出不去"。
                                      * 改成 -55：出岛向左转回 ~55° 把车头摆到主线上，
                                      * 再由 ISL_HEXT_EXIT_STRAIGHT_MM 直线离开。
                                      * 转向能力：GAP_KP 7.12 × 55 = 392 -> 限幅 375，
                                      * 轮距 120mm 下约 358°/s，55° 只需 ~154mm。
                                      * 【11/25 15:30 定稿：exit_deg=360 + -55，这两个必须配套】
                                      * 上一版是"345° 就左转 45°"——转得太早（车还没到交叉口），
                                      * 所以感觉左偏太多、飞出赛道；问题不在角度而在触发点。
                                      * 现在：360° 到 X 交叉口 -> 左转 55°（车头与主线差约 60°）
                                      * -> 325mm 直线离开。
                                      * 【11/25 15:25 曾把 -55 单独改成 -45（当时 exit_deg 还是 345）：】
                                      * 几何上本来就该是 45：外角 60°、而 SEX 在 345°（比闭合点早 15°），
                                      * 车头与主线的夹角 = 60 - 15 = 45°。-55 比需要值多拐 10°，
                                      * 于是 325mm 直行是斜着往赛道外走的，出岛后直接飞出去。
                                      * 调法（一次 5°）：
                                      *   还绕圈/又回到环上 -> -50 ~ -55
                                      *   出岛后向左偏 -> -40
                                      *   SEX 的 run 应仍在 320~340（直行段 325） */
#define ISL_HEXT_EXIT_TURN_V       875.0f   /* 出岛直行段速度（现用 ISL_CROSS_V，留档） */
#define ISL_HEXT_EXIT_TURN         500.0f   /* 出岛打角的固定差速（常量，见 PIVOT_V）。
                                      * 【11/25 16:33：375 -> 500】和 PIVOT_V 350->250 配套，
                                      * 目的是把"枢轴甩出去的横向距离"压下来。见 PIVOT_V。 */
#define ISL_HEXT_EXIT_TURN_MAX_MM  240      /* 【11/25 16:20：80 -> 240】打角段里程上限。
                                      * 80 是按 875mm/s + 358°/s 推的，实际转不到：
                                      * 1.txt 三次实测 55° 都只走到 48.7/50.5/50.5°，
                                      * 用了 271mm（0.19°/mm）。
                                      * 现在降到 350mm/s 转，期望 110~190mm 转到 -55°，
                                      * 240 是留 25% 余量的上限（实到 240 会在日志里打
                                      * HEXT,EXIT 末列为 0，说明"没转到角就被截断"）。 */
#define ISL_HEXT_EXIT_PIVOT_V      250.0f   /* 【11/25 16:33：350 -> 250，与 TURN 375->500 配套】
                                      * 【16:30 实测：枢轴还是太"甩"】
                                      * 两趟真进岛的出岛记录（都是 −71°、末列 1，打角本身没问题）：
                                      *   16:30:34  SEX 333.8  枢轴 121mm  然后盲走 261mm 才扫到线
                                      *   16:31:01  SEX 321.6  枢轴 143mm  然后盲走 263mm 才扫到线
                                      * 盲走期间 n 全程 0，出来的那一帧位图 0E00 / 0780
                                      * （最左 2~3 路亮）—— 说明车已经跑到主线的**右侧** 40~50mm，
                                      * 传感器（±35mm）够不着，只能直着走等线自己扫回来。
                                      * 视觉上就是"左转拐出赛道、再斜着切回来"。
                                      * 横向外移 ≈ R(1-cosθ)，R = 弧长/θ_rad，所以 R ∝ v/ω：
                                      * 差速 750->1000（ω×1.33）、速度 350->250（×0.71）
                                      * => 旋转率 ×1.9，弧长 121->~65mm，外移 50->~27mm，
                                      * 期望枢轴一结束线就在传感器里（不再需要 260mm 盲走）。
                                      * tL = 250-500 = -250、tR = 750，内轮明确反转（FOLLOW_V_MIN=-575
                                      * 不会截断），是真正的原地枢轴。
                                      * 若枢轴看起来太猛/车身晃，把这两项一起回退到 350/375。
                                      * 原 350 的说明：
                                      * 关键关系：°/mm = ω/v，而 ω 只由差速决定。
                                      * 差速恒为 ISL_HEXT_EXIT_TURN(375)，把 v 从 875
                                      * 降到 350，同样的 55° 就只走 1/2.5 的里程：
                                      *   875mm/s：55° 要 271mm，圆弧半径 ~311mm，
                                      *            转完车离主线横向 ~90mm
                                      *            （传感器全宽才 ±35mm）-> 必然盲走
                                      *            -> 交 PD 后满舵打转（1.txt 圈2/圈3）
                                      *   350mm/s：55° 约 110mm，半径 ~126mm，
                                      *            横向 ~48mm -> 线还在传感器边上，
                                      *            PD 能接住。 */
#define ISL_HEXT_JUNC_MIN_DEG     320.0f  /* 交叉点触发出岛：累计偏航至少到这么多 */
#define ISL_HEXT_JUNC_MIN_FR       8U    /* 且这一簇要够长（帧数）。
                                          * 【11/25 15:45 实测 yaw=300.8 就触发了，出岛后直接飞出】
                                          * 环线上 yaw≈300 处另有一个短的两团白（0C3C/0E3C/0E3E…，
                                          * 只持续 4~5 帧），它不是出岛交叉口；真正的 X 交叉口在
                                          * yaw≈342，那一簇持续 20+ 帧（0C3C/0F1E/07E7/03F7/01CF/
                                          * 03CF/038F/078F/070F/0707/0F07/0E07/0E03）。
                                          * 所以用"簇帧数 >= 8"把短图案挡掉，yaw 门槛同时抬到 320。 */
#define ISL_HEXT_JUNC_MM          120     /* 且最近这么多 mm 内见过合格分叉簇 */
/* 【11/25 16:00 出岛的第二个锚点：X 交叉口处两条线并成一团宽白】
 * 15:45 那趟（1.txt）三次绕环出岛的对比：
 *   圈1  yaw 333.2  位图 00FF(n=8,单团)  -> 之后两团白 03CF 连续 3 帧 -> JUNC 341.9 出岛 OK
 *   圈2  yaw 315.2  位图 00FF(n=8,单团)  -> 出口两团白只闪 1 帧(0E07@345.3, fr=1)
 *                                          JUNC 不成立 -> 退回 345 度偏航兜底
 *                                          -> 出岛后 400mm n=0 -> PD 满舵打转(vL 2075/vR -589) -> LOST
 *   圈2  yaw 315.7  位图 007F(n=7,单团)  -> 两团白 2 帧 -> JUNC 330.8 出岛 OK
 * 结论：「两团白」只有 2/3 命中，而「两条线合并成一团宽白」3/3 都命中，
 * 用它补一个第二条锚点。但宽白出现得比交叉口**早**，所以只能记位置、不能直接开火：
 *   圈1 宽白 yaw 326.7（gz -363.3）-> JUNC 341.9，早 15° ≈ 36mm
 *   圈2 宽白 yaw 321.8            -> 交叉口 345.7，早 24° ≈ 60mm
 * 早 60mm 就左转 55°，主线还没到，等于把"出岛后全盲"提前复现。
 * 所以：宽白只**武装**（记 hext_blob_d），开火点是"宽白散开"那一帧 ——
 * 宽白 = 车正压在 X 上；n 从 >=7 掉到 <=5 = 已经过了交叉口，主线回到
 * 传感器下面，这一帧才是该左转的位置。
 * 偏航门不能省：环线前半段确实有宽白（圈1 在 76.3° 见过 n=9 的 01FF，
 * 256.7° 见过 n=7 的 007F），它们在 300° 之前，必须挡掉。 */
#define ISL_HEXT_BLOB_MIN_DEG     320.0f  /* 宽白锚点的偏航门（16:20 从 300 提到 320，与 JUNC 同值）。
                                      * 原因：300~310 这段环线上确实有假宽白（圈1 在 yaw 300.8
                                      * 就被两团白假触发过，出岛后全盲、转进墙）。
                                      * 320 以上的宽白才都是真交叉口，而且它只做“武装”，
                                      * 真正开火靠 BLOB 散开 / JUNC / 345° 三个中的任一个。 */
#define ISL_HEXT_BLOB_N             7U    /* 单团宽白：白路数 >= 7 */
#define ISL_HEXT_BLOB_GAP_MM       30     /* 宽白消失后这么多 mm 内允许"散开开火" */
#define ISL_HEXT_BLOB_DROP_N        5U    /* n 掉到这么多 = 已经压过 X 交叉口 */
#define ISL_HEXT_EXIT_HOLD_MM       420    /* 【11/25 16:45：260 -> 420 —— 这是本轮最关键的一改】
                                        * 【先纠正我之前读错的地方】260 / 261 / 263 反复出现，
                                      * 那不是"车要直走 260mm 才把线扫回来"，那是这个上限本身：
                                      *   16:15  HEXT,LINE,261,2,0
                                      *   16:30#1 HEXT,LINE,261,0,0
                                      *   16:30#2 HEXT,LINE,263,0,0
                                      *   16:35  HEXT,LINE,260,2,0
                                      *   16:40  HEXT,LINE,263,0,0
                                      * 五次末列全是 0 —— 一次都没找到线，全是走满 260mm 到点
                                      * 放弃。而线真正回到传感器（第一帧 n>=3）的位置是：
                                      *   枢轴结束 18490 -> 18788（n=5，线在左侧） = 298mm
                                      *   枢轴结束 18505 -> 18813（n=9）          = 308mm
                                      *   枢轴结束 ~18540 -> 18861（n=3）         = 321mm
                                      * 也就是说每次都只差 40~60mm 就等到了，却在 n=0 的状态下
                                      * 把控制权交回 PD（还带着打角期间攒的旧 e），
                                      * 875mm/s 满速冲出去 —— 现象才是"左转出赛道"。
                                      * 改成 420 后，PD 接手时线已经在传感器里，不需要它去追过期误差。
                                      * 调法：若 HEXT,LINE 仍是 ...,0,0，说明这段真的没线，
                                      *      先看 SOUT 之后 100mm 内有没有 n>=3；有就再加到 480。
                                      * 原值 260 的说明（打角完成后"锁航向找线"段的里程上限）：
                                      * 这一段不看 PD，只锁住刚摆正的航向直行，直到线真的
                                      * 回到传感器（n>=ISL_HEXT_EXIT_LINE_N 连续 2 帧）。
                                      * 【为什么不能立刻交 PD】16:15 那趟逐帧：
                                      *   打角 106mm/55° 期间线一路右移，e 从 1.5 涨到 7.0；
                                      *   打角结束 n=0，但 err 保持 7.0（满量程）；
                                      *   SOUT 后 PD 按 e=7.0 打满右转 -> vL 2000 / vR -660
                                      *   -> 原地右转 360°，越转越远离主线 -> LOST 18923。
                                      * 车头是对的（−55.1°），死在"用打角期间的旧误差去追"。
                                      * 260mm 的依据：按 θ=70° 的落点估算，车头过线 10°、
                                      * 横向偏 ~53mm，走下去约 100~180mm 线就回到 ±35mm 内，
                                      * 260 留 1.5 倍余量。 */
#define ISL_HEXT_EXIT_LINE_N          3U   /* 认"线回来了"的 n 门槛（3 路白） */
#define ISL_HEXT_EXIT_LINE_NMAX       6U   /* 【11/25 17:10】n 上限：>6 是宽白/岔口，不是可循的线。
                                            * 16:44 那趟 HEXT,LINE,353,12,1 就是被 n=12 的 0FFF
                                            * （岔口整片白）骗过去的，PD 拿到对称重心只会直行。 */
#define ISL_HEXT_EXIT_LINE_ERR       4.0f  /* 【11/25 17:10】|err| 上限：只认基本居中的线。
                                            * 【为什么必须有这条】16:58 那趟出岛后车又绕了整整一圈：
                                            *   SEX 345.4 -> 补圆右转 14.6° 到 yaw 360
                                            *   18377 立刻 n=3 bm=0007 e=+5.1  <- 这是**环线自己**
                                            *   HEXT,LINE,9,3,1  -> SOUT -> 交 PD
                                            *   交 PD 后 PD 顺着那条右侧的环线走，
                                            *   gyro -396.3 -> -104.6 = 又右转 291.7°/755mm
                                            *   = 0.386°/mm，正好是六边形环的曲率 -> 绕第二圈。
                                            * 主线在出口处应该在传感器中间附近（|e|<=4）；
                                            * 贴着右边缘 e=+5.1 的那条是环线继续往外弯。
                                            * 所以：只有"窄 + 居中"的线才算找到主线。 */
#define ISL_HEXT_EXIT_YAW_REL     356.0f  /* 【11/25 17:15 惯导出岛时钟】开火所需的累计偏航。
                                      * 【为什么改成"等陀螺"而不是"看图案"】
                                      * 7 次出岛按 SEX 时的累计偏航排开，成功与失败一刀切：
                                      *   yaw 344.5/345.4/352.2/353.4/355.0 -> 打角完看不见主线，
                                      *       要么走满 420mm，要么把环线当主线又绕一圈
                                      *   yaw 354.9/357.5 -> 打角后 2~3mm 就拿到 n=6、e≈2 的主线，
                                      *       两圈跑完
                                      * 也就是"环线与主线合并点"在 yaw≈356，而图案锚点
                                      * （JUNC/BLOB）本身有 ±20mm 抖动，落点常常差 5~20mm
                                      * 就提前开火 —— 差这一点点，车就还骑在环线上。
                                      * 编码器+陀螺的积分航向就是现成的"位置钟"：
                                      * 图案只负责**武装**（确认在环里），
                                      * 真正开火等 yaw >= 356，让每一趟都在同一个航向点出岛。
                                      * 一圈内的陀螺积分漂移只有 1~2°，比图案稳得多。
                                      * 调法：若 HEXT,LINE 仍经常 ...,0（走满 420），加大到 358；
                                      *      若 e 常为负且偏大（线在左侧、过冲），降到 354。 */
#define ISL_HEXT_EXIT_DR_EN          0U  /* 【11/25 17:55 停用】二维惯导是否参与开火。
                                      * 【为什么关】17:42 那趟就是它害的：
                                      *   HEXT,DR,-44,36,58  位置距原点 58mm（闸门 30 还没到）
                                      *   ISL,SEX,3453       却在 yaw 345.3 开火了 —— 只有
                                      *                      "DR 距离降到 30mm" 这条路能解释，
                                      *                      而窗口下沿是 350。
                                      * 结果开火早 5°，补圆角从 9° 变 14.6°，枢轴长 15mm，
                                      * 落点偏 -> 出岛后陀螺又涨 287°/800mm -> 绕两圈。
                                      * 实测单圈 DR 误差 56~59mm（≈3%/圈），比 yaw 窗口差，
                                      * 所以它只能当诊断（HEXT,DR 照打），不能参与开火。
                                      * 要做"全局惯导"更不行：整圈 20m 积分误差 0.5~1m。 */
#define ISL_HEXT_EXIT_DR_MM        30    /* 【11/25 17:35 二维惯导】位置回到岔口原点的判定半径。
                                      * 【为什么二维比一维准】现在 yaw 的零点定在 ISL,FIN，
                                      * 那是"进岛打角还没打完"的位置，之后 PD 还会再拐多少
                                      * 每趟都不一样：实测进岛 PD 时的偏航
                                      *   63.9 / 64.4 / 64.5 / 68.8 / 70.2 / 70.4  （散布 6.5°）
                                      * 这个差量原封不动加到出岛时的 yaw 上：
                                      *   出岛 SEX 偏航 350.7 / 351.1 / 351.1 / 357.7 （散布 7°）
                                      * 而闭环路径"回到起点"这个性质与进岛姿态无关：
                                      * 进岛拐多拐少、路径胖瘦，绕完一圈都回到同一个岔口。
                                      * 所以用 (x,y) 积分判"回到原点"比一维 yaw 更稳。
                                      * 预期误差 10~30mm（编码器 1~2% + 陀螺 1~2°）。 */
#define ISL_HEXT_EXIT_DR_MIN_MM   600    /* 惯导判据的里程护栏：走够这么远才允许"回原点"开火 */
#define ISL_HEXT_EXIT_GUARD_MM     30    /* 【11/25 17:30】护段内"走到这里就开火"的里程。
                                      * 【为什么不能再用 yaw 判】护段把航向锁住后 yaw 是常数，
                                      * 永远涨不到 ISL_HEXT_EXIT_YAW_LO(350)。17:24 那两趟
                                      * 就是这个后果：
                                      *   HEXT,GUARD,3407,785   护段开始
                                      *   HEXT,GUARD_END,3399,987   走了 987mm 才结束，
                                      *       车被锁直着开了快两米，直接开出赛道 -> LOST
                                      * 所以护段期间改用"护段走了多少 mm"开火：
                                      * 从 340° 到闭合点约 11° = 26mm，取 30mm。
                                      * 直行 30mm 在 0.42°/mm 的环上只丢 3mm 横向，可忽略。 */
#define ISL_HEXT_EXIT_GUARD_MAX_MM 120   /* 护段硬上限：超过就释放，绝不长时间锁直。
                                      * 到这儿还没开火说明这圈闭合点已经错过，
                                      * 交给 ISL,SEX_F / ISL,SEXBAD 兜底。 */
#define ISL_HEXT_EXIT_GUARD_EN       0U  /* 【11/25 17:50 停用】护段开关 0 = 关闭。
                                      * 【为什么关掉】护段会把开火点提前 4~5°：
                                      *   yaw 340 进入护段 -> 锁航向 -> 按"护段走 30mm"开火
                                      *   -> SEX 落在 346.6，而窗口里本来应该是 351。
                                      * 三版固件对比：
                                      *   17:15 无护段：SEX 351.1/350.7，补圆 8.8/9.2°，
                                      *        枢轴 21/23mm，SOUT 53/56mm  -> 两圈完成
                                      *   17:31 有护段：SEX 346.6/344.x，补圆 13.3/15.9°，
                                      *        枢轴 36/35mm，SOUT 64mm      -> 出岛后又绕一圈
                                      *   17:34 护段+SETTLE：SEX 346.9，SOUT 181mm -> 拽回环岛
                                      * 而且出岛里程从 53mm 涨到 181~320mm，正好吃掉下一个
                                      * 直角环岛的位置（现场反馈"会错过直角环岛"）。
                                      * 想要"合并区不甩车"的话，先解决开火点精度，再开这个开关。 */
#define ISL_HEXT_EXIT_GUARD_DEG   340.0f  /* 【11/25 17:25 惯导护段起点】从这里开始不再用 PD。
                                      * 出口最后 25~75mm 视觉会失效（环线与主线合并成宽白，
                                      * 加权重心失真 -> 直接 n=0），弯道恢复逻辑就把车打满
                                      * 甩出去。启用本护段后这一段改由陀螺锁航向直行。 */
#define ISL_HEXT_EXIT_YAW_LO      351.0f  /* 【11/25 17:20 闭合窗口下沿】
                                      * 【为什么要窗口，不能只给"到点就开火"的下限】
                                      * 17:11 那两趟出了"假出岛"（用户当场看出来）：
                                      *   JUNC 337 武装 -> 19094 yaw 352.9 到合并点
                                      *   -> 19121 丢线，19152 vL -402/vR 1709 向左甩 130°
                                      *   -> gyro -238.8 掉到 -369.3，yaw 从 353 掉回 222
                                      *   -> GAP 盲走 231mm，车在岔口附近乱走
                                      *   -> 之后 yaw 慢慢爬回 460，此时"yaw>=356"满足了，
                                      *      于是在 1.72 米（两圈）外的任意位置开火：
                                      *      ISL,SEX,4599,1720 / HEXT,EXIT,0,1,1,0 /
                                      *      HEXT,LINE,1,3,1（1mm 就"找到线"）
                                      * 也就是说：只有下限时，一旦在闭合点丢线甩车，积分航向绕一圈
                                      * 回来还会再满足一次，就变成假出岛。
                                      * 所以改成窗口 [LO, HI]：超过 HI 说明这一圈的闭合点已经错过，
                                      * 不许再补开火，交给 ISL,SEX_F / ISL,SEXBAD 兜底。
                                      * LO 取 350：实测真出岛落在 354.9 / 357.5；
                                      * HI 取 385：给甩车后的抖动留 25° 余量，又不至于放到 460。 */
#define ISL_HEXT_EXIT_YAW_HI      385.0f  /* 闭合窗口上沿（见 LO 的说明） */
#define ISL_HEXT_EXIT_ARMED_N        3U   /* 【11/25 17:20】开火那一刻必须在线上（n>=3）。
                                           * 盲走时不许开火 —— 假出岛那次 yaw 爬到 356 时正是 n=0。 */
#define ISL_HEXT_EXIT_DELAY_MM        0    /* 【11/25 16:36 新增：武装 -> 开火 之间先直行 60mm】
                                      * 四趟真进岛的出岛记录摆在一起看，盲走距离惊人地一致：
                                      *   16:15  SEX 332.9  打角 -55.1°  盲走 261mm
                                      *   16:30#1 SEX 333.8  打角 -71.1°  盲走 261mm
                                      *   16:30#2 SEX 321.6  打角 -71.4°  盲走 263mm
                                      *   16:35  SEX 321.6  打角 -72.3°  盲走 260mm
                                      * 打角 55° 和 72° 都盲走 260mm —— 说明盲走距离不是"转多少度"
                                      * 决定的，而是**打角发生在主线的哪一侧**：枢轴几乎是原地转
                                      * （16:35 实测 71mm 转 72.3°），转完车离主线还有 ~90mm，
                                      * 于是要直着走 260mm 才把线扫回 ±35mm 传感器里。
                                      * 90mm 横向 ≈ 0.866 × 104mm —— 也就是 SEX 比 X 交叉口
                                      * **早约 100mm**（约 25° 偏航）就开火了，而图案锚点
                                      * （JUNC 321.6~333.8）本来就比环线闭合点早 10~25°。
                                      * 所以：武装之后先保持直行 ISL_HEXT_EXIT_DELAY_MM 再打角，
                                      * 让枢轴正好落在 X 交叉口上。这 60mm 让"早 45~95mm"
                                      * 收到 "-15~+35mm"，横向偏差落进 ±35mm 传感器内，
                                      * 期望打角一结束线就能看见。
                                      * 调法：HEXT,LINE 如果还是 >100mm 才找到 -> 60 加到 80；
                                      *      如果线在右侧出现(e>0 且 bm 偏右) -> 说明打角过头，减到 40。 */
#define ISL_HEXT_EXIT_SETTLE_MM       80   /* 【11/25 20:12：0 -> 80，重新打开直行段】
                                      * 为什么又回来了：109.txt 第 1 圈偏置已经生效
                                      * （HEXT,OUTB,-20 / OUTBEND,162），车还是绕整圈。
                                      * 逐帧看：SOUT 时 e=+0.1 居中的，53mm 后环线跑到右侧
                                      * 边缘（bm 000F，原始 e=+4.3），偏置只把平衡点挪到
                                      * "环线左侧 20mm"，环线始终在 ±35mm 视野内，PD 继续
                                      * 右打 0.27°/mm × 1240mm = 329° -> 又是一圈。
                                      * 结论：误差偏置够不着，必须靠"锁航向直行"把车带走。
                                      * 几何：环曲率 0.27~0.42°/mm，直行 80mm 横向分离 24mm，
                                      * 100mm 分离 37mm（超出 ±35mm 传感器）——环线自动出视野。
                                      * 取 80 而不是当年被回滚的 120：现在 SEX 落在 351.x、
                                      * need 只有 8~9°（当年护段开着是 346.6 / need 13.3°），
                                      * 出岛点比那时靠后，不需要再走那么长。
                                      * 调法：SOUT 后仍出现 0.3°/mm 的持续右转 -> 加到 100~110；
                                      *      出岛后丢线/冲过下一个直角口 -> 减到 60。
                                      * 以下是 17:50 回滚时的记录（保留作历史）：
                                      * 17:15（无护段、无 SETTLE）出岛总里程只有 53~56mm，
                                      * 两圈都跑完了；17:34（护段+SETTLE120）变成 181mm 并
                                      * 被 PD 拽回环岛，还吃掉了下一个直角环岛的位置。
                                      * 现在回到"打角完直接找线"，直行长度由找线段自己决定（有上限）。
                                      * 原 120 的说明：
                                      * 【为什么不能再"一找到线就交 PD"】
                                      * 17:31 那趟：SEX 346.6 -> 补圆右转 13.3°（正好把一圈
                                      * 走完，车仍骑在环线上）-> HEXT,LINE,10,3,1（10mm 就
                                      * "找到线"）-> SOUT 64 -> 交 PD。
                                      * 但合并点处环线与主线是**相切**的，传感器分不出哪条；
                                      * PD 顺着连续的那条环线走，结果整圈又绕一遍：
                                      *   SOUT 后 gyro -306.2 -> +186 = 右转 325°/700mm
                                      *   = 0.46°/mm，正好是六边形环的曲率。
                                      * 所以合并之后要"直着走出去"再交 PD：
                                      * 环线曲率 0.42°/mm，直行 120mm 后环线已偏 50°、
                                      * 横向偏 51mm（超出 ±35mm 传感器），
                                      * 此时传感器里只剩穿过合并点的那条直线（主线）。
                                      * 调法：若 SOUT 后仍出现 0.4°/mm 的持续右转，加到 150；
                                      *      若 SOUT 后立刻丢线（n=0），减到 90。 */
#define ISL_HEXT_EXIT_FIND_MM       120    /* 【11/25 17:45】SETTLE 之后"等线回来"这一段的里程上限。
                                      * 【为什么要有这一段】17:34 那趟（SETTLE=120 已生效）：
                                      *   HEXT,EXIT,136,35（补圆右转 13.6°）
                                      *   18675 n=6 e=3.1   <- 直行中段还有线
                                      *   18716 n=0 e=6.0   <- 直行 120mm 结束：线没了
                                      *   ISL,SOUT,HEX8,181,0  <- n=0 交出去
                                      *   18810 vL 1653 / vR 308 -> 18854 vL 2094 / vR -449
                                      *   = PD 按打角时留下的旧 e=6.0 打满右转 -> 被拽回环岛
                                      * 赛道在合并点之后是往右弯的，硬直行越久越偏左，
                                      * 所以"固定 120mm"永远会在某个长度上把线走丢。
                                      * 正确做法：SETTLE 段先把环线甩掉，然后直行**等线回来**
                                      * （n>=3 连续 2 帧）再交 PD —— 交出去时误差是新鲜的，
                                      * PD 就不会再按旧误差打满。 */
#define ISL_HEXT_EXIT_STRAIGHT_MM    20    /* 【11/25 16:23：60 -> 20】接上线以后的收尾直行。
                                      * 线已经回来了，就该尽快交 PD，锁直行越久越容易被带偏。 */
#define ISL_HEXT_EXIT_STRAIGHT_MM_OLD 60   /* 【11/25 16:20：180 -> 60，并且改了计程起点】
                                         * 语义变了：这 60mm 从"转到 -55° 那一帧"开始算
                                         * （Island_ExitTurn 里重置 hext_exit_straight_d0），
                                         * 不再是"从 SEX 开始算 180mm"。
                                         * 为什么：180mm 的窗口里大头被打角吃掉了
                                         * （实测打角要 271mm），窗口早就在打角中途用完，
                                         * 于是 total 里程一到 SOUT 就强行收尾，车还歪着。
                                         * 现在打角由角度关门（到角即停），直行只补 60mm
                                         * 让角速度消下来，然后立刻交回 PD。
                                         * 历史：325（15:20）-> 180（15:45）-> 60（16:20）。 */

/* ==================== 出岛后的短程"外偏"（11/25，方案三）====================
 * 【要解决的问题】合并点处环线与主线**相切**，传感器分不出哪条。
 * ISL,SOUT 交回 PD 的那一帧，车还压在合并段上。107.txt 第 1 圈（19:45:10）：
 *   ISL,SEX,3512,920 -> HEXT,EXIT,97,24 -> HEXT,SETTLE,0,4,21
 *   -> HEXT,FIND,2,4,1,21（2mm 就"找到线"）-> ISL,SOUT,HEX8,46,4
 *   -> 之后 963mm 又绕一圈（yaw -412.2 -> -64.6 = 347.6°），
 *      直到第二圈再经过同一个合并点才出去。
 * 交出时残余 e=+2.1，PD 一路右打（10.316 帧 vL 1511 / vR 97），顺着环线走了整圈。
 *
 * 【做法】SOUT 之后一小段里程内，把误差目标往"环外侧"（主线一侧）拉：
 * 环在右边（dir=+1），所以偏置取负，PD 先被带到环线左侧，
 * 环线 0.42°/mm 的右曲率就再也咬不住车头；偏置带渐入/渐出，
 * 关掉那一刻不会阶跃回弹把车甩回环线。
 *
 * 【调法（跑完看日志）】
 *   仍在 SOUT 后 400~1100mm 内出现与合并点同型的 HEXT,FORK 簇 -> 偏置 2.0 加到 2.5/3.0
 *   出岛后向左跑偏、蹭到主线左侧或被 T 口甩出去 -> 偏置减到 1.5，或把 MM 缩到 120
 *   正常：HEXT,OUTB 之后 e 从 +2 附近被拉回 0 附近，且不再重复出现合并点叉口簇。 */
#define ISL_HEXT_OUT_BIAS_EN     0U     /* 【11/25 20:12：1 -> 0，先关掉】
                                      * 109.txt 实测它顶不住：偏置只是把 PD 的平衡点
                                      * 挪到"环线左侧 20mm"，环线仍在 ±35mm 视野里，
                                      * PD 照样一路右打绕整圈（yaw 0.27°/mm × 1240mm）。
                                      * 误差偏置要甩掉 20~35mm 外的线得开到满量程 7，
                                      * 那就不是微调了。改由 SETTLE 锁航向直接走开：
                                      * 直行 80~100mm 后横向已分离 24~37mm，环线自动出视野。
                                      * 代码保留：等 SETTLE 把出岛点钉住以后，若还差一点
                                      * 横向收尾，再把它开成 1U、偏置退回 1.0~1.5。 */
#define ISL_HEXT_OUT_BIAS_MM     160    /* 偏置窗长（mm）：20mm 渐入 + 保持 + 60mm 渐出。 */
#define ISL_HEXT_OUT_BIAS        2.0f   /* 起始偏置（误差单位，e 满量程约 ±7）。 */

/* 【11/21 12:40 等待下限 9000 -> 6000：9000 把唯一的真入口也一起挡住了】
 * 判据本身没坏，坏的是这个下限。用 6 趟同一份固件的日志（rr11/t39 ≈ 1.txt、
 * rr8/t36、t26、rr7、r4/t27、rr10/t38）标定"2 团白 + 跨度>=9"的那个簇：
 *
 *   plan3 起点(dist)   真分叉 rel = dist - isl_d_phase_raw    绝对里程
 *   9119 / 9287        8794 / 8511                          17913 / 17798
 *   9341 / 9376        8609 / 8635                          17950 / 18011
 *   9378 / 9562        8553 / 8425                          17931 / 17987
 *   9597               8383 / 8413                          17980 / 18010
 *
 * 真入口稳定落在 rel 8380~8800，而 9000 的下限把它**全部**挡住。
 * 1.txt 就是最典型的：14 帧合格分叉（rel 8794~8872）一帧没用上 ——
 * 旧配置（hold 150 / 无 ARM）的入口窗口是 rel 9000~9022，只有 22mm 宽，
 * 而 9000 之后再也没有合格分叉（后面 12.9m 里一次都没再出现），于是整车走直通支路
 * 过了 18235/19038 两个 T 口，整圈没进过六边形。
 *
 * 旧注释里"+9147 最早真入口"是旧赛道另一个元素的数字，不是这一格，别再用。
 * 真正的假入口（方形/断路口）在 rel 106 / 1420 / 2566；
 * 新固件日志里最高的一个假分叉是 r4/t27 的 rel 5088（06F2，只有 1 帧就消失）。
 *
 * 【11/21 15:17 68/69 两趟重新标定】
 *   69 有一次在八边形之后的直角弯提前进岛：rel 7358（abs 16476），
 *   而真入口在 rel 8380~8800（68 的 17991/18086、69 的 17853/18020）。
 *   6000 的下限挡不住 7358 这个假入口，所以提高到 8000：
 *   比假入口高 642mm，比最低真入口 8383 低 383mm。
 *   真入口最早 8383，仍有余量；若下一趟真入口被挡，先看 HEXT,ENTRY 的 rel 再微调。 */
#define ISL_HEXT_WAIT_MIN_MM       8000

/* 分叉候选已经出现时，放弃里程往后让这么多 mm。
 * 防止图案刚好在 skip_mm 前一两帧出现、还没来得及确认就被 Island_Advance。 */
#define ISL_SKIP_GRACE_MM       150

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
 * 新赛道一圈 10 个环岛，同一个里程值不可能都适用。
 * 【11/19 23:00 连同使用它的 FIN_F 分支一起删掉了 —— 见 Island_Step 里那段说明：
 *  它其实从来没被真正停用，是个会凭空触发"假进岛"的隐患。】 */

/* 进岛后走够这么远强制出环（兜底，防止图案判据失灵时绕不停）：
 * 六边形周长约 800 mm，八边形约 1200 mm，取 1600 留足余量。 */
#define ISL_EXIT_MM        850       /* 队友版：进岛后走这么远强制出岛 */

/* ==================== 七、丢线找线 / 断线盲走 ==================== */
/* 丢线容忍时间分两种场景 —— 一个值无法同时满足：
 *   普通赛道（直角弯）：要快，130 ms 就开始打角，否则冲出去；
 *   分叉候选已出现（isl_cfm=1）：要慢，等图案稳定再判断，
 *     否则偶尔丢一两帧就被"找线"拐离环岛入口，进不去岛。 */
#define LOST_MS            40U       /* 普通赛道：丢线 56 ms 就打角（约 45 mm @800）。
                                      * 【130 ÷2.29 = 56 ms】判据本质是"允许丢多远"，
                                      * 速度 ×3 后毫秒数必须 ÷3 才是同一段距离。 */
#define LOST_ISL_MS       157U       /* 分叉候选期：放宽到 220 ms，别急着打角。【500 ÷2.29】 */
#define RECOVER_MS        374U       /* 找线/盲走总时长。【1200 ÷2.29 = 525 ms，同一段里程】
                                      * 不除的话 1050 mm/s 下会拐出 1260 mm 的大圈。 */
#define ISL_RECOVER_MS   931U       /* 环岛会话内找线的时长（比普通赛道长）。【3000 ÷2.29 = 1310 ms】
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
/* 【11/26 直角弯打角：RECOVER_TURN 加到 1100，同时加“恢复段转角上限”】
 * 背景：114.txt 用 400/1100/-1000 跑时，第二圈出小六边形、过下一个直角后，
 * 34.6k 丢线，PD 用上一帧 +2.7° 的误差硬转，430 mm 内转了 190°，
 * 接回的是来时的线 -> 掉头往回跑（后面 44.4k 的 GATE 也是假的）。
 *
 * 这次的组合：
 *   · RECOVER_TURN 625 -> 1100：直角弯恢复段转得更狠（差速 1025 -> 1550 一侧），
 *     枢轴过弯更干脆；
 *   · 新增 RECOVER_SWING_MAX_DEG：从最后一次“稳稳看到线”（yaw_lock，
 *     n>0 且角速度 < 60°/s 才刷新）算起，转角到 95° 开始线性收力、
 *     140° 时转向量归 0 —— 直角弯只要 90°，超过 140° 还在转就一定是
 *     转过头了，继续转只会接回来时的线（掉头）。
 *   · RECOVER_V / FOLLOW_V_MIN 保持 450 / -575（50.7 s 那版），
 *     不跟上次一样放到 400 / -1000：内轮反转深度先不动，先看这一版的稳定性。
 * 现场如果又出现“丢线后车头摆过头”，先把 RECOVER_TURN 退到 900 再试。 */
#define RECOVER_V         450.0f     /* 找线速度：慢一点转弯半径更小。【160 ×2.29 = 360】 */
#define RECOVER_TURN     1100.0f     /* 找线时的转向量：625 -> 1100。【220 ×2.29 = 500】 */

/* 【恢复段转角上限（防掉头）】
 * 判据：当前丢线/正在找线时，|gyro_angle - yaw_ok| 超过这个角度就收力。
 * 实现：在 RECOVER_SWING_FADE_DEG 的窗口内把 turn 线性乘到 0（不是硬切），
 * 车最多转到 RECOVER_SWING_MAX_DEG 就停止同方向打角，改直行等线或超时停车。
 * 只在非环岛会话里生效（环岛有自己的绕行方向，不能一刀切）。
 * 直角弯 90°：95° 之前全力转，不会被误伤。 */
#define RECOVER_SWING_MAX_DEG    160.0f   /* 累计转角上限（°）
                                           * 【11/26 140 -> 160】115.txt 里八边形段丢线后
                                           * 车在 95° 就开始收力，直角弯可能没转完；先放宽
                                           * 到 160（115° 起收力），仍然转不到 180°。 */
#define RECOVER_SWING_FADE_DEG    45.0f   /* 最后这么多度内把 turn 线性收掉 */

/* 断线盲走：赛道上有 650 mm 的"断路口区域"没有线，
 * 丢线后不能朝最后方向打满（那会在缺口里原地打转），
 * 而要锁住航向直行冲过去。 */
#define GAP_MM           1500        /* 断路：斜穿 693 + 接线余量（不用于直角弯） */
#define GAP_MS           1245U       /* 断路专用超时。【4000 ÷2.29 = 1750 ms】
                                      * 换算是这样对的：1750 ms × 570 mm/s ≈ 1000 mm，
                                      * 和原来 4000 ms × 250 mm/s ≈ 1000 mm 完全一致，
                                      * 所以"能在哪断掉"这个行为不变。
                                      * 必须和 RECOVER_MS(525) 分开：那个是给直角弯
                                      * 丢线用的，拉到 1750 会让直角弯跑飞时多冲 1.2 m。 */
#define GAP_TURN_BIAS_DEG 0.0f   /* 【11/27 11:30：2.0 -> 0，回滚】断路再偏置已撤掉。
                                  * 加上 2° 之后现场反馈"断路现在往右边偏太多"，
                                  * 所以回到只给几何值 GAP_TURN_DEG(30°) 的原状。
                                  * 需要再微调时：撞左肩 -> 1~2；偏右 -> 0（当前）。
                                  * 原 11/27 10:50 的说明：断路斜穿再额外向右偏 2°。
                                  * 现场反馈：有时穿完断路会撞到断路结束后左边的路肩，
                                  * 说明落点偏左 —— 盲走目标航向再往右掰 2°。
                                  * 650mm 的斜穿上 2° ≈ 23mm 横向，正好是"擦肩"量级。
                                  * 这个偏转只作用于真断路（gap_corner==0）；
                                  * 直角弯丢线（gap_corner=1）的目标仍是原航向不动。
                                  * 调法：还撞左肩 -> 3~4°；从右边出去/压右肩 -> 1° 或 0。 */
#define GAP_TURN_DEG     30.0f       /* 断路斜穿角，右偏为正（20 偏小，实测接上了但角度不足，
                                      * 24 -> 25 -> 26 -> 27（11/25 20:53）-> 30（11/26 现场）。
                                      * 每 1° 约 11 mm 横向：27° 时落点还差约 30 mm，+3° 补上。
                                      * 实测几何：水平 650 mm、右侧接收线低 240 mm，
                                      * atan(240/650) = 20.3°。
                                      * 保持原航向（0°）会从目标白线上方 240 mm 飞过。 */
#define GAP_CONFIRM_N     5U         /* 接线确认：连续这么多帧见到线才交还 PD，
                                      * 防止扫到反光/杂物就提前结束盲走 */
#define GAP_MIN_THIN_MM   100        /* 断路判定前置条件：丢线之前必须已经连续走了
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
#define GAP_V            1250.0f     /* 盲走速度 mm/s。【11/25 20:50：712.5 -> 1250】
                                      * 原值 250×2.29=570，后来按比例提到 712.5。
                                      * 提速理由：断路都是赛道上的干净直道，盲走本质是
                                      * "航向保持直线冲过去"，速度越高越接近真实走线；
                                      * 而且巡航已经 1125（FASTSEG 段 1260），
                                      * 盲走反而比巡航慢会形成减速再加速的顿挫。
                                      * 风险点：丢线判定窗口内车会多冲一段
                                      * （1250 mm/s × 130 ms ≈ 160 mm），
                                      * 若出现"断路线段末端重新见线时冲过头"，
                                      * 先降到 1100，再考虑改 GAP_KP。 */
#define GAP_KP              7.12f     /* 航向保持：航向差 1 度 -> 5.7 mm/s 差速。【2.5 ×2.29】 */
#define GAP_TURN_MAX      375.0f     /* 航向保持差速限幅。【130 ×2.29 = 300】 */

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
#define BAR_CONFIRM_N      5U        /* 之后连续这么多帧回到细线才确认"薄" */
#define BAR_WIDE_MAX      8U        /* 宽白最多持续这么多帧（控制周期 10 ms）。
                                      * 【15 -> 6 帧】15 帧在 350 mm/s 下是 150 ms / 52 mm；
                                      * 换成 800 mm/s 后同样 6 帧 = 60 ms = 48 mm，与原意等价。
                                      * （1050 时曾取 5 帧 = 47 ms = 49 mm；降到 800 后
                                      *   5 帧只有 40 mm，偏紧，所以回到 6 帧）
                                      * 不收紧的后果：宽白会被当成"竖条"（BAR,WIDE 不再触发），
                                      * 整圈的 T 口判据全部错掉 */
#define BAR_DUP_MM        80         /* 两根竖条不可能这么近，用于去重 */

/* ---- 成对判据（中间圈认圈用）----
 * 发车区是"工"字两根竖条，正常通过时实测相距 409 / 415 / 418 / 419 / 425 / 426 mm。
 *
 * 【11/24 19:2x 250~650 -> 200~900：677mm 那次把整圈判据卡死了】
 * 90.txt 第一段（18:13）：
 *   22814 第一根认到了（GATE,FIRST，窗口内）
 *   23243~23392 车在门口横向偏到 e=+6.0、只剩最右 2 路（bm=0003）
 *   第二根竖条从阵列边缘掠过 -> 读不出 n>=11 -> 既没 T 口锁也没 BAR,HIT
 *   23491 由 677 mm 之后另一个整排白顶上 -> GATE,NOPAIR,22814,23491,677
 * 只差 27 mm 就被 650 的上限拒掉，后果是整场没有 Lap_Reset
 * （计划表卡在 3/3 + hext_locked=1 -> 第二圈不走大套小、也进不了最后一个六边形）。
 *
 * 为什么放宽是安全的：真正的防误配不在间距上限，而是这三条前置条件
 *   ① 第一根必须落在 FOLLOW_LAP_MM ± BAR_LAP_TOL_MM 的里程窗口里；
 *   ② 门前必须积够 gate_z_events >= GATE_Z_MIN(3) 次弯道丢线（Z 字特征）；
 *   ③ 第一根前面必须有 >= BAR_THIN_MM(120mm) 的干净细线、且 |err|<BAR_ERR_MAX。
 * 再加 BAR_DUP_MM(80) 去重，普通方框 T 口（间距 450~2000，且门前没有 Z 字）很难凑成一对。
 * 下限 250 -> 200 只是顺手给"斜穿压缩"留余量，实测没出现过小于 409 的情况。 */
#define PAIR_MIN_MM       200
#define PAIR_MAX_MM       900

/* ---- 里程窗口 ----
 * 中心 = FOLLOW_LAP_MM（本圈里程），容差 = BAR_LAP_TOL_MM。
 * 窗口外一律不理（BAR,OUT），只有进窗口的整排白才有资格当"发车门"。
 * 第二圈 lap_start 记在第二根，窗口还要补 gate_gap（见 Lap_Gate）。
 *
 * 【11/24 19:4x 1000 -> 1500】
 * 91.txt 那趟两根竖条在 21686 / 22111（间距 425，完全正常），
 * 但旧窗口下限 22386 把它们整体挡掉 → 两次 BAR,OUT → 不成对 → 没有 Lap_Reset。
 * 单圈里程会随"有没有开负压 / 打滑多少"在 21.9k~23.0k 之间漂（见 FOLLOW_LAP_MM
 * 里那张实测表），±1000（4.3%）根本不够，±1500（6.6%）才覆盖。
 * 放宽窗口的代价由另外三条判据兜着（见上面 PAIR 那段）：
 *   ① 必须在窗口内；② gate_z_events >= 3；③ 第一根前 >=120mm 干净细线且 |err|<3。
 * 现场判读：出现 BAR,OUT 且第二列（本圈里程）与窗口边缘的差 < 200 mm，
 * 就说明窗口又擦边了 —— 先看 LAP 行第三列的实测单圈，再决定调中心还是调容差。 */
/* ---- 里程窗口 ----
 * 【11/24 20:4x 起窗口是自适应的】（见 Lap_Gate）
 *   第一圈：中心 = FOLLOW_LAP_MM(23000)，容差 = BAR_LAP_TOL_MM(2000) → [21000, 25000]
 *   第二圈起：中心 = 上一圈实测单圈，容差 = 实测/8（±12.5%，最少 800）
 * 之所以要这么宽：单圈里程 = 轮子实际滚过的路程，抓地越好越短、速度越高越长。
 *   900 mm/s + 负压 → 21.9k；1350 mm/s → 25.6k（93.txt 实测，+17%）。
 * 窗口外一律不理（BAR,OUT）。放宽窗口的代价由另外三条判据兜着：
 *   ① 必须在窗口内；② gate_z_events >= 3；③ 第一根前 >=120mm 干净细线且 |err|<3。
 * 现场判读：`BAR,OUT` 第二列是 d_cell，跟窗口边缘差 <200mm 就是又擦边了；
 * 但只要有 `LAP` 打出第 4 列（学到的容差），下一圈就自动跟着实测值走。 */
#define BAR_LAP_TOL_MM    2000

/* ---- 门前 Z 字特征 ----
 * 73.txt 门（T3/T4）之前，T2 到 T3 之间连续出现 5 次弯道丢线：
 *   20204 / 20940 / 21208 / 21758 / 22023
 * 门后下一圈起步同样有一串 Z 字弯。
 * 因此门除了里程窗口 + 成对，还要求门之前已经积累 >=3 次 gap_corner；
 * 这样 T1/T2 和途中单个 T 口不会单独成门。 */
#define GATE_Z_MIN        3U

/* ---- 停车点（两圈统一用两竖条门） ----
 * 第一圈：成对确认后直行通过，并 Lap_Reset；
 * 第二圈：成对确认后停车。停车点取两根竖条的中点，
 * 光电管在发车区内，车主体也完整落在两根竖条之间；
 * STOP_MID_OFFSET_MM 正值后移，负值前移，实车再微调。 */
#define STOP_MID_OFFSET_MM   0      /* 停车点取两根竖条中点；正值后移，负值前移 */
#define GATE_GAP_DEFAULT_MM  430    /* 第一圈漏掉成对时，第二圈用默认门间距预测停车点 */
/* 【11/25】第二根竖条漏读的补位余量：走够"预期间距 + 这个值"还没见到第二根，
 * 就用预测位置成对。150 足够让真竖条晚到一点，又不至于把 500 之外的杂物算进来。 */
#define GATE_PRED_MISS_MM    150

/* 里程兜底的额外余量。
 * 【必须给】11/18 16:13：竖条在 20569，而 goal_mm(=FOLLOW_LAP_MM×FOLLOW_LAPS)
 * 正好是 20580 —— 里程兜底抢在竖条门确认之前 11 mm 就把车停了，
 * BAR,HIT 需要的"3 帧回到细线"（约 30~50 mm）根本来不及。
 * 加 600 mm 余量后，门总是先有机会；只有在门完全认不出来时才用里程兜底。
 *
 * 【600 -> 1500（11/19 22:15）】这是"总是提前停车"的直接原因。
 * 里程兜底 = goal_mm(20580) + 这个余量；55.txt 那两趟都是 26 s、
 * 走到 21188 / 21182 mm 就被 LAPS,DONE 停掉 —— 车刚跑完一整圈，
 * 但计划表才到第 4 格（方块格配对失败，见 SQ_MAX_GAP_MM 那段），
 * 后面 5 个环岛全没轮到。用户看到的是"提前停车"，其实是里程判据，不是超时。
 * 1500 让兜底落到 22080 mm：真正常跑完时门（stop_mm）先停，用不到它；
 * 计划表落后时也能多给 1.5 m 的机会，而不是在发车区前面把车掐掉。
 *
 * 【1500 -> 3000（11/24 20:4x）】提速后打滑变大，单圈里程会涨：
 * 900 速是 21.9k，1350 速实测 25.6k（93.txt）。兜底点 = FOLLOW_LAP_MM×2 + 余量
 * = 46000 + 1500 = 47500，两圈真实里程一旦到 48k，兜底就会抢在第二圈门口把车停掉。
 * 余量放到 3000（兜底点 49000），保证"门先有机会"这条原则在提速后仍然成立；
 * 代价是门完全认不出时车多跑 1.5 m。 */
#define MILEAGE_STOP_MARGIN_MM  3000

/* ==================== 八、保护 ==================== */
#define STALL_MS          300U       /* 两轮都不转多久算堵转 */

/* 盲盒收工标志：1 = 车已经按盲盒停在 T 口，别让 Align_Hold 再把它拱走。
 * 每次按发车键清 0（见 main）。定义在 #if 外面：main 里也要用。 */
static uint8_t bb_hold = 0U;

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

/* ==================== 位图"白团"计数 ====================
 * 【11/21 12:21 趟：六边形入口判据抓到的是假岔口】
 * 把每趟"入口前 200mm"的位图列出来后规律很干净：
 *   真入口前面一定有【两团白】：0E07 [3,3]、0707 [3,3]、0C07 [3,2]
 *   假入口前面一次都没有，全是一整条连续宽白：0FF0 [8]、007F [7]、00FF [8]、003F [6]
 * 全日志里 n>=5 且有两团白的帧共 425 个 —— 说明信号本来就存在，
 * 只是判据没要求它（旧判据只要求 n=5~8 且最右四路亮，连续宽白也算）。
 * 所以入口改成"必须是两团白"：runs>=2 且 n<=9（一整条连续的 runs 只有 1 段）。
 * 返回值 = 长度 >= 2 的连续白段数。 */
static uint8_t Line_Lobes(uint16_t raw)
{
  uint16_t b = (uint16_t)(raw & 0x0FFFU);
  uint8_t  cur = 0U, lobes = 0U;

  while (b != 0U) {
    if ((b & 1U) != 0U) {
      cur++;
    } else {
      if (cur >= 2U) lobes++;
      cur = 0U;
    }
    b >>= 1;
  }
  if (cur >= 2U) lobes++;

  return lobes;
}

/* ==================== 位图"白团跨度" ====================
 * 【11/21 12:32 那趟：两团白判据放进了一个假分叉】
 *   HEXT,FORK,5,003B,11016 -> ISL,FABORT,602,541（600mm 只转 54°，自检判假）
 * 假分叉 003B = bits 0,1 + 3,4,5：虽然是两团，但只占阵列左半（跨度 6 路）。
 * 把历次"真/假"位图的跨度列出来，区分非常干净：
 *   真 0E07 -> 0..11  跨度 12
 *   真 0707 -> 0..10  跨度 11
 *   真 0C07 -> 0..11  跨度 12
 *   真 0C38 -> 3..11  跨度  9   （rr8 / rr9 两趟真进岛都是这个）
 *   假 003B -> 0..5   跨度  6
 * 所以再要求"跨度 >= 9"：真岔口是两条线在大半个阵列上分开的，
 * 假分叉只是局部两条线重叠。 */
static uint8_t Line_Span(uint16_t raw)
{
  uint16_t b = (uint16_t)(raw & 0x0FFFU);
  uint8_t  i, lo = 0xFFU, hi = 0U;

  for (i = 0U; i < 12U; i++) {
    if ((b & (1U << i)) != 0U) {
      if (lo == 0xFFU) lo = i;
      hi = i;
    }
  }

  return (uint8_t)((lo == 0xFFU) ? 0U : (hi - lo + 1U));
}

/* ==================== 转向环 ==================== */
static float line_i = 0.0f;

/* ==================== 【11/24 23:3x 运行期可调参数（串口在线调参）】====================
 * 目的：调 PID 不用每次都"改代码 -> 编译 -> 烧录"，在串口上直接发 `KP=450` 就生效。
 * 只把"调参最常动的 11 个量"提成变量，其余常量继续用宏（保持编译期优化）。
 * 全部是 RAM 值：重新上电回到宏里的默认值，比赛前发 `DEFAULT` 也能一键复位。
 * 边界在 tune_items[] 里，越界直接拒绝，不会把车调成飞车。
 * 日志里的 CFG,GAIN / CFG,V 现在打印的是【实际生效值】，不是宏值，
 * 所以每份日志都自带"这趟用的什么参数"。 */
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
 * 【11/20 重构】计划表只保留 3 个真正需要绕行的环岛：
 *   1 大套小 → 2 小六边形 → 3 第 8 格六边形
 * 中间的 4 个方形、八边形、最后方形全部从计划表删除；
 * 它们的 T 口由 Island_Step 的全局 T 口分支锁直行，不推进 plan_idx。
 * 第 8 格入口改成纯图案触发（n=5~8 且最右四路亮），不再依赖八边形出口锚点。
 * 最后一个方形在 HEX8 之后，由 hext_done 标志继续保持 T 直行，直到门/Lap_Reset。
 *
 * 状态（11/19 23:00 精简：原来有 6 个，其中 3 个是同一个动作）：
 *   ISL_NONE     正常循迹
 *   ISL_CROSS    锁直行中。**这一个状态同时承担三种场合**：
 *                  穿 T 口（全局 T 口直行）、进岛穿交叉点、出岛穿岔口
 *                三者的动作完全一样 —— 锁转向、按固定速度直行 N mm，
 *                差别只有"走多远 / 走够时还要求线变窄吗 / 超时多少"，
 *                这三项由进入那一刻填进 isl_lock_* 变量，下面只有一份实现。
 *   ISL_SESSION  六边形会话：正常循迹绕行（只有需要"绕"的格子用它）
 *   ISL_RET      冷却
 */

#define ISL_NONE     0U
#define ISL_CROSS    1U
#define ISL_RET      2U
#define ISL_SESSION  3U

/* ---- 锁直行结束后做什么 ---- */
#define LOCK_AFTER_EVENT   0U   /* 数一次"穿越事件"：够 need_ev 次就前进一格（方形/八边形） */
#define LOCK_AFTER_SESSION 1U   /* 开始绕行会话（顶点进入型六边形） */
#define LOCK_AFTER_ADVANCE 2U   /* 直接前进一格（六边形出岛） */
#define LOCK_AFTER_TONLY   3U   /* 全局 T 口：只锁直行，不推进 plan_idx */

/* 方形/八边形已从计划表删除，T 口直行改由全局分支处理。
 * 【几何修正：八边形的道口也是 90°，不是 45° 斜接】
 *   方形 T 口、八边形 90° 道口：n=11~12（0EFF/0FFF）——全局 T 锁直行；
 *   方框的 90° 直角：n=9~10（0FF8/0FFC/03FF）——绝不能当 T 锁直行。
 * 所以门槛必须是 11，不能用 9：用 9 会把第三个方形的直角弯锁直 60 mm，
 * 车就是从这里直着飞出去的。八边形若某次只读 n=9~10，同样只能交回 PD；
 * 想让它也锁直行，必须再加位置/序列判据，不能把门槛降回 9。 */
#define ISL_T_LOCK_MAX_MM   120    /* T 口锁直行的硬上限：走够这么远无条件放开。
                                    * 正常 T,OUT 是 60~67mm（等 n<=5 才放开）；
                                    * 11/24 出现过 161mm（宽白迟迟不散）-> 车冲过弯后丢线。 */
#define ISL_GLOBAL_T_N    11U    /* 【11/24 20:0x 10 -> 11：10 那版在方形直角弯上误锁直行】
                                  *
                                  * 11:48 那趟把它从 11 降到 10，理由是
                                  * "正方形区那两排十字经常读不满 12 路，只读到 0FFB / 0FFE
                                  *  这类 n=10~11"。**这个理由是数错了**：
                                  *   0FFB = bit0,1,3..11        -> 11 路
                                  *   0FFE = bit1..11            -> 11 路
                                  * 两个都 >= 11，原来的门槛根本没过它们。
                                  * 而真正 <= 10 的是方框的**直角**：
                                  *   0FFC = bit2..11  -> 10 路（左端亮、右端灭）
                                  *   07FE = bit1..10  -> 10 路
                                  *   0FF8 = bit3..11  -> 9  路
                                  *   03FF = bit0..9   -> 10 路
                                  * 92.txt 第二圈就是这么出事的（第一圈全是 0FFF/07FF，没问题）：
                                  *   32728 T,IN,10,07FE  -> 锁直行 61 mm -> T,OUT,61,0（线整条掉了）
                                  *   33724 T,IN,10,0FFC  -> 又锁一次          -> T,OUT,60,0
                                  *   之后一直丢线 -> 36446 STALL
                                  * 也就是车在方形环岛的**直角**上被锁着直行，冲出了赛道。
                                  * 这正是上面那段注释警告过的"门槛不能降到 10/9"。
                                  *
                                  * 恢复 11 之后：真的 T 口（0FFF/0EFF/07FF/0FFB/0FFE）照锁；
                                  * 方框直角（0FFC/07FE/0FF8/03FF）一律交回 PD 转弯。
                                  * ⚠️ 如果哪天出现"T 口只读到 10 路、没锁直行、车拐进方框"，
                                  *    不要降 n，改成给锁加"位置/序列"判据（例如必须在里程窗口内、
                                  *    或前 200 mm 是直行）。 */

/* ==================== 计划表 ====================
 * 【11/19 23:00 删掉了 isl_plan[] 这张表】
 * 原来有两张表描述同一件事：isl_cfg[]（每格的参数）和 isl_plan[]（每格是
 * "方形"还是"六边形"）。两者必须手动保持一致，而漏改过一次就会很难查：
 * 唯一读它的地方是 Island_IsSquare()，漏改的后果是整格走错分支
 * （方形格去等分叉、六边形格只在找 T 口）。
 * 现在"Is 什么类型"直接由 isl_cfg[].kind 回答，只有一份真相。
 * 【11/20 重构后的顺序】
 *   1 大套小 → 2 顶点六边形 → 3 第 8 格六边形
 * 中间的方形/八边形不再占格；它们只走全局 T 口直行。
 * 最后一个方形在 HEX8 之后，靠 hext_done 保持 T 直行，直到门/Lap_Reset。
 *
 * 历史教训（保留）：曾经把第 3 格写成六边形，而赛道上大套小之后只有
 * **一个** 六边形，紧接着就是正方形。于是车走完六边形后 plan_idx 以为还在
 * 等六边形：正方形的 T 口 n=12 被分叉判据拒掉、T 口判据又只在方形格调用，
 * 结果正方形完全不可见，车以 290~300 mm/s 直插 90° 直角冲出赛道。 */
#define ISL_PLAN_N    3U

static uint8_t  isl_state = ISL_NONE;
static uint8_t  isl_cfm   = 0U;
static uint8_t  plan_idx  = 0U;         /* 当前该处理计划表里的第几个（0 基） */
static uint8_t  hext_done = 0U;         /* HEX8 已完成/跳过：之后的最后方形 T 口仍走全局直行 */
static uint8_t  hext_locked = 0U;       /* HEX8 出岛后锁死入口，直到 Lap_Reset/下一次 BIGHEX */
/* 【11/21 11:05 那趟：HEXT 是"进早了"，不是"进不对"，入口要加一条"先在线上"的前提】
 * 两趟对比（同一套 35° 打角参数）：
 *   成功那趟：进岛前 100 mm+ 连续 n=3/4（0070/00F0）→ 绕出 274.3°/729 mm
 *   失败那趟：18652 GAP,DONE（恢复刚结束）→ 18677 只见到 22 mm 线
 *             → 18699 ISL,FIN,HEX8 就在那里判定进岛 → 18721 起 n=0
 *             → 恢复里转 542° → 19371 LOST
 * 也就是入口判据（n=5~8 + 最右四路 + 里程≥9000）在任何位置都能满足，
 * 包括刚恢复完的那 20 mm —— 打角从一个随机姿态开始，必然炸。
 *
 * 用什么判"已经在线上"：**数帧，不用里程**。
 * 里程在 800 mm/s 下会因打滑虚高（实测同一圈 20400~22000，散布 1.5 m），
 * 拿它做 200 mm 门限本身就不可靠；而控制周期固定 10 ms，
 * "连续 15 帧细线" = 150 ms，和打滑/标定无关，且正好对应原来想表达的 ~120 mm。
 * 更新在 Follow_Line_Run 每帧做（n==0 清零、n=1~4 累加、n>=5 宽白保持不清零，
 * 因为进岛那一帧本身就是宽白）。 */
static uint16_t g_clean_frames = 0U;    /* 连续"细线"帧数（10 ms/帧），供 HEXT 入口判据用 */
/* 【11/21 12:35 那趟：真岔口"有时不出现" --> 把当场判据改成"最近见过就算"】
 * 那趟 18500~20900 全是连续宽白（0FF0/00FF/007F），一次合格分叉都没有，
 * 于是整趟没进岛。而之前几趟真岔口出现时，经常只持续 1~2 帧就被 GAP/宽白打断
 * —— 原来的判据要求"进岛判定的那一帧恰好是岔口帧"，太苛刻。
 * 现在：每帧只要看到合格分叉就把里程记下来，进岛判定在 150mm 内见过就算。 */
static int32_t  hext_fork_d  = 0;        /* 这一簇合格分叉的【首帧】里程（0 = 没记过） */
static int32_t  hext_fork_last_d = 0;    /* 这一簇最后一帧的里程；入口锚点用簇末 */
static uint8_t  hext_fork_n  = 0U;       /* 那一帧的路数（日志用） */
static uint16_t hext_fork_bm = 0U;       /* 那一帧的位图（日志用） */
static uint8_t  hext_fork_fr = 0U;       /* 这一簇累计出现了几帧（抗单帧抖动，见下） */
#define ISL_HEXT_FORK_HOLD_MM   200      /* 分叉"有效窗口"：簇末之后多少 mm 之内可进岛 */
/* 【11/21 15:xx 67.txt：入口开始绕行忽早忽晚】
 * 67 两次：
 *   簇首 8814 / 8836，簇末 8888 / 8904，ISL,FIN 8925 / 8948
 *   实际 FIN 都在"簇末 + 37~44 mm"，而"簇首 + 110 mm"会随簇长变化。
 * 所以锚点从簇首改成簇末：记录 hext_fork_last_d，簇末后走 40 mm 才进岛。
 * 窗口仍取 200mm，防止这 40mm 刚好落在短 GAP 里。 */
#define ISL_HEXT_FORK_ARM_MM    20       /* 簇末之后至少走这么远才允许进岛（10->20） */
#define ISL_HEXT_CLEAN_FRAMES   24U     /* 进岛前至少连续 15 帧（150 ms）在细线上 */
/* ==================== 分叉簇必须"持续"才算真分叉（11/24 新增）====================
 * 【问题】88/89 之后这版日志（1.txt，同一固件相邻两趟）里：
 *     run0（负压开）18003 那次进岛用的簇 簇末-簇首 = 0 mm（只有 1 帧）
 *        → 进错位置 → FABORT 301mm → 整格跳过；
 *     run0 23579 那次用的簇 8 mm（2 帧）→ 绕 1502 mm / 577°（1.6 圈）；
 *     run1（负压关）17951 那次用的簇 13 mm（3 帧）→ SEX 345.7°/1000mm 干净出岛。
 * 成因：风扇振动让比较器在线边缘抖，一条宽白被挖掉中间 4 路（0C3C/0E1C/0F1C
 * 这种位图）就长得和六边形岔口一模一样。单帧分辨不出来，但【真岔口会持续
 * 几十毫米到 300 mm，抖出来的假图案通常只闪 1 帧】——所以按"簇持续了几帧"过滤。
 *
 * 门槛用帧数为主、mm 为辅：mm 跨度与速度成正比（900mm/s 时 3 帧 = 13mm，
 * 400mm/s 时 3 帧只有 6mm），所以 mm 只能当"至少真的走了一段"的地板。
 * 现场判读：HEXT,FORK 末列是簇内第几帧；HEXT,ENTRY 末列是入口用的簇总帧数。
 * 若发现真入口被判掉（HEXT,FORK 在真岔口只有 2 帧），把 MIN_FR 降到 2U。
 * 依据（1.txt）：1 帧/2 帧的两次入口全错，3 帧那次正确。
 *
 * 【11/25 20:00：2U -> 3U】108.txt 那趟入岛失败就是 2 帧判据放过去的：
 *   HEXT,ENTRY,8,03BE,21,8512,10,2  —— 簇 2 帧、跨度 10mm、rel 8512
 * 这个假簇比真入口早了约 250mm（107 两趟真入口是 rel 8761/8810，
 * 簇 17 帧/69mm、15 帧/57mm），入口照常右打 60° 后那里没有环线，
 * 车拱进 T 口（n=12 0FFF）、被丢线恢复甩 61° 左转，最后 FABORT。
 * 3U 挡掉的就是这一类"抖出来的 2 帧宽白"。代价：万一某趟真岔口只抖出
 * 2 帧会漏判一次 —— 届时看 HEXT,ENTRY 末列（帧数）就能确认，再决定是否回退。 */
#define ISL_HEXT_FORK_MIN_MM     6       /* 簇最短跨度（速度相关的软门槛） */
#define ISL_HEXT_FORK_MIN_FR    3U       /* 簇最少帧数（真正起作用的门槛） */
/* 【11/27】簇最长跨度：超过它说明车在两条线之间"顺跑"，不是短促横穿。
 * 128 第二圈 HEXT,ENTRY,5,0C07,...,161,20：簇 161mm/20 帧，锚簇末 +20 后
 * 入口比真岔口晚 100~160mm，进环位置错 -> ISL,SEXBAD,1630,287。
 * 107 那趟真入口 5,0C07,...,57,15（57mm）不受影响：57 < 100。 */
#define ISL_HEXT_FORK_MAX_MM   100
static uint8_t  hext_entry_phase = 0U;  /* 0 无 / 1 右转40° / 2 直行1000 / 3 岛内 PD */
static uint8_t  hext_entry_line_cnt = 0U;
static uint16_t hext_entry_lost_cnt = 0U;
static int32_t  hext_entry_pd_d0 = 0;
static int32_t  hext_entry_straight_d0 = 0;
static float    hext_entry_comp_yaw0 = 0.0f;
static float    hext_entry_comp_target = 0.0f;
static int32_t  hext_entry_comp_d0 = 0;
static uint8_t  hext_entry_comp_done = 0U;
static uint8_t  hext_exit_phase = 0U;   /* 0 无 / 2 打角 / 3 锁航向找线 / 4 收尾直行 */
static int32_t  hext_blob_d    = 0;     /* 最近一帧"单团宽白"的里程；0 = 没武装 */
static float    hext_exit_yaw0 = 0.0f;
static int32_t  hext_exit_d0 = 0;
static int32_t  hext_exit_straight_d0 = 0;
static uint8_t  hext_exit_line_cnt = 0U;   /* "线回来了"连续帧计数 */
static int32_t  hext_exit_arm_d = 0;       /* 出岛武装那一刻的里程（0 = 未武装） */
static float    hext_exit_need_deg = 0.0f;
static int32_t  hext_outbias_d0  = 0;       /* 出岛外偏的起始里程；0 = 未生效 */
static float    hext_outbias_val = 0.0f;    /* 本次外偏的带符号大小（负 = 往环外侧/主线一侧拉） */
static uint8_t  hext_guard_on   = 0U;       /* 出岛前的惯导护段是否生效 */
static float    hext_guard_yaw  = 0.0f;     /* 护段锁住的航向（gyro_angle） */
static int32_t  hext_guard_d0   = 0;        /* 护段起始里程 */
/* ---- 二维惯导位置（HEXT 会话内，原点 = ISL,FIN 那一刻的岔口）---- */
static float    hext_dr_x       = 0.0f;     /* 沿"进岛时车头方向"的里程 mm */
static float    hext_dr_y       = 0.0f;     /* 侧向 mm（右为正） */
static int32_t  hext_dr_d0      = 0;        /* 上一次积分时的里程 */
static uint8_t  hext_dr_on      = 0U;       /* 0 = 本趟没在积分 */
static uint8_t  hext_dr_printed = 0U;       /* HEXT,DR 只打一次 */
/* 二维惯导接口（实现放在 Hex_ExitGuard 前面，这里先声明，因为 Island_Step / 入口
 * 都在它前面就要用） */
static void  Hex_Dr_Reset(int32_t dist);
static void  Hex_Dr_Update(int32_t dist);
static float Hex_Dr_R(void);
static uint8_t  sq_events = 0U;         /* 穿越事件计数：每格需要 need_ev 次才前进 */
static int32_t  sq_d0     = 0;          /* 第一次穿越的里程（算两次间隔用） */
static uint32_t isl_t_cfm = 0U;
static uint32_t isl_t_state = 0U;
static uint32_t isl_t_phase = 0U;       /* 计划格起始时刻 */
static int32_t  isl_d0    = 0;          /* 进入锁直行那一刻的里程 */

/* ---- 锁直行的三个参数（进入 ISL_CROSS 之前必须先填好）----
 * 合并 ISL_CROSS / ISL_HIN / ISL_HEXOUT 之后，这三种场合共用一份实现，
 * 差别全部由这三个数表达：
 *   穿 T 口（方形）：走 60 mm，还要求宽白消失（n<=5），超时 1500 ms
 *   进岛穿交叉点  ：走 10 mm，还要求线变窄（n<=10），超时 1200 ms
 *                   且以 in_cross_mm 为上界
 *   出岛穿岔口    ：走 out_cross_mm（170），不等线变窄，超时 3000 ms
 * 每次进入锁直行都必须重新赋值 —— 漏赋值会沿用上一格的数，
 * 后果和"大套小漏清 err_off"那类 bug 一样（第一次对、后面全错）。 */
static int32_t  isl_lock_mm   = 60;     /* 要走够多少 mm */
static uint8_t  isl_lock_nmax = 5U;     /* 走够后还要求 n <= 此值才放开；255 = 不等 */
static uint32_t isl_lock_tmo  = 1500U;  /* 兜底超时 ms */
static uint8_t  isl_lock_after = LOCK_AFTER_EVENT;  /* 放开后做什么 */
static float    isl_yaw0  = 0.0f;       /* 会话起始航向 */
static float    isl_err_off = 0.0f;     /* 切向进岛的误差偏移（等价补线），出岛清零 */
static int32_t  isl_block_dist = 0;     /* 在这之前禁止进入任何环岛（大套小刚结束用） */
static int32_t  isl_ret_dist = 0;       /* 上次会话结束时的里程 */
static uint8_t  isl_out_arm  = 0U;      /* 出岛武装：偏航到 ISL_EXIT_DEG 后置 1，
                                         * 之后任意一帧匹配分叉即出岛 */
static int32_t  isl_d_fin    = 0;       /* 本次会话进岛的里程，强制出岛的基准 */
static int32_t  isl_d_phase  = 0;       /* 当前这一计划格开始时的里程（里程兜底用；
                                         * 盲走结束后会把 glen 补进来，见 GAP,DONE） */
static int32_t  isl_d_phase_raw = 0;    /* 本格"原始"起点：不补盲走里程。
                                         * 只给 HEXT 的入口窗口、自校正闸门和
                                         * ISL,FIN 的相对里程用 —— 这些要的是车在本格的
                                         * 物理位置，不是"扣掉盲走后的剩余预算"。
                                         * 两者分开，避免一次长盲走把入口窗口整体前移。 */
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

/* 发车门状态机：两圈都按“两根竖条成对”确认 */
#define GATE_WAIT_FIRST   0U
#define GATE_WAIT_SECOND  1U
#define GATE_DONE         2U
/* 【11/26 新增】停车后的“软件盲盒”也挂在这条状态机上：
 *   GATE_DONE（已停车）→ GATE_BB_SPIN（原地 180°）→ GATE_BB_STRAIGHT（直行找 T 口）
 *   → GATE_BB_DONE（收工）。动作全在 Follow_Line_Run 末尾按顺序执行，
 *   只是相位由 gate_state 记录，日志里能一眼看出“停完车走到盲盒第几步”。 */
#define GATE_BB_SPIN      3U   /* 盲盒：原地自旋 180°（只用陀螺仪） */
#define GATE_BB_STRAIGHT  4U   /* 盲盒：锁航向直行到 T 口 */
#define GATE_BB_DONE      5U   /* 盲盒：已收工 */
static uint8_t  gate_state = GATE_WAIT_FIRST;
static int32_t  gate_gap_mm = 0;      /* 第一圈成功成对后学到的两竖条间距 */
static int32_t  lap_mm_learned = 0;   /* 【11/24】上一圈实测单圈里程；0 = 用常量窗口 */

/* 【11/26】过了“真断路”之后到进岛之前不再提速。
 * 115.txt：八边形这一段（13.2k~18k）在 FASTSEG 段里跑 1260 mm/s，
 * 每帧 6.3 mm，而 T 口整排白只有 7~8 mm，加上 printf 阻塞 4~6 ms，
 * 采样间隔能到 11 mm —— 直角/T 口判据整帧被跨过，连续丢 6 次线。
 * 真断路的判据是 gap_corner==0；置位后 Island_FastSeg() 返回 0，
 * 直到换圈（Lap_Reset）或重新发车（Island_Reset）才恢复。 */
static uint8_t  fastseg_block = 0U;
static uint8_t  gate_t_count = 0U;    /* HEX8 出岛后经过的 T 口数：T3/T4 是门 */

/* ---- 圈数 / 停车 ---- */
static int32_t  lap_start_mm = 0;  /* 本圈起点里程：发车=0，每过门更新 */
static uint8_t  lap_cnt   = 0U;    /* 已确认过的圈数 */
static int32_t  stop_mm   = 0;     /* 0 = 不停；非 0 = 走到这个里程就停 */
static uint8_t  lap_wrap_pending = 0U;  /* 1 = 计划表刚回绕到第 1 格，等 Island_Step
                                         * 拿到 dist 后补做换圈重置（见 Island_Advance） */
static uint8_t  gate_z_events = 0U;     /* HEX8 出岛后累计的弯道丢线次数；门前的 Z 字特征 */

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
#define ISLK_HEXT    4U     /* 切向进入的六边形：主线与一条边成 30~35°，要打角才能进去，
                             * 顺时针绕一圈，从同一点出来（图纸第 9 个元素标注 35°）。
                             * 入口图案只稳定 1~2 帧，Island_Step 对 HEXT 用
                             * 纯图案触发：n=5~8 且最右四路亮，不再用里程窗口。 */

typedef struct {
  const char *name;          /* 日志里显示的名字 */
  uint8_t  kind;             /* ISLK_LOOP / ISLK_CROSS / ISLK_BSS / ISLK_HEXV / ISLK_HEXT */
  uint8_t  fork_n;           /* 进岛：白路数下限 */
  uint16_t fork_mm;          /* 进岛：图案要持续的距离。
                              * HEXT（第 8 格）已改成窗口内单帧，写 0；
                              * 其它类型仍按这个值做"两帧/持续距离"确认。 */
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
  uint16_t entry_off10;      /* 进岛补线偏置（0.1 单位，满量程 70）；0 = 用全局 ISL_ENTRY_OFF(2.5)。
                              *
                              * 【为什么必须按格配】补线偏置要和入口交叉区的"宽度"配套：
                              * 交叉区里两条分支被加权平均，PD 天然想直行，只有偏置能把
                              * 车推到环线那条分支上；偏置能起作用的里程 = 从判进岛那一刻
                              * 到车真正越过交叉区。
                              *   格1 大六边形：边长 300，交叉区宽，2.5 就够（实测能进）
                              *   格8 六边形：边长 200，交叉区窄，2.5 还没把车推偏就到出口了
                              * （11/19 现场确认：车在格8 岔口根本不拐、直接沿主线走；
                              *   环岛在车的右侧，所以方向没错，是力度不够。） */
  uint8_t  need_ev;          /* 【11/19 23:00 新增】本格需要几次"穿越事件"才前进一格。
                              * 只有 ISLK_CROSS 用得上，其它类型忽略（写 1）：
                              *   2 = 方形：进、出各穿一次 T 口（原来写死在代码里）
                              *   1 = 八边形：只穿一次就前进（它现在是"直穿型"格子）
                              * 有它之后，"方形"和"八边形"就成了同一份代码的两个配置，
                              * 不再需要两套状态机。 */
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
/* 【11/19 22:15 · 上限 1800 -> 2000】速度 ×3 之后车走的线更外，
 * 同一个方块的两个 T 口间距从 1694 涨到 1922~1935，直接顶破了 1800：
 *   55.txt 第 3 格实测 15760 / 17682 / 19617 / 20477
 *     (15760,17682) gap=1922 -> SQ,FAR 被拒
 *     (15760,19617) gap=1935 -> SQ,FAR 又被拒
 *     (19617,20477) gap=860  -> 才配对成功
 * 一个方块吃掉两个方块的赛道，计划表每格落后一格，4 个方块之后落后 4 格，
 * 于是八边形和第 8 格六边形都拿错了规则（略过 / 乱进岛）。
 *
 * 上限只能放到 2000，不能再大：历史测量的"跨格"那一簇起点是 2015
 * （2015 / 2510），2000 和它只差 15 mm。这个窗口本来就窄，
 * 换线、换速度、换地面都要重新量一遍。若再出现 SQ,FAR 连发，
 * 先看日志里那两个 T 口的间距落在哪一簇，不要再盲目加这个数。 */
#define SQ_MIN_GAP_MM     450
#define SQ_MAX_GAP_MM    2000

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
  { "BIGHEX",  ISLK_BSS,    5U,   60,   700,  4200,   180,   825,   0,    0,    0,    0,    0,    0,  1 },  /*  1 大套小（专用）：岛内 290×2.29=660 */
  /* 【11/24 23:1x】skip_mm 500 -> 1500：
   * 这一格的入口不在起点附近（96.txt：真正的候选在 760 / 1186 / 1818 mm），
   * 500 mm 的窗口只够走到大套小出口那个十字就被判死。
   * 配合 ISL_ENTRY_MIN_MM(400)：400mm 之前不理，400~1500mm 之间才有机会认到真入口。 */
  /* 【11/25 15:05 skip_mm 1500 -> 500，恢复已标定的相位锚点】
   * skip_mm 决定"第 2 格什么时候交棒"，也就是第 3 格（HEX8）的相位起点
   * isl_d_phase_raw。改成 1500 后交棒晚约 1000mm，于是 HEX8 入口的 rel
   * 整体左移 1000：本趟第 2 圈六边形的真分叉落在 rel 7851~7905，
   * 而 ISL_HEXT_WAIT_MIN_MM=8000（+ "簇本身必须在下限之后"那条闸）把它判掉
   * -> 整圈没进岛（只有第 1 圈 rel 8622 那个卡线过）。
   * 500 是 WAIT_MIN=8000 / ARM=20 / HOLD=200 这批参数当初标定时用的值，
   * 那时真分叉稳定落在 rel 8800~9200，八角后面 rel~7860 的假簇正好被挡住。
   * 若以后确实需要 skip_mm=1500（让第 2 格多试一次顶点入口），
   * 必须同时把 ISL_HEXT_WAIT_MIN_MM 降到 ~7200，并重新确认八角假簇不会再进。 */
  { "SMALLHEX",ISLK_HEXV,   5U,   48,   340,   850,   170,   850,  +1,   60,   80,    0,  500,   40,  1 },  /*  2 顶点进入：dir -1 -> +1 */
  /* 【11/20 重构】方形 / 八边形 / 第 9 格方形从计划表删除。
   * 它们只保留“T 口锁直行”行为（见 Island_Step 里的全局 T 口分支），
   * 不再占用 plan_idx，也不再有进岛/出岛/偏航/跳格状态。
   * 计划表只剩 3 个真正需要绕行的环岛：大套小、小六边形、第 8 格六边形。 */
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
   * v_island 300 -> 200 -> 300 -> 200（11/19 17:38 又改回 200）
   *   【旧注释"降速 200 是错的"已被 17:38 那趟推翻】
   *   旧结论的依据是 exit_fall=2200 时代的样本（983 mm / +352°），
   *   但 exit_fall 改成 1400 之后，两趟在同一个格7起点上的实测是：
   *     run3  335/200, v_isl=200: 1401 mm -> 128.9°，出岛瞬间 n=4（在线上）
   *                              -> 20077~20327 正常巡线 -> BAR,HIT 20317 -> 完赛
   *     run6  350/210, v_isl=300: 1400 mm ->  89.1°，出岛瞬间 n=0（不在线上）
   *                              -> 丢线 -> 恢复 154 mm -> 漏判终点竖条
   *                              -> 误进 HEX8 -> 冲出赛道
   *   两趟格7起点只差 29 mm（17929 / 17900），进岛点同样 560 mm 强制，
   *   所以差异不在里程基准，而在出岛姿态。
   *
   *   【判据不是"绕了多少度"，是"出岛那一刻车在不在线上"】
   *   整份 1.txt 里 ISL,SEX 出现 0 次，所有环岛都是 SEX_F 兜底退出，
   *   八边形从来没绕到 exit_deg(340)。既然绕不到，就只能挑一个
   *   "兜底退出时刚好压在白线上"的速度 —— 实测 200 那一趟是 n=4。
   *   而且 200 时 1401 mm 只转 128.9°（0.092°/mm），轨迹平缓，
   *   出岛横向偏移小；300 时 89.1°，说明车在岛内被甩来甩去。
   *
   * exit_fall 2200 -> 1400：保留。 */
  /* 【11/19 按赛道图重算 exit_deg 340 -> 274；exit_fall 先缩到 1150，实测不行又退回 1400】
   * 依据是图纸尺寸（赛道图.png，标注单位 mm）：
   *   八边形：8 条边 × 200（顶边 250）-> 周长 = 1600 mm
   *   八边形底部两根竖线相距 400 mm -> 进/出口相距 400 mm，
   *     实际绕行 = 1600 - 400 = 1200 mm，转角 = 360° - 400/1600*360° = 270°
   *
   * 用同图纸的六边形做标定（6 × 200 = 周长 1200 mm）：
   *   日志里 SMALLHEX 每次都是 ISL,SEX_F,850,274°，
   *   850/1200*360 = 255°，实测 274°，误差 7% —— 说明"周长比例"这个算法是对的。
   *   按比例推到八边形：850 * 1600/1200 = 1133 mm，出口角同样是 274°。
   *
   * 【原来的 340 是错的，而且错得很致命】
   * 340° 在 1600 mm 的八边形上对应 1511 mm，比兜底 1400 还远 ——
   * 也就是说 exit_deg 永远不可能先于 exit_fall 触发。
   * 整份 1.txt 里 ISL,SEX 出现 0 次、ISL,SEX_F 出现 7 次，就是这个原因：
   * 车每次都是"还没绕到出口角就被里程强退"，出岛航向随缘，
   * run3 侥幸压在线上（n=4）完赛，run6 没压上（n=0）就跑飞。
   * 改成 274° 之后，正常情况会在 ~1133 mm 处走 ISL,SEX 的
   * "锁定航向直穿岔口"流程，而不是在波浪形/岔口中间被硬吐出去。
   *
   * exit_fall 1150 = 1133 + 17 的余量：只做兜底，正常不该触发。
   *
   * 【11/19 19:30 实测：1150 太早，退回 1400】那趟是第一趟跑这组参数：
   *   17673 ISL,FIN_M,OCT7,561      强制进岛（正常）
   *   18823 ISL,SEX_F,1150,1309     走满 1150 强退，累计偏航只有 130.9°
   *   19004 ISL,SOUT,OCT7,170,0     出岛锁直行结束那一帧 n=0 —— 车已不在线上
   *   19045 n=12 0FFF -> 19063 n=3 0E00 -> 19084 n=3 0380 -> 19107 n=0
   *   19261 GAP,START,19261,1,11    丢线前 thin 只有 11 mm -> 判成弯道
   *   19461 LOST                    满舵找线 1.2 s 无果，停车
   * 也就是说：提前 250 mm 出岛，车是在岔口中间被放出来的，170 mm 直行锁
   * 把它推到线外，之后 100 mm 线从左边缘溜掉。"第 8 格六边形进不去"只是
   * 结果：格 8 起点 19004，车 19107 就丢线，ISL,FIN,HEX8 一次都没轮到
   * （盲走期间 Island_Step 被冻结，入口判据根本没机会跑）。
   *
   * 对照现场唯一跑完过的那趟（335/200，同样是 200 岛内速度）：
   *   exit_fall=1400 -> 1401 mm 出岛、出岛瞬间 n=4（在线上）
   *   -> 正常巡线 -> BAR,HIT 20317 -> 完赛。
   * 所以退回 1400。exit_deg 保持 274：这一趟 274 根本没触发过，不是本次变量。
   *
   * 【11/19 20:26 实测：八边形和修好之前的六边形一样，根本没绕】53.txt：
   *   17955 PLAN,7/9 格7起点 / 18520 ISL,FIN_M,OCT7,561,5,003E
   *         -> 又是在 n=5、bm=003E 的普通细线上强行进岛（真岔口 0FF8 在格内 ~1237）
   *   18520~18824 300 mm：偏航 -503.1° -> -469.9°，只转 33°，线一直是 n=3 0380
   *         -> 车是在主线上跑，不是绕环
   *   19916 ISL,SEX_F,1400,1297  1400 mm 只累计 129.7° = 0.093 °/mm
   *         而真绕八边形要 0.22 °/mm（1600 周长 / 360°），差一倍还多
   * 所以问题在【入口】，不在出岛参数（274 还是 340 都到不了，因为车不转）。
   * 照六边形那套改三处：
   *   fork_mm   30 -> 10    候选一出现就进岛，别把打角浪费在交叉区末尾
   *   force_mm 560 -> 1250  把"里程强制"挪到真岔口之后，让 n>=9 的图案判据
   *                         先有机会（force 判据在代码里排在图案之前，
   *                         560 会把 0..1237 这段图案入口整个屏蔽掉）
   *   entry_deg 60 -> 120 / entry_off10 0 -> 40
   *                         和大套小(130°/2.5) 同级的打角：八边形入口更宽，
   *                         偏置也要给足，否则 PD 仍然顺着直的那条走
   * 注意 exit_deg 保持 274、exit_fall 保持 1400：等入口修好、车真的绕起来，
   * 274° 才第一次有机会被触发，这一趟不用改它。 */
  /* ============ 【11/19 23:30 治本：八边形就是"两个 T 口"，和方形同一份逻辑】============
   * 上面那一整段都在调"怎么把车拐进八边形、绕完怎么出来"，绕了很多轮都没成。
   * 现场确认的事实是：**八边形也是两个道口，进一个、出一个** —— 也就是说它和
   * 方形环岛在结构上是同一个东西；【11/20 现场纠正：八边形的道口也是 90°，不是 45°】。
   *
   * 光电管读数也印证了这一点（53.txt 第 7 格区间 17955~20099 的全部宽白）：
   *   18299 n=9  0FF8
   *   19186 n=9  0FF8
   *   19410 n=10 03FF
   *   19651 n=9  0FF8
   *   19871 n=10 03FF
   * 全都是 9~10 路，**一次 n>=11 都没有**。旧解释是“道口斜 45°”，
   * 但现场已纠正：八边形的道口实际也是 90°；9~10 路只是那一帧有效通道少，
   * 不能据此把八边形门槛降到 9（降到 9 会把方框直角也当 T 口）。
   * OCT7 已从计划表删除，不再存在“八边形 fork_n=9”的配置。
   * 现在全局 T 门槛固定 11；八边形 90° 道口若读 n=9~10，只能交回 PD。
   *
   * 下面这段字段说明只作为历史记录保留，不对应当前计划表。
   *
   * 字段含义（改这一行之前先对一遍，这个表已经被人写错位过两次）：
   *   kind=CROSS      走"穿越型"路径（和方形共用代码）
   *   fork_n=9        历史值（旧解释误以为道口 45°；实际八边形道口也是 90°）
   *   fork_mm=20      CROSS 格不用它（T 口确认走 ISL_CONFIRM_MS），留着对齐格式
   *   out_cross_mm=60 锁直行距离，和方形一样（横线 12 mm + 余量）
   *   skip_mm=2500    里程兜底：这一格走 2500 mm 还没凑齐两次就当它失败、前进一格。
   *                   比一格长（20580/9 ≈ 2286），不会误伤；但能防止"道口这次
   *                   没读出来 → 整圈卡在第 7 格"。
   *   need_ev=2       进一个 T 口 + 出一个 T 口
   *   force_mm=0      不再用里程强制进岛（那是"绕行版"才需要的）
   *   exit_deg/exit_fall/v_island/dir/entry_deg/entry_off10 全部 0（绕行版的量）
   *
   * ⚠️ 如果以后规则要求八边形必须绕行，回退办法：kind 改回 ISLK_LOOP，
   *    恢复 exit_deg=274 / exit_fall=1400 / v_island=600 / entry_deg=120 / entry_off10=40。
   *    但请注意：那套虽然代码还在，实测从来没有真正绕成功过。 */
  /* OCT7 也从计划表删除：八边形的两个 T 口由全局 T 口直行处理，不再占格。 */
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
  /* 【11/19 按赛道图重算 exit_deg 340 -> 274、exit_fall 2000 -> 1000】
   * 格8 这个六边形和格2 的 SMALLHEX 是同一规格：图纸上都是 6 × 200 mm，
   * 周长 1200 mm。SMALLHEX 实测 850 mm / 274°（现成的标定样本），
   * 所以这里出口角同样取 274°、兜底取 1000 mm（850 + 150 余量）。
   *
   * 原来的 exit_fall = 2000 mm：在 1200 mm 的六边形上相当于绕 600°，
   * 整整一圈半还多。就算真进去了也会在岛里绕两圈才出来，
   * 出来时位置和航向都不可能对。这个值明显是从八边形那行抄漏改的。
   *
   * exit_deg 340 同理到不了（340° = 1133 mm > 周长 1200 的 94%，
   * 而且进/出口本身有 400 mm 重叠），必须降到 274 才能触发 ISL,SEX。
   *
   * 【11/19 19:46 那趟（52.txt）曾被误读成"已经绕了半圈"——已作废】
   * 当时看到入口后偏航一路涨到 187° 才回落，就判定"车在环里绕了 3 条边"，
   * 于是把 exit_deg 改成 175、out_cross 改成 60。现场纠正：**车在岔口
   * 根本没拐，那 187° 是主线自己在那一段往右弯**。所以那两个值已经回退，
   * 出岛仍然是 274°/170 mm。
   *
   * 【真正的病灶：入口打角力度不够】（11/19 现场确认，环岛在车右侧 → dir 方向没错）
   *   格1 大套小：n>=5 持续 60 mm → err_off +2.5 → 能进
   *   格8 六边形：n>=5 持续 30 mm → err_off +2.5 → 车直接沿主线走掉
   * 两者偏置量相同，差别在**交叉区宽度**：大六边形边长 300，两条分支分开得慢，
   * 偏置有足够里程把车推过去；小六边形边长 200，交叉区窄，30 mm 就判进岛，
   * 偏置刚生效车已经出交叉区了。
   * 两处一起改：
   *   fork_mm      30 -> 10   候选一出现就进岛，把偏置生效点提到交叉区开头
   *   entry_off10  0 -> 40    本格偏置 2.5 -> 4.0（新增按格字段，格1/格2/格7 不受影响）
   * 预期日志：ISL,FIN,HEX8 之后 100 ms 内应该出现明显右转（偏航每 100 ms >20°），
   *          并且偏航单调涨过 274°、由 ISL,SEX 正常出岛（不再是 SEX_F）。
   *
   * 【11/19 20:26 那趟（53.txt）：确实进环了，但顶点丢线】那一趟是第一次成功：
   *   20760 ISL,FIN,HEX8,12,0FFF -> 20820~21030 一条边 -> 21057 顶点 n=7 00FE
   *   -> 21206 第二条边 -> 21536 -> 21570 n=0 丢线（第三个顶点）
   * 也就是说 4.0 的打角已经能把车送进环，但进环后车"顺着边走到顶点就甩出去"。
   *
   * 【11/19 20:47 现场：入口只是"有右转趋势"，没真正拐进去 → 再激进一档】
   *   entry_off10  40 -> 60   偏置 4.0 -> 6.0（满量程 7，等于把目标点推到阵列最右）
   *   entry_deg   120 -> 160  打角持续到 160°，拐进去之后再多顶 40°
   * 说明：这是故意调过头的一档，用来看"到底是不是偏置不够"。
   *   - 如果车拐进去了（哪怕切内角、压线）→ 说明方向对，下一版把 6.0 往回收（4.5~5）
   *   - 如果还是只有趋势、进不去 → 那就不是偏置问题，是速度太高拐不进 200 mm 的边，
   *     下一步改 v_island 300 -> 200（转弯半径更小），不要再加偏置 */
  /* ============ 【11/20 根治：HEXT 单帧 + 位置窗口；fork_n 保持 5；skip=1500】============
   * 56.txt 两趟的实证：
   *   PLAN,8/9 后 1046 / 1118 mm 才出现 n=6 / n=8 的入口分叉；
   *   再过 163 / 82 mm 就被 skip_mm=1200 判死（ISL,HEXOVER,1209 / 1205）。
   * 三个原因叠在一起：
   *   ① 旧固件 fork_n=9（CFG,HEX8,9,21,...），n=6/8 的入口全被拒；
   *   ② fork_mm=16 在 800/1050 速下要第 3 个控制帧才确认（第 1 帧只记 d0，
   *      第 2 帧 8 / 10.5 mm < 16），而入口实测只稳定 1~2 帧；
   *   ③ skip_mm=1200 离入口只剩 82~224 mm，图案刚出现就判死。
   * 57.txt（800 速三趟）的关键证据：
   *   三趟的 PLAN,8/9 起点分别是 19778 / 18559 / 20518；
   *   第 1 趟 +532 的 00FF、第 2/3 趟 +570 / +500 的 0FF0 / 0FE0，
   *   全是 120° 弯的宽白，不是六边形入口；
   *   第 1 趟只是碰巧在 1000 mm 兜底出岛后回到主线上，看起来"进岛成功"。
   * 58.txt（800 速）：
   *   第 2 趟 +872 进 6,007E -> ISL,FABORT,843,241；
   *   第 3/4 趟在 +976 / +998 进 5,00F8，跑对。
   * 真入口汇总：51 +976、56 +1046、57 +1005、58 +976 / +998。
   * 所以下限取 920：落在 58 的弯道 +872 和真入口 +976 之间。
   * 不能取 800（+872 的弯会进来），也不能取 1000（会误杀 +976）。
   * 不能靠左右侧过滤：同一弯道在不同横向位置会读成左/右两种位图。
   * 所以 HEXT 不再等 fork_mm，改成纯图案触发（n=5~8 且最右四路亮）；
   * fork_n 保持 5：第 8 格历史上 n=5 成功过（11/18 16:00，ISL,FIN,HEX8,5,03E0），
   * 【11/20 重构】方形/八边形已从计划表删除，第 8 格不再有“八边形出口”锚点。
   * 入口改成纯图案触发：n=5~8 且最右四路至少一路亮；中间的方形 T 口由
   * Island_Step 的全局 T 口分支锁直行，不再消耗/推进 plan_idx。
   * skip_mm 放到 20000：从 SMALLHEX 出来到第 8 格要走完中间所有 T 口，
   * 不能再按 1500 的“一格”算。 */
  /* 【11/21 11:35 那趟按"环线曲率"标定出来的出口，不是按图纸猜】
   * 判别方法：六边形边线有固定曲率，实测出岛后 300 mm 走了 +79° = 0.26 °/mm；
   *   **车真正离开环线的标志是这个曲率掉到 ~0**（开始沿主线直走）。
   * 那趟数据：
   *   18864 ISL,SEX,2757,663   ← 判据在 275.7° 就出岛了
   *   之后 300 mm  gyro +79°  = 0.26°/mm   ← 还是环线曲率 -> 根本没出去
   *   之后 1180 mm            = 又绕了约 360°（整整一圈）
   * 所以 275.7° 时车还在边中间；从 275.7° 再过 84.3°(=360-275.7) 才回到入口顶点，
   * 按 0.26°/mm 折算 ≈ 324 mm -> 顶点在进岛后 663+324 ≈ 987 mm。
   * 结论：几何顶点在 360°；但出口还有 15° 打角，若 360° 才触发 SEX，
   * 打角结束时车已经 375°，直行锁晚了 15°（70.txt 的"再多绕一点"）。
   * 所以 exit_deg 360 -> 345：345° 触发 SEX，15° 打角结束正好 360°，
   * 再开始 300mm 强制直行，直行锁落在顶点而不是顶点之后。
   * exit_fall 1500；ISL_OVER_DEG 420（360+15 仍小于 420）。
   * 验证：ISL,SEX 应打出 ~340~350；ISL,SOUT 后 PD 不应再沿环线绕。 */
  /* 【11/25 15:30 exit_deg 345 -> 360：出岛必须发生在环线闭合的 X 交叉口】
   * 345° 时车还在交叉口前约 15°（≈50mm），此时就开始左打角
   * -> 车在主线还没出现的位置就离开了环线 -> "左偏太多、飞出赛道"（本趟实测：
   *    SEX 之后 700mm 就丢线，GAP,START,41025,1 走弯道恢复才救回来）。
   * 360° 正好是 X 交叉口，主线就在那里，此时左转 55° 才能摆到主线上。 */
  /* 【11/25 15:40 exit_deg 360 -> 345：360 是"等偏航凑够"的兜底，
   * 主触发已改为交叉点图案（见 Island_Step 里的 HEXT,JUNC）。
   * 345 是历史实测能到、能触发的值（345.3 / 345.7），留作图案失效时的保险。 */
  { "HEX8",    ISLK_HEXT,   5U,    0,   345,  1500,   400,   575,  +1,  100,    0,    0, 20000,   18,  1 }   /*  3/3 六边形：exit 345°+15°出口打角=360°；
                                                                                                    * 原来 360° 触发，打角完已 375° 才锁直行，晚 15°，70 会多绕一点。
                                                                                                    * out_cross 400 给 15° 打角 + 300mm 直行留空间。
                                                                                                    * 【11/23 16:23 现场反馈"内切角度有点大"：25/130 -> 18/100】
                                                                                                    * 第 8 格只有大套小那个的一半大（2.7mm/度 vs 5.0mm/度），
                                                                                                    * 同样的内缩占半径的比例翻倍，所以偏置要打折：
                                                                                                    *   entry_off10 25 -> 18  （内缩 ≈18mm -> ≈13mm）
                                                                                                    *   entry_deg 130 -> 100  （只覆盖岔口，不把整条边都带偏）
                                                                                                    * 下限提醒：偏置 < 2.0 单位时，岔口 -5 的重心会重新饱和成满舵左，
                                                                                                    * 所以 18 已经接近"还能顶住主线"的下限，别再往下降；
                                                                                                    * 若这趟还内切多，优先继续缩 entry_deg（100 -> 80），不要动偏置量。
                                                                                                    * 【补线偏置本体】沿用大套小那条已经验证过的机制（BS_ERR_OFF），
                                                                                                    * 只是按环的大小打折：
                                                                                                    *   进岛前 100° 把 PD 的**目标点**往环内偏 1.8 单位（≈13mm），
                                                                                                    *   不是加转向偏置。PD 会主动把车开到偏内侧并稳住，
                                                                                                    *   主线在左边拉它时净误差 -5+1.8=-3.2，不再像没有偏置那样
                                                                                                    *   直接满舵左把 yaw 拉回去（77.txt 的 0FE0→0E00）。
                                                                                                    * 代码路径：Island_Step 的 entry_deg/entry_off10 -> isl_err_off
                                                                                                    *         -> Island_ErrOffset(dist) 加进 e（和大套小 bs.err_off 同一条）。
                                                                                                    * 出岛那一侧的偏置走另一条：ISL,SOUT 时置 hext_outbias_val，
                                                                                                    * 同样由 Island_ErrOffset 叠加，见 ISL_HEXT_OUT_BIAS。 */
};

static const IslandCfg *Island_Cfg(void)
{
  return &isl_cfg[plan_idx % ISL_PLAN_N];
}

/* 当前计划格是不是"穿越型"（需要锁直行穿过 T 口/岔口的那一类：方形 + 八边形）。
 * 直接由配置表的 kind 回答，不再维护第二张 isl_plan 表 —— 见上面计划表那段。 */
static uint8_t Island_IsSquare(void)
{
  return (uint8_t)(Island_Cfg()->kind == ISLK_CROSS);
}

/* HEXT 入口硬闸门：出岛后 hext_done=1，或本圈已经锁定 hext_locked=1，
 * 所有入口路径（force / 顶点 / 分叉）都必须先过这一关。 */
static uint8_t Island_HextBlocked(void)
{
  return (uint8_t)(Island_Cfg()->kind == ISLK_HEXT
                   && (hext_done != 0U || hext_locked != 0U));
}

/* HEX8 入口阶段开始：右转 15° -> 直行 20mm -> 岛内 PD */
static void Island_HextEntryStart(int32_t dist)
{
  hext_entry_phase = 1U;
  hext_entry_line_cnt = 0U;
  hext_entry_lost_cnt = 0U;
  hext_entry_pd_d0 = 0;
  hext_entry_straight_d0 = 0;
  hext_entry_comp_yaw0 = 0.0f;
  hext_entry_comp_target = 0.0f;
  hext_entry_comp_d0 = 0;
  hext_entry_comp_done = 1U;   /* 盲走+直行入口，不做 phase4 补偿 */
  hext_exit_phase = 0U;
  hext_outbias_d0  = 0;        /* 新会话：出岛外偏还没开始 */
  hext_outbias_val = 0.0f;
  hext_blob_d     = 0;
  hext_exit_arm_d = 0;
  hext_guard_on   = 0U;
  printf("HEXT,BLIND_START,%ld\r\n", (long)dist);
}

/* HEX8 出口阶段开始：不再单独等 15° 打角完成，SEX 后立刻进入 phase2，
 * 由 15° 航向闭环在直行中完成打角，锁直行的时刻不再晚 15°。 */
static void Island_HextExitStart(float yaw_now, int32_t dist, float yaw_at_sex)
{
  /* 【11/25 17:00】出岛角度不再写死：把这一圈补完所需的右转角。
   * yaw_at_sex 是 ISL,FIN 以来累计的右转量，一圈 = 360。 */
  float need = 360.0f - yaw_at_sex;

  if (need < 0.0f)               need = 0.0f;
  if (need > ISL_HEXT_EXIT_NEED_MAX) need = ISL_HEXT_EXIT_NEED_MAX;
  hext_exit_need_deg = need;

  hext_exit_phase = 2U;
  hext_exit_yaw0 = yaw_now;
  hext_exit_d0 = dist;
  hext_exit_straight_d0 = dist;
  hext_exit_line_cnt    = 0U;
  hext_exit_arm_d       = 0;
  hext_entry_phase = 0U;
}

static void Island_Advance(uint32_t now, const char *why)
{
  uint8_t was = plan_idx;

  if (was == 2U) {
    /* HEX8 结束 == 本圈最后一个环岛结束。后面还有最后一个方形 T 口，
     * 不能立刻 wrap 回 BIGHEX（否则 BSS 会在最后一个方形上误动作）。
     * 保持 plan_idx=2 + hext_done=1：全局 T 口直行接管，等门/换圈再 Lap_Reset。
     * hext_locked=1 是额外的硬闸门：从出岛/放弃这一刻起，到 Lap_Reset 之前
     * 任何 HEXT 入口都不允许，防止 63/64 那种 SOUT 后再来一次 0FC0/0FF0。 */
    hext_done   = 1U;
    hext_locked = 1U;
    gate_z_events = 0U;       /* 从 HEX8 出岛开始重新数门前的 Z 字弯 */
    gate_t_count  = 0U;       /* T1/T2 普通 T 口，T3/T4 是门 */
    printf("ISL,HEXT_LOCK\r\n");
  } else {
    plan_idx = (uint8_t)((plan_idx + 1U) % ISL_PLAN_N);
  }
  isl_t_phase = now;
  isl_cfm     = 0U;
  hext_entry_phase = 0U;
  hext_exit_phase  = 0U;
  hext_blob_d      = 0;
  hext_exit_arm_d  = 0;
  hext_guard_on    = 0U;
  hext_dr_on       = 0U;
  /* 换格不允许把上一格的补线偏置 / 出岛武装带过去：
   * FABORT 路径曾经漏清 isl_err_off，结果车带着 4.0 的固定偏移走进下一格。 */
  isl_err_off = 0.0f;
  isl_out_arm = 0U;
  /* 自校正候选也属于"当前格"：换格后必须重新计 150 ms，
   * 否则上一格残留的 t_rsy 会让新格的第一帧宽白立刻 RESYNC（37.txt 那类误跳）。 */
  rsy_cfm     = 0U;
  t_rsy       = 0U;
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
  isl_d_fin    = 0;
  isl_d_phase  = 0;
  isl_d_phase_raw = 0;
  isl_plan_last = 0xFFU;      /* 强制第一帧 Island_Step 重新记录格起点里程 */
  hext_done     = 0U;         /* 新一圈：HEX8 之后的 T 直行阶段重新开始 */
  hext_locked   = 0U;         /* 新一圈：允许重新进 HEX8 */
  hext_entry_phase = 0U;
  hext_entry_line_cnt = 0U;
  hext_entry_lost_cnt = 0U;
  hext_entry_pd_d0 = 0;
  hext_entry_straight_d0 = 0;
  hext_entry_comp_yaw0 = 0.0f;
  hext_entry_comp_target = 0.0f;
  hext_entry_comp_d0 = 0;
  hext_entry_comp_done = 0U;
  hext_exit_phase  = 0U;
  hext_outbias_d0  = 0;        /* 出新一圈：外偏状态整体清掉 */
  hext_outbias_val = 0.0f;
  hext_blob_d      = 0;
  hext_exit_arm_d  = 0;
  hext_guard_on    = 0U;
  hext_dr_on       = 0U;
  /* 【11/21 14:37】HEXT 入口的跨帧状态必须跟圈/趟一起清：
   * hext_fork_d 是相对 s0 的里程，下一次 GO 的 s0 重新从 0 开始，
   * 如果它还留着上一趟的几万 mm，(d_now - hext_fork_d) 永远是负数，
   * 新的合格分叉永远覆盖不了它 -> 第二趟 HEXT 入口直接失效。
   * g_clean_frames 不清则会带着上一趟的 15+ 帧，绕过“先进稳定线”的门。 */
  g_clean_frames = 0U;
  hext_fork_d    = 0;
  hext_fork_last_d = 0;
  hext_fork_n    = 0U;
  hext_fork_bm   = 0U;
  hext_fork_fr   = 0U;      /* 簇帧数一起清：否则新簇的第一帧就被当成"已持续 3 帧" */
  isl_force_used = 0U;        /* 里程强制进岛：发车时允许一次 */
  /* 【候选状态必须一起清】rsy_cfm / sq_d0 是"看到图案后等确认"的中间态：
   * 上一趟如果停在候选成立的那一刻（rsy_cfm=1），下一次发车第一帧就带着
   * 陈旧的 t_rsy 进入 RESYNC 分支 —— 只要开头 150 ms 内出现一次宽白
   * （发车区那根竖条就够了），就会 PLAN,RESYNC 把 plan_idx 从第 1 格
   * 推到第 2 格，直接跳过大套小。
   * 之前只清了 isl_cfm，漏了这两个，表现为"第一趟正常、第二趟不跑大套小"。
   * 新增候选量时务必加到这里（或像 bs 那样收进结构体统一 memset）。 */
  rsy_cfm      = 0U;
  t_rsy        = 0U;
  sq_d0        = 0;
  /* 发车/终点门 + 圈数：发车时全部清零，本圈起点里程 = 0 */
  bar_cand     = 0U;
  bar_wide     = 0U;
  bar_ok       = 0U;
  bar_mm       = 0;
  bar_hit      = 0U;
  bar_hit_mm   = 0;
  pair_mm      = 0;
  gate_state   = GATE_WAIT_FIRST;
  gate_gap_mm  = 0;
  gate_t_count = 0U;
  last_bar_mm  = -1000;
  lap_start_mm = 0;
  lap_cnt      = 0U;
  /* 【11/24】每次发车从常量窗口开始，学到这一趟的实测单圈后再收紧。
   * 不能放进 Lap_Reset —— 那个在每圈过门时都会调用，会把学到的值抹掉。 */
  lap_mm_learned = 0;
  gate_z_events = 0U;
  stop_mm      = 0;
  fastseg_block = 0U;         /* 发车：提速段重新打开 */
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
   * 【11/20 重构后】计划表只剩 3 个绕行环岛，方形/八边形已删除，
   * 因此不再有 CFG,OCT7；第 8 格是 isl_cfg[2]。
   * 这一版应该是：
   *   PLAN,1/3,HEX,START
   *   CFG,HEX8,5,0,0,20000,1,274,1000,0,0,460
   * 判断方法：
   *   - PLAN 分母是 3，不是 9；
   *   - HEX8 的 skip_mm 是 20000，表示从 SMALLHEX 出来一路等中间所有 T 口；
   *   - entry_off×10=0、entry_deg=0，入口打角由 ISL_HEXT_ENTRY_TURN_* 负责；
   *   - 只要还能看到 CFG,OCT7 或 PLAN,1/9，就说明烧的是旧 hex。 */
  printf("CFG,HEX8,%u,%u,%u,%u,%d,%u,%u,%u,%u,%u\r\n",
         (unsigned)isl_cfg[2].fork_n, (unsigned)isl_cfg[2].fork_mm,
         (unsigned)isl_cfg[2].force_mm, (unsigned)isl_cfg[2].skip_mm,
         (int)isl_cfg[2].dir,
         (unsigned)isl_cfg[2].exit_deg, (unsigned)isl_cfg[2].exit_fall_mm,
         (unsigned)isl_cfg[2].entry_off10, (unsigned)isl_cfg[2].entry_deg,
         (unsigned)isl_cfg[2].v_island);
  /* 第 3、4 字段 = 入口打角差速 / 入口打角目标角（30°），
   * 用来核对烧进去的是不是这一版。 */
  printf("CFG,HEXT,%u,%u,%u,%u\r\n", (unsigned)ISL_HEXT_WAIT_MIN_MM,
         (unsigned)ISL_GLOBAL_T_N, (unsigned)ISL_HEXT_ENTRY_TURN,
         (unsigned)ISL_HEXT_ENTRY_TURN_DEG);
  printf("CFG,HEXT_LOCK,1\r\n");
  /* 确认 HEX8 的绕行方向：必须是 +1（右拐）。 */
  printf("CFG,HEXT_DIR,%d\r\n", (int)isl_cfg[2].dir);
  printf("CFG,V,%d,%d,%d\r\n", (int)FOLLOW_V_FAST,
         (int)FOLLOW_V_SLOW, (int)ISL_FAKE_DEG);
  /* 控制节拍指纹：周期 ms / KD。5ms 这一版必须看到 5,36；若是 10,18 说明烧的是 10ms 版。 */
  printf("CFG,CTL,%u,%d\r\n", (unsigned)CTL_MS, (int)FOLLOW_KD);
  /* 转向环指纹（11/19 提速 ×3 新增）：
   *   CFG,GAIN,<KP>,<KP2>,<KI×10>,<I_MAX>,<TURN_MAX>,<PWM_MAX>,<RAMP_MS>
   * 这几个数决定"同样的偏差能拐出多小的弯"，是提速后最容易改错的一批，
   * 又没有任何别的日志能体现它们。这一行不对就说明烧的是旧 hex：
   *   提速后应为  GAIN,340,90,18,140,1020,6500,700
   *   提速前应为  GAIN,150,40,8,60,450,4200,300 */
  printf("CFG,GAIN,%d,%d,%d,%d,%d,%d,%u\r\n",
         (int)FOLLOW_KP, (int)FOLLOW_KP2, (int)(FOLLOW_KI * 10.0f),
         (int)FOLLOW_I_MAX, (int)FOLLOW_TURN_MAX,
         (int)FOLLOW_PWM_MAX, (unsigned)FOLLOW_RAMP_MS);
  /* 门判据指纹：确认"第二根竖条放宽到 n>=10"这一版有没有真的烧进去 */
  printf("CFG,BAR,%u,%u,%u\r\n", (unsigned)BAR_FULL_N,
         (unsigned)BAR_FULL_N2, (unsigned)BAR_LAP_TOL_MM);
  /* 【11/24】全局 T 口指纹：<n 门槛>,<锁直行 mm>,<锁放开时的 n 上限>。
   * 用来一眼确认"方形直角弯误锁直行"那一版有没有被换掉：
   * 正确值是 11,60,5；若还看到 10,... 就是旧固件。 */
  printf("CFG,TGATE,%u,%d,%u\r\n", (unsigned)ISL_GLOBAL_T_N,
         (int)60, (unsigned)5U);
  printf("CFG,PAIR,%u,%u\r\n", (unsigned)PAIR_MIN_MM, (unsigned)PAIR_MAX_MM);
  /* 【11/25】速度环指纹：<KPS×10>,<KIS×100>，正确值 12,15。
   * 只用于事后核对（日志里看不出 KIS 时排查很费事），不影响控制。 */
  printf("CFG,SPD,%u,%u\r\n", (unsigned)(FOLLOW_KP_SPD * 10.0f),
         (unsigned)(FOLLOW_KI_SPD * 100.0f));
  /* 【11/24】第 2 格入口窗口指纹：<入口最小里程>,<第2格 skip_mm>,<小环前馈×10>。
   * 正确值 400,1500,1500；小环前馈那列是 0 说明还是"纯 PD"的旧版。 */
  printf("CFG,ENTRY,%u,%u,%u\r\n", (unsigned)ISL_ENTRY_MIN_MM,
         (unsigned)isl_cfg[1].skip_mm, (unsigned)(BS_SMALL_FF * 10.0f));
  printf("CFG,START,%u,%d\r\n", (unsigned)START_PRELOAD_MS,
         (int)START_PRELOAD_PWM);
  printf("CFG,HEXT_BLIND,%u,%d,%u\r\n",
         (unsigned)(ISL_HEXT_ENTRY_TURN_DEG * 10.0f),
         (int)ISL_HEXT_ENTRY_TURN,
         (unsigned)ISL_HEXT_ENTRY_TURN_MAX_MM);
  printf("CFG,HEXT_STRAIGHT,%d,%u,%u,%d,%d\r\n",
         (int)ISL_HEXT_ENTRY_STRAIGHT_V,
         (unsigned)ISL_HEXT_ENTRY_STRAIGHT_MM,
         (unsigned)ISL_HEXT_PD_WAIT_MM, (int)HEXT_BLIND_TURN,
         (int)(HEXT_ENTRY_KD * 10.0f));
  /* 【11/25 16:20】出岛指纹：<航向目标×10>,<打角里程上限>,<打角速度>,<直行 mm>。
   * 正确值 -550,240,350,60（打角改成 bang-bang 的那一版）；
   * 若看到 -550,80,875,180 就是旧的"P 环 + 180mm 盲走"版本。 */
  printf("CFG,HEXTOUT,%d,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
          (int)(ISL_HEXT_EXIT_NEED_MAX * 10.0f),
          (unsigned)ISL_HEXT_EXIT_TURN_MAX_MM,
          (int)ISL_HEXT_EXIT_PIVOT_V,
          (unsigned)ISL_HEXT_EXIT_STRAIGHT_MM,
          (unsigned)ISL_HEXT_EXIT_HOLD_MM,
          (unsigned)ISL_HEXT_EXIT_LINE_N,
          (unsigned)(ISL_HEXT_EXIT_YAW_LO),
          (unsigned)ISL_HEXT_EXIT_GUARD_MM,
          (unsigned)ISL_HEXT_EXIT_GUARD_MAX_MM,
          (unsigned)ISL_HEXT_EXIT_SETTLE_MM,
          (unsigned)ISL_HEXT_EXIT_FIND_MM);
  /* 【11/25 新增】出岛外偏指纹：<使能>,<窗长 mm>,<起始偏置×10>。
   * 这一版正确值是 0,160,20 —— 第一列 0 表示外偏已停用，
   * 出岛改由 ISL_HEXT_EXIT_SETTLE_MM=80 的"锁航向直行"负责（见 CFG,HEXTOUT
   * 第 10 列：正确值 80；109.txt 那版是 0）。
   * 看不到这一行说明烧的是更早的固件，别拿旧 hex 的日志分析新问题。 */
  printf("CFG,HEXTOUTB,%u,%u,%d\r\n", (unsigned)ISL_HEXT_OUT_BIAS_EN,
         (unsigned)ISL_HEXT_OUT_BIAS_MM, (int)(ISL_HEXT_OUT_BIAS * 10.0f));
  /* 【11/25 新增】HEXT 入口叉口判据指纹：<簇最少帧数>,<簇最短跨度 mm>。
   * 这一版正确值是 3,6 —— 看到 2,6 说明烧的还是"2 帧假簇也能进岛"的旧固件，
   * 那种固件在 108.txt 里因为 2 帧/10mm 的假叉口直接 FABORT 了。 */
  printf("CFG,HEXTFORK,%u,%u\r\n", (unsigned)ISL_HEXT_FORK_MIN_FR,
         (unsigned)ISL_HEXT_FORK_MIN_MM);
  /* 【11/25 新增】第二格→第 8 格入口提速指纹：<使能>,<巡航倍数×100>,<宽区倍数×100>。
   * 正确值 1,112,108；看不到这一行、或第一列是 0，说明烧的是提速之前的固件。 */
  printf("CFG,FASTSEG,%u,%d,%d\r\n", (unsigned)FASTSEG_V_EN,
         (int)(FASTSEG_V_GAIN * 100.0f), (int)(FASTSEG_V_NEAR_GAIN * 100.0f));
  /* 【11/25 新增】盲走速度指纹：<GAP_V mm/s>。正确值 1250（旧固件是 712 或 570）。 */
  printf("CFG,GAPV,%d\r\n", (int)GAP_V);
  /* 【11/25 新增】断路斜穿角指纹：<GAP_TURN_DEG ×10>。正确值 300（27° 是上一版）。 */
  printf("CFG,GAPTURN,%d\r\n", (int)(GAP_TURN_DEG * 10.0f));
  /* 【11/26 新增】直角弯打角指纹：<RECOVER_V>,<RECOVER_TURN>,<FOLLOW_V_MIN>。
   * 正确值 450,1100,-575（114.txt 掉头那版是 400,1100,-1000）。 */
  printf("CFG,CNR,%d,%d,%d\r\n",
         (int)RECOVER_V, (int)RECOVER_TURN, (int)FOLLOW_V_MIN);
  /* 【11/26 新增】恢复段转角上限指纹：<上限>,<收力窗口>。正确值 140,45。 */
  printf("CFG,SWING,%d,%d\r\n",
         (int)RECOVER_SWING_MAX_DEG, (int)RECOVER_SWING_FADE_DEG);
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

  fastseg_block  = 0U;              /* 新一圈：提速段重新打开（断路在每圈中段） */
  isl_state      = ISL_NONE;
  isl_cfm        = 0U;
  isl_out_arm    = 0U;
  isl_d_fin      = 0;
  isl_d0         = 0;
  isl_err_off    = 0.0f;
  isl_block_dist = 0;
  isl_ret_dist   = 0;
  sq_events      = 0U;
  isl_force_used = 0U;              /* 换圈：里程强制进岛重新允许一次 */
  rsy_cfm        = 0U;
  t_rsy          = 0U;              /* 和 rsy_cfm 成对：只清标志不清时刻会立刻重确认 */
  sq_d0          = 0;
  /* 【必须清 lap_wrap_pending】换圈有两种触发源：过门（Lap_Gate -> 这里）
   * 和计划表回绕（Island_Advance 置标志、下一帧 Island_Step 消费）。
   * 两者可能落在同一控制周期：帧 N 里 Island_Step 先 Island_Advance 回绕置 1，
   * 紧接着 Lap_Gate 认出 GATE,PAIR 又调本函数 —— 如果这里不清标志，
   * 帧 N+1 的 Island_Step 会再消费一次，lap_cnt 多加 1、lap_start_mm 多加
   * 一个 FOLLOW_LAP_MM，第二圈所有按圈里程算的窗口整体错位。
   * 现在 FOLLOW_LAPS=2，第一圈成对过门会进入 Lap_Gate -> 这里；
   * 若不清 lap_wrap_pending，下一帧 Island_Step 会再消费一次，
   * lap_cnt 多加 1、lap_start_mm 多加一个 FOLLOW_LAP_MM，第二圈窗口整体错位。 */
  lap_wrap_pending = 0U;
  bar_cand       = 0U;
  bar_ok         = 0U;
  bar_wide       = 0U;
  bar_hit        = 0U;
  pair_mm        = 0;
  gate_state      = GATE_WAIT_FIRST;
  gate_t_count    = 0U;

  /* 计划表回到第 1 格；isl_d_phase 重设成"这一圈格 1 的起点"，
   * 于是大套小的位置窗口 (dist - lap_start_mm) 每圈都一样。 */
  plan_idx      = 0U;
  isl_d_phase   = dist;
  isl_d_phase_raw = dist;
  isl_plan_last = 0xFFU;            /* 让 Island_Step 下一帧重新记录格起点 */
  hext_done     = 0U;               /* 新一圈：HEX8 之后的 T 直行阶段重新开始 */
  hext_locked   = 0U;               /* 新一圈：允许重新进 HEX8 */
  hext_entry_phase = 0U;
  hext_entry_line_cnt = 0U;
  hext_entry_lost_cnt = 0U;
  hext_entry_pd_d0 = 0;
  hext_entry_straight_d0 = 0;
  hext_entry_comp_yaw0 = 0.0f;
  hext_entry_comp_target = 0.0f;
  hext_entry_comp_d0 = 0;
  hext_entry_comp_done = 0U;
  hext_exit_phase  = 0U;
  hext_outbias_d0  = 0;        /* 换圈：外偏状态整体清掉，第二圈从零开始 */
  hext_outbias_val = 0.0f;
  gate_z_events   = 0U;
  /* 换圈同样要清 HEXT 入口的跨帧状态，见 Island_Reset 的说明。 */
  g_clean_frames = 0U;
  hext_fork_d    = 0;
  hext_fork_last_d = 0;
  hext_fork_n    = 0U;
  hext_fork_bm   = 0U;
  hext_fork_fr   = 0U;      /* 簇帧数一起清：否则新簇的第一帧就被当成"已持续 3 帧" */

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
 * 判据：里程窗口 + 成对。单独一根（方形 T 口）不认。
 *
 * 【11/24 20:4x 窗口改成自适应（第一圈之后不用再手标定）】
 * 93.txt（1350 速版）教训：单圈里程不是几何常量，它 = 轮子实际滚过的路程，
 * 速度越高打滑越多、里程越大：
 *   900 mm/s  -> 21.9k     1350 mm/s -> 25.6k（+17%，25 根竖条全部被窗口拒掉）
 * 也就是说"改一次速度就要重标一次 FOLLOW_LAP_MM"，而漏标一次的代价是
 * 整场没有 Lap_Reset（计划表卡死、第二圈不走大套小、进不了最后一个六边形）。
 * 所以：第一圈用常量窗口，**一旦成对成功就把实测单圈记下来**，
 * 之后所有圈都按"实测值 ± 12.5%"开窗 —— 换速度、换电池、开关负压都自动跟。
 */
static void Lap_Gate(int32_t dist)
{
  int32_t d_cell = dist - lap_start_mm;    /* 本圈已走里程 */
  int32_t d_window = d_cell;
  int32_t win_centre = (int32_t)FOLLOW_LAP_MM;
  int32_t win_tol    = (int32_t)BAR_LAP_TOL_MM;

  /* 学到过就按实测值开窗，容差 12.5%（至少 800mm，防止刚开始学时过窄） */
  if (lap_mm_learned > 0) {
    win_centre = lap_mm_learned;
    win_tol    = lap_mm_learned / 8;
    if (win_tol < 800) win_tol = 800;
  }

  /* lap_start_mm 记的是“上一道门的第二根”，所以第二圈的门窗口
   * 要比 FOLLOW_LAP_MM 少一个门间距；补上 gate_gap 后两圈才同一基准。
   * 15:47 那趟：第二圈 T3 的 d_cell≈22400，补 420 后≈22820，
   * 才落在 23386±900 内；之前差值约 944，刚好被挡住。 */
  if (lap_cnt > 0U && gate_gap_mm > 0) {
    d_window += gate_gap_mm;
  }

  /* ==================== 【11/25】第二根竖条漏读时按里程预测成对 ====================
   * 99a.txt 实测：第一根竖条 22145（n=12）认到了，但第二根该在 22565 左右，
   * 而车在 22472~22574 连续 n=0 / 0000（整排全黑，正好压在第二根上）、
   * 随后 22596 才找回线 —— 竖条检测没数据可判，T 口计数在附近也没触发，
   * 于是门永远配不成对，整场没有 Lap_Reset（第二圈不做环岛、只按里程跑完）。
   * 第一根已经过了"里程窗口 + Z 字 + 前置细线"三重判据，可信度足够；
   * 所以只要走够"预期间距 + 余量"还没见到第二根，就用预测位置补上。
   * 预期间距优先用 gate_gap_mm（上一圈学到的实测值），否则用默认 430。 */
  if (!bar_hit && gate_state == GATE_WAIT_SECOND && pair_mm != 0) {
    int32_t pred  = (gate_gap_mm > 0) ? gate_gap_mm : (int32_t)GATE_GAP_DEFAULT_MM;
    int32_t since = dist - pair_mm;

    if (since >= pred + (int32_t)GATE_PRED_MISS_MM) {
      printf("GATE,PRED,%ld,%ld,%ld\r\n", (long)pair_mm, (long)since, (long)pred);
      bar_hit    = 1U;
      bar_hit_mm = pair_mm + pred;
    }
  }

  if (!bar_hit) return;
  if (gate_state == GATE_DONE) { bar_hit = 0U; return; }

  /* ---- 1) 去重：太近的两根当同一根 ---- */
  {
    int32_t since = bar_hit_mm - last_bar_mm;
    if (since < (int32_t)BAR_DUP_MM) {
      printf("BAR,DUP,%ld,%ld\r\n", (long)bar_hit_mm, (long)since);
      bar_hit = 0U;
      return;
    }
  }

  /* ---- 2) 第一根必须在本圈里程窗口内；第二根只有间距合格才免窗口 ----
   * 第一根用来确认“现在到发车区了”，窗口防普通 T 口；
   * 第二根只要和第一根相距 250~650mm 就是同一道门，
   * 不能因为门中心里程漂移就把第二根判成 BAR,OUT。
   * 但间距不合格的远竖条不能永远带着 WAIT_SECOND 状态，
   * 否则它会跳过窗口，和几百毫米外的另一根误配对（16:00 那趟）。 */
  {
    int32_t gap_pre = (gate_state == GATE_WAIT_SECOND && pair_mm != 0)
                        ? (bar_hit_mm - pair_mm) : 0;
    uint8_t is_second = (uint8_t)(gate_state == GATE_WAIT_SECOND
                                  && pair_mm != 0
                                  && gap_pre >= (int32_t)PAIR_MIN_MM
                                  && gap_pre <= (int32_t)PAIR_MAX_MM);

    if (!is_second
        && (d_window < win_centre - win_tol
            || d_window > win_centre + win_tol)) {
      printf("BAR,OUT,%ld,%ld,%ld\r\n", (long)bar_hit_mm, (long)d_cell, (long)d_window);
      if (gate_state == GATE_WAIT_SECOND) {
        pair_mm = 0;
        gate_state = GATE_WAIT_FIRST;
      }
      bar_hit = 0U;
      return;
    }
  }

  last_bar_mm = bar_hit_mm;
  bar_hit     = 0U;

  /* ---- 3) 门前必须有 Z 字弯，排掉普通 T 口 ---- */
  if (gate_z_events < GATE_Z_MIN) {
    printf("GATE,NOZ,%ld,%d\r\n", (long)bar_hit_mm, (int)gate_z_events);
    return;
  }

  printf("GATE,BAR,%ld,%ld,%ld\r\n", (long)bar_hit_mm, (long)d_cell,
         (long)(pair_mm ? (bar_hit_mm - pair_mm) : 0));

  /* ---- 4) 统一门状态机：两圈都必须两根竖条成对 ---- */
  if (gate_state == GATE_WAIT_FIRST) {
    pair_mm = bar_hit_mm;
    gate_state = GATE_WAIT_SECOND;

    if (lap_cnt + 1U >= (uint8_t)FOLLOW_LAPS) {
      /* 最后一圈：第一根先按“学到的门间距”预测第二根。
       * 若真实第二根也识别到，下面会更新到两根中点的精确停车点；
       * 若第二根漏检，仍能按预测点停车，不再完全不停。 */
      int32_t gap = (gate_gap_mm > 0) ? gate_gap_mm : (int32_t)GATE_GAP_DEFAULT_MM;
      stop_mm = bar_hit_mm + gap / 2 + (int32_t)STOP_MID_OFFSET_MM;
      printf("GATE,PRED,%ld,%ld,%ld\r\n",
             (long)bar_hit_mm, (long)gap, (long)stop_mm);
    }

    printf("GATE,FIRST,%ld,%ld\r\n", (long)bar_hit_mm, (long)d_cell);
    return;
  }

  if (gate_state == GATE_WAIT_SECOND) {
    int32_t gap = bar_hit_mm - pair_mm;

    if (gap < (int32_t)PAIR_MIN_MM || gap > (int32_t)PAIR_MAX_MM) {
      /* 间距不对：这根当新的第一根，重新等第二根 */
      printf("GATE,NOPAIR,%ld,%ld,%ld\r\n",
             (long)pair_mm, (long)bar_hit_mm, (long)gap);
      pair_mm = bar_hit_mm;
      return;
    }

    printf("GATE,PAIR,%ld,%ld,%ld\r\n",
           (long)pair_mm, (long)bar_hit_mm, (long)gap);

    if (lap_cnt + 1U >= (uint8_t)FOLLOW_LAPS) {
      /* 第二圈：成对后停车。
       * 【11/24 20:4x 修 bug】原写法是
       *     pair_mm = 0;
       *     stop_mm = pair_mm + (bar_hit_mm - pair_mm) / 2 + STOP_MID_OFFSET_MM;
       * 先把 pair_mm 清零再拿它算，等于 stop_mm = bar_hit_mm/2（绝对里程的一半），
       * 而当前里程早就大于它 → 车在第二根竖条上立刻停，和"停在发车区"完全不符。
       * 改成和下面兜底路径（GATE,STOP 的 bar_hit_mm + gap/2）同一个式子：
       * 停车点 = 第二根竖条 + 半个门间距，车主体落在发车区内。
       * 想往两根竖条中间挪，就把 STOP_MID_OFFSET_MM 设成负值（约 -gap ≈ -420）。 */
      stop_mm = bar_hit_mm + gap / 2 + (int32_t)STOP_MID_OFFSET_MM;
      pair_mm = 0;
      lap_cnt++;
      gate_state = GATE_DONE;
      printf("GATE,STOP,%d,%ld,%ld,%ld\r\n", (int)lap_cnt,
             (long)bar_hit_mm, (long)gap, (long)stop_mm);
      return;
    }

    /* 第一圈：成对确认后直行通过，并重置下一圈 */
    gate_gap_mm = gap;                /* 学会两根竖条的实际间距 */
    pair_mm = 0;
    lap_cnt++;
    lap_start_mm = bar_hit_mm;
    gate_state = GATE_WAIT_FIRST;
    /* 【11/24 20:4x】把这一圈的实测里程记下来，下一圈窗口改成"实测 ±12.5%"，
     * 从此改速度/换电池/开关负压都不用再手标 FOLLOW_LAP_MM。 */
    if (d_cell > 5000) {              /* 明显不是误判才学 */
      lap_mm_learned = d_cell;
    }
    Lap_Reset(dist);
    /* 【11/24】第三列原来是 `dist - bar_hit_mm`，恒等于 4 mm，是废列。
     * 改成【实测单圈里程】= d_cell（从上一圈起点到这一圈第二根）。
     * 现在它同时是自适应窗口的输入：日志里这一列就是下一圈的开窗中心。 */
    printf("LAP,%d,%ld,%ld,%ld\r\n", (int)lap_cnt,
           (long)bar_hit_mm, (long)d_cell,
           (long)(lap_mm_learned > 0 ? lap_mm_learned / 8 : 0));
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
                || isl_state == ISL_SESSION);
}

/* 第二格之后 → 第 8 格入口 的提速窗口判断。
 * 参数见文件头附近的 FASTSEG_V_GAIN 说明；这里只看"现在是哪一格、到没到第 8 格"。 */
static uint8_t Island_FastSeg(void)
{
  if (!FASTSEG_V_EN) return 0U;
  if (fastseg_block) return 0U;     /* 已过真断路：八边形这一段不快跑 */
  if (plan_idx != 2U || hext_done != 0U) return 0U;
  if (Island_Busy()) return 0U;
  return 1U;
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
#define BS_CAND_MS        178U   /* 大套小入口候选的"掉帧宽容窗口"（同 ISL_FORK_TMO_MS，
                                  * 原来也是函数体里的字面量 250U）。同样不按速度缩：
                                  * 缩短会漏进岛，而漏进岛的代价是整格被跳过。 */
#define BS_FAKE_DEG        35.0f /* 偏航还不到这个值 -> 判为假进岛，退回等入口 */
#define BS_JUNC_MM       150    /* 最近 150 mm 内出现过 n>=fork_n 就算在交叉点 */
#define BS_BIG_MIN_DEG 190.0f   /* 大环：至少转这么多，才允许按图案切段 */
#define BS_BIG_MAX_DEG 300.0f   /* 大环：到这么多度还没遇到交叉点就强制切段 */
#define BS_SML_MIN_DEG 290.0f   /* 小环：至少转这么多，才允许按图案切段 */
#define BS_SML_MAX_DEG 430.0f   /* 小环：到这么多度强制切段 */

#define BS_BIAS        (-215.0f) /* 右转偏置（turn<0 = 右转），顶住交叉点别让 PD 直行。
                                  * 【-95 ×2.29 = -215】它是和 PD 对抗的 mm/s 量。
                                  * 平衡点 = BIAS/KP：-285/450 = -95/150 = 0.63 个 err 单位，
                                  * 只要 BIAS 和 KP 同比例放大，车被顶到的位置就完全一样。 */
/* 小环前馈 BS_SMALL_FF 的定义挪到 ISL_ENTRY_MIN_MM 旁边，见那里。 */
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
    } else if (isl_cfm && (uint32_t)(now - isl_t_cfm) > BS_CAND_MS) {
      /* 超过 BS_CAND_MS 没再看到图案，才算候选失败 */
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
    /* 【11/24】入口段（前 130°）保持"偏置把车压进环"；
     * 之后给恒定前馈，让 PD 只做细纠偏（原来这里 bs.turn 被清 0，小环全靠 PD 硬追）。
     * 【11/25】前馈方向改成**跟着这一趟的实际绕行方向**：
     *   设计上大环右转、小环左转（相反），但进岛方向可能整趟镜像过来
     *   （97.txt 第 2 趟入口位图 0x00E3，小环里 |偏航| 一直在 0~60° 之间来回，
     *    等于在岛里打转而不绕环）。所以用"进岛至今的净转角" bs.yaw0 做基准：
     *   大环向右(>0) -> 小环向左(+FF)；大环向左(<0) -> 小环向右(-FF)。
     *   净转角在整段里单调增长，符号稳定，不会抖。 */
    if (fabsf(gyro_angle - bs.yaw2) < BS_ENTRY_DEG) {
      bs.turn = 0.0f;
      bs.err_off = -BS_ERR_OFF;
    } else {
      float big = gyro_angle - bs.yaw0;      /* 大环绕向（右正） */
      bs.turn = (big > 0.0f) ? BS_SMALL_FF : -BS_SMALL_FF;
      bs.err_off = 0.0f;
    }
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

/* HEXT 岔口入口锚点：
 *   正常簇（跨度 <= ISL_HEXT_FORK_MAX_MM）用簇末——真岔口是 10~20mm 短促横穿，
 *   簇末 +20mm 最稳（67.txt 实测 FIN 落在簇末 +37~44mm）；
 *   跨度 > 100mm 的长簇说明车在两线之间顺跑，簇末已越过真岔口，改锚簇首，
 *   让入口比"簇末 +20"提前 60~80mm（见 ISL_HEXT_FORK_MAX_MM 的 128 数据）。 */
static int32_t Hext_ForkAnchor(void)
{
  if ((hext_fork_last_d - hext_fork_d) > (int32_t)ISL_HEXT_FORK_MAX_MM) {
    return hext_fork_d;
  }
  return hext_fork_last_d;
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
    isl_d_phase_raw = dist;

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

  /* ================= 锁直行（三种场合共用一份实现）=================
   * 11/19 23:00 合并：原来的 ISL_CROSS / ISL_HIN / ISL_HEXOUT 三块代码
   * 做的事完全一样 —— 锁住转向、按固定速度直行 N mm。差别只有三个数，
   * 进锁之前由各自的入口填进 isl_lock_mm / isl_lock_nmax / isl_lock_tmo /
   * isl_lock_after，这里只有一份实现。
   *
   * 放开条件（三者取"或"）：
   *   ① 走够 isl_lock_mm，且（nmax>=255 或 宽白已消失 n<=nmax）
   *   ② （仅进岛型）走够本格的 in_cross_mm —— 窄顶点锁太久会丢线
   *   ③ 超时 isl_lock_tmo
   */
  if (isl_state == ISL_CROSS) {
    int32_t run  = dist - isl_d0;
    uint8_t done = 0U;

    if (run >= isl_lock_mm
        && (isl_lock_nmax >= 255U || n <= isl_lock_nmax)) done = 1U;
    /* T 口直行专用硬上限：等"n<=5"最多等到 ISL_T_LOCK_MAX_MM。
     * 11/24 那趟出现过 T,OUT,161,5 —— 宽白迟迟不散，直行拖到 161mm 才放开，
     * 车已经冲过 T 口后面的弯。正常 T,OUT 是 60~67mm，120mm 不会误伤。
     * 用 isl_lock_after 判断而不另加标志位：T 口是唯一用 LOCK_AFTER_TONLY 的场景。 */
    if (isl_lock_after == LOCK_AFTER_TONLY
        && run >= (int32_t)ISL_T_LOCK_MAX_MM) done = 1U;
    if (isl_lock_after == LOCK_AFTER_SESSION
        && Island_Cfg()->in_cross_mm != 0U
        && run >= (int32_t)Island_Cfg()->in_cross_mm) done = 1U;
    if ((uint32_t)(now - isl_t_state) > isl_lock_tmo) done = 1U;
    /* 【11/25 16:20 改语义】phase2 = 打角段（由 Island_ExitTurn 的 bang-bang
     * 按角度关门，Island_Step 不管它）；phase3 = 打角完成后的直行段，
     * 直行里程从"转到角那一帧"开始算（ExitTurn 里重置 hext_exit_straight_d0）。
     * phase2 只留一个"打角+直行都走满"的硬上限兜底：万一 Island_ExitTurn
     * 没被调用（isl_state 不是 ISL_CROSS），锁也要能放开。 */
    if (hext_exit_phase == 5U
        && (dist - hext_exit_straight_d0) >= (int32_t)ISL_HEXT_EXIT_STRAIGHT_MM) done = 1U;
    if (hext_exit_phase == 2U
        && (dist - hext_exit_d0)
           >= (int32_t)(ISL_HEXT_EXIT_TURN_MAX_MM + ISL_HEXT_EXIT_SETTLE_MM
                        + ISL_HEXT_EXIT_FIND_MM + ISL_HEXT_EXIT_STRAIGHT_MM)) done = 1U;

    if (!done) return 1U;              /* 继续锁直行 */

    /* 锁结束：日志沿用三个旧事件名，历史日志分析的经验仍然对得上 */
    isl_t_state = now;
    Steer_Reset();
    switch (isl_lock_after) {
      case LOCK_AFTER_SESSION:
        printf("ISL,HINOK,%ld,%d\r\n", (long)run, (int)n);
        break;
      case LOCK_AFTER_ADVANCE:
        printf("ISL,SOUT,%s,%ld,%d\r\n", Island_Cfg()->name, (long)run, (int)n);
        break;
      case LOCK_AFTER_TONLY:
        printf("T,OUT,%ld,%d\r\n", (long)run, (int)n);
        break;
      default:
        printf("ISL,OUT,%ld\r\n", (long)run);
        break;
    }
    isl_state = ISL_RET;

    if (isl_lock_after == LOCK_AFTER_TONLY) {
      /* 全局 T 口直行结束：不推进 plan_idx，直接回 NONE（经 ISL_RET 冷却） */
      return 0U;
    }

    if (isl_lock_after == LOCK_AFTER_SESSION) {
      /* 进岛穿过交叉点：交还 PD 开始绕行，绕行角从现在开始算 */
      isl_state = ISL_SESSION;
      isl_yaw0  = gyro_angle;
      isl_d_fin = dist;
      Steer_Reset();
      return 0U;
    }

    if (isl_lock_after == LOCK_AFTER_ADVANCE) {
      /* 出岛穿过岔口：这一格结束 */
      isl_ret_dist = dist;
      sq_events    = 0U;
      /* 【11/25 新增】出岛后短程外偏：交回 PD 之后先把车带离环线，
       * 见 ISL_HEXT_OUT_BIAS 那一大段说明。只在 HEXT 出岛时生效。 */
      if (ISL_HEXT_OUT_BIAS_EN && Island_Cfg()->kind == ISLK_HEXT) {
        hext_outbias_val = (Island_Cfg()->dir > 0) ? -ISL_HEXT_OUT_BIAS
                                                   :  ISL_HEXT_OUT_BIAS;
        hext_outbias_d0  = dist;
        printf("HEXT,OUTB,%d,%d\r\n", (int)(hext_outbias_val * 10.0f), (int)n);
      }
      Island_Advance(now, "HEXDONE");
      return 0U;
    }

    /* ---- LOCK_AFTER_EVENT：数一次"穿越事件" ----
     * 方形需要 2 次（进 + 出各一个 T 口），八边形只需要 1 次。
     * 这个次数就是配置表里的 need_ev，所以"方形"和"八边形"现在是同一份代码。 */
    if (Island_Cfg()->need_ev <= 1U) {
      printf("CROSS,OK,%s,%ld,%ld\r\n", Island_Cfg()->name,
             (long)dist, (long)(dist - isl_d_phase));
      sq_events = 0U;
      sq_d0     = 0;
      Island_Advance(now, "CROSSOK");
      return 0U;
    }

    if (sq_events == 0U) {
      sq_events = 1U;
      sq_d0     = dist;
      printf("SQ,FIRST,%ld,%ld\r\n", (long)dist, (long)(dist - isl_d_phase));
    } else {
      /* 【必须校验两次 T 口的间隔】11/18 15:39 那趟实测四个方形的间隔是
       *   166 / 2015 / 2502 / 2378 mm，正常只有 675~1030。
       * 166  = 同一个口被数了两次（ISL,OUT 后 171 mm 又触发一次）；
       * 2000+ = 已经走到下一个元素了，把远处的 T 口当成了本格的第 2 个。
       * 结果是计划表一路错位：车跑到 20902 mm 才只走到第 7 格（八边形），
       * 根本没回到发车区，单圈里程比正常那趟多了 4000 mm。
       * 所以两次 T 口之间必须落在合理窗口里，超出的当"新的第一个口"。 */
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
        Island_Advance(now, "SQOK");
      }
    }
    return 0U;
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
     * 顶点进岛的 entry_deg = 0：车顺着线自然进去，不需要打角。
     * 偏置大小按格取（entry_off10，0 = 用全局 ISL_ENTRY_OFF）：
     * 交叉区越窄，偏置必须越大 —— 偏置只有在"判进岛 → 越过交叉区"这段里程里
     * 才起作用，2.5 顶不住 200 mm 边长的小六边形（11/19 现场：根本不拐）。 */
    if (Island_Cfg()->entry_deg != 0U
        && yaw < (float)Island_Cfg()->entry_deg) {
      float off = (Island_Cfg()->entry_off10 != 0U)
                      ? ((float)Island_Cfg()->entry_off10 / 10.0f)
                      : ISL_ENTRY_OFF;

      isl_err_off = (Island_Cfg()->dir > 0) ? off : -off;
    } else {
      isl_err_off = 0.0f;
    }

    /* ---- 假进岛自检 ----
     * 八边形/大六边形仍用 300 mm / 25° 的历史判据；
     * HEXT（第 8 格）改用两段更严的判据：
     *   300 mm 处 yaw < 30°  -> 入口打角段没把车带进边，FABORT
     *   600 mm 处 yaw < 120° -> 只是直穿/切内角，不是绕六边形，FABORT
     * 59.txt 的 1001 mm / 63.2° 就是旧判据太松、被 SEX_F 兜底伪装成完成。
     * FABORT 后不前进计划表，留给 skip_mm 或后面的图案重试。 */
    {
      uint8_t fake = 0U;

      if (Island_Cfg()->kind == ISLK_HEXT) {
        if ((run >= (int32_t)ISL_HEXT_COMMIT_MM && yaw < ISL_HEXT_COMMIT_DEG)
            || (run >= (int32_t)ISL_HEXT_FAKE_MM1 && yaw < ISL_HEXT_FAKE_DEG1)
            || (run >= (int32_t)ISL_HEXT_FAKE_MM2 && yaw < ISL_HEXT_FAKE_DEG2)) {
          fake = 1U;
        }
      } else if (run >= (int32_t)ISL_FAKE_MM && yaw < ISL_FAKE_DEG) {
        fake = 1U;
      }

      if (fake) {
        printf("ISL,FABORT,%ld,%d\r\n", (long)run, (int)(yaw * 10.0f));
        isl_out_arm  = 0U;
        isl_err_off  = 0.0f;
        isl_state    = ISL_RET;
        isl_t_state  = now;
        isl_ret_dist = dist;
        hext_entry_phase = 0U;
        hext_exit_phase  = 0U;
        hext_exit_arm_d  = 0;
        Steer_Reset();
        return 0U;
      }
    }

    /* 二维惯导诊断：位置一进到离原点 60mm 内就打一行，跑一趟就能看出
     * "回原点"这个判据到底稳不稳（x/y 分别多少、误差多大）。 */
    if (Island_Cfg()->kind == ISLK_HEXT && hext_dr_on && !hext_dr_printed
        && run >= (int32_t)ISL_HEXT_EXIT_DR_MIN_MM
        && Hex_Dr_R() <= 60.0f) {
      hext_dr_printed = 1U;
      printf("HEXT,DR,%d,%d,%d,%ld\r\n", (int)(hext_dr_x), (int)(hext_dr_y),
             (int)Hex_Dr_R(), (long)run);
    }

    if (yaw >= (float)Island_Cfg()->exit_deg) isl_out_arm = 1U;   /* 偏航到阈值就锁定，抗陀螺抖动 */

    /* ---- 宽白锚点（HEXT 专用，11/25 16:00 加，16:20 改成"只武装"）----
     * 说明见 ISL_HEXT_BLOB_MIN_DEG：X 交叉口 = 两条线并成一团宽白，
     * 这个特征三次绕环 3/3 都命中，但它比交叉口早 15~24°（36~60mm），
     * 直接开火会让车提前左转、落到主线之外。所以分两步：
     *   ① 宽白出现 -> 只记 hext_blob_d（每帧刷新，武装）
     *   ② 宽白散开（n 从 >=7 掉到 <=5）-> 才允许出岛，那一帧正是刚压过 X。 */
    if (Island_Cfg()->kind == ISLK_HEXT
        && yaw >= ISL_HEXT_BLOB_MIN_DEG
        && n >= ISL_HEXT_BLOB_N
        && Line_Lobes(line_raw) == 1U) {
      if (hext_blob_d == 0) {
        printf("HEXT,BLOB,%d,%04X,%ld\r\n", (int)(yaw * 10.0f),
               (unsigned)line_raw, (long)run);
      }
      hext_blob_d = dist;
    }
    if (Island_Cfg()->kind == ISLK_HEXT
        && hext_blob_d != 0
        && (dist - hext_blob_d) <= (int32_t)ISL_HEXT_BLOB_GAP_MM
        && n <= ISL_HEXT_BLOB_DROP_N) {
      if (!isl_out_arm) {
        printf("HEXT,BLOBD,%d,%ld,%ld,%d\r\n", (int)(yaw * 10.0f),
               (long)(dist - hext_blob_d), (long)run, (int)n);
      }
      isl_out_arm = 1U;
      hext_blob_d = 0;
    }

    /* ---- 交叉点触发出岛（HEXT 专用，11/25 15:40）----
     * 【为什么不能只靠偏航】车到 X 交叉口时传感器看到两条线重合，加权重心趋近 0，
     * PD 倾向直行，于是**偏航会卡住上不去**。实测（15:36 那趟）：
     *   18544 yaw 已到 342.3°，之后 350mm yaw 一直停在 -353.2（=342.3°），
     *   于是 exit_deg=360 的 SEX 永远等不到 -> 一路绕到 exit_fall_mm(1500) 里程兜底
     *   -> 出岛位置随机，车在原地打转（vL=-8 / vR=1912）-> 丢线 LOST。
     * 而同一位置的交叉点图案是稳定的 2 团白簇（0C3C/0F1E/07E7/03F7/01CF/03CF/...），
     * 主循环的 hext_fork_last_d 一直在记，所以直接用"人在交叉口"触发出岛，
     * 不再等偏航凑够角度。
     * 门槛：累计偏航 >= 300°（排除进场那个交叉口和环前半段的其它图案），
     *       且最近 120mm 内见过合格分叉簇。 */
    if (Island_Cfg()->kind == ISLK_HEXT
        && yaw >= ISL_HEXT_JUNC_MIN_DEG
        && hext_fork_last_d != 0
        && hext_fork_fr >= ISL_HEXT_JUNC_MIN_FR
        && (dist - hext_fork_last_d) <= (int32_t)ISL_HEXT_JUNC_MM) {
      if (!isl_out_arm) {
        printf("HEXT,JUNC,%d,%ld,%ld,%d\r\n", (int)(yaw * 10.0f),
               (long)(dist - hext_fork_last_d), (long)run, (int)hext_fork_fr);
      }
      isl_out_arm = 1U;
    }

    /* ---- 纯偏航出岛 ----
     * 不再看 Island_Fork(n)。分叉图案判据实测不稳定（n 在 4~8 之间跳），
     * 漏判一帧就要再等一整圈，是"绕一圈出不来"的根源。
     * 这里只看两个量：累计偏航 >= ISL_EXIT_DEG，且进岛后已走够 ISL_EXIT_MIN_MM。
     * 后者是护栏，防止 yaw 偏大时在环道前半段误出（那时进锁直行会撞岛壁）。 */
    /* 记录"武装"时刻：武装（JUNC/BLOB/偏航阈值）到真正打角之间要留
     * ISL_HEXT_EXIT_DELAY_MM 直行，让枢轴落在 X 交叉口上（说明见那个宏）。 */
    if (isl_out_arm && hext_exit_arm_d == 0) hext_exit_arm_d = dist;

    {
      uint8_t hex_fire = 1U;

      if (Island_Cfg()->kind == ISLK_HEXT) {
        uint8_t win;

        /* ① 航向窗口：护段外看 yaw[350,385]，护段内改看护段里程
         *    （护段把航向锁住后 yaw 不再增长，见 ISL_HEXT_EXIT_GUARD_MM） */
        if (hext_guard_on) {
          win = (uint8_t)((dist - hext_guard_d0) >= (int32_t)ISL_HEXT_EXIT_GUARD_MM);
        } else {
          win = (uint8_t)(yaw >= ISL_HEXT_EXIT_YAW_LO && yaw <= ISL_HEXT_EXIT_YAW_HI);
        }

        /* ② 二维惯导：位置回到岔口原点。这一条与进岛姿态无关，
         *    不受"进岛 PD 多拐 6~7°"的影响，所以单独也能开火。 */
        if (ISL_HEXT_EXIT_DR_EN
            && !win && hext_dr_on && run >= (int32_t)ISL_HEXT_EXIT_DR_MIN_MM
            && Hex_Dr_R() <= (float)ISL_HEXT_EXIT_DR_MM) {
          win = 1U;
        }

        /* 开火还要：此刻真的在线上（n>=3）+ 已武装 + 过静默里程 */
        hex_fire = (uint8_t)(win && n >= ISL_HEXT_EXIT_ARMED_N
                             && (dist - hext_exit_arm_d) >= (int32_t)ISL_HEXT_EXIT_DELAY_MM);
      }

      if (isl_out_arm && run >= (int32_t)ISL_EXIT_MIN_MM && hex_fire) {
      printf("ISL,SEX,%d,%ld\r\n", (int)(yaw * 10.0f), (long)run);
      isl_out_arm = 0U;  isl_err_off = 0.0f;
      isl_state      = ISL_CROSS;        /* 锁直行穿过岔口 */
      isl_t_state    = now;
      isl_d0         = dist;
      isl_lock_mm    = (int32_t)Island_Cfg()->out_cross_mm;
      isl_lock_nmax  = 255U;             /* 出岛不等线变窄 */
      isl_lock_tmo   = 3000U;
      isl_lock_after = LOCK_AFTER_ADVANCE;
      if (Island_Cfg()->kind == ISLK_HEXT)
        Island_HextExitStart(gyro_angle, dist, yaw);
      Steer_Reset();
      return 1U;
      }
    }

    /* ---- 里程强制出岛：进岛后走够 exit_fall_mm 无条件出 ----
     * 偏航判据失效（陀螺漂/打滑）时靠它兜底，保证"一定出得去"。
     * 出口流程和正常出岛完全一样（锁直行穿过岔口）。 */
    if (isl_d_fin != 0 && Island_Cfg()->exit_fall_mm != 0U
        && run >= (int32_t)Island_Cfg()->exit_fall_mm) {
      /* 【11/25 16:30】先验"这真的是环"再决定要不要打角。
       * 16:26 那趟：假进岛后在主线上跑了 1500mm，只攒 343.8° = 0.23°/mm，
       * 里程兜底一触发就左打 71°，车直接从直道飞出去。
       * 角密度不够 -> 认输换格，不打角、不锁直行，让车顺着主线跑完。
       * 真环的角密度 0.37~0.44，这条 0.30 卡在中间。 */
      if (Island_Cfg()->kind == ISLK_HEXT
          && yaw < (float)run * ISL_HEXT_EXIT_FALL_RATE) {
        printf("ISL,SEXBAD,%ld,%d\r\n", (long)run, (int)(yaw * 10.0f));
        isl_out_arm  = 0U;
        isl_err_off  = 0.0f;
        isl_state    = ISL_RET;
        isl_t_state  = now;
        isl_ret_dist = dist;
        hext_entry_phase = 0U;
        hext_exit_phase  = 0U;
        hext_blob_d      = 0;
        Steer_Reset();
        Island_Advance(now, "SEXBAD");   /* 换格 + hext_locked，本圈不再进 */
        return 0U;
      }
      printf("ISL,SEX_F,%ld,%d\r\n", (long)run, (int)(yaw * 10.0f));
      isl_out_arm = 0U;  isl_err_off = 0.0f;
      isl_state      = ISL_CROSS;        /* 锁直行穿过岔口 */
      isl_t_state    = now;
      isl_d0         = dist;
      isl_lock_mm    = (int32_t)Island_Cfg()->out_cross_mm;
      isl_lock_nmax  = 255U;
      isl_lock_tmo   = 3000U;
      isl_lock_after = LOCK_AFTER_ADVANCE;
      if (Island_Cfg()->kind == ISLK_HEXT)
        Island_HextExitStart(gyro_angle, dist, yaw);
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
   * in_cross_mm 只当上限兜底用。10 mm 是防抖动：进岛那一帧 n 偶然掉一下不算穿完。
   * ↑ 这段逻辑已经并入上面的 ISL_CROSS 统一实现（LOCK_AFTER_SESSION 分支，
   *   参数：isl_lock_mm=ISL_ENTER_LOCK_MM(10)、nmax=ISL_T_N-1、tmo=1200）。 */

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

  /* ---- 全局 T 口直行（方形/八边形已从计划表删除） ----
   * 计划表只剩 3 个绕行环岛；从 SMALLHEX 出来到第 8 格之前，中间的方形/八边形
   * T 口全部走这里：锁直行 60 mm，然后回到 NONE，**不推进 plan_idx**。
   * 这样 T 口既不会把计划表带偏，也不会阻止后面第 8 格的图案进岛。 */
  if ((plan_idx == 2U || hext_done != 0U) && isl_state == ISL_NONE
      && n >= ISL_GLOBAL_T_N) {
    isl_cfm        = 0U;
    isl_t_state    = now;
    isl_d0         = dist;
    isl_state      = ISL_CROSS;
    isl_lock_mm    = 60;
    isl_lock_nmax  = 5U;       /* 走够 60mm 且宽白消失才放开；另有 120mm 硬上限 */
    isl_lock_tmo   = 1500U;
    isl_lock_after = LOCK_AFTER_TONLY;
    printf("T,IN,%d,%04X,%ld\r\n", (int)n, (unsigned)line_raw, (long)dist);

    /* HEX8 出岛后的 T1/T2 是普通 T 口，T3/T4 就是发车区的两根竖条。
     * 竖条光电检测可能因高速/10 路而漏掉 BAR,HIT；这里用 T 口事件补一根
     * 虚拟竖条，仍然交给 Lap_Gate 做窗口、Z 字和 300~550mm 成对判断。 */
    if (hext_done != 0U) {
      if (gate_t_count < 250U) gate_t_count++;
      if (gate_t_count == 3U || gate_t_count == 4U) {
        printf("GATE,T%d,%ld\r\n", (int)gate_t_count, (long)dist);
        bar_hit    = 1U;
        bar_hit_mm = dist;
      }
    }
    return 1U;
  }

  /* ---- 计划格超时兜底：太久没等到该格的图案，强制前进一格 ---- */
  if (Island_Cfg()->kind != ISLK_HEXT
      && isl_t_phase != 0U && (uint32_t)(now - isl_t_phase) > ISL_PHASE_MS) {
    if (Island_IsSquare()) {
      sq_events = 0U;
    }
    Island_Advance(now, "SKIP");
    return 0U;
  }

  /* ---- 本格的【里程】兜底：走够 skip_mm 还没完成这一格，就前进一格 ----
   * ISL_PHASE_MS 那条是 10 s 才生效，1050 mm/s 下等于 10 m，等于没有兜底。
   * 图案判据一旦漏掉，plan_idx 就永远停在这一格 —— 后面每个环岛都会拿错图案
   * 去处理，表现就是"到了直角环岛反而飞出去"（方形格只认 T 口，六边形格只认
   * 分叉，两种都不对）。
   *
   * 【11/20 重构后】方形/八边形已从计划表删除，不再有“穿越型格”的 skip_mm；
   * 八边形的道口实际也是 90°，旧注释里的“45° 道口”已作废。
   * 这段只服务 BIGHEX / SMALLHEX 两个绕行格和 HEX8 的 20000 mm 兜底。
   * 未单独配置 skip_mm 的六边形格仍沿用默认 ISL_HEX_MAX_MM(1200)；
   * 第 8 格已单独放到 20000（见 HEX8 行），从 SMALLHEX 一路等到第 8 格。
   * 注意判 isl_state == ISL_NONE：正在锁直行/绕行时不受影响。 */
  {
    int32_t skip_mm = (int32_t)Island_Cfg()->skip_mm;

    if (skip_mm == 0 && !Island_IsSquare()) skip_mm = (int32_t)ISL_HEX_MAX_MM;
    /* 候选已经出现：再给 ISL_SKIP_GRACE_MM 的宽限，别在确认前抢跑。
     * 只对 skip_mm>0 的格生效；skip_mm=0 的格仍然表示"不用里程兜底"。 */
    if (skip_mm > 0 && isl_cfm != 0U) skip_mm += (int32_t)ISL_SKIP_GRACE_MM;
    if (skip_mm > 0 && isl_state == ISL_NONE && hext_done == 0U
        && (dist - isl_d_phase) >= skip_mm) {
      printf("ISL,HEXOVER,%ld,%s,%d,%04X\r\n",
             (long)(dist - isl_d_phase), Island_Cfg()->name,
             (int)n, (unsigned)line_raw);
      isl_cfm        = 0U;
      isl_block_dist = 0;
      isl_out_arm    = 0U;
      isl_err_off    = 0.0f;
      sq_events      = 0U;
      Island_Advance(now, "HEXOVER");
      return 0U;
    }
  }

  /* ================= 穿越型格（历史死代码：计划表已无 CROSS 格）=================
   * 【11/20 重构后】方形/八边形已从计划表删除，这个分支不会再执行；
   * 保留只为对照历史 T 口逻辑。八边形的道口实际也是 90°，旧 45° 注释作废。
   * 当前 T 口直行见上面的全局 T 分支（ISL_GLOBAL_T_N=11）。
   * 旧配置差异（仅历史）：
   *   方形   fork_n=11（T 口 n>=11）   need_ev=2   锁 60 mm
   *   八边形 fork_n=9 （旧误判为 45°） need_ev=2  锁 60 mm
   *
   * 【11/19 22:15 改为"单帧可认账"】判据结构改了一处：
   * 原来是 if(!isl_cfm){置候选} else if(时间到){触发}，
   * 也就是"这个分支永远至少需要 2 帧"。现在把"触发"提到 else 外面，
   * 条件变成 (now - isl_t_cfm) >= ISL_CONFIRM_MS：
   *   ISL_CONFIRM_MS = 0  → 第一帧就满足 → 单帧确认（1050 mm/s 必须这样）
   *   ISL_CONFIRM_MS = 10 → 第二帧才满足 → 原来的 2 帧确认（想回退就改回 10）
   * 这个宏还是唯一的开关，不用改函数结构就能在两种行为间切换。 */
  if (Island_IsSquare()) {
    uint8_t t_hit = 0U;

    /* 死代码：当前计划表没有 CROSS 格。
     * 方形旧门槛 11（拐角 9~10 分得开）；八边形旧 9 是错误假设，
     * 现场已确认八边形道口也是 90°，且该格已删除。 */
    if (n >= Island_Cfg()->fork_n) {
      if (!isl_cfm) { isl_cfm = 1U; isl_t_cfm = now; }
      if ((uint32_t)(now - isl_t_cfm) >= ISL_CONFIRM_MS) t_hit = 1U;
    } else {
      isl_cfm = 0U;
    }

    if (t_hit) {
      isl_cfm        = 0U;
      isl_state      = ISL_CROSS;
      isl_t_state    = now;
      isl_d0         = dist;
      isl_lock_mm    = (int32_t)Island_Cfg()->out_cross_mm;   /* 60 */
      isl_lock_nmax  = 5U;           /* 走够 60 mm 且宽白消失才放开 */
      isl_lock_tmo   = 1500U;
      isl_lock_after = LOCK_AFTER_EVENT;
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
   * 没有"第二个 T 口"可以当参照。
   *
   * 【11/19 23:00 起这一段只服务"需要绕行"的六边形格】
   * 八边形已改成直穿型（kind=CROSS），走上面那个分支、用 need_ev=1 前进，
   * 不再进这里。现在没有任何一格配置了 force_mm，这段是留给以后
   * "某个六边形入口实在认不出来"时的备用手段，行为不变。 */
  if (Island_Cfg()->force_mm != 0U && !isl_force_used
      && !Island_HextBlocked()
      && (dist - isl_d_phase) >= (int32_t)Island_Cfg()->force_mm) {
    isl_force_used = 1U;         /* 本格只许强制进岛一次，防止 FABORT 后原地反复重进 */
    printf("ISL,FIN_M,%s,%ld,%d,%04X\r\n", Island_Cfg()->name,
           (long)(dist - isl_d_phase), (int)n, (unsigned)line_raw);
    isl_cfm     = 0U;
    isl_t_state = now;
    isl_yaw0    = gyro_angle;
    isl_out_arm = 0U;
    isl_d_fin   = dist;
    if (Island_Cfg()->in_cross_mm != 0U) {
      isl_state      = ISL_CROSS;
      isl_d0         = dist;
      isl_lock_mm    = ISL_ENTER_LOCK_MM;
      isl_lock_nmax  = (uint8_t)(ISL_T_N - 1U);
      isl_lock_tmo   = 1200U;
      isl_lock_after = LOCK_AFTER_SESSION;
    } else {
      isl_state = ISL_SESSION;
    }
    if (Island_Cfg()->kind == ISLK_HEXT) Island_HextEntryStart(dist);
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
  if (Island_Cfg()->in_cross_mm != 0U && !Island_HextBlocked()
      /* 【11/24】格起点后 400mm 内不认顶点型入口：躲开上一个格出岛留下的十字。
       * 96.txt 就是在 245mm 处把大套小出口十字当成了小六边形入口。 */
      && (dist - isl_d_phase_raw) >= (int32_t)ISL_ENTRY_MIN_MM
      && Island_Pattern(n)) {
    uint8_t t_hit = 0U;

    /* 和方块格同样的单帧/多帧统一写法，见上面那段说明 */
    if (!isl_cfm) { isl_cfm = 1U; isl_t_cfm = now; }
    if ((uint32_t)(now - isl_t_cfm) >= ISL_CONFIRM_MS) t_hit = 1U;

    if (t_hit) {
      isl_cfm     = 0U;
      isl_state      = ISL_CROSS;      /* 先锁直行穿过顶点交叉区 */
      isl_t_state = now;
      isl_yaw0    = gyro_angle;
      isl_out_arm = 0U;
      isl_d_fin   = dist;
      isl_d0      = dist;
      isl_lock_mm    = ISL_ENTER_LOCK_MM;
      isl_lock_nmax  = (uint8_t)(ISL_T_N - 1U);
      isl_lock_tmo   = 1200U;
      isl_lock_after = LOCK_AFTER_SESSION;
      printf("ISL,FIN,%s,%d,%04X,%ld,%ld\r\n", Island_Cfg()->name, (int)n, (unsigned)line_raw, (long)dist,
             (long)(dist - isl_d_phase_raw));
      printf("ISL,HIN,%ld\r\n", (long)dist);
      if (Island_Cfg()->kind == ISLK_HEXT) Island_HextEntryStart(dist);
      Steer_Reset();
      return 1U;                        /* 本帧起锁直行穿顶点 */
    }
    return 0U;
  }

  /* ---- ② 【11/19 23:00 删除】"第二个 T 口之后走够 ISL_ENTRY_MM 就强制进岛" ----
   * 这是第一阶段旧赛道的队友版判据（旧赛道一圈 10 个环岛、大环岛入口固定）。
   * 原注释写着"第二阶段已停用，置大值等于停用"，但实际给的 1660 mm
   * **并不是一个大值**（一格才 2286 mm），所以它一直是活的：
   * 任何一次方形 SQOK 都会把 isl_d_t2 立起来，1660 mm 之后如果正好在
   * 某个六边形格上，就会凭空 ISL,FIN_F 进一次"假岛"。
   * 54.txt 里没出问题只是因为八边形那次 FIN_M 顺手把 isl_d_t2 清了；
   * 八边形改成直穿型以后不再进这里，这条就会直接落到处在格 8 的头上 ——
   * 表现就是"车在第 8 个六边形里反复绕"。所以整段删掉，不留隐患。 */

  /* ---- ①a HEXT 独立入口：锚定"分叉簇末 + 40mm"，不看本帧图案 ----
   * 67.txt：簇首 8814/8836、簇末 8888/8904、实际 FIN 8925/8948。
   * 用簇首 + 110mm 会随簇长飘；用簇末 + 40mm 两次都落在 37~44mm，稳定得多。
   * 这里只用主循环逐帧记下的 hext_fork_d / hext_fork_last_d，本帧 n 是多少都不影响。 */
  if (Island_Cfg()->kind == ISLK_HEXT
      && hext_done == 0U && hext_locked == 0U
      && (isl_ret_dist == 0 || (dist - isl_ret_dist) >= (int32_t)ISL_RET_MM)
      && (dist - isl_d_phase_raw) >= (int32_t)ISL_HEXT_WAIT_MIN_MM
      /* 【簇本身也必须在等待下限之后】只卡"判定这一帧"的下限不够：
       * 下限之前出现过的假簇还在 200mm 的 hold 窗口里，下限一放开就会拿它进岛。
       * 5ms 版实测（11/24 11:48 那趟）：
       *   cluster 7860~7920（八边形后直角弯的假 2 团白）-> 8003 就 FIN，
       *   rel 8003 而真入口在 8800~9200 -> 进错地方 -> SEX_F 兜底 -> 重来。
       * 加上这条后，假簇（7920 < 8000）直接被丢，只有 8800+ 的真簇能进。 */
      && (hext_fork_last_d - isl_d_phase_raw) >= (int32_t)ISL_HEXT_WAIT_MIN_MM
      && g_clean_frames >= ISL_HEXT_CLEAN_FRAMES   /* 先进"稳定在线"门，见 g_clean_frames 注释 */
      && hext_fork_last_d != 0
      /* 【11/24 新增】簇必须真的"持续"：>=3 帧且跨度 >=20mm。
       * 风扇振动会把一条宽白抖成两团白，单帧看起来和岔口一样；真岔口会
       * 持续 100~300mm，假图案只闪 1 帧。这条直接干掉"0 长度簇"进岛。 */
      && hext_fork_fr >= ISL_HEXT_FORK_MIN_FR
      && (hext_fork_last_d - hext_fork_d) >= (int32_t)ISL_HEXT_FORK_MIN_MM
      && (dist - Hext_ForkAnchor()) >= (int32_t)ISL_HEXT_FORK_ARM_MM
      && (dist - Hext_ForkAnchor()) <= (int32_t)ISL_HEXT_FORK_HOLD_MM) {
    /* 日志多看一列：这一簇的帧数与跨度，现场直接判断"是不是抖出来的"。
     * 第 3 列是相对入口锚点（长簇时是簇首，正常是簇末）的里程。 */
    printf("HEXT,ENTRY,%d,%04X,%ld,%ld,%ld,%d\r\n", (int)hext_fork_n, (unsigned)hext_fork_bm,
           (long)(dist - Hext_ForkAnchor()), (long)(dist - isl_d_phase_raw),
           (long)(hext_fork_last_d - hext_fork_d), (int)hext_fork_fr);
    printf("ISL,FIN,%s,%d,%04X,%ld,%ld\r\n", Island_Cfg()->name, (int)n,
           (unsigned)line_raw, (long)dist, (long)(dist - isl_d_phase_raw));

    hext_fork_d      = 0;             /* 用完即清：同一个岔口只进一次 */
    hext_fork_last_d = 0;
    hext_fork_fr     = 0U;
    isl_cfm      = 0U;
    isl_t_state  = now;
    isl_yaw0     = gyro_angle;
    isl_out_arm  = 0U;                /* 新会话：出岛武装清零 */
    isl_d_fin    = dist;
    isl_state    = ISL_SESSION;       /* HEX8 的 in_cross_mm=0：不锁直行，直接进会话 */
    if (Island_Cfg()->kind == ISLK_HEXT) Hex_Dr_Reset(dist);   /* 二维惯导：原点 = 此刻岔口 */
    Island_HextEntryStart(dist);
    Steer_Reset();
    return 1U;
  }

  /* ---- ① 分叉图案进岛（更早，能提前于强制点触发） ----
   * 【11/25】顶点型格（in_cross_mm != 0，当前只有第 2 格 SMALLHEX）在格起点
   * ISL_ENTRY_MIN_MM 内**连分叉入口也不认**。
   * 97.txt 第 1 趟实测：GRID 内 474mm 处以 n=5 / 位图 0x007C 进了岛，
   * 随后 850mm 只转了 96.9°（0.11°/mm，真六边形应该是 0.42°/mm），
   * 最后被 SEX_F(850) 兜底 —— 那是弯道变宽被当成"分叉"，不是环岛入口。
   * 之前只给顶点分支（Island_Pattern / n>=11）加了这道闸，漏了分叉分支，
   * 而实际触发走的正是这条。 */
  if (Island_Fork(n)
      && !(Island_Cfg()->in_cross_mm != 0U
           && (dist - isl_d_phase_raw) < (int32_t)ISL_ENTRY_MIN_MM)
      /* 【11/25 二改】顶点型格（in_cross_mm != 0，当前只有第 2 格 SMALLHEX）
       * 只允许走"顶点入口"（n >= ISL_T_N=11），**禁止分叉入口**。
       * 98.txt 实测：闸门放到 600mm 后，车在 915mm 处又被 n=5 / 0x003E 骗进岛，
       * 之后 850mm 只转了 61.8°，沿主线直走（n 一直 3~4）—— 又一次白跑一趟。
       * 而历史上真正进得去的入口都是 n=12（0FFF）走的顶点分支。
       * 所以对这一类格，n=5~9 的"弯道变宽"一律不再算入口。 */
      && !(Island_Cfg()->in_cross_mm != 0U && n < ISL_T_N)) {
    uint8_t hit = 0U;

    /* 刚出过岛就先别急着再进 */
    if (isl_ret_dist != 0 && (dist - isl_ret_dist) < (int32_t)ISL_RET_MM) {
      isl_cfm = 0U;
      return 0U;
    }

    /* 【11/20 重构：纯图案触发 + 方形区等待下限】
     * 方形/八边形已从计划表删除，第 8 格没有“八边形出口 + 里程窗口”锚点。
     * 入口认图案：n=5~8（真分叉，不是方形 n>=11 / 八边形 n=9~10），
     * 且最右四路至少一路亮（00FF / 003F / 007E / 00FA / 00F8）；
     * 01F0 / 0FF0 / 0FE0 最右四路全灭，不认。
     * 另外本格起点是 SMALLHEX 出口，60.txt 在 +106/+1420/+2566 就误进了
     * 三个方形/直角；真正的第 8 格还在更后面。所以加 9000 mm 等待下限，
     * 先把方形区走完。这不是八边形锚点，只是纯图案的防误进。 */
    /* 【HEXT 必须走独立分支，不能落进下面的通用 fork 逻辑】
     * 63/64/66 的根因就在这里：hext_done/hext_locked 让上面的条件失败后，
     * 代码落进 else if (!isl_cfm) 的通用候选逻辑，fork_mm=0 时下一帧
     * 直接 hit，绕过纯图案/方向/等待/锁的全部检查，SOUT 后又打一次
     * ISL,FIN,HEX8,07E0/0FC0。 */
    if (Island_Cfg()->kind == ISLK_HEXT) {
      /* HEXT 的进岛判定已经在上面的 ①a 独立做完（不看本帧 n，只认簇首+110mm）。
       * 这里必须保留这个分支、并且只把候选清掉：
       * 63/64/66 的根因就是 HEXT 掉进下面的 else if (!isl_cfm) 通用候选逻辑，
       * fork_mm=0 时下一帧直接 hit，绕过图案/方向/等待/锁的全部检查，
       * SOUT 之后又打一次 ISL,FIN,HEX8,07E0/0FC0。 */
      isl_cfm = 0U;
    } else if (!isl_cfm) {
      isl_cfm = 1U;
      isl_t_cfm = now;
      isl_d0 = dist;
    } else {
      isl_t_cfm = now;                 /* 刷新"最后看到图案"的时刻 */
      if ((dist - isl_d0) >= (int32_t)Island_Cfg()->fork_mm) {
        hit = 1U;
      }
    }

    if (hit) {
      isl_cfm     = 0U;
      isl_t_state = now;
      isl_yaw0    = gyro_angle;
      isl_out_arm = 0U;                 /* 新会话：出岛武装清零 */
      isl_d_fin   = dist;
      printf("ISL,FIN,%s,%d,%04X,%ld,%ld\r\n", Island_Cfg()->name,
             (int)n, (unsigned)line_raw, (long)dist,
             (long)(dist - isl_d_phase_raw));
      Steer_Reset();
      /* 进岛先锁直行穿过交叉点（顶点进入型）：
       * 交叉点处加权重心会被两条分支平均成斜向，PD 会带着车斜穿过去。
       * 这几十毫米内不理会 err，按车头方向直着冲过交叉点，过了再交给 PD。 */
      if (Island_Cfg()->in_cross_mm != 0U) {
        isl_state      = ISL_CROSS;    /* 先锁直行穿过交叉区，再由 LOCK_AFTER_SESSION 交给 PD */
        isl_d0         = dist;
        isl_lock_mm    = ISL_ENTER_LOCK_MM;
        isl_lock_nmax  = (uint8_t)(ISL_T_N - 1U);
        isl_lock_tmo   = 1200U;
        isl_lock_after = LOCK_AFTER_SESSION;
        printf("ISL,HIN,%ld\r\n", (long)dist);
      } else {
        isl_state = ISL_SESSION;
      }
      if (Island_Cfg()->kind == ISLK_HEXT) Island_HextEntryStart(dist);
      return 1U;                        /* 本帧起直行锁定 */
    }
  } else if (isl_cfm && (uint32_t)(now - isl_t_cfm) > ISL_FORK_TMO_MS) {
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
   * HEXT（第 8 格）在纯图案进岛之前会长时间停留在本格，期间中间的方形/八边形
   * T 口由全局 T 分支处理；RESYNC 不能把这些 T 当成“计划表落后”去推进 HEX8。 */
  if (isl_state != ISL_NONE || Island_Cfg()->kind == ISLK_HEXT) {
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

/* HEXT 入口线触发条件：必须是岛内单线，不能是岔口宽区/双白团。
 * 4~6 路是实测岛内边的典型宽度；>6 路多为岔口/宽白，n<4 视为还没接到线。 */
static uint8_t Island_EntryLineOK(uint8_t n)
{
  if (n < 4U || n > 6U) return 0U;
  if (Line_Lobes(line_raw) != 1U) return 0U;
  return 1U;
}

/* HEXT（第 8 格）入口打角段。
 * 岔口处两条线加权后的 err 会在 +4~-4 之间跳（59.txt 的 000F/0F00），
 * 再叠加 entry_off 会直接饱和，车在岔口左右满舵来回摆。
 * 这里在前 20°（最多 500 mm）内绕开 PD，直接给固定差速：
 *   v = 420, turn = ±600（dir=+1 取负 = 右转，内轮反转）
 * 15° 后 main 进入 phase2，锁直行 20mm，再交岛内 PD。 */
static uint8_t Island_EntryTurn(float *v, float *turn, int32_t dist)
{
  const IslandCfg *c = Island_Cfg();
  float yaw;

  if (isl_state != ISL_SESSION || c->kind != ISLK_HEXT) return 0U;
  if ((dist - isl_d_fin) >= (int32_t)ISL_HEXT_ENTRY_TURN_MAX_MM) return 0U;

  yaw = (float)c->dir * (gyro_angle - isl_yaw0);
  if (yaw >= ISL_HEXT_ENTRY_TURN_DEG) return 0U;

  *v    = ISL_HEXT_ENTRY_TURN_V;
  /* dir=+1 必须右拐：turn=-600 -> tL=1020、tR=-180，内右轮反转。 */
  *turn = (c->dir > 0) ? -ISL_HEXT_ENTRY_TURN : ISL_HEXT_ENTRY_TURN;
  isl_err_off = 0.0f;
  return 1U;
}

/* HEX8 出岛，四段（11/25 16:23 定型）：
 *   phase2 左打角：bang-bang 到 ISL_HEXT_EXIT_TURN_DEG，速度 PIVOT_V
 *   phase3 锁航向找线：turn=0 直行，直到 n>=LINE_N 连续 2 帧或走满 HOLD_MM
 *   phase4 收尾直行：STRAIGHT_MM，然后由 Island_Step 打 ISL,SOUT 并换格
 * 只在 ISL_CROSS + LOCK_AFTER_ADVANCE（出岛锁）期间使用；
 * 返回 1 表示本帧 v/turn 已填好，main loop 不再走 t_rcv/PD。 */
static uint8_t Island_ExitTurn(float *v, float *turn, int32_t dist, uint8_t n,
                               float err_raw)
{
  const IslandCfg *c = Island_Cfg();

  if (hext_exit_phase == 0U) return 0U;
  if (isl_state != ISL_CROSS || c->kind != ISLK_HEXT
      || isl_lock_after != LOCK_AFTER_ADVANCE) {
    hext_exit_phase = 0U;
    return 0U;
  }

  /* ---- phase2：打角（bang-bang，照大套小的 BS_EXT）----
   * 旧写法是航向 P 环 terr = GAP_KP*(yaw - 目标)，一阶收敛：
   * yaw 到 50.5° 时 terr 只剩 7.12*4.5 = 32，车基本不再转 ——
   * 1.txt 三次实测都停在 48.7 / 50.5 / 50.5°，永远到不了 -55°。
   * 大套小的 BS_EXT 是固定差速 ±215，转到 40° 就停，不存在"差一点到不了"。
   * 这里照它：没到目标就恒定给 ISL_HEXT_EXIT_TURN，到了立刻停、进直行段。
   * 速度另给 ISL_HEXT_EXIT_PIVOT_V(350)：°/mm = ω/v，降速等于把同样角度
   * 压缩到更短里程，转完车才不会被甩到主线 90mm 之外。 */
  if (hext_exit_phase == 2U) {
    float   yaw_now = (float)c->dir * (gyro_angle - hext_exit_yaw0);
    int32_t run     = dist - hext_exit_d0;

    /* 右转！目标 = hext_exit_need_deg（= 360 - SEX 时的 yaw）。
     * 右转时 yaw_now 递增（dir=+1 时 turn<0 让左轮快 = 右转）。 */
    if (yaw_now >= hext_exit_need_deg
        || run >= (int32_t)ISL_HEXT_EXIT_TURN_MAX_MM) {
      printf("HEXT,EXIT,%d,%ld,%d,%d\r\n", (int)(yaw_now * 10.0f), (long)run,
             (yaw_now >= hext_exit_need_deg) ? 1 : 0,
             (int)(hext_exit_need_deg * 10.0f));
      hext_exit_straight_d0 = dist;    /* 找线段里程从"转到角"这一帧开始算 */
      hext_exit_line_cnt    = 0U;
      hext_exit_phase = 3U;            /* 进"锁航向找线"段，见下面 phase3 */
    } else {
      *v    = ISL_HEXT_EXIT_PIVOT_V;
      /* 右转：turn<0 -> tL = v-turn 变大、tR = v+turn 变小 -> 左轮快 -> 右转。
       * 【11/25 17:00：这里从"左转 70°"改成"右转补圆"】
       * 之前 16:12 把这里定成左转，是因为当时以为出岛要"左转回主线 60°"；
       * 实测证明那个推导把 yaw 的起点记错了（yaw 从 FIN 起算，一圈 360 就把
       * 车头带回主线方向），于是每次左转 70° 都把车转成横的。 */
      *turn = (c->dir > 0) ? -ISL_HEXT_EXIT_TURN : ISL_HEXT_EXIT_TURN;
      return 1U;
    }
  }

  /* ---- phase3：锁航向找线（不看 PD）----
   * 打角刚结束时机头已经摆正，但线还没回到传感器底下，
   * 这时 err 是打角期间攒下的残值（+7 满量程），交给 PD 只会满舵打转。
   * 所以这一段保持 turn=0 直着走，直到 n>=3 连续两帧（线真回来了），
   * 或者走满 ISL_HEXT_EXIT_HOLD_MM（认栽，交给后面的兜底）。 */
  if (hext_exit_phase == 3U) {
    /* 【11/25 17:40】合并之后固定直行 SETTLE_MM 再交 PD。
     * 不再用"n>=3 且 |e|<=4 立刻交 PD"——因为合并点两条线相切，
     * 那个判据拿到的很可能就是环线自己，交 PD 后车会再绕一圈
     * （17:31 实测：SOUT 后 gyro −306.2 -> +186 = 右转 325°/700mm）。
     * 直行 SETTLE_MM 后环线已偏出传感器，剩下的才是主线（当前 80mm，
 * 见 ISL_HEXT_EXIT_SETTLE_MM 里 109.txt 的定量说明）。 */
    hext_exit_line_cnt = 0U;

    if ((dist - hext_exit_straight_d0) >= (int32_t)ISL_HEXT_EXIT_SETTLE_MM) {
      printf("HEXT,SETTLE,%ld,%d,%d\r\n", (long)(dist - hext_exit_straight_d0),
             (int)n, (int)(err_raw * 10.0f));
      hext_exit_straight_d0 = dist;      /* 收尾段里程从现在开始算 */
      hext_exit_phase = 4U;
    } else {
      *v    = ISL_CROSS_V;
      *turn = 0.0f;
      return 1U;
    }
  }

  /* ---- phase4：直行等线回来（上限 ISL_HEXT_EXIT_FIND_MM）----
   * 这一段同样不交给 PD：线还没回来时 PD 只会按旧误差打满（拽回环岛）。
   * 线真的回来了（n>=3 连续 2 帧，且不是宽白 n<=6）才进收尾段。 */
  if (hext_exit_phase == 4U) {
    if (n >= ISL_HEXT_EXIT_LINE_N && n <= ISL_HEXT_EXIT_LINE_NMAX
        && fabsf(err_raw) <= ISL_HEXT_EXIT_LINE_ERR) hext_exit_line_cnt++;
    else                                          hext_exit_line_cnt = 0U;

    if (hext_exit_line_cnt >= 2U
        || (dist - hext_exit_straight_d0) >= (int32_t)ISL_HEXT_EXIT_FIND_MM) {
      printf("HEXT,FIND,%ld,%d,%d,%d\r\n", (long)(dist - hext_exit_straight_d0),
             (int)n, (hext_exit_line_cnt >= 2U) ? 1 : 0, (int)(err_raw * 10.0f));
      hext_exit_line_cnt    = 0U;
      hext_exit_straight_d0 = dist;
      hext_exit_phase = 5U;
    } else {
      *v    = ISL_CROSS_V;
      *turn = 0.0f;
      return 1U;
    }
  }

  /* ---- phase5：收尾直行 ISL_HEXT_EXIT_STRAIGHT_MM，然后由 Island_Step 收尾 ---- */
  if (hext_exit_phase == 5U) {
    *v    = ISL_CROSS_V;
    *turn = 0.0f;
    return 1U;
  }

  return 0U;
}

/* 大套小流程的误差目标偏移（等价摄像头的补线），其余恒为 0。
 * 进环靠它把车掰到另一条分支上：PD 会主动开到新目标并稳住，
 * 不像固定偏置那样被 PD 抵消掉（偏置 95 只能让车偏 0.63 个单位）。 */
static float Island_ErrOffset(int32_t dist)
{
  /* 大套小（bs.err_off）+ 通用环岛切向进岛（isl_err_off），两者不会同时非零 */
  float off = bs.err_off + isl_err_off;

  /* ---- 出岛后的短程外偏（ISL_HEXT_OUT_BIAS）----
   * 【只在出岛之后生效】进岛的 phase1/2/3 和岛内 PD 一律不加：
   *   ① hext_outbias_d0 只在 ISL,SOUT 那一刻置起来，进岛时 Island_HextEntryStart 清零；
   *   ② 这里再用 Island_Busy() 兜一道 —— 只要还在 CROSS（穿 T 口/锁直行）
   *      或 SESSION（岛内绕行），就算标志漏清也绝不往 e 上叠偏置。
   *     （107.txt 第 1 圈的偏置问题就是"合并点交回 PD 后还压在环线上"，
   *       跟入口无关；108.txt 那次连岛都没进去，是叉口判据误触发，不是偏置。）
   * 梯形包络：前 20mm 渐入，末 60mm 渐出，走满窗长自动关掉。
   * 为什么要渐入渐出：偏置是直接加在 e 上的，阶跃会让 D 项（KD*(e-last_e)）
   * 打出一记假脉冲；渐出则避免关掉那一刻 e 突然回弹、PD 又把车甩回环线。 */
  if (hext_outbias_d0 != 0) {
    int32_t b_run = dist - hext_outbias_d0;

    if (b_run >= (int32_t)ISL_HEXT_OUT_BIAS_MM) {
      hext_outbias_d0  = 0;
      hext_outbias_val = 0.0f;
      printf("HEXT,OUTBEND,%ld\r\n", (long)b_run);
    } else if (hext_outbias_val != 0.0f && !Island_Busy()) {
      float k = 1.0f;

      if (b_run < 20) {
        k = (float)b_run / 20.0f;                       /* 渐入 */
      } else if (b_run > (int32_t)(ISL_HEXT_OUT_BIAS_MM - 60)) {
        k = (float)((int32_t)ISL_HEXT_OUT_BIAS_MM - b_run) / 60.0f;   /* 渐出 */
      }
      off += hext_outbias_val * k;
    }
  }

  return off;
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

/* ==================== 串口急停 ====================
 * 电脑往 USART3（调试串口）发一个 's'（大小写都认）就立刻停车，
 * 收尾和 PB14 按键停车完全一样：电机 0、FOLLOW,STOP、关负压。
 *
 * 【为什么直接读寄存器，不用 HAL_UART_Receive】
 *   · 不用开中断、不占 DMA，也不进 HAL 的收发状态机；
 *   · 超时/溢出都不会把 5ms 控制循环卡住（HAL 阻塞接收碰到 ORE 会耗在
 *     状态机里，运动中对电机是灾难）；
 *   · 直接在 FIFO 里读走全部字节，命令延迟 < 1ms。
 * 【ORE 处理】STM32F1 上读 SR 再读 DR 就会清 ORE；我们本来就是
 * "读到 RXNE 就读 DR"，所以溢出留不住，不会变成永久收不到。
 * 【要不要额外开中断】不需要。这里每轮主循环都来查一次，比中断还简单。 */
static uint8_t Serial_StopPressed(void)
{
  uint8_t got = 0U;

  /* 一次把 FIFO 里积压的字节全读掉：发 "s\r\n" 时不会只处理第一个 */
  while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET) {
    uint8_t ch = (uint8_t)(huart3.Instance->DR & 0xFFU);

    if (ch == (uint8_t)'s' || ch == (uint8_t)'S') got = 1U;
  }
  return got;
}

/* ==================== HEX8 二维惯导（位置积分） ====================
 * 原点是 ISL,FIN 那一刻的岔口，x 轴沿那一刻的车头方向。
 * 每 5ms 用编码器增量 + 陀螺航向积分一次：
 *     x += Δd·cosψ,  y += Δd·sinψ    （ψ = gyro_angle − isl_yaw0）
 * 闭环路径绕完一圈必然回到原点，与进岛姿态无关 —— 这是它比一维 yaw 稳的原因。
 * 只用距离 4mm 级的 Δd（编码器 5ms 增量），不会有累积的"里程差"问题。
 */
static void Hex_Dr_Reset(int32_t dist)
{
  hext_dr_x = 0.0f;
  hext_dr_y = 0.0f;
  hext_dr_d0 = dist;
  hext_dr_on = 1U;
  hext_dr_printed = 0U;
}

static void Hex_Dr_Update(int32_t dist)
{
  int32_t dd;
  float   psi;

  if (!hext_dr_on) return;

  dd = dist - hext_dr_d0;
  hext_dr_d0 = dist;
  if (dd <= 0) return;
  if (dd > 50) dd = 50;                  /* 掉帧/长间隔时截断，防止一步积飞出界 */

  psi = (gyro_angle - isl_yaw0) * 0.017453292f;   /* 度 -> 弧度 */
  hext_dr_x += (float)dd * cosf(psi);
  hext_dr_y += (float)dd * sinf(psi);
}

static float Hex_Dr_R(void)
{
  return sqrtf(hext_dr_x * hext_dr_x + hext_dr_y * hext_dr_y);
}

/* ==================== HEX8 出岛前的"惯导护段" ====================
 * 【问题：合并处视觉必然失效】
 * 车开到环线与主线的合并点时，两条线并成一团宽白，12 路加权重心失真，
 * 紧接着整排变蓝（n=0）。弯道恢复逻辑判定"弯道丢线"，朝最后见线方向打满，
 * 把车甩出去。16:58 那趟逐帧：
 *   19094  yaw 352.9  n=3 00E0      <- 还正常
 *   19121  e=-3.0  n=6 0FA0         <- 合并成宽白
 *   19152  vL -402 / vR 1709        <- 判定弯道丢线，向左打满
 *          gyro -238.8 -> -369.3    <- 甩了 130°，yaw 从 353 掉回 222
 * 这一圈的闭合点就废了，之后只能在别处"假出岛" -> 现象就是"稳定绕两圈"。
 *
 * 【做法】yaw 进入 [EXIT_GUARD_DEG, EXIT_YAW_HI] 这段（约 25~75mm）不再用
 * PD / GAP，改用陀螺锁住进护段那一刻的航向直行穿过去：
 *   · 直行 76mm 在这个半径(0.42°/mm)上只丢约 8mm 横向，代价可忽略；
 *   · 换到的是"绝不在岔口甩车"，yaw 不会倒退，闭合点不会被浪费；
 *   · 出口是否开火仍由 Island_Step 的窗口判据决定（护段只管方向盘）。
 * 超过 ISL_HEXT_EXIT_YAW_HI 立刻释放给正常循迹（那圈算错过，走 SEX_F/SEXBAD 兜底）。
 *
 * 【为什么不是一直锁到出去】锁太久等于瞎开，而且退出时机本来就该由窗口判据管。
 */
static uint8_t Hex_ExitGuard(float *v, float *turn, int32_t dist)
{
  const IslandCfg *c = Island_Cfg();
  float yaw, terr;

  if (!ISL_HEXT_EXIT_GUARD_EN) {      /* 已停用（见宏说明） */
    hext_guard_on = 0U;
    return 0U;
  }
  if (c->kind != ISLK_HEXT || isl_state != ISL_SESSION || !isl_out_arm) {
    hext_guard_on = 0U;
    return 0U;
  }

  yaw = (float)c->dir * (gyro_angle - isl_yaw0);
  if (yaw < ISL_HEXT_EXIT_GUARD_DEG || yaw > ISL_HEXT_EXIT_YAW_HI
      || (hext_guard_on
          && (dist - hext_guard_d0) >= (int32_t)ISL_HEXT_EXIT_GUARD_MAX_MM)) {
    if (hext_guard_on) {
      printf("HEXT,GUARD_END,%d,%ld\r\n", (int)(yaw * 10.0f),
             (long)(dist - hext_guard_d0));
      hext_guard_on = 0U;
    }
    return 0U;
  }

  if (!hext_guard_on) {
    hext_guard_on  = 1U;
    hext_guard_d0  = dist;
    hext_guard_yaw = gyro_angle;         /* 锁住此刻航向 */
    printf("HEXT,GUARD,%d,%ld\r\n", (int)(yaw * 10.0f), (long)(dist - isl_d_fin));
  }

  /* 航向保持（与 GAP 盲走同一个符号约定） */
  terr = GAP_KP * (gyro_angle - hext_guard_yaw);
  if (terr >  GAP_TURN_MAX) terr =  GAP_TURN_MAX;
  if (terr < -GAP_TURN_MAX) terr = -GAP_TURN_MAX;

  *v    = ISL_CROSS_V;
  *turn = terr;
  return 1U;
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
  /* ---- 精简日志用（11/19）----
   * 原来每 100 ms 无条件打一行 F，一行约 50 字节、在 115200 下阻塞 4.3 ms，
   * 相当于每 10 个控制周期就把一个拉长到 14.3 ms；而且一场 90 s 的日志 800 多行，
   * 真正要看的只有环岛/直角/断路那几段。现在只在这三类"关键部位"打，
   * 其余时间每 2 s 补一行 HB 心跳（能看里程和状态，又不刷屏）。 */
  uint32_t t_hb      = 0U;     /* 上一次心跳打印的时刻 */
  uint32_t t_corner  = 0U;     /* 最近一次"急弯/直角边"触发的时刻 */

  float    err_raw = 0.0f, err_keep = 0.0f, e = 0.0f, last_e = 0.0f;
  float    vL = 0.0f, vR = 0.0f, v_tgt = 0.0f, turn = 0.0f;
  float    err_dir = 0.0f;
  int16_t  uL = 0, uR = 0;
  uint8_t  n = 0U, isl_now = 0U, isl_was = 0U;
  uint8_t  near_wide = 0U;     /* 宽区减速的迟滞状态，见下方速度调度 */
  uint32_t t_wide = 0U;        /* 最后一次 n>=WIDE_NEAR_N 的时刻 */
  uint16_t stall_ms = 0U;
  uint8_t  lap_finish = 0U;     /* 1 = 跑完两圈、靠竖条门停的车（才做盲盒） */
  float    turn_prev = 0.0f;   /* 上一帧转向量，T 口锁定时用 */
  float    gap_yaw = 0.0f;     /* 断线盲走：最后一次看到线时的航向 */
  uint8_t  gap_corner = 0U;    /* 1 = 这次丢线是弯道造成的（不是断路区） */
  uint8_t  gap_ok_n = 0U;      /* 接线确认计数：连续见到线的帧数 */
  float    yaw_ok  = 0.0f;     /* 每一帧见到线就更新，盲走用它当基准 */
  float    yaw_lock = 0.0f;    /* 防掉头基准：只在“见线且没在快速转”时更新 */
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

    /* 串口急停：电脑发 's' 立即停车。和 PB14 一样受 500ms 静默期保护，
     * break 之后走统一收尾（Motor_Set(0,0) / FOLLOW,STOP / 回 main 关负压）。 */
    if ((uint32_t)(now - t0) > 500U && Serial_StopPressed()) {
      printf("SERIAL,STOP,%ld\r\n", (long)(ENC_Dist_mm() - s0));
      break;
    }

    if ((uint32_t)(now - t_ctl) >= CTL_MS) {
      t_ctl = now;

      /* ---------- 1) 读线 ---------- */
      n = Line_Read(&err_raw);

      /* ---------- 1b) 二维惯导积分（只在 HEX8 会话里做）----------
       * 放在读线之后、所有判据之前，保证每个控制帧都积一步。 */
      if (Island_Cfg()->kind == ISLK_HEXT && isl_state == ISL_SESSION) {
        Hex_Dr_Update(ENC_Dist_mm() - s0);
      }

      /* 【每个控制帧都检查】合格分叉 = 两团白 + 跨度>=9 + n=5~9 + 最右四路亮。
       * 67.txt 后改成：记录簇首 + 簇末；入口锚定"簇末 + 40mm"，不再用"簇首+110"。
       * 放在这里而不是 Island_Step 里，是因为 Island_Step 在 t_rcv!=0（丢线）时被冻结，
       * 岔口刚好落在 GAP 里就永远看不到。 */
      if (Island_Cfg()->kind == ISLK_HEXT
          && n >= 5U && n <= 9U
          && (line_raw & 0x000FU) != 0U
          && Line_Lobes(line_raw) >= 2U
          && Line_Span(line_raw) >= 9U) {
        int32_t d_now = ENC_Dist_mm() - s0;

        if (hext_fork_last_d == 0
            || (d_now - hext_fork_last_d) > (int32_t)ISL_HEXT_FORK_HOLD_MM) {
          hext_fork_d      = d_now;        /* 新的一簇：记首帧 */
          hext_fork_last_d = d_now;
          hext_fork_fr     = 1U;           /* 这一簇重新从 1 帧开始数 */
        } else {
          hext_fork_last_d = d_now;        /* 同一簇：簇末跟着刷新 */
          if (hext_fork_fr < 255U) hext_fork_fr++;
        }
        hext_fork_n  = n;
        hext_fork_bm = line_raw;
        printf("HEXT,FORK,%d,%04X,%ld,%d\r\n", (int)n, (unsigned)line_raw,
               (long)(d_now - isl_d_phase_raw), (int)hext_fork_fr);
      }

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

        /* 连续细线帧数（给 HEXT 入口的"先在线上"门用，见 g_clean_frames 注释）：
         *   n=0     丢线     -> 清零
         *   n=1~4   细线     -> 累加
         *   n>=5    宽白     -> 保持不清零（进岛那一帧本身就是宽白，清了就永远进不去） */
        if (n == 0U) {
          g_clean_frames = 0U;
        } else if (n <= 4U) {
          if (g_clean_frames < 250U) g_clean_frames++;
        }
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
         * 只放宽门窗口内且 Z 字满足的第二根/补测；第一根仍保持 11 路：
         * 方框拐角实测 0FF8(9) / 0FFC(10) / 03FF(10) 都只有 9~10 路，
         * 拿 10 路当第一根会把拐角当门；但门窗口 + Z 字环境下，
         * 10 路只可能是第二根漏检后补上来的，允许。 */
        int32_t d_gate = d_now - lap_start_mm;
        if (lap_cnt > 0U && gate_gap_mm > 0) {
          d_gate += gate_gap_mm;
        }
        uint8_t need_n = BAR_FULL_N;
        if (pair_mm != 0
            || (d_gate >= (int32_t)FOLLOW_LAP_MM - (int32_t)BAR_LAP_TOL_MM
                && d_gate <= (int32_t)FOLLOW_LAP_MM + (int32_t)BAR_LAP_TOL_MM
                && gate_z_events >= GATE_Z_MIN)) {
          need_n = BAR_FULL_N2;
        }

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

        /* 【11/27】丢线就打断"连续确认"计数。
         * 旧写法只在 t_rcv==0 时清零，盲走中 n==0 不清，GAP_CONFIRM_N
         * 实际变成了"累计 5 帧"：在缺口里满舵旋转时，阵列扫到来时的线
         * 闪几帧就凑满 -> GAP,DONE 提前触发 -> PD 接回原线 = 原路掉头
         * （119 的 38008 转 147°/144mm、120 的 13344 转 157°/153mm）。 */
        gap_ok_n = 0U;

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
            /* 断路斜穿：几何上要右偏 GAP_TURN_DEG，现场再补 GAP_TURN_BIAS_DEG
             * （撞左肩说明落点偏左，见 GAP_TURN_BIAS_DEG 的说明）。 */
            gap_yaw = gap_corner ? yaw_ok
                                 : (yaw_ok + GAP_TURN_DEG + GAP_TURN_BIAS_DEG);

            printf("GAP,START,%ld,%d,T%d,%d\r\n",
                   (long)gap_d0, (int)gap_corner, (int)(gap_yaw * 10.0f),
                   (int)thin_mm);
            /* 【11/26】真断路（gap_corner==0，前后都是干净细线）一出现，
             * 立刻关掉后面的 FASTSEG 提速：八边形这段要低速过。 */
            if (!gap_corner) fastseg_block = 1U;
            /* 门前的 Z 字路：只数弯道丢线，不计普通直道断路。
             * 73.txt 门（T3/T4）前 T2→T3 之间有 5 次 gap_corner。 */
            if (gap_corner && gate_z_events < 250U) gate_z_events++;
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
        /* 【防掉头基准】只在"看到线、而且车没在快速转"时更新。
         * 直接拿 yaw_ok 当基准不行：恢复段边转边扫到几帧线，
         * yaw_ok 会被一起刷新，累计转角永远清回 0，上限就永远不触发。
         * 用 60 °/s 当门：正常直道/缓弯会更新，直角弯恢复（几百 °/s）不更新。 */
        if (fabsf(gyro_z) < 60.0f) yaw_lock = gyro_angle;
        if (err_raw >  0.3f) err_dir =  1.0f;   /* 记住最后见线方向 */
        if (err_raw < -0.3f) err_dir = -1.0f;
      }

      /* ---------- 3) 误差整形 + 速度调度 ---------- */
      e = Line_Shape(n, err_raw, &err_keep) - FOLLOW_ERR_ZERO
          + Island_ErrOffset(ENC_Dist_mm() - s0);

      /* ---------- 3a) HEXT 入口：线触发接力 + 丢线补偿 ---------- */
      /* 交 PD 后 120mm 内连续 3 帧无 4~6 路单线，说明 90° 的接力点仍然偏外；
       * 再补 25° 盲走，最多补一次，避免反复抽。 */
      if (hext_entry_phase == 3U && isl_state == ISL_SESSION
          && !hext_entry_comp_done) {
        int32_t d_pd = (ENC_Dist_mm() - s0) - hext_entry_pd_d0;

        if (d_pd >= 0 && d_pd <= (int32_t)ISL_HEXT_ENTRY_SETTLE_MM) {
          if (n == 0U) {
            if (hext_entry_lost_cnt < 10U) hext_entry_lost_cnt++;
            if (hext_entry_lost_cnt >= 3U) {
              hext_entry_comp_yaw0 = (float)Island_Cfg()->dir
                                     * (gyro_angle - isl_yaw0);
              hext_entry_comp_target = hext_entry_comp_yaw0
                                       + ISL_HEXT_ENTRY_RECOVER_DEG;
              hext_entry_comp_d0 = ENC_Dist_mm() - s0;
              hext_entry_comp_done = 1U;
              hext_entry_phase = 4U;
              printf("HEXT,RECOVER,%ld,%d\r\n",
                     (long)(ENC_Dist_mm() - s0),
                     (int)(hext_entry_comp_target * 10.0f));
            }
          } else {
            hext_entry_lost_cnt = 0U;
          }
        } else {
          hext_entry_lost_cnt = 0U;
        }
      }

      if (hext_entry_phase == 1U && isl_state == ISL_SESSION) {
        /* HEXT 入口：向右打角 20°（最多 500mm），然后转 phase2 直行。 */
        if (Island_EntryTurn(&v_tgt, &turn, ENC_Dist_mm() - s0)) {
          /* 打角中：v/turn 已由 Island_EntryTurn 填好 */
        } else {
          hext_entry_straight_d0 = ENC_Dist_mm() - s0;
          hext_entry_lost_cnt = 0U;
          hext_entry_comp_done = 1U;
          hext_entry_phase = 2U;
          v_tgt = ISL_HEXT_ENTRY_STRAIGHT_V;
          turn  = 0.0f;
          printf("HEXT,STRAIGHT_START,%ld,%d\r\n",
                 (long)hext_entry_straight_d0,
                 (int)((float)Island_Cfg()->dir
                       * (gyro_angle - isl_yaw0) * 10.0f));
        }
      } else if (hext_entry_phase == 2U && isl_state == ISL_SESSION) {
        /* 15° 打角后直行 ISL_HEXT_ENTRY_STRAIGHT_MM，再交岛内 PD。
         * 【不能写 turn=0】turn=600 打角结束时残转几百 °/s，
         * 只写 0 会让车带着 20~26° 余转冲进环（内切），
         * 所以这里做"航向 P + 角速度阻尼"：见 HEXT_ENTRY_KD 的实测数据。 */
        int32_t d_str = ENC_Dist_mm() - s0 - hext_entry_straight_d0;

        if (d_str >= (int32_t)ISL_HEXT_ENTRY_STRAIGHT_MM
            && n == 0U
            && d_str < (int32_t)(ISL_HEXT_ENTRY_STRAIGHT_MM
                                 + ISL_HEXT_PD_WAIT_MM)) {
          /* 【11/27】交 PD 前先等线：走满直行段但线已经丢了（n==0）时，
           * 保持当前入环航向直行，最多等 ISL_HEXT_PD_WAIT_MM(160mm)，
           * 不把控制权交给还拿着主线旧误差的 PD。
           * 126/127 第一次进岛都是 PD 后 44mm 丢线，59.8° 被旧误差拽回
           * 26.5°/12.3° -> ISL,FABORT,~240；线在 90~120mm 内就回来了，
           * 等它回来再交 PD，入环角就能保住。 */
          v_tgt = ISL_HEXT_ENTRY_STRAIGHT_V;
          turn  = 0.0f;
        } else {
          {
            float yaw_now = (float)Island_Cfg()->dir * (gyro_angle - isl_yaw0);
            float w_now   = (float)Island_Cfg()->dir * gyro_z_f;   /* °/s，绕行方向为正 */
            float terr    = GAP_KP * (yaw_now - ISL_HEXT_ENTRY_TURN_DEG)
                          + HEXT_ENTRY_KD * w_now;

            if (terr >  GAP_TURN_MAX) terr =  GAP_TURN_MAX;
            if (terr < -GAP_TURN_MAX) terr = -GAP_TURN_MAX;

            v_tgt = ISL_HEXT_ENTRY_STRAIGHT_V;
            turn  = terr;
          }

          if (d_str >= (int32_t)ISL_HEXT_ENTRY_STRAIGHT_MM) {
            hext_entry_pd_d0 = ENC_Dist_mm() - s0;
            hext_entry_phase = 3U;
            printf("HEXT,PD,%ld,%d\r\n", (long)hext_entry_pd_d0,
                   (int)((float)Island_Cfg()->dir
                         * (gyro_angle - isl_yaw0) * 10.0f));
          }
        }
      } else if (Island_ExitTurn(&v_tgt, &turn, ENC_Dist_mm() - s0, n,
                                 err_raw)) {
        /* 出岛：先小角打角，再强制直行；不走 PD/找线 */
      } else if (Hex_ExitGuard(&v_tgt, &turn, ENC_Dist_mm() - s0)) {
        /* 出岛前的惯导护段：锁航向穿过合并区，杜绝岔口甩车（见函数说明） */
      } else if (t_rcv != 0U && gap_corner) {
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
          /* 【11/25】第二格之后到第 8 格入口这一段整体提速，见 FASTSEG_V_GAIN。
           * 只放大巡航目标与宽区减速，锁直行/急弯/盲走速度不动。 */
          float v_fast = FOLLOW_V_FAST;
          float v_slow = FOLLOW_V_SLOW;
          float v_near = ISL_NEAR_V;

          if (Island_FastSeg()) {
            v_fast *= FASTSEG_V_GAIN;
            v_slow *= FASTSEG_V_GAIN;
            v_near *= FASTSEG_V_NEAR_GAIN;
          }

          v_tgt = v_fast - (v_fast - v_slow) * (ae / 7.0f);

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

          if (near_wide && v_tgt > v_near) v_tgt = v_near;

          /* 急弯预判：偏差"又大又变快"说明曲率很大。
           * 光电管看不到前方，但 |e| 的增长速率本身就是提前量：
           * 直角弯进弯前 100~200 mm 内 |e| 会从 1 快速涨到 5 以上。
           * 这时立刻压到 CORNER_V，比"等 |e| 到 7 才降速"早得多 ——
           * 之前直角弯冲出去，就是因为降速发生时车已经到弯上了。 */
          {
            float de_abs = fabsf(e - last_e);
            if (fabsf(e) > CORNER_E && de_abs > CORNER_DE && v_tgt > CORNER_V) {
              v_tgt = CORNER_V;
            }
          }
        }

        /* 【11/21 12:25】进岛减速已删除：改用"两团白"判据后不再需要
         * （限速 400 被反馈"触发太早"，且真分叉在 800 速下本来就采集得到）。 */

        turn = FOLLOW_TURN_SIGN * Steer_PD(e, e - last_e)
             + FOLLOW_TURN_BIAS * (v_tgt / FOLLOW_V_FAST)
             + Island_Turn();      /* 大套小交叉点的强制转向偏置，其余恒 0 */

        if (turn >  FOLLOW_TURN_MAX) turn =  FOLLOW_TURN_MAX;
        if (turn < -FOLLOW_TURN_MAX) turn = -FOLLOW_TURN_MAX;

      }
      /* ---------- 3b) 恢复段转角上限（防掉头）----------
       * 直角弯只要转约 90°。从最后一次“稳稳看到线”的航向（yaw_lock）算起，
       * 如果车已经转过 RECOVER_SWING_MAX_DEG 还没接回线，说明转过头了 ——
       * 再顺着同方向转下去，下一个扫到的必然是"来时的线"（114.txt 34.6k
       * 那次 430 mm 转 190°、接回来时线，就是这么掉头的）。
       * 在最后 RECOVER_SWING_FADE_DEG 度内把 turn 线性收到 0：车停止打角、
       * 直行等线（等不到就按原有 LOST 超时停车），物理上转不到 180°。
       * 环岛会话（含大套小）有自己的绕行方向，不参与这个上限。 */
      if ((n == 0U || t_rcv != 0U) && !Island_InSession()) {
        float swung = fabsf(gyro_angle - yaw_lock);
        float fade0 = RECOVER_SWING_MAX_DEG - RECOVER_SWING_FADE_DEG;

        if (swung > fade0) {
          float k = (RECOVER_SWING_MAX_DEG - swung) / RECOVER_SWING_FADE_DEG;
          if (k < 0.0f) k = 0.0f;
          turn *= k;
        }
      }

      /* 急弯/直角边：判据和速度调度里的"急弯预判"完全一致（|e| 大且变化快）。
       * 这里只用来决定"这一段要不要打日志"——直角弯是光电管最容易丢线的地方，
       * 触发后留 600 ms 打印窗口，把进弯、打角、出弯连起来看。 */
      if (fabsf(e) > CORNER_E && fabsf(e - last_e) > CORNER_DE) t_corner = now;

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

      /* ---------- 4b) 发车前置（正向偏置）：线性衰减到 0 ----------
       * 起步瞬间满额，START_PRELOAD_MS 之后归零；衰减是线性 1 - t/T，
       * 这样和 4) 的升速互补：低速段偏置大（破静摩擦），越接近巡航越小。 */
      if (!isl_now) {
        uint32_t tg = (uint32_t)(now - t0);

        if (tg < START_PRELOAD_MS) {
          int32_t ff = (int32_t)(START_PRELOAD_PWM *
                                 (1.0f - (float)tg / (float)START_PRELOAD_MS));

          uL = (int16_t)((int32_t)uL + ff);
          uR = (int16_t)((int32_t)uR + ff);
        }
      }

      Motor_Set(uL, uR);

      /* ---------- 6) 堵转保护 ---------- */
      /* 必须取绝对值：内侧轮允许反转（FOLLOW_V_MIN = -575），
       * 旧写法 vL<20 && vR<20 会把反转的 -120 也算成没转，
       * 于是转弯 / 进岛 / 发车打方向时会随机误报 STALL。 */
      if (fabsf(vL) < 20.0f && fabsf(vR) < 20.0f && v_tgt > 100.0f) stall_ms += 5U;
      else                                           stall_ms  = 0U;
      if (stall_ms > STALL_MS) { printf("STALL\r\n"); break; }

      /* ---------- 7) 停车判定 ----------
       * 优先用竖条门：GATE,STOP 设了 stop_mm（两根竖条中点），
       * 走到那个里程就停 —— 光电管可以已离开发车区，车主体在两根竖条之间。
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
          lap_finish = 1U;
          printf("LAPS,DONE,BAR,%ld\r\n", (long)d_now);
          break;
        }
        /* 冲过停车点 300 mm 还没停下来：强制停，别压过发车区 */
        if (d_now >= stop_mm + (int32_t)300) {
          lap_finish = 1U;
          printf("LAPS,OVER,%ld,%ld\r\n", (long)d_now, (long)stop_mm);
          break;
        }
      } else if ((ENC_Dist_mm() - s0) >= goal_mm + (int32_t)MILEAGE_STOP_MARGIN_MM
                 && !Island_Busy()) {
        printf("LAPS,DONE\r\n"); break;
      }
    }

    /* ---------- 打印：只打"关键部位"（11/19 精简） ----------
     * 判据全部来自现成的状态量，不额外增加判断负担：
     *   环岛     isl_state != ISL_NONE（CROSS/SESSION/HIN/HEXOUT）或大套小 bs.state != BS_IDLE
     *   T 口     isl_now（本帧正在锁直行穿 T 口）
     *   断路     t_rcv != 0（丢线盲走中），出岛后 ISL_OUT_MS 内也算
     *   直角边   t_corner 600 ms 内（|e| 大且变化快）
     * 关键区内仍按 100 ms 一行，关键区外每 1000 ms 一行 HB 心跳。
     * 【11/19 提速 ×3 同步收紧：关键区 100 -> 50 ms、心跳 2000 -> 700 ms】
     * 【11/26 又放回 100 ms / 1000 ms】115.txt 八边形段：1260 mm/s 下
     * 50 ms 一行的 4.3 ms 阻塞 = 5.4 mm，加上 5 ms 一帧的 6.3 mm，
     * 两次采样最坏隔 11.7 mm，比 T 口"整排白"的 7~8 mm 窗口还宽 ——
     * 判据整帧被跨过，连续丢线。放回 100 ms 后最坏采样间隔约 8.6 mm，
     * 同时这一段已关掉 FASTSEG（1260 -> 1125 mm/s，每帧 5.6 mm）。
     * 日志分辨率按"走多远"看：1125 mm/s 下 100 ms = 112 mm，够看环岛动作。 */
    {
      uint8_t key = 0U;

      if (isl_state != ISL_NONE || bs.state != BS_IDLE) key = 1U;
      if (isl_now) key = 1U;
      if (t_rcv != 0U) key = 1U;
      if (t_grace != 0U && (uint32_t)(now - t_grace) < ISL_OUT_MS) key = 1U;
      if (t_corner != 0U && (uint32_t)(now - t_corner) < 600U) key = 1U;

      if (key && (uint32_t)(now - t_prt) >= 100U) {
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
      } else if (!key && (uint32_t)(now - t_hb) >= 1000U) {
        /* 心跳：不在关键区，1000 ms 补一行，确认"车还在跑、跑到哪、线还在不在" */
        t_hb = now;
        printf("HB,%ld,%d,%d,%d,%04X,%d\r\n",
               (long)(ENC_Dist_mm() - s0), (int)(v_tgt * 10.0f),
               (int)(e * 10.0f), (int)n, (unsigned)line_raw,
               (int)(gyro_angle * 10.0f));
      }
    }
  }

  Motor_Set(0, 0);

  /* ==================== 停车后的“软件盲盒” ====================
   * 只有“跑完两圈、靠竖条门停的车”才做：
   * 中途 ABORT / TIMEOUT / 里程兜底停车的位置都不在发车区，
   * 在那里硬做盲盒会把车开到场地外面去。 */
  if (lap_finish) {
    BlindBox_Run(ENC_Dist_mm() - s0);
  }

  printf("FOLLOW,STOP,%ld\r\n", (long)(ENC_Dist_mm() - s0));
}


/* ==================================================================
 *              停车后的“软件盲盒”（2025-11 新规 · PD 版）
 * ==================================================================
 * 规则：
 *   ① 跑完两圈、停车之前必须压过计时器，否则计时不停、成绩无效；
 *   ② 停稳后先原地自旋 180°，再沿白线走到 T 口停下 ——
 *      最终停下时光电管正好压在发车区边沿的那根白线上。
 *
 * 做法（相位挂在发车区状态机上：GATE_DONE -> GATE_BB_SPIN -> GATE_BB_STRAIGHT -> GATE_BB_DONE）：
 *   1) 停车后等 BB_SETTLE_MS 完全静止（期间一个电机输出都不给）；
 *   2) 关负压：吸着地原地转不动，关掉以后车轻、好摆；
 *   3) [GATE_BB_SPIN] 自旋 180°，**只用陀螺仪**：yaw 闭环，粗调 120 ms 长脉冲、
 *      离目标 30° 内改 20 ms 点动，死区 ±5°。这一段完全不看光电管 ——
 *      原地转的时候阵列会扫过各种白线，看了反而会误判；
 *   4) [GATE_BB_STRAIGHT] 转完**锁住航向直行**（航向 P 环 + 每轮速度 PI，
 *      目标速度 BB_FOLLOW_V = 200 mm/s）。不用光电 PD 找线：自旋完车不一定
 *      在线上，PD 拿旧误差会乱打（114.txt 的掉头就是这么来的）；
 *   5) 光电只当终点开关：看到 T 口（整排白 n >= BB_T_N，连续 BB_T_CONFIRM_N 帧）
 *      就断输出自然滑停，状态机进 GATE_BB_DONE；
 *   6) 收工后锁住 Align_Hold，直到下次按发车键。
 *
 * 串口日志（BB, 前缀，和循迹日志分得开）：
 *   BB,START,<停车里程>,<n>,<位图>
 *   BB,SPIN,<目标×10>,<起始 yaw×10>
 *   BB,SPIND,<结束 yaw×10>,<实际转过×10>
 *   BB,STRAIGHT,<锁定的航向×10>
 *   BB,S,<n>,<位图>,<航向偏差×10>,<turn×10>,<已走 mm>   每 50 ms 一行
 *   BB,T,<n>,<位图>,<mm>                  看到 T 口的那一刻
 *   BB,END,<n>,<位图>,<mm>                刹停后的位置
 *   BB,DONE / BB,FAIL / BB,ABORT
 * ================================================================== */
#define BB_ENABLE            1      /* 0 = 整段跳过（只停车，不做盲盒） */
#define BB_BRAKE_KP        0.9f  /* 【11/27 10:45】刹车的 PWM/(mm/s)：反接幅度跟着当前轮速走。
                                  * 【问题】原来完成两圈后只有 Motor_Set(0,0) 断电滑行，
                                  * 现场 1.1 m/s 的车要滑 400~500mm，正好滑出发车区。
                                  * 规则要求"在发车区完全停止后才自旋"，所以必须先刹停。
                                  * 比例反接的好处：速度降下来，反接幅度也自动降下来，
                                  * 一旦 |v| 小于 BB_BRAKE_V_MIN 就完全断电，不会像固定
                                  * -1800 PWM 那样把车往后推半米（116.txt 那个坑）。 */
#define BB_BRAKE_PWM_MAX 1200    /* 反接 PWM 上限，别让轮胎抱死打滑 */
#define BB_BRAKE_V_MIN     40.0f /* 低于这个轮速(mm/s)就算停住了 */
#define BB_BRAKE_MS       400U   /* 刹车最长时间（兜底） */
#define BB_EDGE_N          10U   /* 【11/27 11:00：2 -> 10，回到"整排白"判据】
                                  * 上一次我把它改成 2（压到任意一条线就算完成），
                                  * 结果这一步被跳过：10:55:46 那趟
                                  *   BB,SPIND,-10683,1796  自旋完成
                                  *   BB,DONE,0,3,0038,0    末位 0 = "已在线上，不再移动"
                                  * 而规则要求"光电管恰好压在发车区边沿白线上"，
                                  * 现场也确认应该继续直走到光电管全白。
                                  * 改回 10（与 BB_T_N 同值）：自旋后直行，
                                  * 整排白（n>=10）连续 2 帧才停。 */   /* 【11/27 10:45】"光电管压在边沿白线上"的判据：
                                  * 看到 n>=2 路白就认为已经压住线。
                                  * 原来是 BB_T_N(10)=整排白，结果自旋后车明明已经
                                  * 压在细线上（BB,S 一直是 n=3~4），还硬要直行
                                  * 107mm 去找整排白，正好把车推出发车区。 */
#define BB_EDGE_MAX_MM    200    /* 找边沿白线最多走这么远，防止再冲出去 */
#define BB_SETTLE_MS       500U     /* 停车后等真正静止（这期间一个电机输出都不给） */
#define BB_SPIN_DEG      180.0f     /* 盲盒要求的自旋角度 */
#define BB_SPIN_TOL        5.0f     /* 自旋死区（°）：进这个范围就收手 */
#define BB_SPIN_PWM        600      /* 原地旋转 PWM（左右反向）。【11/27】800 -> 400 -> 600：
                                      * 800 时 120 那趟自旋到 193.2°（过冲 13.2°，直行段 yaw_t
                                      * 被带偏、斜顶发车区竖条原地摇摆）；400 时稳定在 177~179°，
                                      * 但自旋偏慢。600 取中间值：粗调段角速度约为 800 的 3/4，
                                      * 比 400 快 1.5 倍。目标仍然是 BB,SPIND 贴住 ±180°；
                                      * 若过冲回到 >5°，退到 500~550；若还嫌慢再上 650。
                                      * 精调段 BB_SPIN_FINE_MIN=220 是静摩擦下限，不跟着改。 */
#define BB_SPIN_FINE_TOL   3.0f     /* 【11/27 11:20】精调到位判定（°） */
#define BB_SPIN_KP        25.0f     /* 精调 P 增益：PWM per 度 */
#define BB_SPIN_FINE_MAX   250      /* 精调 PWM 上限（压住转速，防一次冲过） */
#define BB_SPIN_FINE_MIN   220      /* 精调 PWM 下限（低于它轮子因静摩擦不动） */
#define BB_SPIN_KD        1.0f     /* 【11/27 12:00：2.5 -> 1.0 + 整体调小】
                                      * 上一版（KD=2.5, KP=60, MAX=500, MIN=300, TOL=1.5）
                                      * 实测自旋 14.1 秒（角度 179.9° 很准但太慢）：
                                      * 阻尼把精调推力抵掉，车只在一两度里"点"，永远
                                      * 满足不了"±1.5° 稳定 20ms"，每轮精调都跑满 1.2 秒上限。
                                      * 这一版整体调小：
                                      *   KP 60->25、MAX 500->250、MIN 300->220
                                      *   KD 2.5->1.0、TOL 1.5->3.0、FINE_MS 1200->600
                                      *   粗调阈值 18->25（粗调更早收手，剩 25° 交给温和精调）
                                      * 3° 的到位精度对后面"直行到全白"完全够（57mm 内就扫到了）。 */      /* 【11/27 11:50】精调角速度阻尼（PWM per °/s）。
                                      * 【问题】纯 P + 下限 300 会在 180° 附近成极限环：
                                      * err=2° 时 60×2=120 被抬到 300，300 PWM 能转 ~40°/s，
                                      * 惯性冲过目标 -> err 变号 -> 反向 300 -> 又冲回来。
                                      * 实测 11:44 那趟自旋 9.6 秒（8 轮精调循环）才收手。
                                      * 加阻尼：PWM = KP*err − KD*ω，快接近时提前刹车。 */
#define BB_SPIN_FINE_MS   600U     /* 精调阶段最长时间 */
#define BB_SPIN_COARSE_DEG 25.0f    /* 【11/27 11:20：30 -> 18】离目标这么多度以上：长脉冲快转。
                                     * 11:10 那趟自旋到 198.5°（超 18.5°）、耗时 12.4s：
                                     * 粗调 PWM 800/120ms 一次能走 20~30°，而阈值是 30°，
                                     * 所以"最后一个粗调脉冲"注定过冲，之后精调 20ms 脉冲
                                     * 每次只能点 15° 量级，永远在 180 两侧摆。
                                     * 阈值收到 18° + 精调改成连续 P 伺服（每 1ms 纠偏）。 */
#define BB_SPIN_PULSE_MS   120U     /* 粗调：每次给电多久 */
#define BB_SPIN_COAST_MS   180U     /* 粗调：断输出后等它滑完再测 */
#define BB_SPIN_FINE_PULSE_MS 20U   /* 精调：一次只点一下，别在 180° 两侧摆 */
#define BB_SPIN_FINE_COAST_MS 120U  /* 精调：滑行等待短一点，采样密一点 */
#define BB_SPIN_TMO_MS   15000U     /* 自旋兜底超时 */
#define BB_FOLLOW_V       170.0f   /* 【11/27 11:20：200 -> 170】慢一点，航向残差大时不容易摆 */    /* 盲盒直行目标速度 mm/s（慢，滑停距离短） */
#define BB_PD_RAMP_MS      300U     /* 直行起步升速时间 */
#define BB_PD_V_MIN       120.0f    /* 升速阶段的低速下限（再低推不动） */
#define BB_T_N             10U      /* n >= 10 = T 口整排白（和门判据一致） */
#define BB_T_CONFIRM_N      2U      /* 连续这么多帧才认 T 口 */
#define BB_PD_MAX_MM       700      /* 直行最多走这么远（正常 ~200 mm） */
#define BB_PD_TMO_MS      5000U     /* 直行兜底超时 */

/* 中途打断：PB14 或串口 's'（和循迹时的急停同一套） */
static uint8_t BB_Abort(void)
{
  return (uint8_t)(Key_Pressed() || Serial_StopPressed());
}

/* 等待期间也一直采陀螺：自旋靠 yaw 闭环，不能断积分 */
static void BB_Wait(uint16_t ms)
{
  uint32_t t0 = HAL_GetTick();

  while ((uint32_t)(HAL_GetTick() - t0) < ms) {
    Gyro_Poll_1ms(HAL_GetTick());
  }
}

/* 按轮速比例反接刹车，刹到停住就断电。
 * 和"固定 -1800 PWM 反接 250ms"的区别：反接幅度 = KP × 当前轮速，
 * 速度降下来幅度也降下来，|v| < BB_BRAKE_V_MIN 立即断输出，
 * 不会出现"刹完往后倒半米"（116.txt 那个坑）。 */
static void BB_BrakeStop(void)
{
  uint32_t t0 = HAL_GetTick();

  while ((uint32_t)(HAL_GetTick() - t0) < BB_BRAKE_MS) {
    float   wL = 0.0f, wR = 0.0f;
    float   pL, pR;

    Speed_Update(&wL, &wR);
    if (fabsf(wL) < BB_BRAKE_V_MIN && fabsf(wR) < BB_BRAKE_V_MIN) break;

    pL = -wL * BB_BRAKE_KP;
    pR = -wR * BB_BRAKE_KP;
    if (pL >  BB_BRAKE_PWM_MAX) pL =  BB_BRAKE_PWM_MAX;
    if (pL < -BB_BRAKE_PWM_MAX) pL = -BB_BRAKE_PWM_MAX;
    if (pR >  BB_BRAKE_PWM_MAX) pR =  BB_BRAKE_PWM_MAX;
    if (pR < -BB_BRAKE_PWM_MAX) pR = -BB_BRAKE_PWM_MAX;

    Motor_Set((int16_t)pL, (int16_t)pR);
    BB_Wait(10U);
  }
  Motor_Set(0, 0);
}

/* 【11/26 删除反接刹车】原来是“给反向 PWM 直到 5 ms 没再走或 250 ms”，
 * 116.txt 两次都出问题：到 T 口后 BB,END 的里程是 -485 / -515 mm，
 * 也就是车被反向 PWM 从 T 口往后推了半米（250 ms 的 -1800 PWM 足够倒冲 2 m/s）。
 * 现在到 T 口只断输出，让车自己滑停。 */

/* 自旋 180°：只用陀螺仪闭环，完全不看光电管。
 * 返回 1 = 转完（角度以 BB,SPIND 为准）；2 = 被按键/串口打断。 */
static uint8_t BB_Spin180(void)
{
  float    yaw0   = gyro_angle;
  float    target = yaw0 + BB_SPIN_DEG;
  uint32_t t0 = HAL_GetTick();
  int      i;

  printf("BB,SPIN,%d,%d\r\n", (int)(target * 10.0f), (int)(yaw0 * 10.0f));

  for (i = 0; i < 160; i++) {
    float    err;
    int8_t   d;

    Gyro_Poll_1ms(HAL_GetTick());
    err = target - gyro_angle;
    if (fabsf(err) <= BB_SPIN_FINE_TOL) break;

    if (fabsf(err) > BB_SPIN_COARSE_DEG) {
      /* ---- 粗调：长脉冲快转（离目标 >18°，不怕冲） ---- */
      d = (err > 0.0f) ? 1 : -1;
      Motor_Set((int16_t)(d * BB_SPIN_PWM), (int16_t)(-d * BB_SPIN_PWM));
      {
        uint32_t tp = HAL_GetTick();
        while ((uint32_t)(HAL_GetTick() - tp) < BB_SPIN_PULSE_MS) {
          Gyro_Poll_1ms(HAL_GetTick());
        }
      }
      Motor_Set(0, 0);
      {
        uint32_t tp = HAL_GetTick();
        while ((uint32_t)(HAL_GetTick() - tp) < BB_SPIN_COAST_MS) {
          Gyro_Poll_1ms(HAL_GetTick());
        }
      }
    } else {
      /* ---- 精调：连续 P 伺服（每 1ms 纠偏），不再用"脉冲+滑行" ----
       * 脉冲方式一次甩 15~20°，容差只有 ±5°，车就在目标两侧来回点
       * （实测 4.0s / 10.7s / 12.4s，且过冲 18°）。 */
      uint32_t tq    = HAL_GetTick();
      uint16_t ok_ms = 0U;

      Motor_Set(0, 0);
      while ((uint32_t)(HAL_GetTick() - tq) < BB_SPIN_FINE_MS) {
        int16_t p;
        float   e2;

        Gyro_Poll_1ms(HAL_GetTick());
        e2 = target - gyro_angle;

        if (fabsf(e2) <= BB_SPIN_FINE_TOL) {
          ok_ms++;
          Motor_Set(0, 0);
          if (ok_ms >= 20U) break;          /* 稳定 20ms 才算到位 */
          continue;
        }
        ok_ms = 0U;

        /* PD：err 提供推力，ω 提供提前刹车（d*gyro_z_f = 往目标转的角速度） */
        {
          int8_t dd  = (e2 > 0.0f) ? 1 : -1;
          float  cmd = BB_SPIN_KP * e2 - BB_SPIN_KD * ((float)dd * gyro_z_f);

          p = (int16_t)cmd;
          if (p >  BB_SPIN_FINE_MAX) p =  BB_SPIN_FINE_MAX;
          if (p < -BB_SPIN_FINE_MAX) p = -BB_SPIN_FINE_MAX;

          /* 只有"还在往目标方向推"时才给下限（克服静摩擦）；
           * 阻尼已经反向刹车时（p 与 err 异号）不允许被下限抬回去。 */
          if ((float)p * e2 > 0.0f) {
            if (p >= 0 && p <  (int16_t)BB_SPIN_FINE_MIN) p =  (int16_t)BB_SPIN_FINE_MIN;
            if (p <  0 && p > -(int16_t)BB_SPIN_FINE_MIN) p = -(int16_t)BB_SPIN_FINE_MIN;
          }
        }
        Motor_Set(p, (int16_t)(-p));
      }
      Motor_Set(0, 0);
      BB_Wait(100U);                        /* 让残余角速度消掉 */
    }

    if ((uint32_t)(HAL_GetTick() - t0) >= BB_SPIN_TMO_MS) break;
    if (BB_Abort()) { Motor_Set(0, 0); return 2U; }
  }

  Motor_Set(0, 0);
  BB_Wait(200U);
  printf("BB,SPIND,%d,%d\r\n",
         (int)(gyro_angle * 10.0f), (int)((gyro_angle - yaw0) * 10.0f));

  return 1U;
}

/* ==================== 盲盒第 2 步：直行到 T 口 ====================
 * 【11/26 改成“直行”】上一版是“交给 PD 沿白线走”，114.txt 实测：
 * 自旋结束后光电读到的 n=0（不在线上），PD 拿不到线就一直按旧误差走，
 * 而且这段本来就在发车区边沿，线可能偏、可能被竖条打断，PD 很容易乱打。
 * 现在只做两件事：
 *   ① 陀螺仪锁住自旋结束时的航向，直行（航向 P 环 + 每轮速度 PI）；
 *   ② 光电只当“终点开关”：整排白 n>=BB_T_N 连续 2 帧 = 到 T 口，断输出滑停。
 * 这样即使一开始不在线上，只要航向没错，几厘米后也一定会扫到竖条。 */
#define BB_STRAIGHT_KP         6.0f   /* 航向保持：差 1° 给 6 mm/s 差速 */
#define BB_STRAIGHT_TURN_MAX 300.0f   /* 航向差速限幅 */

/* 直行到 T 口。返回 1 = 已到 T 口并刹停；0 = 超时/走满；2 = 被按键/串口打断。 */
static uint8_t BB_StraightToT(uint8_t *n_out, uint16_t *raw_out, int32_t *mm_out)
{
  int32_t  s0 = ENC_Dist_mm();
  uint32_t t0 = HAL_GetTick();
  uint32_t t_ctl = t0, t_prt = t0;
  float    yaw_t = gyro_angle;          /* 自旋结束时的航向：就按这个方向直行 */
  float    vL = 0.0f, vR = 0.0f;
  float    err = 0.0f;
  uint8_t  n = 0U, t_cnt = 0U, ret = 0U;

  Speed_PID_Init();
  printf("BB,STRAIGHT,%d\r\n", (int)(yaw_t * 10.0f));

  for (;;) {
    uint32_t now = HAL_GetTick();

    Gyro_Poll_1ms(now);

    if (BB_Abort()) { ret = 2U; break; }
    if ((uint32_t)(now - t0) >= BB_PD_TMO_MS) break;
    if ((ENC_Dist_mm() - s0) >= (int32_t)BB_PD_MAX_MM) break;

    if ((uint32_t)(now - t_ctl) >= CTL_MS) {
      float   v_tgt, turn, tL, tR;
      int16_t uL, uR;

      t_ctl = now;
      n = Line_Read(&err);          /* 只用来认 T 口，不参与转向 */

      /* ---- T 口：整排白连续 BB_T_CONFIRM_N 帧 ---- */
      if (n >= BB_T_N) {
        if (t_cnt < 255U) t_cnt++;
        if (t_cnt >= BB_T_CONFIRM_N) {
          ret = 1U;
          printf("BB,T,%d,%04X,%ld\r\n",
                 (int)n, (unsigned)line_raw, (long)(ENC_Dist_mm() - s0));
          break;
        }
      } else {
        t_cnt = 0U;
      }

      /* ---- 速度：慢速起步，之后恒速 ---- */
      v_tgt = BB_FOLLOW_V;
      {
        uint32_t tg = (uint32_t)(now - t0);
        if (tg < BB_PD_RAMP_MS) v_tgt *= (float)tg / (float)BB_PD_RAMP_MS;
        if (v_tgt < BB_PD_V_MIN) v_tgt = BB_PD_V_MIN;
      }

      /* ---- 航向保持：偏了多少度，就给多少反向差速 ---- */
      turn = -BB_STRAIGHT_KP * (gyro_angle - yaw_t);
      if (turn >  BB_STRAIGHT_TURN_MAX) turn =  BB_STRAIGHT_TURN_MAX;
      if (turn < -BB_STRAIGHT_TURN_MAX) turn = -BB_STRAIGHT_TURN_MAX;

      /* 【11/27 11:20】差速不允许超过 v_tgt 的一半：
       * 否则内轮会反转，车在原地来回摆、里程数变负（11:10 那趟
       * BB,S 的 mm 从 469 一路掉到 -995，最后 BB,FAIL）。 */
      if (turn >  v_tgt * 0.5f) turn =  v_tgt * 0.5f;
      if (turn < -v_tgt * 0.5f) turn = -v_tgt * 0.5f;

      tL = v_tgt - turn;
      tR = v_tgt + turn;
      if (tL < FOLLOW_V_MIN) tL = FOLLOW_V_MIN;
      if (tR < FOLLOW_V_MIN) tR = FOLLOW_V_MIN;

      Speed_Update(&vL, &vR);
      uL = (int16_t)Inc_PID(&pidL, tL, vL);
      uR = (int16_t)Inc_PID(&pidR, tR, vR);
      Motor_Set(uL, uR);

      if ((uint32_t)(now - t_prt) >= 50U) {
        t_prt = now;
        printf("BB,S,%d,%04X,%d,%d,%ld\r\n",
               (int)n, (unsigned)line_raw,
               (int)((gyro_angle - yaw_t) * 10.0f), (int)(turn * 10.0f),
               (long)(ENC_Dist_mm() - s0));
      }
    }
  }

  /* 【11/26 116.txt】到 T 口只断输出，自然滑停：
   * 原来这里是 BB_Brake(1)（反接 -1800 PWM），实测把车从 T 口往后推了
   * 485 / 515 mm，看起来就是“停车后往后冲”。 */
  Motor_Set(0, 0);
  BB_Wait(300U);               /* 等它滑停再读位置，不再给任何反向输出 */

  *n_out   = Line_Read(&err);
  *raw_out = line_raw;
  *mm_out  = ENC_Dist_mm() - s0;
  printf("BB,END,%d,%04X,%ld\r\n",
         (int)*n_out, (unsigned)*raw_out, (long)*mm_out);

  return ret;
}

/* 停车后的完整盲盒（发车区状态机的后半段）：
 * 停稳 → 关负压 → GATE_BB_SPIN 自旋 180° → GATE_BB_STRAIGHT 锁航向直行到 T 口 → 刹停 */
static void BlindBox_Run(int32_t d_stop)
{
#if BB_ENABLE
  uint8_t  n = 0U, ret;
  uint16_t raw = 0U;
  int32_t  mm = 0;
  float    e = 0.0f;

  bb_hold = 1U;                       /* 收工后别再让 Align_Hold 把车拱走 */
  Motor_Set(0, 0);

  n = Line_Read(&e);
  printf("BB,START,%ld,%d,%04X\r\n", (long)d_stop, (int)n, (unsigned)line_raw);

  /* ---- 0) 先刹停 ----
   * 完成两圈那一刻车速还有 1 m/s 左右，原来这里只断电滑行，
   * 800ms 的等待里会滑出 400~500mm —— 直接滑出发车区。
   * 规则要求"在发车区完全停止后才自旋"，所以先按轮速比例刹到停。 */
  BB_BrakeStop();
  printf("BB,BRAKED,%ld\r\n", (long)(ENC_Dist_mm() - d_stop));

  /* ---- 1) 原地等 500 ms 完全静止（这期间一个电机输出都不给） ---- */
  BB_Wait(BB_SETTLE_MS);
  if (BB_Abort()) { printf("BB,ABORT,SETTLE\r\n"); return; }

  /* ---- 2) 关负压：吸着地原地转不动，关掉以后车轻、好摆 ---- */
  Fan_Stop();
  Motor_Set(0, 0);
  BB_Wait(300U);

  /* ---- 3) 发车区状态机：GATE_DONE（已停车）→ GATE_BB_SPIN ---- */
  gate_state = GATE_BB_SPIN;
  ret = BB_Spin180();
  if (ret == 2U) { printf("BB,ABORT,SPIN\r\n"); return; }

  /* ---- 4) 状态机：GATE_BB_STRAIGHT —— 锁航向直行到 T 口 ----
   * 不再用光电 PD 找线：自旋完车不一定在线上，PD 拿旧误差会乱打；
   * 这里只用陀螺仪保航向，光电只负责“看到整排白就停”。 */
  n = Line_Read(&e);
  if (n >= BB_EDGE_N) {               /* 自旋后已经压在边沿白线上：不用再走 */
    gate_state = GATE_BB_DONE;
    printf("BB,DONE,0,%d,%04X,0,%d\r\n",
           (int)n, (unsigned)line_raw, (int)(gyro_angle * 10.0f));
    return;
  }

  gate_state = GATE_BB_STRAIGHT;
  ret = BB_StraightToT(&n, &raw, &mm);
  gate_state = GATE_BB_DONE;
  if (ret == 2U) { printf("BB,ABORT,STRAIGHT\r\n"); Motor_Set(0, 0); return; }
  if (ret == 0U) {
    printf("BB,FAIL,%d,%04X,%ld\r\n", (int)n, (unsigned)raw, (long)mm);
    Motor_Set(0, 0);
    return;
  }

  Motor_Set(0, 0);
  printf("BB,DONE,1,%d,%04X,%ld,%d\r\n",
         (int)n, (unsigned)raw, (long)mm, (int)(gyro_angle * 10.0f));
#else
  (void)d_stop;
#endif
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

  /* 盲盒收工后要保持“光电管压在 T 口白线上”这个位置：这时再按线回正，
   * 会把车拱回线中心、离开白线，所以直接停手，直到下次发车。 */
  if (bb_hold) { Motor_Set(0, 0); return; }

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
/* ==================== 负压风扇：一次性初始化 + 运行期起停 ====================
 * 上电只建立 50 Hz 信号并保持 1.0 ms 零油门（风扇不转），
 * 发车时 Fan_Start() 走电调解锁窗口，停车后 Fan_Stop() 回零油门。
 * 这样待发阶段地面不被吸住（Align_Hold 的原地对正还能动），也不白耗电。 */
void Init_brushless_motor(void)
{
  /* PA11 idles low; the first high level then also lasts exactly the idle
     width, because the DMA writes the first slot 0.1 ms after TIM1 starts. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  ESC_BuildDmaTable(ESC_STOP_PULSE_SLOTS);   /* 1.0 ms = 零油门，风扇不转 */
  esc_period_count = 0U;
  esc_arming      = 0U;
  fan_on          = 0U;
  hdma_tim1_up.XferCpltCallback = ESC_DmaTransferComplete;
  if (HAL_DMA_Start_IT(&hdma_tim1_up,(uint32_t)esc_dma_table,(uint32_t)&GPIOA->BSRR,ESC_PULSE_SLOTS) != HAL_OK)
    Error_Handler();
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  printf("FAN,IDLE\r\n");   /* 信号已出、风扇停用；按键发车才真正起转 */
}

/* 起负压：电调要先认 0.9 ms 低油门满 2 s（自检/解锁），再跳 2.0 ms 全速。
 * 阻塞约 2 s。返回 1 = 已全速；0 = 没起（宏关闭 / 超时）。
 * 从调用到 main 打完 GO 之前，车必须被扶稳。 */
uint8_t Fan_Start(void)
{
  uint32_t t0;

  if (!IF_start_Brushless_motor) return 0U;
  if (fan_on) return 1U;              /* 已经在转，重复调用直接成功 */

  printf("FAN,START\r\n");
  fan_on           = 1U;
  esc_period_count = 0U;
  ESC_SetPulseWidth(ESC_START_PULSE_SLOTS);   /* 只改变化的槽位，不整表重写 */
  esc_arming = 1U;                            /* 交给 DMA 完成中断去计时 */
  __HAL_DMA_ENABLE_IT(&hdma_tim1_up, DMA_IT_TC);

  t0 = HAL_GetTick();
  while (esc_arming && (uint32_t)(HAL_GetTick() - t0) < FAN_START_TMO_MS) {
    HAL_Delay(10);
  }
  if (esc_arming) {                   /* 超时：退回零油门，别停在 0.9 ms 半解锁状态 */
    esc_arming = 0U;
    fan_on     = 0U;
    __HAL_DMA_DISABLE_IT(&hdma_tim1_up, DMA_IT_TC);
    ESC_SetPulseWidth(ESC_STOP_PULSE_SLOTS);
    printf("FAN,TMO\r\n");
    return 0U;
  }
  printf("FAN,ON\r\n");
  return 1U;
}

/* 停负压：回到 1.0 ms 零油门。电调保持解锁，下次 Fan_Start 直接再走一遍
 * 0.9 ms -> 2.0 ms 即可，不需要整机复位。 */
void Fan_Stop(void)
{
  if (!fan_on) return;                /* 本来就没转：不打印，避免一次发车多条 FAN */

  fan_on     = 0U;
  esc_arming = 0U;
  __HAL_DMA_DISABLE_IT(&hdma_tim1_up, DMA_IT_TC);
  ESC_SetPulseWidth(ESC_STOP_PULSE_SLOTS);
  printf("FAN,OFF\r\n");
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

  /* USER CODE BEGIN 2 */
  int8_t imu = dodo_BMI270_init();       /* 初始化陀螺仪，0 = 成功 */
  printf("IMU,%d\r\n", (int)imu);
  Motor_Set(0, 0);                       /* 标定期间轮子不转 */

  /* 【11/27 顺序调整：风扇信号链必须在标定之前建立】
   * 1) Init_brushless_motor() 只把 PA11 拉成 50 Hz + 1.0 ms 零油门，风扇本身
   *    不转（真正起转是按键时的 Fan_Start：0.9 ms 满 2 s -> 2.0 ms），所以它
   *    不会污染零偏测量 —— 旧注释"必须先标定再初始化风扇"的前提已不成立。
   * 2) 反过来，把它留在 Gyro_CalibBias() 后面是有害的：标定判失败会进 for(;;)
   *    永久卡死，PA11 就停在 MX_GPIO_Init() 写低的那个电平上，电调连信号都收不到，
   *    表现为"上电后负压怎么都不转"，按键也没反应（主循环根本没进去）。
   *    118.txt 15:06:59 那次就是这样：IMU,0 之后只剩 CAL_BAD，没有 FAN,IDLE。
   * 3) 提前之后：任何情况下 PA11 都有合法 PWM，日志里一定能看到 FAN,IDLE，
   *    也便于区分"信号没出"和"电调没响应"。
   *
   * ⚠ 附带处理：电调收到 1.0 ms 会先走一遍解锁鸣叫。鸣叫是电机绕组发声，
   *   会给陀螺加一点振动，所以这里等 600 ms 让叫声过去再标零偏。
   *   若上电仍看到 GB 值偏脏（σ 明显大于 0.5 °/s），把这个延时加大。 */
  Init_brushless_motor();
  HAL_Delay(600U);

  /* 零偏在【风扇静止】时采：此刻风扇只有零油门信号，不转。
   * 风扇全速之后的那一刻，Gyro_RebaseBiasFan() 还会补一次"带风扇"的零偏。 */
  Gyro_CalibBias();                      /* 采零偏：调用时车必须静止、风扇未转 */

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
      bb_hold = 0U;                   /* 新一趟：解除盲盒收工锁，待发对正恢复工作 */

#if BB_TEST_MODE
      /* ---- 盲盒单独调试：不起负压、不跑两圈，直接执行盲盒 ----
       * 车要提前摆到“两圈跑完停在发车区”的那个位置和朝向上。
       * 清角度/里程是为了让 BB,SPIN 的目标和直行基准都从 0 开始算。 */
      Gyro_ResetAngle();
      ENC_Reset();
      printf("GO,BBTEST\r\n");
      BlindBox_Run(0);
      printf("STOP\r\n");
      Motor_Set(0, 0);
#else
      /* ---- 1) 先起负压（阻塞约 2 s：0.9 ms 满 2 s 解锁 -> 2.0 ms 全速）----
       * 这段时间车必须被扶稳：风扇会把车往下压，位置和姿态都可能微动。
       * 顺序上负压必须在"清零里程/角度"之前，否则起转振动的积分会带进比赛。
       * 起不来（宏关闭 / 电调没响应）也照常发车，只是没有下压力 —— 见 FAN,TMO。 */
      {
        uint8_t fan_ok = Fan_Start();

        /* ---- 2) 风扇全速后补测零偏 ----
         * 静标那一版是"无风扇"环境采的，实际跑车时风扇一直在转，
         * 振动整流出的直流偏置要在这里补掉，否则长距离偏航会漂。
         * 判读 FAN,GB,<零偏×10>,<σ×10>；出现 FAN,VIB 说明采样期间车在动，
         * 这次不替换零偏（保留静标那版）。 */
        if (fan_ok) Gyro_RebaseBiasFan();

        /* ---- 3) 风扇到位了，再清里程与角度，然后发车 ---- */
        Gyro_ResetAngle();            /* 起转期间的角度作废 */
        ENC_Reset();
        printf("GO\r\n");
#if (RUN_MODE == 0)
        Drive_Straight_PID(742);      /* 直行 1 m（视频③） */
#elif (RUN_MODE == 1)
        Spin_90x4();                  /* 原地转 4×90°（视频②） */
#else
        Follow_Line_Run();            /* 循迹跑两圈（视频①） */
#endif
        printf("STOP\r\n");
        Fan_Stop();                   /* 停车即关负压（车已经停稳，掉吸力不会跑偏） */
      }
#endif
    }
    /* 空闲时也把串口积压的字节读掉：否则上一次停车前发的 's' 会留在 FIFO 里，
     * 下一次 GO 之后 500ms 静默期一过就立刻把车停掉（看起来像"发车就停"）。 */
    (void)Serial_StopPressed();

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
