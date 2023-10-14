/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "math.h"
#include "stdio.h"
#include "string.h"
#include "LCD_char.h"
#include "FLASH_SECTOR_H7.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RR

#define PI 3.1415926535897932384626433832795
#define cos120 -0.5
#define sin120 0.86602540378443864676372317075294
#define RAD2DEG 57.29577951308232087679815

#ifdef ER
#define KST
#define L1 90.0
#define L2 124
#define dp 40.7
#else
#define L1 60.0
#define L2 111.5
#define dp 40.7
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */

enum master_register_name
{
	_SET_VX				= 0x10,
	_SET_VY				= 0x11,
	_SET_V_YAW			= 0x12,
	_SET_X				= 0x13,
	_SET_Y				= 0x14,
	_SET_YAW			= 0x15,

	_READ_ENCD_X		= 0x20,
	_READ_ENCD_Y		= 0x21,
	_READ_ENCD_YAW		= 0x22,
	_READ_ENCD0			= 0x23,
	_READ_ENCD1			= 0x24,
	_READ_ENCD2			= 0x25,
	_READ_ENCD3			= 0x26,

	_RESET_ORIGIN		= 0x30,
	_RESET_YAW			= 0x31,

	_SET_COORD			= 0x40,
	_LOCK_MOVE			= 0X41,

	_SET_PID_TOLERANCE	= 0x50,
	_SET_PID_MODE		= 0x51
};

enum thrower_register_name
{
	_SET_RPM			= 0x10,
	_SET_KP_M1 			= 0xC1,
	_SET_KI_M1 			= 0xC2,
	_SET_KD_M1 			= 0xC3,
	_SET_KP_M2 			= 0xC4,
	_SET_KI_M2 			= 0xC5,
	_SET_KD_M2 			= 0xC6,
	_READ_HALL			= 0xD1
};

enum rotator_register_name
{
	_STEER_R = 0xA0,
	_STEER_L = 0xA1,
	_STEER_P = 0xA2,

	_OFFSET_R = 0xB0,
	_OFFSET_L = 0xB1,
	_OFFSET_P = 0xB2,
};

enum fifo_state
{
	_FULL = 0,
	_IDLE
};

enum LCD_register
{
	_4_BIT_MODE = 0x20,
	_EXTENDED_INSTRUCTION_MODE = 0x24,
	_GRAPHIC_MODE = 0x26,
	_DISPLAY_ON = 0x0C,
	_DISPLAY_CLEAR = 0x01,
	_CURSOR_INC = 0x06,
};

enum servo_channel
{
	_SERVO0,
	_SERVO1,
	_SERVO2,
	_SERVO3,
	_SERVO4,
	_SERVO5,
	_SERVO6,
	_SERVO7,
};

enum menu
{
	_MAIN,

	_TR_SET,

	_PID,
	_PID_WHEEL0,
	_PID_WHEEL1,
	_PID_WHEEL2,
	_PID_WHEEL3,
	_PID_THROWER_R,
	_PID_THROWER_L,

	_ROBOT_SET,
	_CALL_WHEEL,
	_CALL_ROTATOR,
	_CHECK_HW,
	_READ_SENS,
	_BLT_SET,
	_ABOUT,
};

enum submenu_tr_set
{
	_TARGET_TR	= 1,
	_POWER_TR	= 2,
	_ANGLE_TR	= 3
};

enum submenu_pid
{
	_KP_M1 = 1,
	_KI_M1,
	_KD_M1,
	_KP_M2,
	_KI_M2,
	_KD_M2,
};

enum submenu_robot_set
{
	_R_SPEED = 1,
	_R_MODE,
};

enum submenu_call_wheels
{
	_W0 = 1,
	_W1,
	_W2,
	_W3,
};

enum gripper_ch
{
	_GRIPPER1 = 0,
	_GRIPPER2,
};

enum thrower_ch
{
	_THROWER1 = 0,
	_THROWER2,
};

enum ring_grip_state
{
	_GRIP = 0, _RELEASE
};

typedef struct
{
	double vx, vy, v_yaw, x, y, yaw, last_vx, last_vy, last_v_yaw;
}VROBOT_t;

typedef struct
{
	uint8_t CH;
	uint32_t pwm;
	double offset;
}SERVO_t;

typedef struct
{
	uint32_t delay, delay_speed, timer, rot_inc;
	double arm_degree, arm_target_deg, grip_degree[2];
	_Bool state, rot_move_flag, R_empty , L_empty, start_delay_flag;
}PICKER_t;

typedef struct
{
	double z, last_z;
	uint32_t timer, delay;
	_Bool start_flag, move_flag;
}LIFT_RING_t;

typedef struct
{
	uint32_t delay, timer;
	_Bool start_flag, move_flag, state, last_state, ring_empty, last_ring_empty;
}GRIPPER_t;

typedef struct
{
	uint32_t timer, delay;
	_Bool start_flag, move_flag, ring_empty;
}PUSHER_t;

typedef struct
{
	double theta, last_theta;
	uint32_t timer, delay;
	_Bool start_flag, move_flag, toggle;
}THROWER_ROT_t;

typedef struct // Sulthon
{
	float x_ljoystick, y_ljoystick, x_rjoystick, y_rjoystick; // x and y (Left and Right joystick)
	int x_ljoystick_int, y_ljoystick_int, x_rjoystick_int, y_rjoystick_int;
	int16_t lthrower_speed, rthrower_speed;// (Left and Right Thrower) speed
	_Bool isDo1, isDo2, isDo3, isAng1, isAng2, isAng3,
			isAng4, isAng5, isEn1, isEn2, isEn3; // State tiap pole
}COM_BLT_t;

char menu_str[40][32] =
{
		{"1.Thrower Setting  "},//0
		{"2.PID              "},
		{"3.Robot Setting    "},
		{"4.Call Angle Wheels"},
		{"5.Call Angle ROT   "},
		{"6.Check Hardware   "},
		{"7.Read Sensors     "},
		{"8.Bluetooth Setting"},
		{"9.About Robot      "},
		//sub menu Thrower Setting
		{"1.Thrower Setting: "},//8
		{" Thrower           "},
		{" 	Power:           "},
		{" 	Angle:           "},
		//sub menu PID
		{"2.PID:             "},//12
		{" Wheel0            "},
		{" Wheel1            "},
		{" Wheel2            "},
		{" Wheel3            "},
		{" Thrower R         "},
		{" Thrower L         "},
		//sub menu Robot Setting
		{"3.Robot Setting:   "},//19
		{" Speed:            "},
		{" Mode:             "},
		//sub menu Call Angle wheels
		{"4.Call Angle Wheel:"},//22
		{" Angle Wheel0:     "},
		{" Angle Wheel1:     "},
		{" Angle Wheel2:     "},
		{" Angle Wheel3:     "},
		//sub menu Check Hardware
		{"6.Check Hardware:  "},//27
		{" Thrower           "},
		{" Gripper           "},
		{" Swerve Wheels     "},
		//Read Sensors
		{"7.Read Sensors:    "},//31
		{" Encoders          "},
		{" LiDAR             "},
		//About Robot
		{"9.About Robot:     "},//34
		{"BANDHAYUDHA ER v1.0"},
		{" ABU ROBOCON 2023  "},
		{"     H-53 KRI      "},
};
const uint8_t 	ln_menu[31] =
{
		9, 4, 7, 4, 4, 4, 4, 4, 4, 3, 5, 4, 4, 3, 0, 4,
};

COM_BLT_t 	com_blt; // Sulthon
SERVO_t 	servo1, servo2, servo3, servo4, servo5, servo6, servo7, servo8;
VROBOT_t 	manual, last_manual, sens;
FDCAN_TxHeaderTypeDef can1_tx_header, can2_tx_header;
FDCAN_RxHeaderTypeDef can1_rx_header, can2_rx_header;

PICKER_t ring_pic;
LIFT_RING_t lift_R, lift_L;
GRIPPER_t grip_R, grip_L;
PUSHER_t pusher_R, pusher_L;
THROWER_ROT_t trot_R, trot_L;

_Bool 		button_state = 0;
char 		lcd_str_buff[255];
uint8_t 	lcd_rows = 64, lcd_cols = 128,
			pixel_buffer[(128 * 64)/8], last_pixel_buffer[(128 * 64)/8],
			menu_state = _MAIN, menu_str_inc = 0, menu_mode = 0, menu_dis_limit;
int8_t 		ui_encd_val = 0, ui_last_encd_val = 0, menu_pos = 0, last_menu_pos = -1,
			cursor_enter = 0;
uint32_t 	last_tim5 = 100, bz_t;
//perlu disimpan
uint16_t 	power_pelempar[12] = {1300, 2050, 2120, 2580, 2710, 2600, 2120, 2010}; //1300, 2200, 2630, 2210
int16_t 	lst_speed;
float	 	angle_pelempar[12] = {18.0, -24.0, 29.0, 13.8, -9.5, -1.5, 73.5, -68.1}; //18.0, 29.0, -9.5, 73.5
float		kp_set[10], ki_set[10], kd_set[10],
			offset_angle_wheel[4], offset_angle_R, offset_angle_L, offset_angle_P;
int8_t 		target_index = 0, robot_speed, robot_mode,
			pole_order[11] = {2, 5, 4, 6, 7, 8, 1, 3, 9, 10, 11};
//-------------

_Bool 		can2_fifo_state = _IDLE;
uint8_t 	blt1_rx_buff[4],blt_rx_buff[5], BUFFER_LEN = 5, blt_tx_buff[5], //Sulthon
			can2_tx_data[8], can2_rx_data[8], can1_tx_data[8], can1_rx_data[8],
			can2_queue = 0, wait_request;

uint32_t v_test = 0;
double t_rpm_test, last_t_rpm_test,
		angle_thrR = 0, last_angle_thrR = 0,
		angle_thrL = 0, last_angle_thrL = 0;
_Bool toggle_thr = 0, launch_R = 0, launch_L = 0, ctrl_state = 0, last_ctrl_state = 0, next_bt = 0,
		grip_state = 0, thrower_en = 0, grip_bt = 0;
uint32_t shoot_timer_R, shoot_timer_L, acc_shoot_timer, t_rot_thrR, t_rot_thrL;

uint8_t ring_stack = 10;
float down_grip = -38, sa = 180, upper_ring_stack = -45;

uint32_t adc;
double current, distance, distance_filter, actual_distance;

//---------------------------------------------------

uint32_t flashmem_buffer[42];

_Bool thrower_ring_empty = 0, picker_ring_empty = 0;

double 	j, acc, vel, dis, last_dis, d_robot, last_d_robot, max_acc = 0.0000007,
		dest = 100, max_j = 0.0000000000001,
		theta_des, x_robot, y_robot, x_start, y_start;
uint8_t s_curve_state = 0, trpz_state = 0;
uint32_t s_time, ts[7];
_Bool robot_move_flag = 0, stop = 0, r_test = 0, toggle = 0, hold_picker = 0,
		ready_R = 0, ready_L = 0, change_param_tR = 0, change_param_tL = 0,
		launch_R_prepare = 0, launch_L_prepare = 0,
		launch_L_permission = 0, launch_R_permission = 0, thrower_off = 0, start_trial = 0,
		ring_is_empty = 0, place_R_permission = 1, place_L_permission = 1,
		pemutar_start = 0;

uint8_t throw_order = 0, last_throw_order = 10,
		robot_state = 0, state_picker = 0, state_place = 0, ring_order_R = 0, ring_order_L = 0,
		robot_move_state = 0;

double speed_mul = 1;

_Bool RR_ambil = 0, RR_lempar = 0, RR_tail=0, RR_grip_prepare = 0, RR_tailgrip = 0;
uint8_t RR_ring_ready = 0;
int8_t Dir_negation = 1;

uint32_t dir_t, bz_t;
int xss = 0;


/*
 * 1 1690 0.8
 * 4 2600 0.8
 * 6
 *
 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM5_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM12_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI2_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void robot_init ()
{
	  ring_pic.arm_degree = 180.0;
	  ring_pic.arm_target_deg = 180.0;
	  ring_pic.rot_inc = 3;
	  ring_pic.R_empty = 1;
	  ring_pic.L_empty = 1;
	  ring_pic.delay = 500;

	  lift_R.z = -25;
	  lift_L.z = -25;
	  lift_R.last_z = -25;
	  lift_L.last_z = -25;

	  grip_R.ring_empty = 1;
	  grip_L.ring_empty = 1;
	  grip_R.last_ring_empty = 1;
	  grip_L.last_ring_empty = 1;
	  grip_R.state = _RELEASE;
	  grip_L.state = _RELEASE;
	  grip_R.last_state = _RELEASE;
	  grip_L.last_state = _RELEASE;
	  grip_R.delay = 350;
	  grip_L.delay = 350;

	  pusher_R.ring_empty = 1;
	  pusher_L.ring_empty = 1;
	  pusher_R.delay = 500;
	  pusher_L.delay = 500;

	  trot_R.theta = 0;
	  trot_R.last_theta = 0;
	  trot_L.theta = 0;
	  trot_L.last_theta = 0;
}

void save_data2flash ()
{
	int16_t d_temp[12];
	for (uint8_t i = 0; i < 6; i++)
	{
		flashmem_buffer[i] = (power_pelempar[i*2] << 16) | power_pelempar[i*2+1];
	}
	for (uint8_t i = 0; i < 6; i++)
	{
		d_temp[i*2] = (angle_pelempar[i*2] * 100.0);
		d_temp[i*2+1] = (angle_pelempar[i*2+1] * 100.0);
	 	flashmem_buffer[i+6] = ((uint16_t)d_temp[i*2] << 16) | (uint16_t)d_temp[i*2+1];
	}
	Flash_Write_Data(0x08100000 , flashmem_buffer, 12);
}

void flash_get_data ()
{
	int16_t d_temp[12];
	Flash_Read_Data(0x08100000 , flashmem_buffer, 12);
	for (uint8_t i = 0; i < 6; i++)
	{
		power_pelempar[i*2] = (flashmem_buffer[i] >> 16) & 0xffff;
		power_pelempar[i*2+1] = flashmem_buffer[i] & 0xffff;
	}
	for (uint8_t i = 0; i < 6; i++)
	{
		d_temp[i*2] = (flashmem_buffer[i+6] >> 16) & 0xffff;
		d_temp[i*2+1] = flashmem_buffer[i+6] & 0xffff;
		angle_pelempar[i*2] = d_temp[i*2] / 100.0;
		angle_pelempar[i*2+1] = d_temp[i*2+1] / 100.0;
	}
}

void beep (uint32_t t)
{
	BUZZER_GPIO_Port->BSRR = BUZZER_Pin;
	HAL_Delay(t);
	BUZZER_GPIO_Port->BSRR = BUZZER_Pin<<16;
}

void delay_us (uint32_t us)
{
	TIM12->CNT = 0;
	while (TIM12->CNT < us) ;
}

void lcd_push_enable()
{
	LCD_EN_GPIO_Port->BSRR = LCD_EN_Pin;
	delay_us(10);
	LCD_EN_GPIO_Port->BSRR = LCD_EN_Pin<<16;
	delay_us(10);
}

void lcd_command(uint8_t cmd)
{
	LCD_RS_GPIO_Port->BSRR = LCD_RS_Pin<<16;
	LCD_RW_GPIO_Port->BSRR = LCD_RW_Pin<<16;
	uint32_t last_output = GPIOE->ODR & ~0x78;
	GPIOE->ODR = last_output | ((cmd >> 1) & 0x78);
	lcd_push_enable();
	last_output = GPIOE->ODR & ~0x78;
	GPIOE->ODR = last_output | ((cmd << 3) & 0x78);
	lcd_push_enable();
}

void lcd_data(uint8_t data)
{
	LCD_RS_GPIO_Port->BSRR = LCD_RS_Pin;
	LCD_RW_GPIO_Port->BSRR = LCD_RW_Pin<<16;
	uint32_t last_output = GPIOE->ODR & ~0x78;
	GPIOE->ODR = last_output | ((data >> 1) & 0x78);
	lcd_push_enable();
	last_output = GPIOE->ODR & ~0x78;
	GPIOE->ODR = last_output | ((data << 3) & 0x78);
	lcd_push_enable();
}

void invert_bitmap(uint8_t bitmap)
{
	LCD_RS_GPIO_Port->BSRR = LCD_RS_Pin;
	LCD_RW_GPIO_Port->BSRR = LCD_RW_Pin<<16;
	HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, ((bitmap >> 0) & 1) ? SET : RESET);
	HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, ((bitmap >> 1) & 1) ? SET : RESET);
	HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, ((bitmap >> 2) & 1) ? SET : RESET);
	HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, ((bitmap >> 3) & 1) ? SET : RESET);
	lcd_push_enable();
	HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, ((bitmap >> 4) & 1) ? SET : RESET);
	HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, ((bitmap >> 5) & 1) ? SET : RESET);
	HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, ((bitmap >> 6) & 1) ? SET : RESET);
	HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, ((bitmap >> 7) & 1) ? SET : RESET);
	lcd_push_enable();

}

void lcd_bitmap(const uint8_t* bitmap)
{
	uint8_t x, y;
	for(y = 0; y < 64; y++)
	{
		if(y < 32)
		{
			for(x = 0; x < 8; x++)
			{
				lcd_command(0x80 | y);
				lcd_command(0x80 | x);
				lcd_data(bitmap[2*x + 16*y]);
				lcd_data(bitmap[2*x+1 + 16*y]);
			}
		}
		else
		{
			for(x = 0; x < 8; x++)
			{
				lcd_command(0x80 | (y-32));
				lcd_command(0x88 | x);
				lcd_data(bitmap[2*x + 16*y]);
				lcd_data(bitmap[2*x+1 + 16*y]);
			}
		}

	}
}

void clear_graphic()
{
	uint8_t x, y;
	for(y = 0; y < 64; y++)
	{
		if(y < 32)
		{
			for(x = 0; x < 8; x++)
			{
				lcd_command(0x80 | y);
				lcd_command(0x80 | x);
				lcd_data(0);
				lcd_data(0);
			}
		}
		else
		{
			for(x = 0; x < 8; x++)
			{
				lcd_command(0x80 | (y-32));
				lcd_command(0x88 | x);
				lcd_data(0);
				lcd_data(0);
			}
		}

	}
	memset(pixel_buffer, 0, sizeof(pixel_buffer));
}

void lcd_init()
{
	HAL_Delay(500);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, SET);
	LCD_RS_GPIO_Port->BSRR = LCD_RS_Pin<<16;
	LCD_RW_GPIO_Port->BSRR = LCD_RW_Pin<<16;
	HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, RESET);
	HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, RESET);
	HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, SET);
	HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, RESET);
	lcd_push_enable();
	lcd_command(_4_BIT_MODE);
	delay_us(101);
	lcd_command(_GRAPHIC_MODE);
	delay_us(101);
	lcd_command(_DISPLAY_ON);
	delay_us(101);
	clear_graphic();
	HAL_Delay(100);
	LCD_BL_GPIO_Port->BSRR = LCD_BL_Pin;
}

void lcd_setpixel(uint8_t x, uint8_t y)
{
	if (y < lcd_rows && x < lcd_cols)
	{
	  pixel_buffer[x/8 + y*(lcd_cols/8)] |= (0x80 >> (x%8));
	}
}

void lcd_setpixel_inverse (uint8_t x, uint8_t y)
{
	if (y < lcd_rows && x < lcd_cols)
	{
	  pixel_buffer[x/8 + y*(lcd_cols/8)] &= ~(0x80 >> (x%8));
	}
}

void lcd_drawline(int x0, int y0, int x1, int y1, _Bool color)
{
  int dx = (x1 >= x0) ? x1 - x0 : x0 - x1;
  int dy = (y1 >= y0) ? y1 - y0 : y0 - y1;
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;


  for (;;)
  {
	  if(color)
	  {
		  lcd_setpixel(x0, y0);
	  }
	  else
	  {
		  lcd_setpixel_inverse(x0, y0);
	  }

    if (x0 == x1 && y0 == y1) break;
    int e2 = err + err;
    if (e2 > -dy)
    {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx)
    {
      err += dx;
      y0 += sy;
    }
  }
}

void lcd_drawrectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, _Bool color)
{
	/* Check input parameters */
	if (
		x >= lcd_cols ||
		y >= lcd_rows
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= lcd_cols) {
		w = lcd_cols - x;
	}
	if ((y + h) >= lcd_rows) {
		h = lcd_rows - y;
	}

	/* Draw 4 lines */
	lcd_drawline(x, y, x + w, y, color);         /* Top line */
	lcd_drawline(x, y + h, x + w, y + h, color); /* Bottom line */
	lcd_drawline(x, y, x, y + h, color);         /* Left line */
	lcd_drawline(x + w, y, x + w, y + h, color); /* Right line */
}

void lcd_print_char (uint8_t x, uint8_t y, char ch)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		for (int8_t n = 4; n >= 0; n--)
		{
			if ((fonts[(uint8_t)ch][i]>>n)&1)
				lcd_setpixel ( (4-n)+x, i+y);
			else
				lcd_setpixel_inverse ( (4-n)+x, i+y);
		}
	}
}

void lcd_print_string(uint8_t x, uint8_t y, char *str)
{
	while(*str)
	{
		if (*str == '\n')
		{
			x = 6;
			y += 9;
		}
		lcd_print_char(x, y, *str++);
		x+=6;
	}
}

void lcd_update(void)
{
  lcd_bitmap(pixel_buffer);
}

void menu_cursor (uint8_t pos)
{
	lcd_print_char(0, pos*9, 0);
}

void menu_cursor_invisible (uint8_t pos)
{
	lcd_print_char(0, pos*9, ' ');
}

void back_cursor (uint8_t pos)
{
	lcd_print_char(0, pos*9, 1);
}

void menu_update ()
{
	menu_dis_limit = ln_menu[menu_state];
	if (menu_dis_limit > 7) menu_dis_limit = 7;
	switch (menu_state)
	{
	case _TR_SET:
		sprintf (lcd_str_buff,
				"1.Thrower Setting:\n"
				"Target-%02d[POLE-%02d]:\n"
				"Power:%04d\n"
				"Angle:%03.1f   ",
				target_index, pole_order[target_index],
				power_pelempar[target_index],
				angle_pelempar[target_index]
				);
		lcd_print_string (6, 0, lcd_str_buff);
		break;
	case _PID:
		lcd_print_string (6, 0,
				"2.PID:\n"
				"wheel-0\n"
				"wheel-1\n"
				"wheel-2\n"
				"wheel-3\n"
				"thrower R\n"
				"thrower L"
				);
		break;
	case _PID_WHEEL0:
	case _PID_WHEEL1:
	case _PID_WHEEL2:
	case _PID_WHEEL3:
		sprintf (lcd_str_buff,
				"PID wheel-%d:\n"
				"KP:%04.5f\n"
				"KI:%04.5f\n"
				"KD:%04.5f",
				menu_state-3,
				kp_set[menu_state-3],
				ki_set[menu_state-3],
				kd_set[menu_state-3]
				);
		lcd_print_string (6, 0, lcd_str_buff);
		break;
	case _PID_THROWER_R:
		sprintf (lcd_str_buff,
				"PID thrower R\n"
				"KP:%04.5f\n"
				"KI:%04.5f\n"
				"KD:%04.5f",
				kp_set[4],
				ki_set[4],
				kd_set[4]
				);
		lcd_print_string (6, 0, lcd_str_buff);
		break;
	case _PID_THROWER_L:
		sprintf (lcd_str_buff,
				"PID thrower L\n"
				"KP:%04.5f\n"
				"KI:%04.5f\n"
				"KD:%04.5f",
				kp_set[4],
				ki_set[4],
				kd_set[4]
				);
		lcd_print_string (6, 0, lcd_str_buff);
		break;
	case _ROBOT_SET:
		lcd_print_string (6, 0, "3.Robot Setting:");
		switch (robot_speed)
		{
		case 0: sprintf (lcd_str_buff, " Speed:%c  ", 3); break;
		case 1: sprintf (lcd_str_buff, " Speed:%c%c ", 3, 3); break;
		case 2: sprintf (lcd_str_buff, " Speed:%c%c%c", 3, 3, 3); break;
		}
		lcd_print_string (6, 9, lcd_str_buff);
		switch (robot_mode)
		{
		case 0: lcd_print_string (6, 18, " Mode:Normal"); break;
		case 1: lcd_print_string (6, 18, " Mode:Retry1"); break;
		case 2: lcd_print_string (6, 18, " Mode:Retry2"); break;
		case 3: lcd_print_string (6, 18, " Mode:Retry3"); break;
		case 4: lcd_print_string (6, 18, " Mode:Trial "); break;
		}
		break;
	case _CALL_WHEEL:
		sprintf (lcd_str_buff,
				"4.Call Angle Wheels:\n"
				"offset W0:%03.1f    \n"
				"offset W1:%03.1f    \n"
				"offset W2:%03.1f    \n"
				"offset W3:%03.1f    ",
				offset_angle_wheel[0],
				offset_angle_wheel[1],
				offset_angle_wheel[2],
				offset_angle_wheel[3]
				);
		lcd_print_string (6, 0, lcd_str_buff);
		break;

	case _CALL_ROTATOR:
		sprintf (lcd_str_buff,
				"5.Call Angle ROT:\n"
				"offset R:%03.1f    \n"
				"offset L:%03.1f    \n"
				"offset P:%03.1f    ",
				offset_angle_R,
				offset_angle_L,
				offset_angle_P
				);
		lcd_print_string (6, 0, lcd_str_buff);
		break;
	default:
		for (uint8_t i = 0; i < menu_dis_limit; i++)
		{
			lcd_print_string (6, i*9, menu_str[i+menu_pos+menu_str_inc]);
		}
		break;
	}
	lcd_update ();
}

void menu_enter_param ()
{
	switch (menu_state)
	{
	case _TR_SET:
		if (cursor_enter != 0)
		{
			if (menu_mode == 0)
			{
				menu_mode = 1;
			}
			else
			{
				menu_mode = 0;
				save_data2flash ();
			}
		}
		break;
	case _PID_WHEEL0:
	case _PID_WHEEL1:
	case _PID_WHEEL2:
	case _PID_WHEEL3:
	case _PID_THROWER_R:
	case _PID_THROWER_L:
	case _ROBOT_SET:
	case _CALL_WHEEL:
	case _CALL_ROTATOR:
		if (cursor_enter != 0)
		{
			if (menu_mode == 0)
			{
				menu_mode = 1;
			}
			else
			{
				menu_mode = 0;
			}
		}
		break;
	}
}

void menu_change_param ()
{
	switch (menu_state)
	{
	case _TR_SET:
		switch (cursor_enter)
		{
		case _TARGET_TR:
			if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
			{
				target_index++;
				if (target_index > 10) target_index = 10;
			}
			else
			{
				target_index--;
				if (target_index < 0) target_index = 0;
			}
			break;
		case _POWER_TR:
			if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
			{
				power_pelempar[target_index]+=2;
				if (power_pelempar[target_index] > 4000) power_pelempar[target_index] = 4000;
			}
			else
			{
				power_pelempar[target_index]-=2;
				if (power_pelempar[target_index] < 0) power_pelempar[target_index] = 0;
			}
			break;
		case _ANGLE_TR:
			if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
			{
				angle_pelempar[target_index]+=0.1;
				if (angle_pelempar[target_index] > 90) angle_pelempar[target_index] = 90;
			}
			else
			{
				angle_pelempar[target_index]-=0.1;
				if (angle_pelempar[target_index] < -90) angle_pelempar[target_index] = -90;
			}
			break;
		}
		break;
	case _PID_WHEEL0:
	case _PID_WHEEL1:
	case _PID_WHEEL2:
	case _PID_WHEEL3:
	case _PID_THROWER_R:
	case _PID_THROWER_L:
		switch (cursor_enter)
		{
		case _KP_M1:
			if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
			{
				kp_set[menu_state-3]+=0.01;
				if (kp_set[menu_state-3] > 30) kp_set[menu_state-3] = 30;
			}
			else
			{
				kp_set[menu_state-3]-=0.01;
				if (kp_set[menu_state-3] < 0) kp_set[menu_state-3] = 0;
			}
			break;
		case _KI_M1:
			if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
			{
				ki_set[menu_state-3]+=0.01;
				if (ki_set[menu_state-3] > 30) ki_set[menu_state-3] = 30;
			}
			else
			{
				ki_set[menu_state-3]-=0.01;
				if (ki_set[menu_state-3] < 0) ki_set[menu_state-3] = 0;
			}
			break;
		case _KD_M1:
			if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
			{
				kd_set[menu_state-3]+=0.01;
				if (kd_set[menu_state-3] > 30) kd_set[menu_state-3] = 30;
			}
			else
			{
				kd_set[menu_state-3]-=0.01;
				if (kd_set[menu_state-3] < 0) kd_set[menu_state-3] = 0;
			}
			break;
		}
		break;
	case _ROBOT_SET:
		switch (cursor_enter)
		{
		case _R_SPEED:
			if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
			{
				robot_speed++;
				if (robot_speed > 2) robot_speed = 2;
			}
			else
			{
				robot_speed--;
				if (robot_speed < 0) robot_speed = 0;
			}
			break;
		case _R_MODE:
			if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
			{
				robot_mode++;
				if (robot_mode > 4) robot_mode = 0;
			}
			else
			{
				robot_mode--;
				if (robot_mode < 0) robot_mode = 4;
			}
			break;
		}
		break;
	case _CALL_WHEEL:
		if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
		{
			offset_angle_wheel[cursor_enter-1]+=0.1;
			if (offset_angle_wheel[cursor_enter-1] > 180) offset_angle_wheel[cursor_enter-1] = 180;
		}
		else
		{
			offset_angle_wheel[cursor_enter-1]-=0.1;
			if (offset_angle_wheel[cursor_enter-1] < -180) offset_angle_wheel[cursor_enter-1] = -180;
		}
		break;
	case _CALL_ROTATOR:
		if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
		{
			switch (cursor_enter)
			{
			case 1:
				offset_angle_R+=0.1;
				if (offset_angle_R > 180) offset_angle_R = 180;
				break;
			case 2:
				offset_angle_L+=0.1;
				if (offset_angle_L > 180) offset_angle_L = 180;
				break;
			case 3:
				offset_angle_P+=0.1;
				if (offset_angle_P > 180) offset_angle_P = 180;
				break;
			}
		}
		else
		{
			switch (cursor_enter)
			{
			case 1:
				offset_angle_R-=0.1;
				if (offset_angle_R < -180) offset_angle_R = -180;
				break;
			case 2:
				offset_angle_L-=0.1;
				if (offset_angle_L < -180) offset_angle_L = -180;
				break;
			case 3:
				offset_angle_P-=0.1;
				if (offset_angle_P < -180) offset_angle_P = -180;
				break;
			}
		}
		break;
	}
}

void main_menu ()
{
	if (TIM5->CNT != last_tim5)
	{
		last_tim5 = TIM5->CNT;
		beep(10);
		menu_dis_limit = ln_menu[menu_state];
		if (menu_dis_limit > 7) menu_dis_limit = 7;

		if (menu_mode == 0)
		{
			if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
			{
				ui_encd_val++;
				if (ui_encd_val > menu_dis_limit-1)
				{
					ui_encd_val = menu_dis_limit-1;
				}
				if (ui_last_encd_val == 6)
				{
					if (menu_pos+6 < ln_menu[menu_state]-1)
						menu_pos++;
				}
				else menu_cursor_invisible (ui_last_encd_val);
				cursor_enter++;
				if (cursor_enter > ln_menu[menu_state]-1) cursor_enter = ln_menu[menu_state]-1;
			}
			else
			{
				ui_encd_val--;
				if (ui_encd_val < 0)
				{
					ui_encd_val = 0;
				}
				if (ui_last_encd_val == 0)
				{
					menu_pos--;
					if (menu_pos < 0) menu_pos = 0;
				}
				else menu_cursor_invisible (ui_last_encd_val);
				cursor_enter--;
				if (cursor_enter < 0) cursor_enter = 0;
			}
			if (menu_state != _MAIN && ui_encd_val == 0) back_cursor (ui_encd_val);
			else menu_cursor (ui_encd_val);

			ui_last_encd_val = ui_encd_val;
		}
		else
		{
			menu_change_param ();
		}
		menu_update ();
	}
	//KLICK
	if (!HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin))
	{
		if (!button_state)
		{
			button_state = 1;
			beep(50);
			menu_enter_param ();
			switch (menu_state)
			{
			case _MAIN:
				switch (cursor_enter)
				{
				case 0:
					menu_str_inc = 8;
					menu_state = _TR_SET;
					break;
				case 1:
					menu_str_inc = 12;
					menu_state = _PID;
					break;
				case 2:
					menu_str_inc = 19;
					menu_state = _ROBOT_SET;
					break;
				case 3:
					menu_str_inc = 22;
					menu_state = _CALL_WHEEL;
					break;
				case 4:
					menu_state = _CALL_ROTATOR;
					break;
				case 5:
					menu_str_inc = 28;
					menu_state = _CHECK_HW;
					break;
				case 6:
					menu_str_inc = 32;
					menu_state = _READ_SENS;
					break;
				case 8:
					menu_str_inc = 35;
					menu_state = _ABOUT;
					break;
				}
				menu_pos = 0;
				last_menu_pos = 0;
				ui_encd_val = 1;
				cursor_enter = 1;
			break;
			case _PID:
				switch (cursor_enter)
				{
				case 0:
					menu_state = _MAIN;
					menu_str_inc = 0;
					break;
				case 1:
					menu_state = _PID_WHEEL0;
					break;
				case 2:
					menu_state = _PID_WHEEL1;
					break;
				case 3:
					menu_state = _PID_WHEEL2;
					break;
				case 4:
					menu_state = _PID_WHEEL3;
					break;
				case 5:
					menu_state = _PID_THROWER_R;
					break;
				case 6:
					menu_state = _PID_THROWER_L;
					break;
				}
				menu_pos = 0;
				last_menu_pos = 0;
				ui_encd_val = 1;
				cursor_enter = 1;
				break;
				//////BACK menu
			case _PID_WHEEL0:
			case _PID_WHEEL1:
			case _PID_WHEEL2:
			case _PID_WHEEL3:
			case _PID_THROWER_R:
			case _PID_THROWER_L:
				if (cursor_enter == 0)
				{
					menu_state = _PID;
					ui_encd_val = 1;
					cursor_enter = 1;
				}
				break;
			default:
				switch (cursor_enter)
				{
				case 0:
					menu_state = _MAIN;
					menu_str_inc = 0;
				}
				break;
			}
			///////
			clear_graphic ();
			menu_cursor_invisible (ui_last_encd_val);
			menu_cursor (ui_encd_val);
			menu_update ();
			ui_last_encd_val = ui_encd_val;
		}
	}
	else
	{
		if (button_state) button_state = 0;
	}
}

/******************************************************************************************/

void bluetooth_at_cmd (char *cmd)
{
	uint32_t ln = 0;
	while (cmd[ln] != 0) ln++;
	HAL_UART_Transmit(&huart4, (uint8_t*)cmd, ln, 1000);
}


void can_config ()
{
	FDCAN_FilterTypeDef  sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0xA00;
	sFilterConfig.FilterID2 = 0xA00;

	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) Error_Handler();
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) Error_Handler();

	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x000;//0x202
	sFilterConfig.FilterID2 = 0x000;

	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) Error_Handler();
	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) Error_Handler();

	  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//	  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY, 0);
	  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
	  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_TX_FIFO_EMPTY, 0);

	  can1_tx_header.Identifier = 0x300;
	  can1_tx_header.IdType = FDCAN_STANDARD_ID;
	  can1_tx_header.TxFrameType = FDCAN_DATA_FRAME;
	  can1_tx_header.DataLength = FDCAN_DLC_BYTES_5;
	  can1_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	  can1_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	  can1_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	  can1_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	  can1_tx_header.MessageMarker = 0;

	  can2_tx_header.Identifier = 0x201;
	  can2_tx_header.IdType = FDCAN_STANDARD_ID;
	  can2_tx_header.TxFrameType = FDCAN_DATA_FRAME;
	  can2_tx_header.DataLength = FDCAN_DLC_BYTES_5;
	  can2_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	  can2_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	  can2_tx_header.FDFormat = FDCAN_FD_CAN;
	  can2_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	  can2_tx_header.MessageMarker = 0;
}

void can_wait_transmit (FDCAN_HandleTypeDef *hfdcan)
{
	uint32_t time_out = HAL_GetTick();
	while (HAL_FDCAN_GetTxFifoFreeLevel (hfdcan) == 0)
	{
		if (HAL_GetTick() - time_out >= 1000) break;
	}
}


void wheels_control_send_speed ()
{
	int16_t data_temp[3];
	if (HAL_FDCAN_GetTxFifoFreeLevel (&hfdcan2) > 0)
	{
		can2_tx_data[0] = _SET_VX;
		data_temp[0] = manual.vx * 10;
		data_temp[1] = manual.vy * 10;
		data_temp[2] = manual.v_yaw * 10;
		can2_tx_header.Identifier = 0x201;
		can2_tx_header.DataLength = FDCAN_DLC_BYTES_7;
		for (uint8_t i = 0; i < 3; i++)
		{
			can2_tx_data[i*2+1] = (data_temp[i] >> 8) & 0xff;
			can2_tx_data[i*2+2] = data_temp[i] & 0xff;
		}
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &can2_tx_header, can2_tx_data) == HAL_OK)
		{
			can2_fifo_state = _FULL;
		}
	}
}

void robot_gotoxy (double x, double y, double yaw)
{
	int16_t data_temp[3];
	if (HAL_FDCAN_GetTxFifoFreeLevel (&hfdcan2) > 0)
	{
		can2_tx_data[0] = _SET_X;
		data_temp[0] = x * 10;
		data_temp[1] = y * 10;
		data_temp[2] = yaw * 10;
		can2_tx_header.Identifier = 0x201;
		can2_tx_header.DataLength = FDCAN_DLC_BYTES_7;
		for (uint8_t i = 0; i < 3; i++)
		{
			can2_tx_data[i*2+1] = (data_temp[i] >> 8) & 0xff;
			can2_tx_data[i*2+2] = data_temp[i] & 0xff;
		}
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &can2_tx_header, can2_tx_data) == HAL_OK)
		{
			can2_fifo_state = _FULL;
		}
	}
}

void can_reset_origin_robot ()
{
	if (HAL_FDCAN_GetTxFifoFreeLevel (&hfdcan2) > 0)
	{
		can2_tx_data[0] = _RESET_ORIGIN;
		can2_tx_header.Identifier = 0x201;
		can2_tx_header.DataLength = FDCAN_DLC_BYTES_1;
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &can2_tx_header, can2_tx_data) == HAL_OK)
		{
			can2_fifo_state = _FULL;
		}
	}
}

void can_reset_yaw_robot ()
{
	if (HAL_FDCAN_GetTxFifoFreeLevel (&hfdcan2) > 0)
	{
		can2_tx_data[0] = _RESET_YAW;
		can2_tx_header.Identifier = 0x201;
		can2_tx_header.DataLength = FDCAN_DLC_BYTES_1;
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &can2_tx_header, can2_tx_data) == HAL_OK)
		{
			can2_fifo_state = _FULL;
		}
	}
}

void can_set_coord_robot (double x, double y, double yaw)
{
	int16_t data_temp[3];
	if (HAL_FDCAN_GetTxFifoFreeLevel (&hfdcan2) > 0)
	{
		can2_tx_data[0] = _SET_COORD;
		data_temp[0] = x * 10;
		data_temp[1] = y * 10;
		data_temp[2] = yaw * 10;
		can2_tx_header.Identifier = 0x201;
		can2_tx_header.DataLength = FDCAN_DLC_BYTES_7;
		for (uint8_t i = 0; i < 3; i++)
		{
			can2_tx_data[i*2+1] = (data_temp[i] >> 8) & 0xff;
			can2_tx_data[i*2+2] = data_temp[i] & 0xff;
		}
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &can2_tx_header, can2_tx_data) == HAL_OK)
		{
			can2_fifo_state = _FULL;
		}
	}
}

void can_lock_move ()
{
	if (HAL_FDCAN_GetTxFifoFreeLevel (&hfdcan2) > 0)
	{
		can2_tx_data[0] = _LOCK_MOVE;
		can2_tx_header.Identifier = 0x201;
		can2_tx_header.DataLength = FDCAN_DLC_BYTES_1;
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &can2_tx_header, can2_tx_data) == HAL_OK)
		{
			can2_fifo_state = _FULL;
		}
	}
}

void can_read_request (uint8_t cmd)
{
	if (can2_fifo_state == _IDLE && wait_request == 0)
	{
		can2_tx_header.Identifier = 0x201;
		can2_tx_header.DataLength = FDCAN_DLC_BYTES_1;
		can2_tx_data[0] = cmd;
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &can2_tx_header, can2_tx_data) == HAL_OK)
		{
			wait_request = cmd;
			can2_fifo_state = _FULL;
		}
	}
}

void can_set_thrower (_Bool ch, uint16_t rpm_L, uint16_t rpm_R)
{
	if (HAL_FDCAN_GetTxFifoFreeLevel (&hfdcan1) > 0)
	{
		uint32_t addr;
		if (ch == _THROWER1) addr = 0x0300;
		else addr = 0x0302;
		can1_tx_header.Identifier = addr;
		can1_tx_header.DataLength = FDCAN_DLC_BYTES_5;
		can1_tx_data[0] = _SET_RPM;
		can1_tx_data[1] = (rpm_L >> 8) & 0xFF;
		can1_tx_data[2] = rpm_L & 0xFF;
		can1_tx_data[3] = (rpm_R >> 8) & 0xFF;
		can1_tx_data[4] = rpm_R & 0xFF;

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can1_tx_header, can1_tx_data);
	}
}


int8_t can_set_rotator (uint8_t reg, double value)
{
	int8_t status = 0;
	_Bool error_input = 0;
	if (reg == _STEER_R)
	{
		if (value > 90 || value < -34) error_input = 1;
	}
	else if (reg == _STEER_L)
	{
		if (value < -90 || value > 34) error_input = 1;
	}
	else if (reg == _STEER_P)
	{
		if (value < 80 || value > 280) error_input = 1;
	}
	if (!error_input)
	{
		if (HAL_FDCAN_GetTxFifoFreeLevel (&hfdcan1) > 0)
		{
			int16_t temp = value * 100;
			can1_tx_header.Identifier = 0x3A5;
			can1_tx_header.DataLength = FDCAN_DLC_BYTES_3;
			can1_tx_data[0] = reg;
			can1_tx_data[1] = (temp >> 8) & 0xFF;
			can1_tx_data[2] = temp & 0xFF;

			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can1_tx_header, can1_tx_data) == HAL_OK)
				status = 1;
			else status = -1;
		}
	}
	return status;
}

/******************************************************************************************/

void servo_init ()
{
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim4);

#ifdef ER
	servo1.offset = -4.0;
	servo2.offset = -7.0;
	servo3.offset = 0;
	servo4.offset = 0;
	servo5.offset = 0;
	servo6.offset = 0;
	servo7.offset = -2.5;
	servo8.offset = 5;
#else
	servo1.offset = 5.0;
	servo2.offset = 0;
	servo3.offset = 0;
	servo4.offset = 0;
	servo5.offset = 0;
	servo6.offset = 5.0;
	servo7.offset = 0;
	servo8.offset = 0;
#endif
}

void set_servo (SERVO_t *hservo, double angle)
{
	if (hservo == &servo7 || hservo == &servo8)
		hservo->pwm = ((angle + hservo->offset) / 55.0 * 260.0) + 500.0;
	else
	{
#ifdef KST
		hservo->pwm =  ((angle + hservo->offset) * 5.55555555555555555556 + 999);
#else
		hservo->pwm =  ((angle + hservo->offset) * 10.6666666666666666667 + 499);
#endif
	}
}

void set_gripper (uint8_t hgrip, double z, _Bool ring)
{
	double b, alpha1, alpha2, alpha;
	b = sqrt (z * z + dp * dp);
	alpha1 = asin (dp / b);
	alpha2 = acos ((L1 * L1 + b * b - L2 * L2) / (2.0 * L1 * b));
	alpha = (alpha1 + alpha2) * RAD2DEG;
	if (hgrip == _GRIPPER1)
	{
		set_servo (&servo1, alpha);
		set_servo (&servo3, (ring == _GRIP)? 120 : 0);
	}
	else
	{
		set_servo (&servo2, alpha);
		set_servo (&servo4, (ring == _GRIP)? 140 : 0);
	}
}

/******************************************************************************************/

void generate_trajectory (double x0, double y0, double x1, double y1)
{
	if (!robot_move_flag)
	{
		double dx, dy;
		dx = x1 - x0;
		dy = y1 - y0;
		dest = sqrt (dx * dx + dy * dy);
//		if (dx > 0 && dy > 0)
		{
			theta_des = atan2 (dy, dx);
		}
//		else if (dx > 0 && dy <= 0)
//		{
//			theta_des = atan2 (-dy, dx);
//		}
//		else if (dx <= 0 && dy > 0)
//		{
//			theta_des = atan2 (dy, -dx);
//		}
//		else if (dx <= 0 && dy <= 0)
//		{
//			theta_des = atan2 (-dy, -dx);
//		}
		x_start = x0;
		y_start = y0;

		vel = 0;
		acc = 0;
		dis = 0;
		last_dis = 0;

		TIM7->CNT = 0;
		TIM7->ARR = 99;
		HAL_TIM_Base_Start_IT(&htim7);
		robot_move_flag = 1;
		for (uint8_t i = 0; i < 7; i++)
		{
			ts[i] = 0;
		}
	}

}

void trapezoid_motion (double acc_t, double dcc_t)
{
	if (robot_move_flag)
	{
		vel += acc;
		dis += vel;
		ts[trpz_state]++;
		switch (trpz_state)
		{
		case 0:
			acc = max_acc;
			if (fabs (dis - last_dis) >= acc_t * dest)
			{
				last_dis = dis;
				trpz_state = 1;
			}
			break;
		case 1:
			acc = 0;
			if (fabs (dis - last_dis) >= (1.0 - acc_t - dcc_t) * dest)
			{
				last_dis = dis;
				trpz_state = 2;
			}
			break;
		case 2:
			acc = -max_acc;
			if (fabs (dis - last_dis) >= dcc_t * dest)
			{
				last_dis = dis;
				trpz_state = 0;
				vel = 0;
				acc = 0;
				robot_move_flag = 0;
				HAL_TIM_Base_Stop_IT(&htim7);
			}
			break;
		}

		d_robot = dis;
		x_robot = d_robot * cos (theta_des) + x_start;
		y_robot = d_robot * sin (theta_des) + y_start;
	}
}

void s_curve (double j_time, double j0_time, double c_time)
{
	if (robot_move_flag)
	{
		ts[s_curve_state]++;
		acc += j;
		vel += acc;
		dis += vel;
		switch (s_curve_state)
		{
		case 0:
			j = max_j;
			if (fabs (dis - last_dis) >= j_time * dest)
			{
				last_dis = dis;
				s_curve_state = 1;
			}
			break;
		case 1:
			j = 0;
			if (fabs (dis - last_dis) >= j0_time * dest)
			{
				last_dis = dis;
				s_curve_state = 2;
			}
			break;
		case 2:
			j = -max_j;
			if (fabs (dis - last_dis) >= j_time * dest)
			{
				last_dis = dis;
				s_curve_state = 3;
			}
			break;
		case 3:
			j = 0;
			acc = 0;
			if (fabs (dis - last_dis) >= c_time * dest)
			{
				last_dis = dis;
				s_curve_state = 4;
			}
			break;
		case 4:
			j = -max_j;
			if (fabs (dis - last_dis) >= j_time * dest)
			{
				last_dis = dis;
				s_curve_state = 5;
			}
			break;
		case 5:
			j = 0;
			if (fabs (dis - last_dis) >= j0_time * dest)
			{
				last_dis = dis;
				s_curve_state = 6;
			}
			break;
		case 6:
			j = max_j;
			if (fabs (dis - last_dis) >= j_time * dest)
			{
				last_dis = dis;
				s_curve_state = 0;
				j = 0;
				vel = 0;
				acc = 0;
				robot_move_flag = 0;
				HAL_TIM_Base_Stop_IT(&htim7);
			}
			break;
		}
		d_robot = dis;
		x_robot = d_robot * cos (theta_des) + x_start;
		y_robot = d_robot * sin (theta_des) + y_start;
	}
}

/******************************************************************************************/

void picker_rot_motion ()
{
	if (ring_pic.arm_degree != ring_pic.arm_target_deg)
	{
		if (HAL_GetTick() - ring_pic.timer > ring_pic.delay_speed)
		{
			ring_pic.timer = HAL_GetTick();
			if (ring_pic.arm_degree < ring_pic.arm_target_deg)
			{
				if ((ring_pic.arm_degree + ring_pic.rot_inc) > ring_pic.arm_target_deg)
				{
					ring_pic.arm_degree = ring_pic.arm_target_deg;
				}
				else
				{
					ring_pic.arm_degree += ring_pic.rot_inc;
				}
			}
			else
			{
				if ((ring_pic.arm_degree - ring_pic.rot_inc) < ring_pic.arm_target_deg)
				{
					ring_pic.arm_degree = ring_pic.arm_target_deg;
				}
				else
				{
					ring_pic.arm_degree -= ring_pic.rot_inc;
				}
			}
			can_set_rotator(_STEER_P, ring_pic.arm_degree);
		}
	}
	else
	{
		if (ring_pic.rot_move_flag)
		{
			if (!ring_pic.start_delay_flag)
			{
				ring_pic.start_delay_flag = 1;
				can_wait_transmit (&hfdcan1);
				can_set_rotator(_STEER_P, ring_pic.arm_degree);
				ring_pic.timer = HAL_GetTick();
			}
			if (HAL_GetTick() - ring_pic.timer >= ring_pic.delay)
			{
				ring_pic.start_delay_flag = 0;
				ring_pic.rot_move_flag = 0;
			}
		}
	}
}

void picker_set_angle (double deg, uint32_t t_delay)
{
	ring_pic.arm_target_deg = deg;
	if (ring_pic.arm_target_deg != ring_pic.arm_degree)
	{
		ring_pic.delay_speed = t_delay;
		if (!ring_pic.rot_move_flag)
		{
			ring_pic.rot_move_flag = 1;
		}
	}
}

void thrower_rot_motion (THROWER_ROT_t *htrot)
{
	if (htrot->move_flag)
	{
		if (!htrot->start_flag)
		{
			htrot->start_flag = 1;
			htrot->timer = HAL_GetTick();
		}
		if (HAL_GetTick() - htrot->timer < htrot->delay)
		{
			if (!htrot->toggle)
			{
				htrot->toggle = 1;
				can_wait_transmit (&hfdcan1);
				can_set_rotator((htrot == &trot_R)? _STEER_R : _STEER_L, htrot->theta);
			}
		}
		else
		{
			htrot->start_flag = 0;
			htrot->toggle = 0;
			htrot->move_flag = 0;
		}
	}
}

void thrower_set_angle (THROWER_ROT_t *htrot, double theta)
{
	double d_theta;
	htrot->theta = theta;
	if (htrot->last_theta != htrot->theta)
	{
		d_theta = fabs (htrot->last_theta - htrot->theta);
		htrot->delay = (uint32_t)(d_theta * 9.5);
		htrot->last_theta = htrot->theta;
		htrot->move_flag = 1;
	}
}

void lift_ring_motion (LIFT_RING_t *hlift)
{
	if (hlift->move_flag)
	{
		if (!hlift->start_flag)
		{
			hlift->start_flag = 1;
			hlift->timer = HAL_GetTick();
		}
		if (HAL_GetTick() - hlift->timer < hlift->delay)
		{
			double b, alpha1, alpha2, alpha;
			b = sqrt (hlift->z * hlift->z + dp * dp);
			alpha1 = asin (dp / b);
			alpha2 = acos ((L1 * L1 + b * b - L2 * L2) / (2.0 * L1 * b));
			alpha = (alpha1 + alpha2) * RAD2DEG;
			set_servo((hlift == &lift_R)? &servo1 : &servo2, alpha);
		}
		else
		{
			hlift->start_flag = 0;
			hlift->move_flag = 0;
		}
		if (HAL_GetTick() - hlift->timer < 100)
			BUZZER_GPIO_Port->BSRR = BUZZER_Pin;
		else
			BUZZER_GPIO_Port->BSRR = BUZZER_Pin<<16;
	}
}

void set_lift_ring_z (LIFT_RING_t *hlift, double z)
{
	double dz;
	hlift->z = z;
	if (hlift->last_z != hlift->z)
	{
		dz = fabs (hlift->last_z - hlift->z);
		hlift->delay = (uint32_t)(dz * 3.5);
		hlift->last_z = hlift->z;
		hlift->move_flag = 1;
	}
}

void grip_ring_motion (GRIPPER_t *hgrip)
{
	if (hgrip->move_flag)
	{
		if (!hgrip->start_flag)
		{
			hgrip->start_flag = 1;
			hgrip->timer = HAL_GetTick();
		}
		if (HAL_GetTick() - hgrip->timer < hgrip->delay)
		{
			if (hgrip == &grip_R) set_servo (&servo3, (hgrip->state == _GRIP)? 117 : 0);
			else set_servo (&servo4, (hgrip->state == _GRIP)? 134 : 0);
		}
		else
		{
			hgrip->start_flag = 0;
			hgrip->move_flag = 0;
			if (hgrip->state == _GRIP)
			{
				hgrip->ring_empty = 0;
			}
			else
			{
				hgrip->ring_empty = 1;
				if (hgrip == &grip_R) pusher_R.ring_empty = 0;
				else pusher_L.ring_empty = 0;
			}
		}
		if (HAL_GetTick() - hgrip->timer < 50)
			BUZZER_GPIO_Port->BSRR = BUZZER_Pin;
		else
			BUZZER_GPIO_Port->BSRR = BUZZER_Pin<<16;
	}
}

void set_grip_ring (GRIPPER_t *hgrip, _Bool state)
{
	hgrip->state = state;
	if (hgrip->last_state != hgrip->state)
	{
		hgrip->last_state = hgrip->state;
		hgrip->move_flag = 1;
	}
}



void ring_pusher_motion (PUSHER_t *hpusher)
{
	if (hpusher->move_flag)
	{
		if (!hpusher->start_flag)
		{
			hpusher->start_flag = 1;
			hpusher->timer = HAL_GetTick();
		}
		if (HAL_GetTick() - hpusher->timer < hpusher->delay)
		{
			set_servo((hpusher == &pusher_R)? &servo7 : &servo8, 68);
		}
		else
		{
			if (hpusher == &pusher_R)
			{
				set_servo(&servo7, 0);
				ready_R = 1;
				ring_order_R++;
				ring_order_R %= 5;
			}
			else
			{
				set_servo(&servo8, 0);
				ready_L = 1;
				ring_order_L++;
				ring_order_L %= 5;
			}
			hpusher->move_flag = 0;
			hpusher->start_flag = 0;
			hpusher->ring_empty = 1;
		}
	}
}

void push_ring (PUSHER_t *hpusher)
{
	hpusher->move_flag = 1;
}

void trial_lempar ()
{
	//--------------------------------------------------------------------------------------
	switch (state_picker)
	{
	case 0://putar
		if (grip_R.ring_empty || (pusher_L.ring_empty && !grip_L.ring_empty))
		{
			if (!lift_L.move_flag && !lift_R.move_flag)
			{
				if (pemutar_start)
				{
					pemutar_start = 0;
					picker_set_angle (250.0, 20);
				}
				else
				{
					picker_set_angle (250.0, 10);
				}
				state_picker = 1;
			}
		}
//		else state_picker = 4;
		break;
	case 1://turun
		if (!ring_pic.rot_move_flag)
		{
			if (!pusher_R.ring_empty)
			{
				if (!launch_R_prepare) change_param_tR = 1;
			}
			if (grip_R.ring_empty && !ring_is_empty)//
			{
				set_lift_ring_z(&lift_R, upper_ring_stack - ((10 - ring_stack) * 15));
			}
			//----------------------------------------------------------------
			if (pusher_L.ring_empty && !grip_L.ring_empty)
			{
				if (!trot_L.move_flag && place_L_permission)
				{
					place_L_permission = 0;
					set_lift_ring_z(&lift_L, -190);
				}
			}
			//----------------------------------------------------------------
			if (!lift_R.move_flag && !lift_L.move_flag && !trot_L.move_flag)
			{
				if (grip_R.ring_empty && !ring_is_empty)//
				{
					ring_stack--;
					if (ring_stack == 0)
					{
						ring_stack = 10;
//						ring_is_empty = 1;
					}
				}
				state_picker = 2;
			}
		}
		break;
	case 2://ambil & letakkan
		if (grip_R.ring_empty && !ring_is_empty)
		{
			if (!lift_R.move_flag)
			{
				set_grip_ring (&grip_R, _GRIP);
			}
		}
		//----------------------------------------------------------------
		if (pusher_L.ring_empty && !grip_L.ring_empty)
		{
			if (!lift_L.move_flag)
			{
				set_grip_ring (&grip_L, _RELEASE);
			}
		}
		//----------------------------------------------------------------
		if (!lift_R.move_flag && !lift_L.move_flag)
		{
			state_picker = 3;
		}
		break;
	case 3://angkat
		if (grip_L.last_ring_empty != grip_L.ring_empty)
		{
			grip_L.last_ring_empty = grip_L.ring_empty;
			set_lift_ring_z(&lift_L, -25);
		}
		if (grip_R.last_ring_empty != grip_R.ring_empty)
		{
			grip_R.last_ring_empty = grip_R.ring_empty;
			set_lift_ring_z(&lift_R, -25);
		}
		if (!grip_R.move_flag && !grip_L.move_flag)
		{
			state_picker = 4;
		}
		break;
	//--------------------------------------------------------------------------------------
	case 4://putar
		if (grip_L.ring_empty || (pusher_R.ring_empty && !grip_R.ring_empty))
		{
			if (!lift_L.move_flag && !lift_R.move_flag)
			{
				picker_set_angle (105.0, 10);
				state_picker = 5;
			}
		}
		break;
	case 5://turun
		if (!ring_pic.rot_move_flag)
		{
			if (!pusher_L.ring_empty)
			{
				if (!launch_L_prepare) change_param_tL = 1;
			}
			if (grip_L.ring_empty && !ring_is_empty)
			{
				set_lift_ring_z(&lift_L, upper_ring_stack - ((10 - ring_stack) * 15));
			}
			//----------------------------------------------------------------
			if (pusher_R.ring_empty && !grip_R.ring_empty)
			{
				if (!trot_R.move_flag && place_R_permission)
				{
					place_R_permission = 0;
					set_lift_ring_z(&lift_R, -185);
				}
			}
			//----------------------------------------------------------------
			if (!lift_R.move_flag && !lift_L.move_flag && !trot_R.move_flag)
			{
				if (grip_L.ring_empty && !ring_is_empty)
				{
					ring_stack--;
					if (ring_stack == 0)
					{
						ring_stack = 10;
//						ring_is_empty = 1;
					}
				}
				state_picker = 6;
			}
		}
		break;
	case 6://ambil & letakkan
		if (grip_L.ring_empty && !ring_is_empty)
		{
			if (!lift_L.move_flag)
			{
				set_grip_ring (&grip_L, _GRIP);
			}
		}
		//----------------------------------------------------------------
		if (pusher_R.ring_empty && !grip_R.ring_empty)
		{
			if (!lift_R.move_flag)
			{
				set_grip_ring (&grip_R, _RELEASE);
			}
		}
		//----------------------------------------------------------------
		if (!lift_R.move_flag && !lift_L.move_flag)
		{
			state_picker = 7;
		}
		break;
	case 7://angkat
		if (grip_L.last_ring_empty != grip_L.ring_empty)
		{
			grip_L.last_ring_empty = grip_L.ring_empty;
			set_lift_ring_z(&lift_L, -25);
		}
		if (grip_R.last_ring_empty != grip_R.ring_empty)
		{
			grip_R.last_ring_empty = grip_R.ring_empty;
			set_lift_ring_z(&lift_R, -25);
		}
		if (!grip_R.move_flag && !grip_L.move_flag)
		{
			state_picker = 0;
		}
		break;
	}
	//--------------------------------------------------------------------------------------
	if (pusher_R.ring_empty)
	{
		thrower_set_angle (&trot_R, -25);
		if (ready_R)
		{
			ready_R = 0;
			place_R_permission = 1;
//			if (state_picker == 0) state_picker = 4;
			state_picker = 4;
		}
	}
	else
	{
		if (change_param_tR)
		{
			change_param_tR = 0;
			uint16_t rpm_L, rpm_R;
			double diff = 0.7;
			uint8_t r_order_temp = ring_order_R;
			if (ring_order_R == 4) r_order_temp = 2;

			if (r_order_temp == 2)
			{
				diff = 0.9;
			}
			rpm_L = power_pelempar[r_order_temp*2];
			rpm_R = power_pelempar[r_order_temp*2] * diff;
			can_wait_transmit (&hfdcan1);
			can_set_thrower (_THROWER1, rpm_L, rpm_R);
			thrower_set_angle (&trot_R, angle_pelempar[r_order_temp*2]);
//			set_servo(&servo7, 25);
			launch_R_prepare = 1;
		}
	}
	if (pusher_L.ring_empty)
	{
		thrower_set_angle (&trot_L, 23);
		if (ready_L)
		{
			ready_L = 0;
			place_L_permission = 1;
			state_picker = 0;
		}
	}
	else
	{
		if (change_param_tL)
		{
			change_param_tL = 0;
			uint16_t rpm_L, rpm_R;
			double diff = 0.7;
			uint8_t r_order_temp = ring_order_L;
			if (ring_order_L == 4) r_order_temp = 1;

			if (r_order_temp == 1)
			{
				diff = 0.91;
			}
			else if (r_order_temp == 2) diff = 0.92;
			rpm_L = power_pelempar[r_order_temp*2 + 1];
			rpm_R = power_pelempar[r_order_temp*2 + 1] * diff;
			can_wait_transmit (&hfdcan1);
			can_set_thrower (_THROWER2, rpm_L, rpm_R);
			thrower_set_angle (&trot_L, angle_pelempar[r_order_temp*2 + 1]);
//			set_servo(&servo8, 25);
			launch_L_prepare = 1;
		}
	}
	//--------------------------------------------------------------------------------------
	if (launch_R)
	{
		if (launch_R_permission)
		{
			launch_R = 0;
			launch_R_permission = 0;
			push_ring (&pusher_R);
		}
	}
	if (launch_L)
	{
		if (launch_L_permission)
		{
			launch_L = 0;
			launch_L_permission = 0;
			push_ring (&pusher_L);
		}
	}
	//--------------------------------------------------------------------------------------
	picker_rot_motion ();
	thrower_rot_motion (&trot_R);
	thrower_rot_motion (&trot_L);
	lift_ring_motion (&lift_R);
	lift_ring_motion (&lift_L);
	grip_ring_motion (&grip_R);
	grip_ring_motion (&grip_L);
	ring_pusher_motion (&pusher_R);
	ring_pusher_motion (&pusher_L);
	//--------------------------------------------------------------------------------------
//	if (thrower_off)
//	{
//		thrower_off = 0;
//		can_wait_transmit (&hfdcan1);
//		can_set_thrower (_THROWER1, 0, 0);
//		can_wait_transmit (&hfdcan1);
//		can_set_thrower (_THROWER2, 0, 0);
//	}
	//--------------------------------------------------------------------------------------
	if (launch_R_prepare)
	{
		if (!trot_R.move_flag)
		{
			launch_R_prepare = 0;
			launch_R_permission = 1;
		}
	}
	if (launch_L_prepare)
	{
		if (!trot_L.move_flag)
		{
			launch_L_prepare = 0;
			launch_L_permission = 1;
		}
	}
}
_Bool last_button = 0;
_Bool stop_button = 0;
void kalibrasi_lempar()
{
	uint32_t speed = 2.7475*actual_distance + 1165.4; // 2,7475x + 1165,4
	if (speed > 2500) speed = 2500;

	if(com_blt.rthrower_speed != lst_speed){
		lst_speed = com_blt.rthrower_speed;
		can_wait_transmit(&hfdcan1);
		//can_set_thrower (_THROWER1, com_blt.rthrower_speed, com_blt.rthrower_speed*0.85);
		can_set_thrower (_THROWER1, speed, speed*0.85);
	}

	if(last_button){
		last_button = 0;
		set_servo (&servo7, 65);
		HAL_Delay(300);
		set_servo (&servo7, 15);
	}
	if(stop_button){
		can_wait_transmit(&hfdcan1);
		can_set_thrower (_THROWER1, 0, 0);
		stop_button =0;
	}
	if (RR_tail)
	{
		 if (!RR_grip_prepare) {
			 RR_grip_prepare = 1;
			 HAL_Delay(500);
			can_wait_transmit(&hfdcan1);
			 can_set_rotator (_STEER_P, 270);
		 }
	}

}

void trial_lempar_RR ()
{
//	uint32_t speed = 0.00000001*pow(actual_distance,4) - 0.00002*pow(actual_distance,3)+ 0.0114*pow(actual_distance,2) + 0.7854*actual_distance + 1270.1; //1E-08x4 - 2E-05x3 + 0,0114x2 + 0,7854x + 1270,1
//	if (speed > 2500) speed = 2500;

	if (RR_ambil)
	{
		if (RR_ring_ready == 0)
		{
			manual.vx = 0;
			manual.vy = 0;
			manual.v_yaw = 0;
			wheels_control_send_speed();
			can_wait_transmit(&hfdcan2);
			can_lock_move();
			for (uint8_t i = 0; i < 2; i++)
			{
//				can_wait_transmit(&hfdcan1);
//				can_set_rotator (_STEER_P, 270);
//				HAL_Delay(700);
				set_servo (&servo6, 133);
				HAL_Delay(100);
				set_gripper (_GRIPPER1, -70 - ((7-ring_stack)*15), _RELEASE);
				HAL_Delay(300);
				set_gripper (_GRIPPER1, -70 - ((7-ring_stack)*15), _GRIP);
				HAL_Delay(500);
				set_gripper (_GRIPPER1, -45, _GRIP);
				HAL_Delay(450);
				can_wait_transmit(&hfdcan1);
				can_set_rotator (_STEER_P, 82);
				if (i == 0)
				{
					HAL_Delay(1250);
					set_gripper (_GRIPPER1, -95, _GRIP);
					HAL_Delay(300);
					set_gripper (_GRIPPER1, -95, _RELEASE);
					HAL_Delay(500);
					set_gripper (_GRIPPER1, -45, _RELEASE);
					HAL_Delay(100);
					can_wait_transmit(&hfdcan1);
					can_set_rotator (_STEER_P, 268);
					HAL_Delay(700);
				}
				RR_ring_ready++;
				ring_stack--;
				if (ring_stack == 0) {
					ring_stack = 5;
					break;
				}

			}
//			lst_speed = com_blt.rthrower_speed;
			set_servo (&servo6, 120);
			//nyala
			can_wait_transmit(&hfdcan1);
			can_set_thrower (_THROWER1, com_blt.rthrower_speed*0.75, com_blt.rthrower_speed);
		}
		RR_ambil = 0;
		if(com_blt.isEn1){
			 // USER CODE BEGIN ENEMY 3
			 com_blt.isEn1 = 0;
			 blt_tx_buff[0]= 0; blt_tx_buff[1]= 8; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
			 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
		 }
		 if(com_blt.isEn2){
			 // USER CODE BEGIN ENEMY 2
			 com_blt.isEn2 = 0;
			 blt_tx_buff[0]= 0;blt_tx_buff[1]= 16; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
			 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
		 }
		 if(com_blt.isEn3){
			 // USER CODE BEGIN ENEMY 1
			 com_blt.isEn3 = 0;
			 blt_tx_buff[0]= 0;blt_tx_buff[1]= 32;blt_tx_buff[2]= 0;blt_tx_buff[3]= 0;blt_tx_buff[4]= 0;
			 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN); //mengirimkan state ke android
		 }
		 if(com_blt.isDo1){
			 // USER CODE BEGIN DOMAIN 1
			 com_blt.isDo1 = 0;
			 blt_tx_buff[0]= 8; blt_tx_buff[1]= 0; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
			 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
		 }
		 if(com_blt.isDo2){
			 // USER CODE BEGIN DOMAIN 2
			 com_blt.isDo2 = 0;
			 blt_tx_buff[0]= 16; blt_tx_buff[1]= 0; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
			 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
		 }
		 if(com_blt.isAng1){
			 // USER CODE BEGIN ANGKOR 1
			 com_blt.isAng1 = 0;
			 blt_tx_buff[0]= 64; blt_tx_buff[1]= 0; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
			 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN); //mengirimkan state ke android
		 }
		 if(com_blt.isAng2){
			 // USER CODE BEGIN ANGKOR 2
			 com_blt.isAng2 = 0;
			 blt_tx_buff[0]= 128; blt_tx_buff[1]= 0; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
			 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
		 }
		 if(com_blt.isAng3){
			 // USER CODE BEGIN ANGKOR 3
			 com_blt.isAng3 = 0;
			 blt_tx_buff[0]= 0; blt_tx_buff[1]= 1; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
			 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
		 }
		 if(com_blt.isAng4){
			 // USER CODE BEGIN ANGKOR 4
			 com_blt.isAng4 = 0;
			 blt_tx_buff[0]= 0; blt_tx_buff[1]= 2; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
			 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
		 }
		 if(com_blt.isAng5){
			 // USER CODE BEGIN ANGKOR 5
			 com_blt.isAng5 = 0;
			 blt_tx_buff[0]= 0; blt_tx_buff[1]= 4; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
			 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
		 }
	}
//	if(RR_ring_ready > 0)
//	{
//		if(com_blt.rthrower_speed != lst_speed){
//			can_wait_transmit(&hfdcan1);
//			can_set_thrower (_THROWER1, com_blt.rthrower_speed, com_blt.rthrower_speed*0.5);
//		}
//
//	}

	if(com_blt.rthrower_speed != lst_speed){
		lst_speed = com_blt.rthrower_speed;

		// Mengirim kecepatan Real ketika speed diubah
		blt_tx_buff[0]= 0; blt_tx_buff[1]= 128; blt_tx_buff[2]= 0;
		blt_tx_buff[3]= (com_blt.rthrower_speed) & 0xFF; blt_tx_buff[4]= ((com_blt.rthrower_speed)>>8 )& 0xFF;
		HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);

		// Mengirim jarak ketika speed berubah
		blt_tx_buff[0]= 0; blt_tx_buff[1]= 64; blt_tx_buff[2]= 0;
		blt_tx_buff[3]= (int)actual_distance & 0xFF; blt_tx_buff[4]= ((int)actual_distance >> 8) & 0xFF;
		HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);

		can_wait_transmit(&hfdcan1);
		can_set_thrower (_THROWER1, com_blt.rthrower_speed*0.75, com_blt.rthrower_speed);
	}

	if (RR_lempar)
	{
		if (RR_ring_ready != 0)
		{
			manual.vx = 0;
			manual.vy = 0;
			manual.v_yaw = 0;
			wheels_control_send_speed ();
			can_wait_transmit(&hfdcan2);
			can_lock_move();
			set_servo (&servo7, 45);
			HAL_Delay(300);
			set_servo (&servo7, 16);
			RR_ring_ready--;
			if (RR_ring_ready == 1)
			{
				set_servo (&servo6, 133);
				HAL_Delay(100);
				set_gripper (_GRIPPER1, -95, _GRIP);
				HAL_Delay(300);
				set_gripper (_GRIPPER1, -95, _RELEASE);
				HAL_Delay(500);
				set_gripper (_GRIPPER1, -45, _RELEASE);
				HAL_Delay(100);
				can_wait_transmit(&hfdcan1);
				can_set_rotator (_STEER_P, 268);
				HAL_Delay(500);
				set_servo (&servo6, 120);
			}
			if (RR_ring_ready == 0)
			{
				HAL_Delay(500);
				can_wait_transmit(&hfdcan1);
				can_set_thrower (_THROWER1, 0, 0);
			}
		}
		RR_lempar = 0;
	}
	if (HAL_GetTick() - bz_t < 200)
	{
		BUZZER_GPIO_Port->BSRR = BUZZER_Pin;
	}
	else
	{
		BUZZER_GPIO_Port->BSRR = BUZZER_Pin<<16;
	}
	if (RR_tail)
	{
		 if (!RR_grip_prepare) {
			 RR_grip_prepare = 1;
			 HAL_Delay(500);
			can_wait_transmit(&hfdcan1);
			 can_set_rotator (_STEER_P, 268);
		 }
	}
}

void ring_prepare (_Bool thrower_ch)
{
	if (thrower_ch == 0)
	{
		  //persiapan pelempar L
		  if (sa < 180)
		  {
			  can_set_rotator(_STEER_L, 23);
		  }
		  //mengambil ring untuk pelempar R & meletakkan ring untuk pelempar L
		  set_gripper (_GRIPPER1, -25, _RELEASE);
		  set_gripper (_GRIPPER2, -25, _GRIP);
		  beep(300);

		  for (double deg = sa; deg <= 260; deg+=2)
		  {
			  can_set_rotator(_STEER_P, deg);
			  HAL_Delay(6);
		  }
		  can_wait_transmit (&hfdcan1);
		  can_set_rotator(_STEER_P, 260);

		  beep(200);
		  set_gripper (_GRIPPER1, down_grip, _RELEASE);
		  if (sa < 180) set_gripper (_GRIPPER2, -190, _GRIP);
		  HAL_Delay(400);
		  set_gripper (_GRIPPER1, down_grip, _GRIP);
		  if (sa < 180) set_gripper (_GRIPPER2, -190, _RELEASE);
		  HAL_Delay(400);
		  set_gripper (_GRIPPER1, -25, _GRIP);
		  if (sa < 180) set_gripper (_GRIPPER2, -25, _RELEASE);
		  beep(300);
		  if (sa < 180)
		  {
			  can_set_rotator(_STEER_L, angle_pelempar[throw_order]);
			  throw_order++;
			  throw_order %= 8;
			  HAL_Delay(700);
		  }

		  down_grip -= 15.0;
		  if (down_grip <= -195.0) down_grip = -48;
	}
	else
	{
		  //persiapan pelempar R
		  can_set_rotator(_STEER_R, -19);
		  //mengambil ring untuk pelempar L & meletakkan ring untuk pelempar R
		  set_gripper (_GRIPPER1, -25, _GRIP);
		  if (sa < 180) set_gripper (_GRIPPER2, -25, _RELEASE);
		  beep(300);

		  for (double deg = 260; deg >= 110; deg-=2)
		  {
			  can_set_rotator(_STEER_P, deg);
			  HAL_Delay(6);
		  }
		  can_wait_transmit (&hfdcan1);
		  can_set_rotator(_STEER_P, 110);

		  beep(200);
		  set_gripper (_GRIPPER1, -190, _GRIP);
		  set_gripper (_GRIPPER2, down_grip, _RELEASE);
		  HAL_Delay(400);
		  set_gripper (_GRIPPER1, -190, _RELEASE);
		  set_gripper (_GRIPPER2, down_grip, _GRIP);
		  HAL_Delay(400);
		  set_gripper (_GRIPPER1, -25, _RELEASE);
		  set_gripper (_GRIPPER2, -25, _GRIP);
		  beep(300);
		  can_set_rotator(_STEER_R, angle_pelempar[throw_order]);
		  throw_order++;
		  throw_order %= 8;
		  HAL_Delay(700);

		  down_grip -= 15.0;
		  if (down_grip <= -195.0) down_grip = -45;

		  sa = 110;
	}
}

void launch (_Bool thrower_ch)
{
	if (thrower_ch == 0)
	{
		set_servo(&servo7, 67);
		HAL_Delay(300);
		set_servo(&servo7, 0);
	}
	else
	{
		set_servo(&servo8, 67);
		HAL_Delay(300);
		set_servo(&servo8, 0);
	}
}

void tes_pelempar ()
{
	  if (next_bt)
	  {
		  can_set_thrower (_THROWER1, 1000, 1000*0.9);
		  can_wait_transmit (&hfdcan1);
		  can_set_thrower (_THROWER2, 1000, 1000*0.9);
		  can_wait_transmit (&hfdcan1);
		  switch (robot_state)
		  {
		  case 0:
			  ring_prepare (0);
			  launch (1);
			  robot_state = 1;
			  break;
		  case 1:
			  ring_prepare (1);
			  launch (0);
			  robot_state = 0;
			  break;
		  }
		  next_bt = 0;
	  }
}

void trial ()
{
	  if (grip_state)
	  {
		  if (sa < 180) can_set_rotator(_STEER_L, 22);
		  set_gripper (_GRIPPER1, -25, _RELEASE);
		  set_gripper (_GRIPPER2, -25, _GRIP);
		  beep(300);
		  for (double deg = sa; deg <= 260; deg+=2)
		  {
			  can_set_rotator(_STEER_P, deg);
			  HAL_Delay(6);
		  }
		  beep(200);
		  set_gripper (_GRIPPER1, down_grip, _RELEASE);
		  if (sa < 180) set_gripper (_GRIPPER2, -170, _GRIP);
		  HAL_Delay(300);
		  set_gripper (_GRIPPER1, down_grip, _GRIP);
		  if (sa < 180) set_gripper (_GRIPPER2, -170, _RELEASE);
		  HAL_Delay(400);

		  down_grip -= 15.0;
		  if (down_grip <= -192.0) down_grip = -42;

		  can_set_rotator(_STEER_R, -22);
		  set_gripper (_GRIPPER1, -25, _GRIP);
		  if (sa < 180) set_gripper (_GRIPPER2, -25, _RELEASE);
		  beep(300);
		  for (double deg = 260; deg >= 110; deg-=2)
		  {
			  can_set_rotator(_STEER_P, deg);
			  HAL_Delay(6);
		  }
		  beep(200);
		  set_gripper (_GRIPPER1, -170, _GRIP);
		  set_gripper (_GRIPPER2, down_grip, _RELEASE);
		  HAL_Delay(300);
		  set_gripper (_GRIPPER1, -170, _RELEASE);
		  set_gripper (_GRIPPER2, down_grip, _GRIP);
		  HAL_Delay(400);
		  set_gripper (_GRIPPER1, -25, _RELEASE);
		  set_gripper (_GRIPPER2, -25, _GRIP);

		  down_grip -= 15.0;
		  if (down_grip <= -192.0) down_grip = -42;

		  sa = 110;
		  grip_state = 0;
	  }
	  if (ctrl_state != last_ctrl_state)
	  {
		  last_ctrl_state = ctrl_state;
	  }
	  if (!ctrl_state)
	  {
		  wheels_control_send_speed ();
	  }
	  else
	  {
		  if (HAL_GetTick() - t_rot_thrR > 50)
		  {
			  t_rot_thrR = HAL_GetTick();
			  angle_thrR -= manual.vx / 600;
			  if (angle_thrR > 70) angle_thrR = 70;
			  else if (angle_thrR < -30) angle_thrR = -30;
		  }
		  if (angle_thrR != last_angle_thrR)
		  {
			  last_angle_thrR = angle_thrR;
			  can_set_rotator(_STEER_R, angle_thrR);
		  }
		  if (HAL_GetTick() - t_rot_thrL > 50)
		  {
			  t_rot_thrL = HAL_GetTick();
			  angle_thrL -= manual.v_yaw / 200;
			  if (angle_thrL > 30) angle_thrL = 30;
			  else if (angle_thrL < -70) angle_thrL = -70;
		  }
		  if (angle_thrL != last_angle_thrL)
		  {
			  last_angle_thrL = angle_thrL;
			  can_set_rotator(_STEER_L, angle_thrL);
		  }
	  }

	  //------------------------------------------------------------
	  if (thrower_en)
	  {
		  t_rpm_test = power_pelempar[0];
	  }
	  else
	  {
		  t_rpm_test = 0;
	  }

//	  t_rpm_test = power_pelempar[throw_order];
//	  if (last_t_rpm_test != t_rpm_test)
//	  {
//		  last_t_rpm_test = t_rpm_test;
//		  uint16_t rpm_L, rpm_R;
//		  rpm_L = t_rpm_test;
//		  rpm_R = t_rpm_test * 0.9;
//		  can_set_thrower (_THROWER1, rpm_L, rpm_R);
//	  }

	  if (launch_R)
	  {
		  uint32_t dt = HAL_GetTick() - shoot_timer_R;
		  if (dt < 300)
		  {
			  set_servo(&servo7, 67);
		  }
		  else
		  {
			  set_servo(&servo7, 0);
			  launch_R = 0;
			  HAL_Delay(1000);
			  throw_order++;
			  throw_order %= 4;
		  }
	  }
	  if (launch_L)
	  {
		  uint32_t dt = HAL_GetTick() - shoot_timer_L;
		  if (dt < 300)
		  {
			  set_servo(&servo8, 67);
		  }
		  else
		  {
			  set_servo(&servo8, 0);
			  launch_L = 0;
		  }
	  }
	  if (throw_order != last_throw_order)
	  {
		  last_throw_order = throw_order;
		  uint16_t rpm_L, rpm_R;
		  double diff;

		  if (throw_order == 2) diff = 0.9;
		  else diff = 0.7;
		  rpm_L = power_pelempar[throw_order];
		  rpm_R = power_pelempar[throw_order] * diff;
		  can_set_thrower (_THROWER1, rpm_L, rpm_R);
		  HAL_Delay(100);
		  can_set_rotator(_STEER_R, angle_pelempar[throw_order]);
	  }
}

void com_bluetooth_init(){ // Sulthon
	com_blt.isAng1 = 0;
	com_blt.isAng2 = 0;
	com_blt.isAng3 = 0;
	com_blt.isAng4 = 0;
	com_blt.isAng5 = 0;
	com_blt.isDo1 = 0;
	com_blt.isDo2 = 0;
	com_blt.isDo3 = 0;
	com_blt.isEn1 = 0;
	com_blt.isEn2 = 0;
	com_blt.isEn3 = 0;
	com_blt.lthrower_speed = 0;
	com_blt. rthrower_speed = 1700;
}

/******************************************************************************************/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		TIM2->CCR1 = servo1.pwm;
		TIM2->CCR2 = servo2.pwm;
		TIM2->CCR3 = servo3.pwm;
		TIM2->CCR4 = servo4.pwm;
		SERVO0_GPIO_Port->BSRR = SERVO0_Pin<<16;
		SERVO1_GPIO_Port->BSRR = SERVO1_Pin<<16;
		SERVO2_GPIO_Port->BSRR = SERVO2_Pin<<16;
		SERVO3_GPIO_Port->BSRR = SERVO3_Pin<<16;
	}
	if (htim->Instance == TIM3)
	{
		TIM3->CCR1 = servo5.pwm;
		TIM3->CCR2 = servo6.pwm;
		SERVO4_GPIO_Port->BSRR = SERVO4_Pin<<16;
		SERVO5_GPIO_Port->BSRR = SERVO5_Pin<<16;
	}
	if (htim->Instance == TIM4)
	{
		TIM4->CCR1 = servo7.pwm;
		TIM4->CCR2 = servo8.pwm;
		SERVO6_GPIO_Port->BSRR = SERVO6_Pin<<16;
		SERVO7_GPIO_Port->BSRR = SERVO7_Pin<<16;
	}
	if (htim->Instance == TIM7)
	{
//		trapezoid_motion (0.4, 0.4);
	}
}

/******************************************************************************************/

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) SERVO0_GPIO_Port->BSRR = SERVO0_Pin;
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) SERVO1_GPIO_Port->BSRR = SERVO1_Pin;
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) SERVO2_GPIO_Port->BSRR = SERVO2_Pin;
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) SERVO3_GPIO_Port->BSRR = SERVO3_Pin;
	}
	if (htim->Instance == TIM3)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) SERVO4_GPIO_Port->BSRR = SERVO4_Pin;
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) SERVO5_GPIO_Port->BSRR = SERVO5_Pin;
	}
	if (htim->Instance == TIM4)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) SERVO6_GPIO_Port->BSRR = SERVO6_Pin;
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) SERVO7_GPIO_Port->BSRR = SERVO7_Pin;
	}
}

/******************************************************************************************/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//bluetooth--------------------------------------------------------
		if (huart->Instance == UART4) // Sulthon
		{
			// Jika Berasal dari keydown event
			if((blt_rx_buff[0]&3)==0){
			 if(blt_rx_buff[1]==1){
				 // Button A
				 if(RR_tailgrip){
					 RR_tailgrip = 0;
					 set_servo(&servo6, 120);
				 }
				 else{
					 RR_tailgrip = 1;
					 set_servo(&servo6, 167);
				 }
			 }
			 if(blt_rx_buff[1]==2){
				 // Button B : Menggunakan motor untuk memutar arah gripper lalu servo untuk membuka dan mengambil cincin
				 RR_ambil = 1;
			 }
			 if(blt_rx_buff[1]==4){
				 // Button X : Menggunakan servo untuk melepaskan griper lalu meluncurkan cincin
				 RR_lempar = 1;
			 }
			 if(blt_rx_buff[1]==8){
				 // Button Y

			 }
			 if(blt_rx_buff[1]==16){
				 // Button THUMBR

			 }
			 if(blt_rx_buff[1]==32){
				 // Button THUMBL
				 if(!RR_tail){
					 RR_tail = 1;
					 set_servo(&servo2, 102);
				 }else{
					 RR_tail = 0;
					 set_servo(&servo2, 80);
				 }
			 }
			 if(blt_rx_buff[1]==64){
				 // Button SELECT
				 com_blt.rthrower_speed -= 10;
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 128; blt_tx_buff[2]= 0;
					blt_tx_buff[3]= (com_blt.rthrower_speed) & 0xFF; blt_tx_buff[4]= ((com_blt.rthrower_speed)>>8 )& 0xFF;
					HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);

			 }
			 if(blt_rx_buff[1]==128){
				 // Button START
				 com_blt.rthrower_speed += 10;
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 128; blt_tx_buff[2]= 0;
				 blt_tx_buff[3]= com_blt.rthrower_speed & 0xFF; blt_tx_buff[4]= (com_blt.rthrower_speed>>8 )& 0xFF;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
			 }
			 if(blt_rx_buff[2]==1){
				 // Button R1
				 if(speed_mul == 1){
					 speed_mul = 0.2;
				 }else{
					 speed_mul = 1;
				 }
			 }
			 if(blt_rx_buff[2]==2){
				 // Button L1
				 if(Dir_negation==1){
					 Dir_negation = -1;
				 }else{
					 Dir_negation = 1;
				 }
				 bz_t = HAL_GetTick();

			 }
			 if(blt_rx_buff[2]==4){
				 // Button R2

			 }
			 if(blt_rx_buff[2]==8){
				 // Button L2

			 }
			 if(blt_rx_buff[2]==16){
				// DPAD UP
			 }
			 if(blt_rx_buff[2]==32){
				 // DPAD DOWN
			 }
			 if(blt_rx_buff[2]==64){
				 // DPAD RIGHT
			 }
			 if(blt_rx_buff[2]==128){
				 // DPAD LEFT
			 }
			 HAL_UART_Receive_IT(&huart4, blt_rx_buff, BUFFER_LEN);
			}
			// Jika Berasal dari communication event
			if((blt_rx_buff[0]&1)==1){
			 // Ring Stack Gripper
			 if((blt_rx_buff[0]&0b00000100)==4 ){
				 // USER CODE BEGIN INCREMENT LEFT THROWER SPEED
				 com_blt.lthrower_speed++;
				 if(ring_stack<5){
					 ring_stack++;
				 }
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 64; blt_tx_buff[2]= 0;
				 blt_tx_buff[3]= ring_stack & 0xFF; blt_tx_buff[4]= (ring_stack >> 8) & 0xFF;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
			 }
			 if((blt_rx_buff[0]&0b00001000)==8){
				 // USER CODE BEGIN DECREMENT LEFT THROWER SPEED
				 com_blt.lthrower_speed--;
				 if(ring_stack>1){
					 ring_stack--;
				 }
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 64; blt_tx_buff[2]= 0;
				 blt_tx_buff[3]= ring_stack & 0xFF; blt_tx_buff[4]= (ring_stack >> 8) & 0xFF;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
			 }
			 // Right Thrower Speed
			 if((blt_rx_buff[0]&0b00010000)==16){
				 // USER CODE BEGIN INCREMENT RIGHT THROWER SPEED
				 com_blt.rthrower_speed += 10;
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 128; blt_tx_buff[2]= 0;
				 blt_tx_buff[3]= com_blt.rthrower_speed & 0xFF; blt_tx_buff[4]= (com_blt.rthrower_speed>>8 )& 0xFF;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
			 }
			 if((blt_rx_buff[0]&0b00100000)==32){
				 // USER CODE BEGIN DECREMENT RIGHT THROWER SPEED
				 com_blt.rthrower_speed -= 10;
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 128; blt_tx_buff[2]= 0;
				 blt_tx_buff[3]= com_blt.rthrower_speed & 0xFF; blt_tx_buff[4]= (com_blt.rthrower_speed>>8 )& 0xFF;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
			 }
			 // Domain Pole
			 if((blt_rx_buff[1]&0b00010000)==16){
				 // USER CODE BEGIN DOMAIN 1
				 if(com_blt.isDo1) com_blt.isDo1 = 0;
				 else com_blt.isDo1 = 1;
				 ring_stack=4;
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 64; blt_tx_buff[2]= 0;
				 blt_tx_buff[3]= ring_stack & 0xFF; blt_tx_buff[4]= (ring_stack >> 8) & 0xFF;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
				 blt_tx_buff[0]= 8; blt_tx_buff[1]= 0; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
			 }
			 if((blt_rx_buff[1]&0b00100000)==32){
				 // USER CODE BEGIN DOMAIN 2
				 if(com_blt.isDo2) com_blt.isDo2 = 0;
				 else com_blt.isDo2 = 1;
				 ring_stack=5;
				 stop_button = 1;
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 64; blt_tx_buff[2]= 0;
				 blt_tx_buff[3]= ring_stack & 0xFF; blt_tx_buff[4]= (ring_stack >> 8) & 0xFF;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
				 blt_tx_buff[0]= 16; blt_tx_buff[1]= 0; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
			 }
			 if((blt_rx_buff[1]&0b01000000)==64){
				 // USER CODE BEGIN DOMAIN 3
				 last_button = 1;
				 if(com_blt.isDo3){
					 com_blt.isDo3 = 0;
				 }
				 else{
					 com_blt.isDo3 = 1;
				 }
				 blt_tx_buff[0]= 32; blt_tx_buff[1]= 0; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
			 }
			 // Angkor Pole
			 if((blt_rx_buff[1]&0b10000000)==128){
				 // USER CODE BEGIN ANGKOR 1
				 if(com_blt.isAng1) com_blt.isAng1 = 0;
				 else com_blt.isAng1 = 1;
				 ring_stack=1;
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 64; blt_tx_buff[2]= 0;
				 blt_tx_buff[3]= ring_stack & 0xFF; blt_tx_buff[4]= (ring_stack >> 8) & 0xFF;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
				 blt_tx_buff[0]= 64; blt_tx_buff[1]= 0; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN); //mengirimkan state ke android
			 }
			 if((blt_rx_buff[2]&0b00000001)==1){
				 // USER CODE BEGIN ANGKOR 2
				 if(com_blt.isAng2) com_blt.isAng2 = 0;
				 else com_blt.isAng2 = 1;
				 ring_stack=2;
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 64; blt_tx_buff[2]= 0;
				 blt_tx_buff[3]= ring_stack & 0xFF; blt_tx_buff[4]= (ring_stack >> 8) & 0xFF;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
				 blt_tx_buff[0]= 128; blt_tx_buff[1]= 0; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
			 }
			 if((blt_rx_buff[2]&0b00000010)==2){
				 // USER CODE BEGIN ANGKOR 3
				 if(com_blt.isAng3) com_blt.isAng3 = 0;
				 else com_blt.isAng3 = 1;
				 ring_stack=3;
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 64; blt_tx_buff[2]= 0;
				 blt_tx_buff[3]= ring_stack & 0xFF; blt_tx_buff[4]= (ring_stack >> 8) & 0xFF;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 1; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
			 }
			 if((blt_rx_buff[2]&0b00000100)==4){
				 // USER CODE BEGIN ANGKOR 4
				 if(com_blt.isAng4) com_blt.isAng4 = 0;
				 else com_blt.isAng4 = 1;
				 ring_stack=4;
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 64; blt_tx_buff[2]= 0;
				 blt_tx_buff[3]= ring_stack & 0xFF; blt_tx_buff[4]= (ring_stack >> 8) & 0xFF;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 2; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
			 }
			 if((blt_rx_buff[2]&0b00001000)==8){
				 // USER CODE BEGIN ANGKOR 5
				 if(com_blt.isAng5) com_blt.isAng5 = 0;
				 else com_blt.isAng5 = 1;
				 ring_stack=5;
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 64; blt_tx_buff[2]= 0;
				 blt_tx_buff[3]= ring_stack & 0xFF; blt_tx_buff[4]= (ring_stack >> 8) & 0xFF;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 4; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
			 }
			 // Enemy Pole
			 if((blt_rx_buff[2]&0b00010000)==16){
				 // USER CODE BEGIN ENEMY 3
				 if(com_blt.isEn1) com_blt.isEn1 = 0;
				 else com_blt.isEn1 = 1;
				 ring_stack=1;
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 64; blt_tx_buff[2]= 0;
				 blt_tx_buff[3]= ring_stack & 0xFF; blt_tx_buff[4]= (ring_stack >> 8) & 0xFF;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 8; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
			 }
			 if((blt_rx_buff[2]&0b00100000)==32){
				 // USER CODE BEGIN ENEMY 2
				 if(com_blt.isEn2) com_blt.isEn2 = 0;
				 else com_blt.isEn2 = 1;
				 ring_stack=2;
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 64; blt_tx_buff[2]= 0;
				 blt_tx_buff[3]= ring_stack & 0xFF; blt_tx_buff[4]= (ring_stack >> 8) & 0xFF;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
				 blt_tx_buff[0]= 0;blt_tx_buff[1]= 16; blt_tx_buff[2]= 0; blt_tx_buff[3]= 0; blt_tx_buff[4]= 0;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
			 }
			 if((blt_rx_buff[2]&0b01000000)==64){
				 // USER CODE BEGIN ENEMY 1
				 if(com_blt.isEn3) com_blt.isEn3 = 0;
				 else com_blt.isEn3 = 1;
				 ring_stack=3;
				 blt_tx_buff[0]= 0; blt_tx_buff[1]= 64; blt_tx_buff[2]= 0;
				 blt_tx_buff[3]= ring_stack & 0xFF; blt_tx_buff[4]= (ring_stack >> 8) & 0xFF;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN);
				 blt_tx_buff[0]= 0;blt_tx_buff[1]= 32;blt_tx_buff[2]= 0;blt_tx_buff[3]= 0;blt_tx_buff[4]= 0;
				 HAL_UART_Transmit_IT(&huart4, blt_tx_buff, BUFFER_LEN); //mengirimkan state ke android
			 }
			 HAL_UART_Receive_IT(&huart4, blt_rx_buff, BUFFER_LEN);
			}
			// Jika Berasal dari motion event
			if((blt_rx_buff[0]&2)==2){
			 if((blt_rx_buff[0]&4)==4){
				 // jika input berasal dari Right joystick
				 com_blt.x_rjoystick_int = (blt_rx_buff[0]>>4) | (blt_rx_buff[1]<<4) | ((blt_rx_buff[2]&0x3F)<<12);
				 com_blt.y_rjoystick_int = (blt_rx_buff[2]>>6) | (blt_rx_buff[3]<<2) | (blt_rx_buff[4]<<10);

				 //Jika X Negatif
				 if((blt_rx_buff[2]&32)==32) com_blt.x_rjoystick_int |= 0xFFFC0000;
				 //Jika Y Negatif
				 if((blt_rx_buff[4]&128) == 128) com_blt.y_rjoystick_int |= 0xFFFC0000;

				 // Konvert integer ke float
				 com_blt.x_rjoystick = com_blt.x_rjoystick_int / 100000.0;
				 com_blt.y_rjoystick = com_blt.y_rjoystick_int / 100000.0;
				 manual.vx = com_blt.x_rjoystick * -1200 * speed_mul * Dir_negation;
				 manual.vy = com_blt.y_rjoystick * 1200 * speed_mul * Dir_negation;
	//			 tx_len = sprintf(blt_tx_buff, "xr:%d yr:%d \n", xRight, yRight);
			 }else{
				 // jika input berasal dari Left joystick
				 com_blt.x_ljoystick_int = (blt_rx_buff[0]>>4) | (blt_rx_buff[1]<<4) | ((blt_rx_buff[2]&0x3F)<<12);
				 com_blt.y_ljoystick_int = (blt_rx_buff[2]>>6) | (blt_rx_buff[3]<<2) | (blt_rx_buff[4]<<10);

				 //Jika X Negatif
				 if((blt_rx_buff[2]&32)==32) com_blt.x_ljoystick_int |= 0xFFFC0000;
				 //Jika Y Negatif
				 if((blt_rx_buff[4]&128) == 128) com_blt.y_ljoystick_int |= 0xFFFC0000;

				 // Konvert integer ke float
				 com_blt.x_ljoystick = com_blt.x_ljoystick_int / 100000.0;
				 com_blt.y_ljoystick = com_blt.y_ljoystick_int / 100000.0;
				 manual.v_yaw = com_blt.x_ljoystick * 500 * speed_mul;
	//			 tx_len = sprintf(blt_tx_buff, "xl:%d yl:%d \n", xLeft, yLeft);
			 }
			 HAL_UART_Receive_IT(&huart4, blt_rx_buff, BUFFER_LEN);
			}
		}
//	//bluetooth--------------------------------------------------------
//	if (huart->Instance == UART4)
//	{
//		switch (blt1_rx_buff[0])
//		{
//		case 0xA5:
//#if ER
//			manual.vx = ((double)blt1_rx_buff[1] - 50) * 24 * speed_mul;
//			manual.vy = (50 - (double)blt1_rx_buff[2]) * 24 * speed_mul;
//			manual.v_yaw = ((double)blt1_rx_buff[3] - 50) * 10 * speed_mul;
//#else
//			manual.vx = ((double)blt1_rx_buff[1] - 50) * -24 * speed_mul;
//			manual.vy = (50 - (double)blt1_rx_buff[2]) * -24 * speed_mul;
//			manual.v_yaw = ((double)blt1_rx_buff[3] - 50) * 5 * speed_mul;
//#endif
//			break;
//		case 60:
//			if (blt1_rx_buff[1] == 6) speed_mul = 1;
//			else if (blt1_rx_buff[1] == 14) speed_mul = 0.2;
//			else if (blt1_rx_buff[1] == 7) ;
//			break;
//		case 0xFE:
//			switch (blt1_rx_buff[1])
//			{
//			case 0x01:
//				launch_R = 1;
//				shoot_timer_R = HAL_GetTick();
//				break;
//			case 0x02:
//				launch_L = 1;
//				shoot_timer_L = HAL_GetTick();
//				break;
//			case 0x03:
//				grip_bt = 1;
//				RR_ambil = 1;
//				break;
//			case 0x04:
////				if (ctrl_state) ctrl_state = 0;
////				else ctrl_state = 1;
//				RR_lempar = 1;
//				next_bt = 1;
//				break;
//			}
//			break;
//		}
//		//A0 60 6 6 6
//		//A1 60 14 14 14
//		//A2 60 7 7 7
//		//next 254 4 4 4
//		HAL_UART_Receive_IT(&huart4, blt1_rx_buff, 4);
//		v_test++;
//		if (v_test > 10)
//		{
//			v_test = 0;
//			HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
//		}
//	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if (hfdcan->Instance == FDCAN1)
	{
		if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
		{
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can1_rx_header, can1_rx_data);
			HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
		}
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	v_test++;
	if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0)
	{
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &can2_rx_header, can2_rx_data);
		HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

		switch (can2_rx_data[0])
		{
		case _READ_ENCD_X:
			if (wait_request == _READ_ENCD_X)
			{
				sens.x = (double)((can2_rx_data[1] << 24) | (can2_rx_data[2] << 16) |
						 (can2_rx_data[3] << 8) | can2_rx_data[4]) / 100.0;
			}
			break;
		case _READ_ENCD_Y:
			if (wait_request == _READ_ENCD_Y)
			{
				sens.y = (double)((can2_rx_data[1] << 24) | (can2_rx_data[2] << 16) |
						 (can2_rx_data[3] << 8) | can2_rx_data[4]) / 100.0;
			}
			break;
		case _READ_ENCD_YAW:
			if (wait_request == _READ_ENCD_YAW)
			{
				sens.yaw = (double)((can2_rx_data[1] << 24) | (can2_rx_data[2] << 16) |
						 (can2_rx_data[3] << 8) | can2_rx_data[4]) / 100.0;
			}
			break;
		}

		wait_request = 0;
	}
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan2)
    {
    	can2_fifo_state = _IDLE;
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY , 0);
    }
}

void ADC_toDistance(){

	double alpha = 0.9998;

	double V = (double)adc/65535.0 * 3.3;
	current = V/120.0;

	distance = (current-0.004)/0.016 * 980.0 + 20.0;

	distance_filter = alpha * distance_filter + (1.0-alpha) * distance;

	actual_distance = (distance_filter-56.7)/69.0 * 70.0 + 30.0; // 56.7 = 30 cm || 125.7 = 100 cm

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adc = HAL_ADC_GetValue(&hadc1);
	ADC_toDistance();
}

/******************************************************************************************/

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
  MX_TIM5_Init();
  MX_FDCAN1_Init();
  MX_TIM12_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  MX_FDCAN2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  robot_init ();
  can_config ();
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
//  HAL_UART_Receive_IT(&huart4, blt1_rx_buff, 4);
  HAL_UART_Receive_IT(&huart4, blt_rx_buff, BUFFER_LEN); // Sulthon
  HAL_TIM_Base_Start(&htim12);
  lcd_init();
  lcd_bitmap (bandhayudha_logo);
  servo_init ();
  com_bluetooth_init(); //Sulthon
  HAL_ADC_Start_IT(&hadc1);

#ifdef ER
  set_servo (&servo5, 118 - 90);
  set_servo (&servo6, 67 + 90);
  set_servo (&servo7, 0);
  set_servo (&servo8, 0);
  set_gripper (_GRIPPER1, -25, _RELEASE);
  set_gripper (_GRIPPER2, -25, _RELEASE);
  beep (100);
  HAL_Delay(100);
  beep (100);
  HAL_Delay(100);
  beep (100);

#if 0
  save_data2flash ();
#else
  flash_get_data ();
#endif
  HAL_Delay(5000);
#else
//  set_servo (&servo1, 0);
  ring_stack = 5;
  //set_servo (&servo2, 0); //
  set_servo (&servo2, 45);
  set_servo (&servo7, 17);
  set_servo (&servo6, 133);
  set_gripper (_GRIPPER1, -45, _RELEASE);
  HAL_Delay(10000);
#endif
//  HAL_TIM_Base_Start_IT(&htim7);
	clear_graphic();
	lst_speed = com_blt.rthrower_speed;
//	uint8_t t = 0;

//  bluetooth_at_cmd("AT+NAME=RR-bndyd");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef ER
	  if (!start_trial)
	  {
		  if (next_bt)
		  {
				for (uint8_t b = 0; b < 3; b++)
				{
				  beep (50);
				  HAL_Delay(50);
				}
				next_bt = 0;
				launch_R = 0;
				launch_L = 0;
				//====================================
//				if (t == 0)
//				{
//					t = 1;
//					robot_gotoxy (100, 0, 0);
//				}
//				else
//				{
//					t = 0;
//					robot_gotoxy (0, 0, 0);
//				}
				//====================================
//				start_trial = 1;
//				pemutar_start = 1;

				switch (robot_move_state)
				{
				case 0:
					can_wait_transmit (&hfdcan2);
					can_reset_origin_robot ();
					can_wait_transmit (&hfdcan2);
					robot_gotoxy(-510, 50, 0);
					break;
				case 1:
					set_servo (&servo5, 118);
					set_servo (&servo6, 67);
					can_wait_transmit (&hfdcan2);
					robot_gotoxy(30, 98, 0);
					break;
				case 2:
					can_wait_transmit (&hfdcan2);
					can_set_coord_robot (30, 98, 0);
					can_wait_transmit (&hfdcan2);
					robot_gotoxy(30, 95, 0);
					HAL_Delay(800);
					set_servo (&servo5, 118 - 90);
					set_servo (&servo6, 67 + 90);
					HAL_Delay(200);
					robot_gotoxy(30, 98, 0);
					break;
				case 3:
					can_wait_transmit (&hfdcan2);
					can_lock_move ();
					launch_R = 0;
					launch_L = 0;
					start_trial = 1;
					pemutar_start = 1;
					break;
				}
				robot_move_state++;
				robot_move_state %= 4;
		  }
		  if (grip_bt)
		  {
			  grip_bt = 0;
			  if (grip_state)
			  {
				  grip_state = 0;
				  set_servo (&servo5, 118 - 90);
				  set_servo (&servo6, 67 + 90);
			  }
			  else
			  {
				  grip_state = 1;
				  set_servo (&servo5, 118);
				  set_servo (&servo6, 67);
			  }
		  }
		  wheels_control_send_speed ();
	  }
	  else
	  {
		  trial_lempar ();
	  }
	  main_menu ();
#else
//	  sprintf (lcd_str_buff, "%d\n%0.2fcm",adc, actual_distance);
//	  lcd_print_string(0, 0, lcd_str_buff);
//	  lcd_update();
	  wheels_control_send_speed ();
//	  if (manual.vx != 0 || manual.vy != 0 || manual.v_yaw != 0)
//	  {
//		  set_servo (&servo6, 121);
//	  }
	  trial_lempar_RR ();
//	  kalibrasi_lempar();
#endif
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 16;
  RCC_OscInitStruct.PLL.PLLR = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 13;
  hfdcan1.Init.NominalTimeSeg1 = 86;
  hfdcan1.Init.NominalTimeSeg2 = 13;
  hfdcan1.Init.DataPrescaler = 25;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 2;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 2;
  hfdcan2.Init.NominalTimeSeg1 = 47;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 2;
  hfdcan2.Init.DataSyncJumpWidth = 12;
  hfdcan2.Init.DataTimeSeg1 = 12;
  hfdcan2.Init.DataTimeSeg2 = 12;
  hfdcan2.Init.MessageRAMOffset = 11;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 0;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 1;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.Date = 13;
  sDate.Year = 23;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3002;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 199;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 99;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 199;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin
                          |SERVO0_Pin|SERVO1_Pin|SERVO2_Pin|SERVO3_Pin
                          |SERVO4_Pin|SERVO5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SERVO6_Pin|SPI2_CS_Pin|LCD_BL_Pin|LCD_RST_Pin
                          |LCD_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SERVO7_GPIO_Port, SERVO7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LCD_RW_Pin|LCD_RS_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin
                           SERVO0_Pin SERVO1_Pin SERVO2_Pin SERVO3_Pin
                           SERVO4_Pin SERVO5_Pin */
  GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin
                          |SERVO0_Pin|SERVO1_Pin|SERVO2_Pin|SERVO3_Pin
                          |SERVO4_Pin|SERVO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BUILTIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SERVO6_Pin LCD_EN_Pin */
  GPIO_InitStruct.Pin = SERVO6_Pin|LCD_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_CS_Pin LCD_BL_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin|LCD_BL_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SERVO7_Pin */
  GPIO_InitStruct.Pin = SERVO7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SERVO7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RW_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_RW_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

#ifndef ER
	#ifndef RR
		#error "mohon pilih ER atau RR!"
	#endif
#endif

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
