#include <string.h>
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "spi.h"
#include "main.h"
#include "moto_ctrl.h"
#include "dot_matrix.h"
#include "uart_osc.h"
#include "common.h"
#include "esp8266.h"
#include "mpu9250.h"
#include "moto_ctrl.h"

extern mpu9250_t g_mpu9250 ;
extern moto_ctrl_t g_moto_ctrl ;
uint32_t g_SysMode = 2 ;

uint32_t ReadUserButton0( void )
{
	static uint8_t btn0_down = 0 ;
	static uint8_t btn0_up = 1 ;
	if ( HAL_GPIO_ReadPin( USER_PB0_GPIO_Port , USER_PB0_Pin ) == GPIO_PIN_RESET )
	{
		if ( btn0_down == 1 && btn0_up == 1 )
		{
			btn0_up = 0 ;
			return 1 ;
		}
		else
		{
			btn0_down = 1 ;
    	HAL_Delay( 50 );
		}
	}
	else
	{
		btn0_down = 0;
		btn0_up = 1 ;
	}
	return 0 ;
}

uint32_t ReadUserButton1( void )
{
	static uint8_t btn1_down = 0 ;
	static uint8_t btn1_up = 1 ;
	if ( HAL_GPIO_ReadPin( USER_PB1_GPIO_Port , USER_PB1_Pin ) == GPIO_PIN_RESET )
	{
		if ( btn1_down == 1 && btn1_up == 1 )
		{
			btn1_up = 0 ;
			return 1 ;
		}
		else
		{
			btn1_down = 1 ;
    	HAL_Delay( 50 );
		}
	}
	else
	{
		btn1_down = 0;
		btn1_up = 1 ;
	}
	return 0 ;
}

// index 0 : all , 1~4 : LED1~4 
// mode 0 : off , 1 : on , 2 : toggle
void LEDCtrl( uint32_t index , uint32_t mode )
{
	 switch( index )
	 {
		 case 0 :
			 if ( mode ==  0 ) 
			 {
  				HAL_GPIO_WritePin( LED1_GPIO_Port , LED1_Pin, GPIO_PIN_SET );
  				HAL_GPIO_WritePin( LED2_GPIO_Port , LED2_Pin, GPIO_PIN_SET );
			 }
			 else if ( mode ==  1 )
			 {
  				HAL_GPIO_WritePin( LED1_GPIO_Port , LED1_Pin, GPIO_PIN_RESET );
 				  HAL_GPIO_WritePin( LED2_GPIO_Port , LED2_Pin, GPIO_PIN_RESET );
			 }
			 break;
		 case 1 :
			 if ( mode ==  0 ) 
			 {
  				HAL_GPIO_WritePin( LED1_GPIO_Port , LED1_Pin, GPIO_PIN_SET );
			 }
			 else if ( mode ==  1 )
			 {
  				HAL_GPIO_WritePin( LED1_GPIO_Port , LED1_Pin, GPIO_PIN_RESET );
			 }
			 else if ( mode ==  2 )
			 {
  				HAL_GPIO_TogglePin( LED1_GPIO_Port , LED1_Pin );
			 }
			 break;
		 case 2 :
			 if ( mode ==  0 ) 
			 {
  				HAL_GPIO_WritePin( LED2_GPIO_Port , LED2_Pin, GPIO_PIN_SET );
			 }
			 else if ( mode ==  1 )
			 {
  				HAL_GPIO_WritePin( LED2_GPIO_Port , LED2_Pin, GPIO_PIN_RESET );
			 }
			 else if ( mode ==  2 )
			 {
  				HAL_GPIO_TogglePin( LED2_GPIO_Port , LED2_Pin );
			 }
			 break;

		 default :
			 break;
	 }
}

void SleepSystem( void )
{
//  MX_GPIO_DeInit();
	
	HAL_PWR_EnterSLEEPMode( PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFE );
}

	
void StopSystem( void )
{
	HAL_PWR_EnterSTOPMode( PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI );
}

void Get_IR_Sensor( void )
{
  uint8_t data0 , data1;
  data0 = GPIOC->IDR & 0x0E;	  //  pin 3,2,1 -> IR 4,3,5
  data1 = GPIOB->IDR & 0x28;	  //  pin 5,3   -> IR 2,1
	g_moto_ctrl.ir_sensor = (data1<<2 ) | data0 ;
}

void IR_Sensor_Init( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  HAL_SPI_MspDeInit( &hspi3 );
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, IR_3_Pin|IR_4_Pin|IR_5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_DeInit(GPIOC, IR_3_Pin|IR_4_Pin|IR_5_Pin);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = IR_1_Pin|IR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = IR_3_Pin|IR_4_Pin|IR_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void SetWorkMode( uint32_t mode)
{
		switch ( mode )
		{
			case 0 :   // balance control
				LEDCtrl( 1 , 0 );
				LEDCtrl( 2 , 1 );
				
				HAL_TIM_PWM_MspInit(&htim8);
				HAL_TIM_Base_Start_IT(&htim6);
				IR_Sensor_Init();
				break;
			case 1 :   // dot matrix
				LEDCtrl( 1 , 1 );
				LEDCtrl( 2 , 0 );
				HAL_TIM_Base_Stop_IT(&htim6);
				Motor_Stop();
				HAL_TIM_PWM_MspDeInit(&htim8);
				dot_matrix_init();
				break ;
			case 2 :   // show osc wave
				LEDCtrl( 1 , 1 );
				LEDCtrl( 2 , 1 );
				Motor_Stop();
				HAL_TIM_PWM_MspDeInit(&htim8);
				IR_Sensor_Init();
				HAL_TIM_Base_Start_IT(&htim6);
				break ;
		}
}

void	UserTask( void )
{
	if ( ReadUserButton0() == 1 )
	{ 
		LEDCtrl( 0 , 0 );
		HAL_TIM_Base_Stop_IT(&htim6);
		HAL_TIM_PWM_MspDeInit(&htim8);
		HAL_PWR_EnterSTANDBYMode();			
	}
	
	if ( ReadUserButton1() == 1 )
	{ 
		g_SysMode++;
		if ( g_SysMode > 2 ) g_SysMode = 0 ;
		SetWorkMode( g_SysMode );		
	}

	switch ( g_SysMode )
	{
		case 1 :   // dot matrix
			show_dot_matrix();
			break;
		case 2 :
			Uart_OSC_ShowWave( g_mpu9250.angle_x , g_mpu9250.gyro_scale_y , g_mpu9250.Angle_Complement_1st, g_moto_ctrl.ir_sensor  ) ;
//		Uart_OSC_ShowWave( g_moto_ctrl.right_pwm , g_moto_ctrl.left_pwm   , g_mpu9250.Angle_Complement_1st ) ;

//		Uart_OSC_ShowWave( g_mpu9250.angle_x , g_mpu9250.gyro_scale_y  , g_mpu9250.Angle_Kalman , g_mpu9250.Angle_Complement_1st ) ;
			break;
	}
}

void SystemTimer1msCallback( int x ) 
{
	static uint32_t i1 = 0,i2=0,i3=0 ;
	
	if ( i1 >= 99 ){ i1 = 0;i2++ ;}
	
	else {
	if(i1%4==0){i3++;}
	if(i3>=99){i3=0;}
	i1 ++ ;}
		
	Get_IR_Sensor();
  MPU9250_Get_Accel_Gyro_Temp();
  MPU9250_Data_Process();
	if(i2<40)
  x=i3;
	
	
	else
	x=100;
	Moto_Balance_PID_Ctrl(x);	

	switch ( g_SysMode )
	{
		case 0 :   // banlance car
	    if ( i1 == 99 ) LEDCtrl( 2 , 2 );
		  break;
		case 2 :
	    if ( g_mpu9250.angle_x > 0 ) LEDCtrl( 2 , 1 );
	    else if ( g_mpu9250.angle_x < 0	) LEDCtrl( 2 , 0 );
      break ;
	}
}


