#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "mpu9250.h"
#include "moto_ctrl.h"
#include "common.h"



extern TIM_HandleTypeDef htim8;
extern mpu9250_t g_mpu9250 ;
moto_ctrl_t g_moto_ctrl ;

void Moto_Balance_PID_Ctrl(int x)
{
	float KP = 10 ; 
	float KI = 0 ; 
	float KD = 0.9 ; 
	const int16_t MOTO_THRESHOLD=580; // PWM = 18KHz
	static float angle_D = 0 ;
  static float angle_D_last = 0 ;

	float ctrl ;
	float angle_diff = 0 ;
	float angle_FTM = 0 ;
	int16_t dir ;
	int16_t PWM ;
	
	
	
	
	

	
	if(x<6){
		angle_FTM= g_mpu9250.Angle_Complement_1st-1;
		angle_diff += g_mpu9250.Angle_Complement_1st  ;
	  goto aa;
	}
/*  else if(10<x<18){
  	angle_FTM= g_mpu9250.Angle_Complement_1st-3 ;
		goto aa;
	}
	else if(18<x<22){
		angle_FTM= g_mpu9250.Angle_Complement_1st+8 ;
	  goto aa;
	  }
	else if(25<x<30){
		angle_FTM= g_mpu9250.Angle_Complement_1st+3 ;
	  goto aa;
	  }*/
		else{
		angle_FTM= g_mpu9250.Angle_Complement_1st-6 ;
		angle_diff += g_mpu9250.Angle_Complement_1st  ;
		goto aa;
		}
	
		aa:
	ctrl = (angle_FTM)* KP 
	        + angle_diff * KI 
	        + g_mpu9250.gyro_scale_y * KD ;
	
	if ( ctrl > 0 )  
	{
		dir = 1;   // backward
	}
	else
	{
		dir = 0 ;  // forward
	}
	
	PWM =  abs( (int)ctrl ) +  MOTO_THRESHOLD ;
  
//  if (PWM > 700 ) PWM = 700;	
	g_moto_ctrl.right_pwm = PWM ;
	
			
	if ( dir == 0 )
	{
			left_forward( PWM );
			right_forward( PWM );
	}
	else
	{
			left_backward( PWM );
			right_backward( PWM );
	}




}


void Moto_Ctrl_Init( void ) 
{
	memset( &g_moto_ctrl , 0 , sizeof( moto_ctrl_t ) );
}

void right_backward(uint16_t value)
{		
		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
	
		HAL_GPIO_WritePin(MT1_A_GPIO_Port, MT1_A_Pin, GPIO_PIN_SET);		
		HAL_GPIO_WritePin(MT1_B_GPIO_Port, MT1_B_Pin, GPIO_PIN_RESET);
			
		TIM_SetCompare4(TIM8, value);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);	
}

void right_forward(uint16_t value)
{
		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
		HAL_GPIO_WritePin(MT1_A_GPIO_Port, MT1_A_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(MT1_B_GPIO_Port, MT1_B_Pin, GPIO_PIN_SET);
		
		TIM_SetCompare4(TIM8, value);		
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

void left_backward(uint16_t value)
{
		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
		HAL_GPIO_WritePin(MT2_B_GPIO_Port, MT2_B_Pin, GPIO_PIN_SET);		
		HAL_GPIO_WritePin(MT2_A_GPIO_Port, MT2_A_Pin, GPIO_PIN_RESET);
				
	  TIM_SetCompare3(TIM8, value);		
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);	
}

void left_forward(uint16_t value)
{
		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
		HAL_GPIO_WritePin(MT2_B_GPIO_Port, MT2_B_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(MT2_A_GPIO_Port, MT2_A_Pin, GPIO_PIN_SET);
					
		TIM_SetCompare3(TIM8, value);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);	
}

void Motor_Stop(void)
{
		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
		HAL_GPIO_WritePin(MT1_A_GPIO_Port, MT2_A_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(MT1_B_GPIO_Port, MT2_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MT2_A_GPIO_Port, MT2_A_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(MT2_B_GPIO_Port, MT2_B_Pin, GPIO_PIN_RESET);
}


void TIM_SetCompare3(TIM_TypeDef* TIMx, uint16_t Compare)
{
  /* Check the parameters */
  //assert_param(IS_TIM_LIST8_PERIPH(TIMx));
  /* Set the Capture Compare1 Register value */
  TIMx->CCR3 = Compare;
}

void TIM_SetCompare4(TIM_TypeDef* TIMx, uint16_t Compare)
{
  /* Check the parameters */
  //assert_param(IS_TIM_LIST6_PERIPH(TIMx));
  /* Set the Capture Compare2 Register value */
  TIMx->CCR4 = Compare;
}

/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////

void SpeedCaptureTimeOut( int32_t channel )
{
	if ( channel == 0 )
	{
		g_moto_ctrl.left_speed = 0 ;
//		g_moto_ctrl.left_direction = 0 ;
		g_moto_ctrl.left_capture_first = 0 ;
	}
	else
	{
		g_moto_ctrl.right_speed = 0 ;
//		g_moto_ctrl.right_direction = 0 ;
		g_moto_ctrl.right_capture_first = 0 ;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static int32_t start_flag[4]         = { 0,0,0,0 };
	static int32_t ok_flag[4]         = { 0,0,0,0 };
	
  if (htim->Instance == TIM2) 
	{
		
		if ( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 )
		{
			if ( g_moto_ctrl.left_capture_first == 0 )
			{
				__HAL_TIM_SET_COUNTER( htim , 0 );
				g_moto_ctrl.left_capture_first = 1 ;
				return;
			}
				
			if ( start_flag[ 0 ] == 0 ) 
			{
		    g_moto_ctrl.left_start_val[ 0 ] = HAL_TIM_ReadCapturedValue( htim , TIM_CHANNEL_1 );
				start_flag[ 0 ] = 1 ;
			}
			else{
		    g_moto_ctrl.left_end_val[ 0 ] = HAL_TIM_ReadCapturedValue( htim , TIM_CHANNEL_1 );
				
				start_flag[ 0 ] = 0 ;
				if ( ok_flag[ 1 ] == 1 ) 
				{
					Speed_Driection_Calculate( 0 , g_moto_ctrl.left_start_val[ 0 ] , g_moto_ctrl.left_end_val[0] ,
                                     g_moto_ctrl.left_start_val[ 1 ] , g_moto_ctrl.left_end_val[1] ) ;
          __HAL_TIM_SET_COUNTER( htim , 0 );

					ok_flag[ 1 ] = 0 ;
				}
				else
				{
					ok_flag[ 0 ] = 1 ;
				}
			}				
		}
		else if ( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 )
		{
			if ( g_moto_ctrl.left_capture_first == 0 )
			{
				__HAL_TIM_SET_COUNTER( htim , 0 );
				g_moto_ctrl.left_capture_first = 1 ;
				return;
			}

			if ( start_flag[ 1 ] == 0 ) 
			{
		    g_moto_ctrl.left_start_val[ 1 ] = HAL_TIM_ReadCapturedValue( htim , TIM_CHANNEL_2 );
				start_flag[ 1 ] = 1 ;
			}
			else{
		    g_moto_ctrl.left_end_val[ 1 ] = HAL_TIM_ReadCapturedValue( htim , TIM_CHANNEL_2 );
				start_flag[ 1 ] = 0 ;
				if ( ok_flag[ 0 ] == 1 ) 
				{
					Speed_Driection_Calculate( 0 , g_moto_ctrl.left_start_val[ 0 ] , g_moto_ctrl.left_end_val[0] ,
                                     g_moto_ctrl.left_start_val[ 1 ] , g_moto_ctrl.left_end_val[1] ) ;
          __HAL_TIM_SET_COUNTER( htim , 0 );
					
					ok_flag[ 0 ] = 0 ;
				}
				else
				{
					ok_flag[ 1 ] = 1 ;
				}
			}				
		}
	}
	else if (htim->Instance == TIM3) 
	{
		if ( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3 )
		{
			if ( g_moto_ctrl.right_capture_first == 0 )
			{
				__HAL_TIM_SET_COUNTER( htim , 0 );
				g_moto_ctrl.right_capture_first = 1 ;
				return;
			}
			
			if ( start_flag[ 2 ] == 0 ) 
			{
		    g_moto_ctrl.right_start_val[ 0 ] = HAL_TIM_ReadCapturedValue( htim , TIM_CHANNEL_3 );
				start_flag[ 2 ] = 1 ;
			}
			else{
		    g_moto_ctrl.right_end_val[ 0 ] = HAL_TIM_ReadCapturedValue( htim , TIM_CHANNEL_3 );
				start_flag[ 2 ] = 0 ;
				if ( ok_flag[ 3 ] == 1 ) 
				{
					Speed_Driection_Calculate( 1 , g_moto_ctrl.right_start_val[ 0 ] , g_moto_ctrl.right_end_val[0] ,
                                     g_moto_ctrl.right_start_val[ 1 ] , g_moto_ctrl.right_end_val[1] ) ;
          __HAL_TIM_SET_COUNTER( htim , 0 );
					ok_flag[ 3 ] = 0 ;
				}
				else
				{
					ok_flag[ 2 ] = 1 ;
				}
			}				
		}
		else if ( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 )
		{
			if ( g_moto_ctrl.right_capture_first == 0 )
			{
				__HAL_TIM_SET_COUNTER( htim , 0 );
				g_moto_ctrl.right_capture_first = 1 ;
				return;
			}

			if ( start_flag[ 3 ] == 0 ) 
			{
		    g_moto_ctrl.right_start_val[ 1 ] = HAL_TIM_ReadCapturedValue( htim , TIM_CHANNEL_4 );
				start_flag[ 3 ] = 1 ;
			}
			else{
		    g_moto_ctrl.right_end_val[ 1 ] = HAL_TIM_ReadCapturedValue( htim , TIM_CHANNEL_4 );
				start_flag[ 3 ] = 0 ;
				if ( ok_flag[ 2 ] == 1 ) 
				{
					Speed_Driection_Calculate( 1 , g_moto_ctrl.right_start_val[ 0 ] , g_moto_ctrl.right_end_val[0] ,
                                     g_moto_ctrl.right_start_val[ 1 ] , g_moto_ctrl.right_end_val[1] ) ;
          __HAL_TIM_SET_COUNTER( htim , 0 );
					ok_flag[ 2 ] = 0 ;
				}
				else
				{
					ok_flag[ 3 ] = 1 ;
				}
			}				
		}
	}
}

void Speed_Driection_Calculate( int32_t channel  , int32_t start1 , int32_t end1 , int32_t start2 , int32_t end2 )
{
	int32_t count1 , count2 ;
	int32_t phase_diff ;
	int32_t half_period ;
	int32_t direction ;
	
	if ( end1 < start1 )
	{
	  return ;
	}
	else
	{
	  count1  = end1 - start1 ;
	}

	if ( end2 < start2 )
	{
	  return ;
	}
	else
	{
	  count2  = end2 - start2 ;
	}

	half_period  = count1 >> 1 ;
	
	if ( start2 > start1 ) 
	{
		phase_diff = start2 - start1 ;
		if ( phase_diff > half_period ) 
		{
			direction = 1;
		}
		else
		{
			direction = 0;
		}
	}
	else
	{
		phase_diff = start1 - start2 ;
		if ( phase_diff > half_period ) 
		{
			direction = 0;
		}
		else
		{
			direction = 1;
		}
	}	
	if ( channel == 0 ) 
	{
		g_moto_ctrl.left_speed = count1 ;
		g_moto_ctrl.left_direction = direction ;
		g_moto_ctrl.left_half_period = half_period ;
		g_moto_ctrl.left_phase_diff = phase_diff ;
	}
	else
	{
		g_moto_ctrl.right_speed = count1 ;
		g_moto_ctrl.right_direction = direction ;
		g_moto_ctrl.right_half_period = half_period ;
		g_moto_ctrl.right_phase_diff = phase_diff ;
	}
}
