#include "spi.h"
#include "dot_matrix.h"

uint8_t g_sys_mode = 0 ;

uint8_t g_dot_start = 0 ;
uint16_t g_dot_cnt = 0 ;

uint8_t g_ShowData[ 51 ]  = {0x00,0x00,0x00,0x00,0x0C,0x12,0x10,0x7C,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x76,0x24,0x18,0x18,0x18,0x24,0x6E,0x00,0x00,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void dot_matrix_init( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;
	uint8_t i ;

  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_5);
  HAL_GPIO_DeInit(GPIOC, DOT_EN_Pin|DOT_SHIFT_Pin 
                          |DOT_LAT_Pin);

	HAL_SPI_MspInit( &hspi3 );
  HAL_GPIO_WritePin(GPIOC, DOT_EN_Pin|DOT_SHIFT_Pin 
                          |DOT_LAT_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = DOT_EN_Pin|DOT_SHIFT_Pin 
                          |DOT_LAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin( DOT_LAT_GPIO_Port  , DOT_LAT_Pin , GPIO_PIN_RESET );
  HAL_GPIO_WritePin( DOT_EN_GPIO_Port  , DOT_EN_Pin , GPIO_PIN_SET );
	for ( i = 0 ; i < 20 ; i++ )
	{
  	HAL_GPIO_WritePin( DOT_SHIFT_GPIO_Port  , DOT_SHIFT_Pin , GPIO_PIN_RESET );
		HAL_GPIO_WritePin( DOT_SHIFT_GPIO_Port  , DOT_SHIFT_Pin , GPIO_PIN_SET );
	}
}
	

void show_dot_matrix( void )
{
	uint32_t i , j , k ;
	
  g_dot_cnt++ ;
	
	if ( g_dot_cnt == 5 ) 
	{
		g_dot_cnt = 0 ;
		if ( g_dot_start < 50 ) g_dot_start++ ;
		else g_dot_start = 0 ;			
	}
	
	for ( i = 0 ; i < 20 ; i++ )
	{
//		p = g_ShowData[i ];
		j = g_dot_start + i ;
		
		if ( j > 50 ) j -= 51 ;
		
		HAL_SPI_Transmit( &hspi3, g_ShowData + j  , 1, 0 );

		if ( i== 0 ) {
      HAL_GPIO_WritePin( DOT_EN_GPIO_Port  , DOT_EN_Pin , GPIO_PIN_RESET );
      HAL_GPIO_WritePin( DOT_SHIFT_GPIO_Port  , DOT_SHIFT_Pin , GPIO_PIN_RESET );
  		HAL_GPIO_WritePin( DOT_SHIFT_GPIO_Port  , DOT_SHIFT_Pin , GPIO_PIN_SET );
      HAL_GPIO_WritePin( DOT_EN_GPIO_Port  , DOT_EN_Pin , GPIO_PIN_SET );
		}
		else
		{
    	HAL_GPIO_WritePin( DOT_SHIFT_GPIO_Port  , DOT_SHIFT_Pin , GPIO_PIN_RESET );
  		HAL_GPIO_WritePin( DOT_SHIFT_GPIO_Port  , DOT_SHIFT_Pin , GPIO_PIN_SET );
		}
  	HAL_GPIO_WritePin( DOT_LAT_GPIO_Port  , DOT_LAT_Pin , GPIO_PIN_RESET );
		HAL_GPIO_WritePin( DOT_LAT_GPIO_Port  , DOT_LAT_Pin , GPIO_PIN_SET );
		for ( k = 0 ; k < 8000 ; k++) ;
	}
}
