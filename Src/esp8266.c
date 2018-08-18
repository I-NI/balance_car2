#include "esp8266.h"
#include "common.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_tx;

esp8266_t g_esp8266 ;

/*
Function : ESP8266ModeCtrl
  Select ESP8266 work mode , 0 Disable, 1 Program mode, 2 WiFi mode		
*/

void ESP8266ModeCtrl( uint8_t state)
{

	if(state == 0 ) 
	{
  	HAL_GPIO_WritePin(WIFI_RESET_GPIO_Port,WIFI_RESET_Pin,GPIO_PIN_SET);
  	HAL_GPIO_WritePin(WIFI_EN_GPIO_Port,WIFI_EN_Pin ,GPIO_PIN_RESET);
	}
	else if(state == 1 ) 
	{
  	HAL_GPIO_WritePin(WIFI_RESET_GPIO_Port,WIFI_RESET_Pin,GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(WIFI_EN_GPIO_Port,WIFI_EN_Pin ,GPIO_PIN_SET);
  	HAL_GPIO_WritePin( WIFI_BOOT_GPIO_Port,WIFI_BOOT_Pin, GPIO_PIN_RESET );
  	HAL_Delay( 400 ) ;
	  HAL_GPIO_WritePin(WIFI_RESET_GPIO_Port,WIFI_RESET_Pin,GPIO_PIN_SET);
	}		
	else
	{
  	HAL_GPIO_WritePin(WIFI_RESET_GPIO_Port,WIFI_RESET_Pin,GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(WIFI_EN_GPIO_Port,WIFI_EN_Pin ,GPIO_PIN_SET);
  	HAL_GPIO_WritePin( WIFI_BOOT_GPIO_Port,WIFI_BOOT_Pin, GPIO_PIN_SET );
  	HAL_Delay( 400 ) ;
	  HAL_GPIO_WritePin(WIFI_RESET_GPIO_Port,WIFI_RESET_Pin,GPIO_PIN_SET);
	}
}


/*
Function : ESP8266ModeCheck
  Check if need to flash proggram to ESP8266 
  If Reset with USER_PB0 pushed , then enter flash mode
  Else enter work mode.

*/
void ESP8266ModeCheck( void )
{
	int i = 0 ;
	g_esp8266.work_mode = 0 ;  // 0 Disable, 1 Program mode, 2 WiFi mode
	g_esp8266.host_uart = NULL ;
	g_esp8266.esp8266_uart = NULL ;
	
	if ( HAL_GPIO_ReadPin( USER_PB0_GPIO_Port , USER_PB0_Pin ) == GPIO_PIN_RESET )
	{
    HAL_Delay(100);
    if ( HAL_GPIO_ReadPin( USER_PB0_GPIO_Port , USER_PB0_Pin ) == GPIO_PIN_RESET )
		{
			g_esp8266.work_mode = 1 ;
		}
	}	
	ESP8266ModeCtrl( g_esp8266.work_mode ) ;
	if ( g_esp8266.work_mode == 0 )
	{
		return ;
	}
	else if ( g_esp8266.work_mode == 1 )
	{
		Host_ESP8266_UART_INIT( );
		LEDCtrl( 0 , 0 );
		while ( 1 )
		{
			Host_ESP8266_UART_Tranceiver();
			HAL_Delay( 50 );
			if ( i == 10 ) 
			{
				i = 0 ;
				LEDCtrl( 0 , 2 );
			}
			else i++;
		}
	}
}

void Host_ESP8266_UART_INIT( )
{
	int i ; 
	
	g_esp8266.uart_dma_rx[ 0 ].huart       = &huart1 ;
	g_esp8266.uart_dma_rx[ 0 ].hdma_rx     = &hdma_usart1_rx ;
	g_esp8266.uart_dma_rx[ 0 ].hdma_tx     = &hdma_usart1_tx ;
	g_esp8266.uart_dma_rx[ 0 ].rx_buf_head = 0 ;
	g_esp8266.uart_dma_rx[ 0 ].rx_buf_tail = 0 ;
	g_esp8266.uart_dma_rx[ 0 ].rx_buf_full = 0 ;
	g_esp8266.uart_dma_rx[ 0 ].tx_busy      = 0 ;
	for ( i = 0 ;  i < MAX_UART_BUF_NUM ; i ++ )
	{
	    g_esp8266.uart_dma_rx[ 0 ].uart_rx_buf[ 0 ][ i ] = 0;
	    g_esp8266.uart_dma_rx[ 0 ].rx_buf_size[ i ] = 0;
	}
  HAL_UART_Receive_DMA( g_esp8266.uart_dma_rx[ 0 ].huart , 
												( uint8_t *)g_esp8266.uart_dma_rx[ 0 ].uart_rx_buf[ 0 ] ,
												UART_RX_BUF_SIZE );

	g_esp8266.uart_dma_rx[ 1 ].huart       = &huart3 ;
	g_esp8266.uart_dma_rx[ 1 ].hdma_rx     = &hdma_usart3_rx ;
	g_esp8266.uart_dma_rx[ 1 ].hdma_tx     = &hdma_usart3_tx ;
	g_esp8266.uart_dma_rx[ 1 ].rx_buf_head = 0 ;
	g_esp8266.uart_dma_rx[ 1 ].rx_buf_tail = 0 ;
	g_esp8266.uart_dma_rx[ 1 ].rx_buf_full = 0 ;
	g_esp8266.uart_dma_rx[ 1 ].tx_busy      = 0 ;
	for ( i = 0 ;  i < MAX_UART_BUF_NUM ; i ++ )
	{
	    g_esp8266.uart_dma_rx[ 1 ].uart_rx_buf[ 1 ][ i ] = 0;
	    g_esp8266.uart_dma_rx[ 1 ].rx_buf_size[ i ] = 0;
	}
  HAL_UART_Receive_DMA( g_esp8266.uart_dma_rx[ 1 ].huart , 
												( uint8_t *)g_esp8266.uart_dma_rx[ 1 ].uart_rx_buf[ 0 ] ,
												UART_RX_BUF_SIZE );
	
	__HAL_UART_ENABLE_IT( g_esp8266.uart_dma_rx[ 0 ].huart , UART_IT_IDLE);    //使能空闲终中断
	__HAL_UART_ENABLE_IT( g_esp8266.uart_dma_rx[ 1 ].huart , UART_IT_IDLE);    //使能空闲终中断
}

void Uart_RxIDLE_Handler( UART_HandleTypeDef * huart )
{
	int32_t temp ;
	int32_t i ;
	
	
	if(huart->Instance == USART1) {	
		i = 0 ;
	} else if(huart->Instance == USART3) {	
		i = 1 ;
	} else  
	{	
		return ;
	}	
	
	if( __HAL_UART_GET_FLAG( g_esp8266.uart_dma_rx[ i ].huart , UART_FLAG_IDLE) != RESET )
	{
		__HAL_UART_CLEAR_IDLEFLAG( g_esp8266.uart_dma_rx[ i ].huart );
		__HAL_UART_CLEAR_PEFLAG( g_esp8266.uart_dma_rx[ i ].huart );

//		temp = g_uart_dma[ i ].huart->Instance->SR;  
//		temp = g_uart_dma[ i ].huart->Instance->DR;
		
		HAL_UART_DMAStop( g_esp8266.uart_dma_rx[ i ].huart );
		temp  = __HAL_DMA_GET_COUNTER( g_esp8266.uart_dma_rx[ i ].hdma_rx ) ;

		g_esp8266.uart_dma_rx[ i ].rx_buf_size[ g_esp8266.uart_dma_rx[ i ].rx_buf_tail ] =  UART_RX_BUF_SIZE - temp;                           
		if ( g_esp8266.uart_dma_rx[ i ].rx_buf_tail == ( MAX_UART_BUF_NUM - 1 ) )
			g_esp8266.uart_dma_rx[ i ].rx_buf_tail = 0 ;
		else
			g_esp8266.uart_dma_rx[ i ].rx_buf_tail ++ ;

//		LEDCtrl( g_wifi_uart_map[ i ] , 2 );

		HAL_UART_Receive_DMA( g_esp8266.uart_dma_rx[ i ].huart , 
													( uint8_t *)g_esp8266.uart_dma_rx[ i ].uart_rx_buf[ g_esp8266.uart_dma_rx[ i ].rx_buf_tail ] ,
													UART_RX_BUF_SIZE );
			
		if ( g_esp8266.uart_dma_rx[ i ].rx_buf_tail == g_esp8266.uart_dma_rx[ i ].rx_buf_head ) 
		{
			g_esp8266.uart_dma_rx[ i ].rx_buf_full = 1 ;
		}
	}	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t i ;

	if(huart->Instance == USART1) {	
		i = 0 ;
	} else if(huart->Instance == USART3) {	
		i = 1 ;
	} else  
	{	
		return ;
	}	

	g_esp8266.uart_dma_rx[ i ].rx_buf_size[ g_esp8266.uart_dma_rx[ i ].rx_buf_tail ] =  UART_RX_BUF_SIZE ;                           
	if ( g_esp8266.uart_dma_rx[  i ].rx_buf_tail == ( MAX_UART_BUF_NUM - 1 ) )
		g_esp8266.uart_dma_rx[ i ].rx_buf_tail = 0 ;
	else
		g_esp8266.uart_dma_rx[ i ].rx_buf_tail ++ ;

	HAL_UART_Receive_DMA( g_esp8266.uart_dma_rx[  i ].huart , 
												( uint8_t *)g_esp8266.uart_dma_rx[ i ].uart_rx_buf[ g_esp8266.uart_dma_rx[ i ].rx_buf_tail ] ,
												UART_RX_BUF_SIZE );

	if ( g_esp8266.uart_dma_rx[ i ].rx_buf_tail == g_esp8266.uart_dma_rx[ i ].rx_buf_head ) 
	{
		g_esp8266.uart_dma_rx[ i ].rx_buf_full = 1 ;
	}
//	LEDCtrl( g_wifi_uart_map[ i ] , 2 );

}


void Host_ESP8266_UART_Tranceiver( void )
{
	int32_t i ;
	int32_t j ;
	
 	for ( i = 0 ; i < MAX_UART_PORT_NUM ; i++ )
	{
		while ( g_esp8266.uart_dma_rx[ i ].rx_buf_full == 1 || 
				 g_esp8266.uart_dma_rx[ i ].rx_buf_head != g_esp8266.uart_dma_rx[ i ].rx_buf_tail  )
		{
		
			if ( i == 0 ) j =1 ;
			else j = 0 ;

			if (  HAL_UART_Transmit_DMA( g_esp8266.uart_dma_rx[ j ].huart , 
			                         g_esp8266.uart_dma_rx[ i ].uart_rx_buf[ g_esp8266.uart_dma_rx[ i ].rx_buf_head ] , 
			                         g_esp8266.uart_dma_rx[ i ].rx_buf_size[ g_esp8266.uart_dma_rx[ i ].rx_buf_head ]  )  == HAL_OK ) 
			{
				if ( g_esp8266.uart_dma_rx[ i ].rx_buf_head == ( MAX_UART_BUF_NUM - 1 ) )
					g_esp8266.uart_dma_rx[ i ].rx_buf_head = 0 ;
				else
					g_esp8266.uart_dma_rx[ i ].rx_buf_head ++ ;
				g_esp8266.uart_dma_rx[ i ].rx_buf_full = 0 ;
			}
//			else
//			{
//				HAL_Delay( 1 );
//			}
			//HAL_GPIO_WritePin( LED2_GPIO_Port , LED2_Pin, GPIO_PIN_RESET );			
		}			
  }
}
