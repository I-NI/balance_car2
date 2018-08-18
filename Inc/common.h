#ifndef __COMMON_H
#define	__COMMON_H
#include "main.h"

#define IR_5_Pin GPIO_PIN_1
#define IR_5_GPIO_Port GPIOC
#define IR_3_Pin GPIO_PIN_2
#define IR_3_GPIO_Port GPIOC
#define IR_4_Pin GPIO_PIN_3
#define IR_4_GPIO_Port GPIOC

#define IR_1_Pin GPIO_PIN_3
#define IR_1_GPIO_Port GPIOB
#define IR_2_Pin GPIO_PIN_5
#define IR_2_GPIO_Port GPIOB


uint32_t ReadUserButton0( void );
uint32_t ReadUserButton1( void );
void LEDCtrl( uint32_t index , uint32_t mode );
void SetWorkMode( uint32_t mode);
void	UserTask( void );
void  IR_Sensor_Init( void );
void Get_IR_Sensor( void );
void SystemTimer1msCallback( int x ) ;
#endif
