#include "stm32f1xx_hal.h"
#include "mpu9250.h"

extern mpu9250_t g_mpu9250 ;




void Kalman_Filter(float angle, float gyro)  
{ 
	static const float Q_angle=0.001;  
	static const float Q_gyro=0.003;
	static const char  C_0 = 1;
	static const float R_angle=0.5;

	static float Q_bias, angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
  
	g_mpu9250.Angle_Kalman += (gyro - Q_bias) * DeltaTime;       //先验估计
      
  Pdot[0] = Q_angle - PP[0][1] - PP[1][0];     //Pk-先验估计误差协方差的微分
  Pdot[1] = -PP[1][1];  
  Pdot[2] = -PP[1][1];  
  Pdot[3] = Q_gyro;  
      
  PP[0][0] += Pdot[0] * DeltaTime;   //Pk--先验估计误差协方差的微分的积分=先验估计误差协方差
  PP[0][1] += Pdot[1] * DeltaTime;     
  PP[1][0] += Pdot[2] * DeltaTime;  
  PP[1][1] += Pdot[3] * DeltaTime;  
      
  angle_err = angle - g_mpu9250.Angle_Kalman ;    //zk-先验估计
      
  PCt_0 = C_0 * PP[0][0];  
  PCt_1 = C_0 * PP[1][0];  
      
  E = R_angle + C_0 * PCt_0;  
      
  K_0 = PCt_0 / E;  
  K_1 = PCt_1 / E;  
      
  t_0 = PCt_0;  
  t_1 = C_0 * PP[0][1];  
      
  PP[0][0] -= K_0 * t_0;     //后验估计误差协方差
  PP[0][1] -= K_0 * t_1;  
  PP[1][0] -= K_1 * t_0;  
  PP[1][1] -= K_1 * t_1;  
      
  g_mpu9250.Angle_Kalman += K_0 * angle_err; //后验估计
  Q_bias  += K_1 * angle_err;    //后验估计
  g_mpu9250.Gyro_Kalman = gyro - Q_bias ;    //输出值(后验估计)的微分=角速度
}  

void Complementary_Filter_1st_Order (float angle, float gyro )  
{  
    const float K1 =0.01;
    g_mpu9250.Angle_Complement_1st = K1 * angle + ( 1-K1 ) * ( g_mpu9250.Angle_Complement_1st + gyro * DeltaTime );  
}  

void Complementary_Filter_2nd_Order(float angle, float gyro )  
{  
    const float K1 =0.2;  
    float x1,x2,y1;
	
    x1 = ( angle - g_mpu9250.Angle_Complement_2nd ) * ( 1 - K1 ) * ( 1 - K1 ) ;  
    y1 = y1 + x1 * DeltaTime ;  
    x2 = y1 + 2 * (1-K1) * ( angle - g_mpu9250.Angle_Complement_2nd ) + gyro ;   
    g_mpu9250.Angle_Complement_2nd = g_mpu9250.Angle_Complement_2nd + x2 * DeltaTime;  
}  

void Complementary_Filter (float angle, float gyro )
{
   g_mpu9250.Angle_Complement_1st += ( ( angle - g_mpu9250.Angle_Complement_1st) * 0.3 + gyro ) * 0.01 ;  
}
