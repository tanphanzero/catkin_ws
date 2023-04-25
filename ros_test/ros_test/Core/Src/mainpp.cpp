/*
 * mainpp.cpp
 *
 *  Created on: Feb 28, 2023
 *      Author: Dell
 */

#define _USE_MATH_DEFINES

#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;


ros::NodeHandle nh;
geometry_msgs::Twist vel_pub;
std_msgs::Float64 set1_lin, set2_lin, set1_ang, set2_ang, u11_s, u21_s;
//double set1 = 5;
//double set2 = 15;
std_msgs::Float64 y11, y21, y11_lin, y21_lin;
const double L = 0.745, R = 0.325/2;
const double T = 0.01;
int16_t N1, preN1;
double y12, y13, y14, ym11, ym12, ym13;
double ec11, ec12, ec13, em11, em12, em13;
double delkp11, delkp12, delkp13;
double delki11, delki12, delki13;
double Kp1, Ki1;
const double gammap1 = 0.00005, gammai1 = 0.00005;
double alpha1, beta1;
double u11, u12;


int16_t N2, preN2;
double y22, y23, y24, ym21, ym22, ym23;
double ec21, ec22, ec23, em21, em22, em23;
double delkp21, delkp22, delkp23;
double delki21, delki22, delki23;
double Kp2, Ki2;
const double gammap2 = 0.00005, gammai2 = 0.00005;
double alpha2, beta2;
double u21, u22;
int time_parameter = 0;

double rpm_to_mps(double rpm, double R)
{
	return rpm*2*M_PI*R/60;
}

double mps_to_rpm(double mps, double R)
{
	return 60*mps/(2*M_PI*R);
}

void _run(void)
{
	// Left wheel
	N1 = __HAL_TIM_GET_COUNTER(&htim1);
	if(abs((int)(N1 - preN1)) < 40000) y13 = ((N1 - preN1)*60.0*100/4000);
//	y13 = 20;
	y11.data = 0.9753*y12 + 0.02469*y14;
	y12 = y11.data; y14 = y13;
	preN1 = N1;
	y11_lin.data = rpm_to_mps(y11.data, R);
	ec11 = set1_ang.data - y11.data;
	ym11 = 0.00122*set1_ang.data + 0.00078*set1_ang.data + 1.921*ym12 - 0.923*ym13;
	em11 = y11.data - ym11;
	delkp11 = 1.921*delkp12 - 0.923*delkp13 + 0.07685*ec12 - 0.07685*ec13;
	Kp1 += -gammap1*em11*delkp11;
	delki11 = 1.921*delki12 - 0.923*delki13 + 0.00039*ec12 + 0.00025*ec13;
	Ki1 += -gammai1*em11*delki11;

	alpha1 = Kp1*(ec11 - ec12);
	beta1 = T/2*Ki1*(ec11 + ec12);
	u11 = u12 + alpha1 + beta1;

	if(u11 < 0)
		{
			if(u11 < 0){
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -u11);
			}
		}
		else
		{
			if(u11 >= 0){
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, u11);
			}
		}

		if(set1_ang.data == 0) u11 = 0;

	u12 = u11;
	ec13 = ec12; ec12 = ec11; em13 = em12; em12 = em11;	ym13 = ym12; ym12 = ym11;
	delkp13 = delkp12; delkp12 = delkp11; delki13 = delki12; delki12 = delki11;

	u11_s.data = u11;

	// Right wheel
	N2 = __HAL_TIM_GET_COUNTER(&htim3);
	if(abs((int)(N2 - preN2)) < 40000) y23 = ((N2 - preN2)*60.0*100/4000);
//	y23 = 20;
	y21.data = 0.9753*y22 + 0.02469*y24;
	y22 = y21.data; y24 = y23;
	preN2 = N2;
	y21_lin.data = rpm_to_mps(y21.data, R);
	ec21 = set2_ang.data - y21.data;
	ym21 = 0.00122*set2_ang.data + 0.00078*set2_ang.data + 1.921*ym22 - 0.923*ym23;
	em21 = y21.data - ym21;
	delkp21 = 1.921*delkp22 - 0.923*delkp23 + 0.07685*ec22 - 0.07685*ec23;
	Kp2 += -gammap2*em21*delkp21;
	delki21 = 1.921*delki22 - 0.923*delki23 + 0.00039*ec22 + 0.00025*ec23;
	Ki2 += -gammai2*em21*delki21;

	alpha2 = Kp2*(ec21 - ec22);
	beta2 = T/2*Ki2*(ec21 + ec22);
	u21 = u22 + alpha2 + beta2;

	if(u21 < 0)
	{
		if(u21 < 0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -u21);
		}
	}
	else
	{
		if(u21 >= 0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, u21);
		}
	}

	if(set2_ang.data == 0) u21 = 0;
	u22 = u21;
	ec23 = ec22; ec22 = ec21; em23 = em22; em22 = em21;	ym23 = ym22; ym22 = ym21;
	delkp23 = delkp22; delkp22 = delkp21; delki23 = delki22; delki22 = delki21;
	u21_s.data = u21;
}
/*
*/
void _get_cmd_vel(const geometry_msgs::Twist& msg)
{
	set1_lin.data = msg.linear.x - msg.angular.z*L/2;
	set2_lin.data = msg.linear.x + msg.angular.z*L/2;
	set1_ang.data = mps_to_rpm(set1_lin.data,R);
	set2_ang.data = mps_to_rpm(set2_lin.data,R);
}

void ex_cmd_vel(void)
{
	vel_pub.linear.x = (y21_lin.data + y11_lin.data)/2;
	vel_pub.angular.z = (y21_lin.data - y11_lin.data)/L;
}
//std_msgs::Int32 num_msg;

//int num_test = 0;
//ros::Publisher num_pub("num", &num_msg);
//ros::Publisher left_set_pub("left_set", &set1_lin);
//ros::Publisher right_set_pub("right_set", &set2_lin);
//ros::Publisher left_pub("left", &y11_lin);
//ros::Publisher right_pub("right", &y21_lin);
//ros::Publisher pub = nh.advertise<geometry_msgs::Twist> ("vel_pub", 1000);
//ros::Publisher u21_pub("u21", &u21_s);

ros::Publisher pub("vel_pub", &vel_pub);
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &_get_cmd_vel);

//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);
//char hello[] = "Hello world from STM32!";

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2)
		nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2)
		nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
  nh.subscribe(sub_cmd_vel);
//  nh.advertise(left_set_pub);
//  nh.advertise(right_set_pub);
//  nh.advertise(left_pub);
//  nh.advertise(right_pub);
    nh.advertise(pub);

//  nh.advertise(u11_pub);
//  nh.advertise(u21_pub);
//  nh.advertise(chatter);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1|TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1|TIM_CHANNEL_2);



}

void loop(void)
{

//	num_msg.data = num_test;
//	num_pub.publish(&num_msg);
//	num_test++;
//	nh.spinOnce();
//
//
//  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
//  str_msg.data = hello;
//  chatter.publish(&str_msg);

  _run();
  ex_cmd_vel();
//  left_set_pub.publish(&set1_lin);
//  right_set_pub.publish(&set2_lin);
//  left_pub.publish(&y11_lin);
//  right_pub.publish(&y21_lin);
  pub.publish(&vel_pub);
//  u11_pub.publish(&u11_s);
//  u21_pub.publish(&u21_s);
  nh.spinOnce();


  HAL_Delay(10);
}


