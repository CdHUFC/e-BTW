/*
 * DCMotors.h
 *
 *  Created on: 7 de dez de 2018
 *      Author: esteves
 */

#ifndef DCMOTORS_H_
#define DCMOTORS_H_

#include "DCMotors.h"
#include "gpio.h"
#include <stdint.h>

#define DELTA_COL_LIN 		11 // 2.7 mm
#define DELTA_CHAR_H 		8  // 1.9 mm
#define DELTA_CHAR_V 		14 // 3.4 mm

#define P_FRACTION 			1.0     //Proportional factor of control loop 0.001 - 10.0 (1.0)
#define STEP_MARGIN 		10     //10 - 1000 (1)

#define MIN_DUTYCYCLE 		175   //0 - 255 (125)
#define MAX_DUTYCYCLE 		255  //0 - 255 (255)

#define PIERCE_TIME			100

#define UP					1
#define DOWN				0

typedef struct{
	signed long setPoint;
	uint16_t actualPoint;
	uint8_t stepStatusOld;

	TIM_HandleTypeDef* timer;

	GPIO_TypeDef* left_port;
	GPIO_TypeDef* right_port;

	uint16_t left_pin;
	uint16_t right_pin;

	uint8_t dutyCycle;
}MotorControl_t;

typedef struct{
	GPIO_TypeDef* A_PORT;
	uint16_t A_PIN;

	GPIO_TypeDef* B_PORT;
	uint16_t B_PIN;
}MotorControl_Simple_t;

void motorBegin(MotorControl_t* motor, TIM_HandleTypeDef* timer, GPIO_TypeDef* left_port, uint16_t left_pin, GPIO_TypeDef* right_port, uint16_t right_pin);
void updateAxis(MotorControl_t* motor,signed long setPoint);
void motorBackward(MotorControl_Simple_t* motor, uint32_t time);
void motorForward(MotorControl_Simple_t* motor, uint32_t time);
void motorSimpleBegin(MotorControl_Simple_t* motor, GPIO_TypeDef* a_port, uint16_t a_pin, GPIO_TypeDef* b_port, uint16_t b_pin);
void pierce(MotorControl_Simple_t* motor, uint32_t time);
void updateAxis_Simple(MotorControl_t* motor, uint32_t time, uint8_t direction);
double myABS(double num1);

#endif /* DCMOTORS_H_ */
