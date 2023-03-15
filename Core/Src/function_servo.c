#include <function_servo.h>
#include "main.h"

// Master function?
void Servo_modLucruServo(int modServo){
	if(modServo == 0){
		TIM3 -> CCR4 = 500;   // 0%
	}

	if(modServo == 1){
		Servo_servoStergereX1();
	}

	if(modServo == 2){
		Servo_servoTreapta1();
	}

	if(modServo == 3){
		Servo_servoTreapta2();
	}

	if(modServo == 4){
		Servo_servoTreapta3();
	}

	if(modServo == 5){
		Servo_servoSpalareParbriz();
	}

	if(modServo == 6){
		Servo_servoSpalareLuneta();
	}
}

//Functiile pentru modul de lucru al servo motorului

void Servo_servoStergereX1(){
	TIM3 -> CCR4 = 500;   // 0%
	HAL_Delay(200);

	TIM3 -> CCR4 = 1000; // 25%
	HAL_Delay(200);

	TIM3 -> CCR4 = 1450; // 55%
	HAL_Delay(200);

	TIM3 -> CCR4 = 2000; // 85%
	HAL_Delay(200);

	TIM3 -> CCR4 = 2400; // 100%
	HAL_Delay(250);

	TIM3 -> CCR4 = 500;   // 0%
	HAL_Delay(200);

}

void Servo_servoTreapta1(){
	TIM3 -> CCR4 = 500;   // 0%
	HAL_Delay(150);

	TIM3 -> CCR4 = 1000; // 25%
	HAL_Delay(150);

	TIM3 -> CCR4 = 1450; // 55%
	HAL_Delay(150);

	TIM3 -> CCR4 = 2000; // 85%
	HAL_Delay(150);

	TIM3 -> CCR4 = 2400; // 100%
	HAL_Delay(150);

	TIM3 -> CCR4 = 500;   // 0%
	HAL_Delay(200);
}

void Servo_servoTreapta2(){
	TIM3 -> CCR4 = 500;   // 0%
	HAL_Delay(100);

	TIM3 -> CCR4 = 1000; // 25%
	HAL_Delay(100);

	TIM3 -> CCR4 = 1450; // 55%
	HAL_Delay(100);

	TIM3 -> CCR4 = 2000; // 85%
	HAL_Delay(100);

	TIM3 -> CCR4 = 2400; // 100%
	HAL_Delay(100);

	TIM3 -> CCR4 = 500;   // 0%
	HAL_Delay(200);
}

void Servo_servoTreapta3(){
	TIM3 -> CCR4 = 500;   // 0%
	HAL_Delay(75);

	TIM3 -> CCR4 = 1000; // 25%
	HAL_Delay(75);

	TIM3 -> CCR4 = 1450; // 55%
	HAL_Delay(75);

	TIM3 -> CCR4 = 2000; // 85%
	HAL_Delay(75);

	TIM3 -> CCR4 = 2400; // 100%
	HAL_Delay(75);

	TIM3 -> CCR4 = 500;   // 0%
	HAL_Delay(200);
}

void Servo_servoSpalareParbriz(){
	HAL_Delay(2000);
}

void Servo_servoSpalareLuneta(){
	HAL_Delay(2000);
}
