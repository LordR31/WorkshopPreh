#include "servo.h"
#include "main.h"

// Master function?
void modLucruServo(int modServo){
	if(modServo == 0){
		TIM3 -> CCR4 = 500;   // 0%
	}

	if(modServo == 1){
		servoStergereX1();
	}

	if(modServo == 2){
		servoTreapta1();
	}

	if(modServo == 3){
		servoTreapta2();
	}

	if(modServo == 4){
		servoTreapta3();
	}

	if(modServo == 5){
		servoSpalareParbriz();
	}

	if(modServo == 6){
		servoSpalareLuneta();
	}
}

//Functiile pentru modul de lucru al servo motorului

void servoStergereX1(){
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

void servoTreapta1(){
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

void servoTreapta2(){
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

void servoTreapta3(){
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

void servoSpalareParbriz(){
	HAL_Delay(2000);
}

void servoSpalareLuneta(){
	HAL_Delay(2000);
}
