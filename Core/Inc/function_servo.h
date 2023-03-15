// Master function?
void Servo_modLucruServo(int modLucru);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////  Codificare mod lucru  ///////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	/*
	 * mode = 0 -> ecran gol
	 * mode = 1 -> stergereX1
	 * mode = 2 -> treapta1
	 * mode = 3 -> treapta2
	 * mode = 4 -> treapta3
	 * mode = 5 -> spalareParbriz
	 * mode = 6 -> spalareLuneta
	 */

// Functii prototip pentru modul de lucru al servo motorului
void Servo_servoStergereX1();
void Servo_servoTreapta1();
void Servo_servoTreapta2();
void Servo_servoTreapta3();
void Servo_servoSpalareParbriz();
void Servo_servoSpalareLuneta();
