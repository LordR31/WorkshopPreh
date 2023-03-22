// Master function?
int Servo_master_function(int modLucru, int *counter, int *abort);

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

int Servo_up_movement(int *counter, int timeUnit, int *abort);
int Servo_down_movement(int *counter, int timeUnit, int *abort);

int Servo_spalare_parbriz(int *counter);
int Servo_spalare_luneta(int *counter);
