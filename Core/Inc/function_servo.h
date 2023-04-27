// Master function?
int Servo_master_function(int mod_servo, int *timer, int *abort_signal, int *stop, int *is_first_swipe);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////  Codificare mod lucru  ///////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	/*
	 * mode = 0 -> ecran gol
	 * mode = 1 -> stergereX1
	 * mode = 2 -> treapta1
	 * mode = 3 -> treapta2
	 * mode = 4 -> treapta3
	 * mode = 5 -> spalare
	 */

// Functii prototip pentru modul de lucru al servo motorului

int Servo_up_movement(int *timer, int start_time, int end_time, int *abort_signal, int *is_first_swipe);
int Servo_down_movement(int *timer, int start_time, int end_time, int *abort_signal, int *stop);

int Servo_spalare(int *timer, int* abort_signal, int *stop, int* is_first_swipe);
