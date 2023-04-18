// TODO Tweak timer time numbers

#include <function_servo.h>
#include "main.h"

// Master function
int Servo_master_function(int mod_servo, int *timer, int *abort_signal,
		int *is_first_swipe) {
	int mod_lucru_sistem = 1;
	switch (mod_servo) {
	case 0:
		TIM3->CCR4 = 500; // 0%
		mod_lucru_sistem = 0;
		break;
	case 1: // swipe x1
		Servo_up_movement(timer, 0, 300, abort_signal, is_first_swipe);
		mod_lucru_sistem = Servo_down_movement(timer, 0, 300, abort_signal);
		break;
	case 2: // treapta 1
		Servo_up_movement(timer, 1000, 125, abort_signal, is_first_swipe);
		Servo_down_movement(timer, 100, 100, abort_signal);
		mod_lucru_sistem = 2;
		break;
	case 3: // treapta 2
		Servo_up_movement(timer, 500, 175, abort_signal, is_first_swipe);
		Servo_down_movement(timer, 75, 200, abort_signal);
		mod_lucru_sistem = 2;
		break;
	case 4: // treapta 3
		Servo_up_movement(timer, 100, 175, abort_signal, is_first_swipe);
		Servo_down_movement(timer, 75, 200, abort_signal);
		mod_lucru_sistem = 2;
		break;
	case 5: // spalare
		mod_lucru_sistem = Servo_spalare(timer, abort_signal, is_first_swipe);
		break;
	}

	return mod_lucru_sistem;
}

// Functiile pentru modul de lucru al servo motorului

int Servo_up_movement(int *timer, int start_time, int end_time,
		int *abort_signal, int *is_first_swipe) {
	*timer = 0; // Reseteaza timer-ul
	if (*is_first_swipe == 0) {
		while (*timer < start_time) {
			if (*abort_signal)
				return 0;
			continue;
		}
	}
	*timer = 0;
	TIM3->CCR4 = 2400;
	while (*timer < end_time) {
		if (*abort_signal)
			return 0;
		continue;
	}

	if (*abort_signal) {
		return 0;
	}

	return 0;
}

int Servo_down_movement(int *timer, int start_time, int end_time,
		int *abort_signal) {
	*timer = 0; // Reseteaza timer-ul
	while (*timer < start_time) {
		if (*abort_signal)
			return 0;
		continue;
	}
	TIM3->CCR4 = 500; // se trece la urmatorul pas
	while (*timer < end_time) {
		if (*abort_signal)
			return 0;
		continue;
	}

	if (*abort_signal) // Daca se primeste semnalul de abort se iese fortat din functie
	{
		return 0;
	}

	return 0;
}

int Servo_spalare(int *timer, int *abort_signal, int *is_first_swipe) {
	for (int i = 0; i < 3; i++) {
		if (*abort_signal)
			return 0;

		if (i == 0) {
			if (TIM3->CCR4 == 2400) {
				Servo_down_movement(timer, 150, 200, abort_signal);
				Servo_up_movement(timer, 100, 300, abort_signal, is_first_swipe);
			} else {
				Servo_up_movement(timer, 100, 300, abort_signal,is_first_swipe);
				Servo_down_movement(timer, 150, 200, abort_signal);
			}
		}

		if (i > 0) {
			Servo_up_movement(timer, 100, 300, abort_signal, is_first_swipe);
			Servo_down_movement(timer, 150, 200, abort_signal);
		}
		*timer = 0;

		while (*timer < 300) {
			if (*abort_signal)
				return 0;
			continue;
		}
	}

	return 0;
}

