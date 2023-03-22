// TODO Tweak timer time numbers

#include <function_servo.h>
#include "main.h"

#define ARRAY_SIZE 48

int pasiServo[ARRAY_SIZE] = {500, 500, 500, 500, 500, 550, 600, 650, 700, 750,
							 800, 850, 900, 950, 1000, 1050, 1100, 1150, 1200, 1250, 1300, 1350,
							 1400, 1450, 1500, 1550, 1600, 1650, 1700, 1750, 1800, 1850, 1900, 1950,
							 2000, 2050, 2100, 2150, 2200, 2250, 2300, 2350, 2400, 2500, 2500, 2500,
							 2500, 2500};

// Master function
int Servo_master_function(int modServo, int *timer, int *abort)
{
	int modLucruSistem = 1;
	switch (modServo)
	{
	case 0:
		TIM3->CCR4 = 500; // 0%
		modLucruSistem = 0;
		break;
	case 1: // swipe x1
		Servo_up_movement(timer, 30, abort);
		modLucruSistem = Servo_down_movement(timer, 30, abort);
		break;
	case 2: // treapta 1
		Servo_up_movement(timer, 15, abort);
		modLucruSistem = Servo_down_movement(timer, 15, abort);
		break;
	case 3: // treapta 2
		Servo_up_movement(timer, 10, abort);
		modLucruSistem = Servo_down_movement(timer, 10, abort);
		break;
	case 4: // treapta 3
		Servo_up_movement(timer, 5, abort);
		modLucruSistem = Servo_down_movement(timer, 5, abort);
		break;
	case 5: // spalare parbriz
		modLucruSistem = Servo_spalare_parbriz(timer);
		break;
	case 6: // spalare luneta
		modLucruSistem = Servo_spalare_luneta(timer);
		break;
	}

	return modLucruSistem;
}

// Functiile pentru modul de lucru al servo motorului

int Servo_spalare_parbriz(int *timer)
{ // Reseteaza timer-ul si asteapta 2s
	*timer = 0;
	while (1)
	{
		if (*timer == 2000)
			break;
	}
	return 0;
}

int Servo_spalare_luneta(int *timer)
{ // Reseteaza timer-ul si asteapta 2s
	*timer = 0;
	while (1)
	{
		if (*timer == 2000)
			break;
	}
	return 0;
}

int Servo_up_movement(int *timer, int timeUnit, int *abort)
{
	int i = 0;	// Initializeaza un index
	*timer = 0; // Reseteaza timer-ul
	while (1)
	{
		if (*timer == timeUnit)
		{							   // Cand timer-ul este egal cu unitatea de timp presetata
			TIM3->CCR4 = pasiServo[i]; // se trece la urmatorul pas al servomotorului
			i++;					   // se incrementeaza indexul
			if (i >= ARRAY_SIZE)	   // se verifica daca s-a ajuns la final si se iese (in caz true)
			{
				break;
			}
			*timer = 0; // Se reseteaza timer-ul si se reia
		}

		if (*abort) // Daca se primeste semnalul de abort se iese fortat din functie
		{
			return 0;
		}
	}
	return 0;
}
int Servo_down_movement(int *timer, int timeUnit, int *abort)
{
	int i = 38; // Initializeazaun index
	*timer = 0; // Reseteaza timer-ul
	while (1)
	{
		if (*timer == timeUnit)
		{							   // Cand timer-ul este egal cu unitatea de timp presetata
			TIM3->CCR4 = pasiServo[i]; // se trece la urmatorul pas
			i--;					   // se decrementeaza indexul
			if (i < 0)				   // se verifica daca s-a ajuns la final si se iese (in caz true)
			{
				break;
			}
			*timer = 0; // Se reseteaza timer-ul si se reia
		}

		if (*abort) // Daca se primeste semnalul de abort se iese fortat din functie
		{
			return 0;
		}
	}
	return 0;
}
