#include <function_display.h>
#include <function_icons.h>
#include "lcd_st7565.h"

/// Variabile de control local ///

int was_screen_cleared = 0; // Verifica daca ecranul a fost curatat. Se foloseste pentru treptele de viteza (functioneaza cu logica negativa)
int is_display_busy = 0;	// Verifica daca pe display este incarcat deja o matrice a evita suprapunerile de imagini

void Display_master_function(int mod_lucru_display) // Functia Master care primeste modul de lucru al display-ului si apeleaza functia corespunzatoare
{
	switch (mod_lucru_display)
	{
	case 0:
		Icons_clear_screen(); // Apeleaza functia de curatare a ecranului
		was_screen_cleared = 0;
		is_display_busy = 0;
		break;
	case 1:
		Display_stergere_x1();
		break;
	case 2:
		Display_treapta_1();
		break;
	case 3:
		Display_treapta_2();
		break;
	case 4:
		Display_treapta_3();
		break;
	case 5:
		Display_spalare_parbriz();
		break;
	case 6:
		Display_spalare_luneta();
		break;
	}

	if (is_display_busy == 0) // Daca display-ul nu este incarcat atunci curata ecranul si reseteaza ambele variabile de control
	{
		Icons_clear_screen(); // Apeleaza functia de curatare a ecranului
		was_screen_cleared = 0;
		is_display_busy = 0;
	}
}

int Display_start_screen(int *timer)
{
	is_display_busy = 1;
	is_display_busy = Icons_start_screen(timer);

	return 1;
}

// Functiile pentru afisat modul de lucru

void Display_stergere_x1()
{
	if (is_display_busy == 0)
	{
		is_display_busy = 1;
		Icons_load_x1();
	}
}

void Display_treapta_1()
{
	is_display_busy = 1;

	if (was_screen_cleared != 0)
	{						  // Verifica daca este nevoie sa curete ecranul pt afisaj
		Icons_clear_screen(); // (trecere de la o treapta la alta)
		was_screen_cleared = 0;
	}

	Icons_load_treapta_1();
}

void Display_treapta_2()
{
	is_display_busy = 1;

	if (was_screen_cleared != 1)
	{						  // Verifica daca este nevoie sa curete ecranul pt afisaj
		Icons_clear_screen(); // (trecere de la o treapta la alta)
		was_screen_cleared = 1;
	}

	Icons_load_treapta_2();
}

void Display_treapta_3()
{
	is_display_busy = 1;

	if (was_screen_cleared != 2)
	{						  // Verifica daca este nevoie sa curete ecranul pt afisaj
		Icons_clear_screen(); // (trecere de la o treapta la alta)
		was_screen_cleared = 2;
	}

	Icons_load_treapta_3();
}

void Display_spalare_parbriz()
{
	if (is_display_busy == 0)
	{
		is_display_busy = 1;
		Icons_load_spalare_parbriz();
	}
}

void Display_spalare_luneta()
{
	if (is_display_busy == 0)
	{
		is_display_busy = 1;
		Icons_load_spalare_luneta();
	}
}
