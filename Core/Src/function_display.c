#include <function_display.h>
#include <function_icons.h>
#include "lcd_st7565.h"

int cleared = 0;		// Pt treptele de viteza
int displayBusy = 0;	// Pt a evita suprapunerile de imagini

//Master function?
void Display_modLucruDisplay(int modDisplay){
	if (displayBusy == 0 || modDisplay == 0){
		Icons_clearScreen();
		cleared = 0;
		displayBusy = 0;
	}

	if(modDisplay == 1){
		Display_displayStergereX1();
	}

	if(modDisplay == 2){
		Display_displayTreapta1();
	}

	if(modDisplay == 3){
		Display_displayTreapta2();
	}

	if(modDisplay == 4){
		Display_displayTreapta3();
	}

	if(modDisplay == 5){
		Display_displaySpalareParbriz();
	}

	if(modDisplay == 6){
		Display_displaySpalareLuneta();
	}

}

// Functiile pentru afisat modul de lucru

void Display_displayStergereX1(){
	if(displayBusy == 0){
		displayBusy = 1;
		Icons_loadX1();
	}
}

void Display_displayTreapta1(){
		displayBusy = 1;

		if(cleared != 0){	// Verifica daca este nevoie sa curete ecranul pt afisaj
			Icons_clearScreen();	// (trecere de la o treapta la alta)
			cleared = 0;
		}

		Icons_loadTreapta1();
}

void Display_displayTreapta2(){
		displayBusy = 1;

		if(cleared != 1){	// Verifica daca este nevoie sa curete ecranul pt afisaj
			Icons_clearScreen();	// (trecere de la o treapta la alta)
			cleared = 1;
		}

		Icons_loadTreapta2();
}

void Display_displayTreapta3(){
		displayBusy = 1;

		if(cleared != 2){	// Verifica daca este nevoie sa curete ecranul pt afisaj
			Icons_clearScreen();	// (trecere de la o treapta la alta)
			cleared = 2;
		}

		Icons_loadTreapta3();
}

void Display_displaySpalareParbriz(){
	if(displayBusy == 0){
		displayBusy = 1;
		Icons_loadSpalareParbriz();
	}
}

void Display_displaySpalareLuneta(){
	if(displayBusy == 0){
		displayBusy = 1;
		Icons_loadSpalareLuneta();
	}
}
