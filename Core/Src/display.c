#include "display.h"
#include "icons.h"
#include "lcd_st7565.h"

int cleared = 0;		// Pt treptele de viteza
int displayBusy = 0;	// Pt a evita suprapunerile de imagini

//Master function?
void modLucruDisplay(int modDisplay){
	if (displayBusy == 0 || modDisplay == 0){
		clearScreen();
		cleared = 0;
		displayBusy = 0;
	}

	if(modDisplay == 1){
		displayStergereX1();
	}

	if(modDisplay == 2){
		displayTreapta1();
	}

	if(modDisplay == 3){
		displayTreapta2();
	}

	if(modDisplay == 4){
		displayTreapta3();
	}

	if(modDisplay == 5){
		displaySpalareParbriz();
	}

	if(modDisplay == 6){
		displaySpalareLuneta();
	}

}

// Functiile pentru afisat modul de lucru

void displayStergereX1(){
	if(displayBusy == 0){
		displayBusy = 1;
		loadX1();
	}
}

void displayTreapta1(){
		displayBusy = 1;

		if(cleared != 0){	// Verifica daca este nevoie sa curete ecranul pt afisaj
			clearScreen();	// (trecere de la o treapta la alta)
			cleared = 0;
		}

		loadTreapta1();
}

void displayTreapta2(){
		displayBusy = 1;

		if(cleared != 1){	// Verifica daca este nevoie sa curete ecranul pt afisaj
			clearScreen();	// (trecere de la o treapta la alta)
			cleared = 1;
		}

		loadTreapta2();
}

void displayTreapta3(){
		displayBusy = 1;

		if(cleared != 2){	// Verifica daca este nevoie sa curete ecranul pt afisaj
			clearScreen();	// (trecere de la o treapta la alta)
			cleared = 2;
		}

		loadTreapta3();
}

void displaySpalareParbriz(){
	if(displayBusy == 0){
		displayBusy = 1;
		loadSpalareParbriz();
	}
}

void displaySpalareLuneta(){
	if(displayBusy == 0){
		displayBusy = 1;
		loadSpalareLuneta();
	}
}
