// Master function
void Display_master_function(int mod_lucru_display);

// Start screen
int Display_start_screen(int* timer);

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

// Functii prototip pentru afisajul pe display a modului de lucru
void Display_stergere_x1();
void Display_treapta_1();
void Display_treapta_2();
void Display_treapta_3();
void Display_spalare_parbriz();
void Display_spalare_luneta();
