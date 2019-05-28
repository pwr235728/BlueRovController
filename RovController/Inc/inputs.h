/*
 * inputs.h
 *
 *  Created on: 22.03.2019
 *      Author: proxima
 */

#ifndef INPUTS_H_
#define INPUTS_H_
#include "main.h"


#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))


// Kolejność potenjcometrów w boforze czytanym przez DMA z ADC
#define POT1 6
#define POT2 5
#define POT3 7
#define POT4 4
#define POT5 3
#define POT6 2
#define POT_X 0
#define POT_Y 1

// Parametry filtorwania
#define p_def  0.5f
#define q_def  (1.0f - p_def)

// Zmierzone poprawki do ADC
#define ADC_MIN 0
#define ADC_MAX  (4095.0f * 0.98f)

#define JOY_X_CENTER -700
#define JOY_Y_CENTER 200

	/*
	 Zmierzone wartosci potrzebne do kalibracji joystika
	 srodek osi X: -700
	 srodek osi Y: 200

	 max wszystkiego to 98% zakresu adc
	 */

volatile typedef struct
{
	volatile uint32_t joy_lock;
	volatile uint32_t display_motors;

}buttons_t;

volatile typedef struct
{
	volatile float joy_x;
	volatile float joy_y;

	volatile float pot6;
	volatile float pot5;
	volatile float pot4;
	volatile float pot2;
	volatile float pot1;
	volatile float pot3;
}pots_t;


// Konfiuracja i rozpoczęcie pomiarów
void pots_start(pots_t* adc_out);

// Filtorwanie odczytów z adc
void pots_filter(pots_t* pots_new_values, pots_t* pots_filtered, float p, float q);

// Pobranie stanu przełączników
void read_buttons(buttons_t *buttons);


// zamiana zakresu ADC 4096 wartosci na zakres -10000 do 10000
float _map(float x, float in_min, float in_max, float out_min, float out_max);


// Oblicza wartosc z adc na "os" w zakresie +/- 1.00f  z zadana martwa strefa
float calculate_axis(uint16_t adc_value, float dead_zone);
#endif /* INPUTS_H_ */
