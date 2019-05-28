/*
 * inpus.c
 *
 *  Created on: 22.03.2019
 *      Author: proxima
 */

#include "inputs.h"

// Konfiguracja ADC i DMA do odczytu wartości potencjometrów
void pots_start(pots_t* adc_out)
{
	// Konfiguracja adres�w
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
			LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA), adc_out,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

	// Ilosc danych do przeslania
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 8);

	// Uruchamiam przerwania DMA
	//LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	//LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
	//LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);

	// Ucuhomienia kana�u DMA
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	// Uruchomienie ADC
	LL_ADC_Enable(ADC1);

	// Od tej pory wszystkie pomiary z ADC b�d� same si� aktualizowac

	LL_ADC_REG_StartConversion(ADC1);
}


void pots_filter(pots_t* pots_new_values, pots_t* pots_filtered, float p, float q )
{
	float *pots_raw = (float*)pots_filtered;
	float *new_raw = (float*)pots_new_values;
	for (int i = 0; i < 8; i++) {
		pots_raw [i] = p*pots_raw [i] + q*new_raw[i];
	}
}

// Pobranie stanu przełączników
void read_buttons(buttons_t *buttons)
{
	buttons->joy_lock = LL_GPIO_ReadInputPort(JOY_EN_GPIO_Port) & JOY_EN_Pin;
	buttons->display_motors = LL_GPIO_ReadInputPort(ZW_GPIO_Port) & ZW_Pin;
}


// funkcje pomocnicze
// zamiana zakresu ADC 4096 wartosci na zakres -10000 do 10000
float _map(float x, float in_min, float in_max, float out_min, float out_max)
{
	float out = (x - in_min) * (out_max - out_min) / (in_max - in_min)
			+ out_min;

	// obcinanie do zakresy min-max gdy by x by� poza podanym zakresem
	return MAX(MIN(out, out_max), out_min);
}


// Oblicza wartosc z adc na "os" w zakresie +/- 1.00f  z zadana martwa strefa
float calculate_axis(uint16_t adc_value, float dead_zone)
{
	// korekcja na niepoprawny maksymalny zakres adc
	float out = _map((float) adc_value, ADC_MIN, ADC_MAX, -1.0f, 1.0f);

	if (out >= -dead_zone && out <= dead_zone) {
		return 0.0f; // jestesmy w deadzone
	} else if (out > 0) {
		return _map(out, dead_zone, 1.0f, 0.0f, 1.0f);
	} else {
		return _map(out, -1.0f, -dead_zone, -1.0f, 0.0f);
	}
}

float pots_zero(pots_t* pots)
{
	float* f = (float*)pots;
	for(int i = 0; i<8;i++)
		f[i] = 0.0f;
}

