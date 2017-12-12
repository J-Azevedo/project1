#ifndef __WORDS_RECOGNIZER_H
#define __WORDS_RECOGNIZER_H

#include "arm_math.h"

/******************************************************************************************************
********	Defines
*******************************************************************************************************/
#define FREERTOS

/******************************************************************************************************
********	Estruturas de dados
*******************************************************************************************************/
typedef struct {
	arm_matrix_instance_f32 transpost_A; // transition between states probability matrix A - NxN
	arm_matrix_instance_f32 mu; // DxN Vector significado
	arm_matrix_instance_f32* inv_chol_sigma; // DxDxN matriz da covariância
	float* log_sqrt_det_sigma; // N
} Word;

typedef struct {
	unsigned char N; // Numero de estados
	unsigned char D; // Numero de frequencias guardadas por cada frame do sinal
	unsigned char number_of_words; // Numero total de palavras
	unsigned int sample_rate;
	unsigned int frame_size;
	unsigned char overlap;
	
	unsigned int NFFT; // Next Pow of 2 of overlap //length of the signal you want to calculate the Fourier transform of?
	float* hamming_window; // Hamming window
	float* f; // Linear space
	float* x;
	float* X;
	unsigned int* peak_locations;
	
	Word* words;
} Words;

/******************************************************************************************************
********	Headers das funções Publicas
*******************************************************************************************************/
unsigned char WordsRecognizerInit(Words* words);
unsigned char WordRecognize(Words* words, float* audio_signal, unsigned int audio_signal_size);

#endif
