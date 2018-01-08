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
	arm_matrix_instance_f32 transpost_A; // NxN transition probability matrix
	arm_matrix_instance_f32 mu; // DxN mean vector (D = number of features)
	arm_matrix_instance_f32* inv_chol_sigma; // DxDxN covariance matrix
	float* log_sqrt_det_sigma; // Nx1 initial state distribution vector?
} Word;

typedef struct {
	unsigned char N; // number of states
	unsigned char D; // number of features 
	unsigned char number_of_words; // total number of words
	unsigned int sample_rate;
	unsigned int frame_size;
	unsigned char overlap;
	
	unsigned int NFFT; // Next Pow of 2 of overlap //length of the signal you want to calculate the Fourier transform of (128)
	float* hamming_window; // Hamming window
	float32_t* f; // Linear space
	float* x; // signal after hamming window process (complex) (255)
	float* X; // signal (real)
	unsigned int* peak_locations;
	
	Word* words;
} Words;

/******************************************************************************************************
********	Headers das funções Publicas
*******************************************************************************************************/
unsigned char wordRecognizerInit(Words* words);
unsigned char wordRecognize(Words* words, float* audio_signal, unsigned int audio_signal_size);

#endif
