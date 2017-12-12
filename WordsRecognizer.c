//#include "stm32f4_discovery.h"
#include "arm_const_structs.h"

#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif

#include "WordsRecognizer.h"

#ifdef FREERTOS

#include "FreeRtos.h"
#define malloc(size) pvPortMalloc(size)
#define free(ptr) vPortFree(ptr)

#endif

/******************************************************************************************************
********	Defines
*******************************************************************************************************/
#ifndef PI
#define PI	3.14159265358979f
#endif
#ifndef NULL
#define NULL 0
#endif
#define EULER 2.718281828

/******************************************************************************************************
********	Variaveis externas
*******************************************************************************************************/

/******************************************************************************************************
********	Headers das funções Privadas
*******************************************************************************************************/
static void WordHammingWindow(Words* words);
static void WordNextPow2(Words* words);
static void WordLinSpace(Words* words);

static void WordFindPeaks(Words* words, float* single_sided, unsigned int* peak_locations);

static void WordExtractFeaturesFrame(Words* words, float* audio_frame, float* frame_frequencies);
static void WordExtractFeatures(Words* words, float* audio, unsigned int audio_size, arm_matrix_instance_f32* frame_frequencies);
static float WordLogLikelihood(Words* words, unsigned char word_position, arm_matrix_instance_f32* frame_frequencies);
static void WordStateLakelihood(Words* words, unsigned char word_position, arm_matrix_instance_f32* frame_frequencies, arm_matrix_instance_f32* B);

static void WordMvnpdf(Words* words, unsigned char word_position, unsigned char state, arm_matrix_instance_f32* X, float* quadform);
static float WordForward(Words* words, unsigned char word_position, arm_matrix_instance_f32* frame_frequencies, arm_matrix_instance_f32* B);

/******************************************************************************************************
********	Funcoes Privadas
*******************************************************************************************************/
static void WordHammingWindow(Words* words){
	unsigned int i;
	float w;
	unsigned int half_frame_size = words->frame_size/2;
	unsigned int last_frame = words->frame_size-1;
	float x = (float)2.0*PI*((float)1.0/last_frame);
	
	if(words->hamming_window)
		free(words->hamming_window);
	words->hamming_window = (float*)malloc(words->frame_size*sizeof(float));
	
	for(i = 0; i < half_frame_size; i++){
		w = (float)0.54 - (float)0.46 * arm_cos_f32(x * i);
		words->hamming_window[i] = w;
		words->hamming_window[last_frame-i] = w;
	}
}

static void WordNextPow2(Words* words){
	unsigned int pow2 = 1;
	
	while(pow2 < words->frame_size)
		pow2 <<= 1;
	
	words->NFFT = pow2;
}

static void WordLinSpace(Words* words){
	unsigned int i;
	unsigned int size = words->NFFT/2+1;
	float step = words->sample_rate/2.0/(words->NFFT/2.0);
	
	if(words->f)
		free(words->f);
	words->f = (float*)malloc(size*sizeof(float));
	
	for(i = 0; i < size; i++)
		words->f[i] = step * i;
}

static void WordFindPeaks(Words* words, float* single_sided, unsigned int* peak_locations){
	unsigned int i, j, k;
	unsigned int end = words->NFFT/2;
	
	for(i = 0; i < words->D; i++)
		peak_locations[i] = 0;
	
	for(i = 1; i < end; i++)
		if(single_sided[i] > single_sided[i-1] && single_sided[i] > single_sided[i+1])
			for(j = 0; j < words->D; j++)
				if(peak_locations[j] == 0 || single_sided[i] > single_sided[peak_locations[j]]){
					for(k = words->D-1; k > j; k--)
						peak_locations[k] = peak_locations[k-1];
					peak_locations[j] = i;
					break;
				}
}

static void WordExtractFeaturesFrame(Words* words, float* audio_frame, float* frame_frequencies){
	unsigned int i, j;
	
	//Floating-point vector multiplication between audio_frame and hamming_window, putting the result on x
	arm_mult_f32(audio_frame, words->hamming_window, words->x, words->frame_size); 	
	
	for(i = words->frame_size; i < words->NFFT; i++)
		words->x[i] = 0.0;
	
	j = 2 * words->NFFT - 1;
	for(i = words->NFFT-1; i > 0; i--){
		words->x[j--] = 0.0;
		words->x[j--] = words->x[i];
	}
	words->x[j--] = 0.0;
	
	
	/*first - points to an instance of the floating-point CFFT structure.
	second - points to the complex data buffer of size 2*fftLen. Processing occurs in-place
	third - flag that selects forward (ifftFlag=0) or inverse (ifftFlag=1) transform
	fourth - flag that enables (bitReverseFlag=1) or disables (bitReverseFlag=0) bit reversal of output*/
	arm_cfft_f32(&arm_cfft_sR_f32_len128, words->x, 0, 1); //computes a 64-point inverse complex FFT including bit reversal
	
	/*first - points to complex input buffer
	second - points to real output buffer
	third - number of complex samples in the input vector*/
	arm_cmplx_mag_f32(words->x, words->X, words->NFFT/2+1); //Computes the magnitude of the elements of a complex data vector.
	
	/*first - points to the input vector
	second - scale factor to be applied
	third -	points to the output vector
	fourth - number of samples in the vector*/
	arm_scale_f32(words->X, 1.0/words->frame_size, words->x, words->NFFT/2+1); //Multiplies a floating-point vector by a scalar
	
	WordFindPeaks(words, words->x, words->peak_locations);
	for(i = 0; i < words->D; i++)
		frame_frequencies[i] = words->f[words->peak_locations[i]];
}

static void WordExtractFeatures(Words* words, float* audio, unsigned int audio_size, arm_matrix_instance_f32* frame_frequencies){
	unsigned int i;
	unsigned int step = words->frame_size * (100 - words->overlap) / 100;
	unsigned int num_frames = audio_size / step - 1;
	
	for(i = 0; i < num_frames; i++)
		WordExtractFeaturesFrame(words, &audio[i * step], &((frame_frequencies->pData)[i * words->D]));
}

static void WordMvnpdf(Words* words, unsigned char word_position, unsigned char state, arm_matrix_instance_f32* X, float* quadform){
	unsigned int i, j;
	static arm_matrix_instance_f32 X0 = {0,0,NULL};
	static arm_matrix_instance_f32 xRinv = {0,0,NULL};
	float* mu = &(words->words[word_position].mu.pData[state*words->D]);
	
	if(X->numCols != X0.numCols || X->numRows != X0.numRows){
		if(X0.pData)
			free(X0.pData);
		
		if(xRinv.pData)
			free(xRinv.pData);
		
		X0.numCols = X->numCols;
		X0.numRows = X->numRows;
		X0.pData = (float*)malloc(X0.numCols*X0.numRows*sizeof(float));
	
		xRinv.numCols = X->numCols;
		xRinv.numRows = X->numRows;
		xRinv.pData = (float*)malloc(xRinv.numCols*xRinv.numRows*sizeof(float));
	}
	
	for(i = 0; i < X0.numRows; i++)
		arm_sub_f32(&(X->pData[i * X->numCols]), mu, &(X0.pData[i * X0.numCols]), words->D);
	
	arm_mat_mult_f32(&X0, &(words->words[word_position].inv_chol_sigma[state]), &xRinv);
	
	for(i = 0; i < xRinv.numRows; i++){
		quadform[i] = 0.0;
		for(j = 0; j < xRinv.numCols; j++){
			xRinv.pData[i*xRinv.numCols+j] *= xRinv.pData[i*xRinv.numCols+j];
			quadform[i] += xRinv.pData[i*xRinv.numCols+j];
		}
		quadform[i] *= -0.5;
		quadform[i] -= words->words[word_position].log_sqrt_det_sigma[state];
		quadform[i] = powf(EULER, quadform[i]);
	}
}

static float WordForward(Words* words, unsigned char word_position, arm_matrix_instance_f32* frame_frequencies, arm_matrix_instance_f32* B){
	arm_matrix_instance_f32 alpha, alpha_col, B_col, alpha_tmp;
	unsigned int i, j;
	float alpha_sum;
	float log_likelihood = 0.0;
	
	alpha.numRows = B->numCols;
	alpha.numCols = B->numRows;
	alpha.pData = (float*)malloc(alpha.numCols*alpha.numRows*sizeof(float));
	
	alpha_tmp.numRows = B->numRows;
	alpha_tmp.numCols = 1;
	alpha_tmp.pData = (float*)malloc(alpha_tmp.numRows*sizeof(float));
	
	alpha_col.numRows = alpha.numCols;
	alpha_col.numCols = 1;
	alpha_col.pData = alpha.pData;
	
	B_col.numRows = B->numCols;
	B_col.numCols = 1;
	B_col.pData = (float*)malloc(B_col.numRows*sizeof(float));
	
	for(i = 0; i < alpha_tmp.numRows; i++)
		alpha_col.pData[i] = B->pData[i * B->numCols];
	
	for(i = 1; i < frame_frequencies->numRows; i++){
		arm_mat_mult_f32(&words->words[word_position].transpost_A, &alpha_col, &alpha_tmp);
		
		for(j = 0; j < alpha_tmp.numRows; j++)
			B_col.pData[j] = B->pData[j * B->numCols + i];
		arm_mult_f32(B_col.pData, alpha_tmp.pData, &alpha.pData[i * alpha.numCols], alpha_tmp.numRows);
		
		alpha_sum = 0.0;
		for(j = 0; j < alpha.numCols; j++)
			alpha_sum += alpha.pData[i * alpha.numCols + j];
		
		if(alpha_sum == (float)0.0)
			alpha_sum = 0.0000000000000000000001;
		
		for(j = 0; j < alpha.numCols; j++)
			alpha.pData[i * alpha.numCols + j] /= alpha_sum;
		
		for(j = 0; j < alpha_tmp.numRows; j++)
			alpha_col.pData[j] = alpha.pData[i * alpha.numCols + j];
		
		log_likelihood += log(alpha_sum);
	}
	
	free(alpha.pData);
	free(alpha_tmp.pData);
	free(B_col.pData);
	
	return log_likelihood;
}

static void WordStateLakelihood(Words* words, unsigned char word_position, arm_matrix_instance_f32* frame_frequencies, arm_matrix_instance_f32* B){	
	unsigned int i;
	
	for(i = 0; i < words->N; i++)
		WordMvnpdf(words, word_position, i, frame_frequencies, &(B->pData[i * B->numCols]));
}

static float WordLogLikelihood(Words* words, unsigned char word_position, arm_matrix_instance_f32* frame_frequencies){
	arm_matrix_instance_f32 B; //create a floating-point matrix structure
	float log_likelihood;
	
	B.numCols = frame_frequencies->numRows;
	B.numRows = words->N;
	B.pData = (float*)malloc(B.numRows*B.numCols*sizeof(float));

	WordStateLakelihood(words, word_position, frame_frequencies, &B);
	log_likelihood = WordForward(words, word_position, frame_frequencies, &B);
	
	free(B.pData);
	
	return log_likelihood;
}

/******************************************************************************************************
********	Funcoes Publicas
*******************************************************************************************************/
unsigned char WordsRecognizerInit(Words* words){
	words->hamming_window = NULL;
	words->f = NULL;
	
	WordHammingWindow(words);
	WordNextPow2(words);
	WordLinSpace(words);
	
	words->x = (float*)malloc(words->NFFT*2*sizeof(float));
	words->X = (float*)malloc((words->NFFT/2+1)*sizeof(float));
	words->peak_locations = (unsigned int*)malloc(words->D*sizeof(unsigned int));
	
	return 0;
}

unsigned char WordRecognize(Words* words, float* audio_signal, unsigned int audio_signal_size){
	arm_matrix_instance_f32 frame_frequencies;
	unsigned int i, prediction;
	float likelihood, max_likelihood;
	
	
	frame_frequencies.numCols = words->D;
	frame_frequencies.numRows = audio_signal_size / (words->frame_size * (100 - words->overlap) / 100) - 1;
	frame_frequencies.pData = (float*)malloc(frame_frequencies.numRows*frame_frequencies.numCols*sizeof(float));
	
	WordExtractFeatures(words, audio_signal, audio_signal_size, &frame_frequencies);// Extract the relevant features
	
	max_likelihood = -1000000000.0;
	prediction = 0;
	for(i = 0; i < words->number_of_words; i++){
		likelihood = WordLogLikelihood(words, i, &frame_frequencies);
		if(likelihood > max_likelihood){
			max_likelihood = likelihood;
			prediction = i;
		}
	}
	free(frame_frequencies.pData);
	
	return prediction;
}
