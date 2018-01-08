#ifndef __WORDS_DATA_H
#define __WORDS_DATA_H

#include "wordRecognizer.h"

extern float word1_trasnpost_A[];

extern float word1_transpost_mu[];

extern float word1_inv_chol_sigma1[];

extern float word1_inv_chol_sigma2[];

extern float word1_inv_chol_sigma3[];

extern float word1log_sqrt_det_sigma[];

extern arm_matrix_instance_f32 word1_inv_chol_sigma[];

extern float word2_trasnpost_A[];

extern float word2_transpost_mu[];

extern float word2_inv_chol_sigma1[];

extern float word2_inv_chol_sigma2[];

extern float word2_inv_chol_sigma3[];

extern float word2log_sqrt_det_sigma[];

extern arm_matrix_instance_f32 word2_inv_chol_sigma[];

extern float word3_trasnpost_A[];

extern float word3_transpost_mu[];

extern float word3_inv_chol_sigma1[];

extern float word3_inv_chol_sigma2[];

extern float word3_inv_chol_sigma3[];

extern float word3log_sqrt_det_sigma[];

extern arm_matrix_instance_f32 word3_inv_chol_sigma[];

extern Word words[];

extern Words vocabulary;

#endif
