#include "WordsRecognizer.h"

const float word1_transpost_A[] = {
  9.630407e-01, 2.916240e-02, 5.894941e-02, 
  3.021860e-02, 8.969766e-01, 2.632133e-01, 
  6.740688e-03, 7.386099e-02, 6.778373e-01
};

const float word1_transpost_mu[] = {
  4.659819e+02, 7.686544e+02, 9.250010e+02, 1.243409e+03, 1.602473e+03, 1.974255e+03, 
  3.652759e+02, 1.729593e+03, 2.189885e+03, 2.375559e+03, 2.399597e+03, 2.212108e+03, 
  2.332944e+03, 1.645662e+03, 2.047098e+03, 2.024786e+03, 2.510773e+03, 1.741820e+03
};

const float word1_inv_chol_sigma1[] = { //for state 1
  4.179116e-03, -1.260834e-03, 1.263526e-04, 3.878722e-04, -1.371985e-04, -2.630226e-04, 
  0, 3.094341e-03, 7.844901e-05, -6.339753e-05, -1.097173e-04, -2.041553e-04, 
  0, 0, 2.069227e-03, 1.029858e-04, 9.387063e-05, -2.421297e-05, 
  0, 0, 0, 1.417312e-03, -1.757506e-04, -1.161877e-04, 
  0, 0, 0, 0, 1.229454e-03, -2.008905e-04, 
  0, 0, 0, 0, 0, 1.216246e-03
};

const float word1_inv_chol_sigma2[] = { //for state 2
  5.878287e-03, 1.456290e-03, 1.654089e-03, 1.166713e-03, 8.866525e-04, 5.069780e-04, 
  0, 9.425860e-04, -1.973921e-04, 8.226222e-05, 6.020395e-05, 1.806415e-04, 
  0, 0, 1.061329e-03, 1.125052e-05, 1.194236e-04, 1.585190e-04, 
  0, 0, 0, 1.052429e-03, 3.679888e-05, 8.707103e-05, 
  0, 0, 0, 0, 1.002518e-03, 6.308180e-05, 
  0, 0, 0, 0, 0, 9.821327e-04
};

const float word1_inv_chol_sigma3[] = { //for state 3
  1.139135e-03, -5.605872e-04, -4.236666e-04, 3.378612e-04, -6.849609e-05, 1.418351e-04, 
  0, 1.021998e-03, 4.033669e-04, -2.396933e-04, 1.326809e-04, 1.876778e-04, 
  0, 0, 9.158377e-04, -2.558447e-04, -1.779454e-04, 1.068796e-04, 
  0, 0, 0, 8.928897e-04, -1.516666e-04, -1.675897e-05, 
  0, 0, 0, 0, 1.202272e-03, -7.063206e-05, 
  0, 0, 0, 0, 0, 1.041292e-03
};

const float word1log_sqrt_det_sigma[] = {
  3.189495e+01, 
  3.412565e+01, 
  3.575740e+01
};

const arm_matrix_instance_f32 word1_inv_chol_sigma[] = {
  {
    .numCols = 6,
    .numRows = 6,
    .pData = (float*)word1_inv_chol_sigma1
  },
  {
    .numCols = 6,
    .numRows = 6,
    .pData = (float*)word1_inv_chol_sigma2
  },
  {
    .numCols = 6,
    .numRows = 6,
    .pData = (float*)word1_inv_chol_sigma3
  }
};

const float word2_transpost_A[] = {
  5.473815e-01, 4.323333e-01, 5.393269e-01, 
  1.655698e-01, 3.411167e-01, 1.388379e-01, 
  2.870487e-01, 2.265499e-01, 3.218352e-01
};

const float word2_transpost_mu[] = {
  4.474639e+02, 5.268643e+02, 1.067351e+03, 1.358614e+03, 1.593587e+03, 1.841156e+03, 
  9.530743e+02, 1.070762e+03, 7.633872e+02, 1.107216e+03, 1.535846e+03, 1.778758e+03, 
  5.192524e+02, 1.275132e+03, 1.351459e+03, 1.596116e+03, 1.815287e+03, 1.989645e+03
};

const float word2_inv_chol_sigma1[] = {
  4.234982e-03, 1.545445e-03, -8.161106e-04, -5.125741e-04, -1.836984e-04, -2.679798e-04, 
  0, 4.457678e-03, -7.318173e-04, -4.221251e-04, -4.899218e-05, -1.008159e-04, 
  0, 0, 1.931982e-03, -5.712512e-05, -4.673208e-05, -1.190793e-04, 
  0, 0, 0, 1.729351e-03, -9.065422e-05, -6.853867e-05, 
  0, 0, 0, 0, 1.570389e-03, -1.409740e-04, 
  0, 0, 0, 0, 0, 1.468647e-03
};

const float word2_inv_chol_sigma2[] = {
  1.972754e-03, 3.976283e-04, -6.008558e-04, -2.951285e-04, 1.859773e-06, 9.517344e-05, 
  0, 2.092296e-03, 3.058768e-04, 3.148341e-04, 1.752183e-04, 1.331805e-04, 
  0, 0, 1.764348e-03, -1.826808e-04, -8.166695e-05, 9.048805e-05, 
  0, 0, 0, 1.425013e-03, 5.020645e-05, -6.047980e-06, 
  0, 0, 0, 0, 1.326310e-03, 4.125676e-05, 
  0, 0, 0, 0, 0, 1.321497e-03
};

const float word2_inv_chol_sigma3[] = {
  5.117887e-03, -5.216560e-04, 5.450051e-04, -2.910761e-04, -6.599273e-04, -8.067736e-04, 
  0, 1.772204e-03, 1.766359e-04, 2.916713e-04, 3.049971e-04, 9.881983e-05, 
  0, 0, 1.305868e-03, 5.924206e-05, 1.560174e-04, 8.033545e-05, 
  0, 0, 0, 1.314344e-03, -2.830505e-05, -3.033107e-06, 
  0, 0, 0, 0, 1.302637e-03, -8.925568e-05, 
  0, 0, 0, 0, 0, 1.216612e-03
};

const float word2log_sqrt_det_sigma[] = {
  3.095294e+01, 
  3.303208e+01, 
  3.272727e+01
};

const arm_matrix_instance_f32 word2_inv_chol_sigma[] = {
  {
    .numCols = 6,
    .numRows = 6,
    .pData = (float*)word2_inv_chol_sigma1
  },
  {
    .numCols = 6,
    .numRows = 6,
    .pData = (float*)word2_inv_chol_sigma2
  },
  {
    .numCols = 6,
    .numRows = 6,
    .pData = (float*)word2_inv_chol_sigma3
  }
};

const float word3_transpost_A[] = {
  9.661814e-01, 2.208035e-02, 2.131314e-02, 
  2.234232e-02, 9.481451e-01, 2.612492e-02, 
  1.147626e-02, 2.977452e-02, 9.525619e-01
};

const float word3_transpost_mu[] = {
  4.001444e+02, 1.115104e+03, 1.650865e+03, 1.830581e+03, 1.847293e+03, 1.815791e+03, 
  1.466069e+03, 1.820326e+03, 2.088933e+03, 2.039071e+03, 2.338588e+03, 2.024753e+03, 
  5.404997e+02, 6.570561e+02, 8.835878e+02, 1.157476e+03, 1.553058e+03, 1.945513e+03
};

const float word3_inv_chol_sigma1[] = {
  1.348257e-02, 8.201984e-04, 1.152567e-03, -3.113044e-04, 1.572978e-05, 2.639770e-04, 
  0, 1.296468e-03, 3.127201e-06, 6.299450e-05, 2.035219e-05, -2.226304e-05, 
  0, 0, 1.331902e-03, -1.959250e-05, 9.152369e-05, 1.172626e-04, 
  0, 0, 0, 1.270939e-03, -6.843007e-05, 7.664645e-06, 
  0, 0, 0, 0, 1.225598e-03, -1.315494e-04, 
  0, 0, 0, 0, 0, 1.161087e-03
};

const float word3_inv_chol_sigma2[] = {
  9.678897e-04, 4.089142e-05, 5.733333e-05, 7.158151e-05, 2.521967e-05, 2.273725e-04, 
  0, 8.989968e-04, 1.796671e-05, 1.021167e-04, 6.343723e-05, 1.512228e-04, 
  0, 0, 9.049773e-04, -1.961641e-05, -1.115347e-05, 1.006973e-04, 
  0, 0, 0, 9.376837e-04, -7.481828e-05, 2.678700e-05, 
  0, 0, 0, 0, 1.028133e-03, 4.587215e-05, 
  0, 0, 0, 0, 0, 1.012430e-03
};

const float word3_inv_chol_sigma3[] = {
  2.857597e-03, 1.354640e-04, 1.549565e-04, -2.038416e-04, -2.908053e-04, -2.948855e-04, 
  0, 2.730508e-03, 3.464888e-04, 3.624386e-05, -3.246340e-04, -3.587318e-04, 
  0, 0, 2.217665e-03, -4.809549e-05, -3.024598e-04, -3.401348e-04, 
  0, 0, 0, 1.777317e-03, -2.775375e-04, -1.910390e-04, 
  0, 0, 0, 0, 1.458478e-03, -3.084625e-04, 
  0, 0, 0, 0, 0, 1.423606e-03
};

const float word3log_sqrt_det_sigma[] = {
  3.219271e+01, 
  3.619610e+01, 
  3.177629e+01
};

const arm_matrix_instance_f32 word3_inv_chol_sigma[] = {
  {
    .numCols = 6,
    .numRows = 6,
    .pData = (float*)word3_inv_chol_sigma1
  },
  {
    .numCols = 6,
    .numRows = 6,
    .pData = (float*)word3_inv_chol_sigma2
  },
  {
    .numCols = 6,
    .numRows = 6,
    .pData = (float*)word3_inv_chol_sigma3
  }
};

const Word words[] = {
  [0] = {
    .transpost_A = {
      .numCols = 3,
      .numRows = 3,
      .pData = (float*)word1_transpost_A
    },
    .mu = {
      .numCols = 3,
      .numRows = 6,
      .pData = (float*)word1_transpost_mu
    },
    .inv_chol_sigma = (arm_matrix_instance_f32*)word1_inv_chol_sigma,
    .log_sqrt_det_sigma = (float*)word1log_sqrt_det_sigma
  },
  [2] = {
    .transpost_A = {
      .numCols = 3,
      .numRows = 3,
      .pData = (float*)word2_transpost_A
    },
    .mu = {
      .numCols = 3,
      .numRows = 6,
      .pData = (float*)word2_transpost_mu
    },
    .inv_chol_sigma = (arm_matrix_instance_f32*)word2_inv_chol_sigma,
    .log_sqrt_det_sigma = (float*)word2log_sqrt_det_sigma
  },
  [1] = {
    .transpost_A = {
      .numCols = 3,
      .numRows = 3,
      .pData = (float*)word3_transpost_A
    },
    .mu = {
      .numCols = 3,
      .numRows = 6,
      .pData = (float*)word3_transpost_mu
    },
    .inv_chol_sigma = (arm_matrix_instance_f32*)word3_inv_chol_sigma,
    .log_sqrt_det_sigma = (float*)word3log_sqrt_det_sigma
  },
};

Words vocabulary = {
		.N = 3,		// Numero de estados (fonemas?)
		.D = 6,  // Numero de frequencias guardadas por cada frame do sinal
		.number_of_words = 2,
		.sample_rate = 8000, //Fs=16000
		.frame_size = 80, //buffer_mic_size=8000
		.overlap = 50,
		.words = (Word*)words
};
