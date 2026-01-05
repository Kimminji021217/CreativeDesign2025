// silence vs rest
#ifndef SVM_MODEL_SILENCE_H
#define SVM_MODEL_SILENCE_H

#define FEAT_DIM 5

static const float svm_w_silence[5]     = { -1.09323702f, 0.38069883f, -0.95743482f, -0.62359134f, -0.48924356f, };
static const float svm_mu_silence[5]    = { -0.39932236f, -1.92020725f, -2.01828943f, -2.23934369f, -2.75789605f, };
static const float svm_sigma_silence[5] = { 0.80530384f, 1.26895659f, 1.42084620f, 1.51816914f, 1.44271209f, };
static const float svm_bias_silence = -1.65813164f;

#endif // SVM_MODEL_SILENCE_H
