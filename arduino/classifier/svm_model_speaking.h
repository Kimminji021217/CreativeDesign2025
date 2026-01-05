// speaking vs rest
#ifndef SVM_MODEL_SPEAKING_H
#define SVM_MODEL_SPEAKING_H

#define FEAT_DIM 5

static const float svm_w_speaking[5]     = { 0.95551591f, 1.20817750f, -0.07147999f, -1.50180175f, 0.01371082f, };
static const float svm_mu_speaking[5]    = { -0.39932236f, -1.92020725f, -2.01828943f, -2.23934369f, -2.75789605f, };
static const float svm_sigma_speaking[5] = { 0.80530384f, 1.26895659f, 1.42084620f, 1.51816914f, 1.44271209f, };
static const float svm_bias_speaking = -0.63028096f;

#endif // SVM_MODEL_SPEAKING_H
