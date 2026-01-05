// snoring vs rest
#ifndef SVM_MODEL_SNORING_H
#define SVM_MODEL_SNORING_H

#define FEAT_DIM 5

static const float svm_w_snoring[5]     = { -0.17144630f, -0.87495033f, 0.03488554f, 1.63922235f, 0.28860801f, };
static const float svm_mu_snoring[5]    = { -0.39932236f, -1.92020725f, -2.01828943f, -2.23934369f, -2.75789605f, };
static const float svm_sigma_snoring[5] = { 0.80530384f, 1.26895659f, 1.42084620f, 1.51816914f, 1.44271209f, };
static const float svm_bias_snoring = -0.63610069f;

#endif // SVM_MODEL_SNORING_H
