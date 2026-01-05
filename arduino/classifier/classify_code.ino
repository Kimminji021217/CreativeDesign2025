#include <PDM.h>
#include <arduinoFFT.h>

// ==========================================
// 설정값
// ==========================================
#define FS           16000      // 16 kHz
#define FFT_SAMPLES  1024       // Nfft (MATLAB과 동일하게)
#define FEAT_DIM     5          // 5-band 에너지
#define SEC_SAMPLES  16000      // ★ 1초당 샘플 수 (FS * 1초)

// 프레임 파라미터 (MATLAB의 frameLen=Nfft, hop=Nfft와 대응)
#define FRAME_LEN    FFT_SAMPLES
#define HOP_SAMPLES  FFT_SAMPLES

// ==========================================
// SVM 파라미터 (MATLAB에서 출력한 값 붙여넣기)
// ==========================================
#include "svm_model_silence.h"
#include "svm_model_speaking.h"
#include "svm_model_snoring.h"

const int CLASS_SILENCE = 0;    // ClassNames(0)
const int CLASS_SPEAKING = 1;   // ClassNames(1)
const int CLASS_SNORING  = 2;   // ClassNames(2)

// ==========================================
// 5개 주파수 대역 (MATLAB bandEdges와 동일해야 함)
// [0 300], [300 600], [600 1200], [1200 2400], [2400 4000]
// ==========================================
const float bandEdgesHz[FEAT_DIM][2] = {
  {   0.0f,   300.0f },
  { 300.0f,   600.0f },
  { 600.0f,  1200.0f },
  {1200.0f,  2400.0f },
  {2400.0f,  4000.0f }
};

// ==========================================
// 버퍼 및 FFT 관련 전역 변수
// ==========================================

// [CHANGED] 1초(16000 샘플) 버퍼
int16_t secBuffer[SEC_SAMPLES];
volatile int secSamplesCollected = 0;
volatile bool secReady = false;

// FFT 버퍼
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];

// Hann window (MATLAB의 hann(Nfft,'periodic')와 대응)
double hannWindow[FFT_SAMPLES];

ArduinoFFT<double> FFT  = ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, FS);

// ==========================================
// Hann window 초기화
// ==========================================
void initHannWindow() {
  // MATLAB: win = hann(Nfft,'periodic');
  for (int n = 0; n < FFT_SAMPLES; ++n) {
    hannWindow[n] = 0.5 * (1.0 - cos(2.0 * PI * n / (FFT_SAMPLES - 1)));
  }
}

// ==========================================
// SVM 추론 함수
// ==========================================
float compute_score(const float *feat,
                    const float *w, const float *mu,
                    const float *sigma, float bias) {
  float s = bias;
  for (int i = 0; i < FEAT_DIM; ++i) {
    float z = (feat[i] - mu[i]) / sigma[i];
    s += w[i] * z;
  }
  return s;
}

// ==========================================
// PDM 콜백: 마이크 데이터 수신
//  -> 1초(16000 샘플) 버퍼 채우기
// ==========================================
void onPDMdata() {
  int bytesAvailable = PDM.available();
  if (bytesAvailable <= 0) return;

  int samples = bytesAvailable / 2;  // int16_t 샘플 수
  static int16_t pdmBuffer[256];    // 임시 버퍼

  if (samples > (int)(sizeof(pdmBuffer)/sizeof(pdmBuffer[0]))) {
    samples = sizeof(pdmBuffer)/sizeof(pdmBuffer[0]);
  }

  PDM.read(pdmBuffer, samples * 2);

  for (int i = 0; i < samples; ++i) {
    if (secReady) continue; // 이전 결과 아직 처리 중이면 버림

    if (secSamplesCollected < SEC_SAMPLES) {
      secBuffer[secSamplesCollected++] = pdmBuffer[i];
      if (secSamplesCollected >= SEC_SAMPLES) {
        secReady = true;          // 1초 구간 준비 완료
        secSamplesCollected = 0;  // 다음 1초 수집을 위해 초기화
      }
    }
  }
}

// ==========================================
// Setup
// ==========================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }  // PC 연결 기다림

  Serial.println("Initializing PDM...");

  // Hann window 초기화
  initHannWindow();

  // PDM 마이크 설정
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, FS)) {  // mono, 16 kHz
    Serial.println("Failed to start PDM!");
    while (1);
  }

  // 마이크 gain 조절 (필요에 따라 조정)
  PDM.setGain(24);

  Serial.println("Ready. Recording 1 sec slices and classifying.");
}

// ==========================================
// 1초 버퍼에서 feature 계산 (MATLAB 1초 버전과 대응)
// - 1초 안을 1024-sample 프레임으로 나눠서
//   각 프레임 FFT -> band 에너지 -> 프레임 평균 -> log10
// ==========================================
void compute_features_1sec(const int16_t *x, float featOut[FEAT_DIM]) {
  const int N_HALF   = FFT_SAMPLES / 2;
  const int nFrames  = (SEC_SAMPLES - FRAME_LEN) / HOP_SAMPLES + 1;

  float bandEnergySum[FEAT_DIM];
  for (int b = 0; b < FEAT_DIM; ++b) {
    bandEnergySum[b] = 0.0f;
  }

  float freqRes = (float)FS / (float)FFT_SAMPLES;

  for (int n = 0; n < nFrames; ++n) {
    int startIdx = n * HOP_SAMPLES;

    // 1) frame 추출 + [-1,1] 스케일 + Hann window
    for (int i = 0; i < FRAME_LEN; ++i) {
      // MATLAB의 x = double(x_int16)/32768; 과 동일하게
      float xScaled = (float)x[startIdx + i] / 32768.0f;
      vReal[i] = (double)xScaled * hannWindow[i];
      vImag[i] = 0.0;
    }

    // 2) FFT 수행
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();

    // 3) one-sided power spectrum
    static float P[FFT_SAMPLES / 2 + 1];
    for (int k = 0; k <= N_HALF; ++k) {
      double mag = vReal[k];
      P[k] = (float)(mag * mag);
    }

    // 4) band 에너지 합산
    for (int b = 0; b < FEAT_DIM; ++b) {
      float f1 = bandEdgesHz[b][0];
      float f2 = bandEdgesHz[b][1];

      int k1 = (int)ceilf(f1 / freqRes);
      int k2 = (int)floorf(f2 / freqRes);

      if (k1 < 0)      k1 = 0;
      if (k2 > N_HALF) k2 = N_HALF;
      if (k2 < k1)     k2 = k1;

      float E = 0.0f;
      for (int k = k1; k <= k2; ++k) {
        E += P[k];
      }
      if (E <= 0.0f) E = 1e-12f;

      bandEnergySum[b] += E;
    }
  }

  // 5) 프레임 평균 + log10
  for (int b = 0; b < FEAT_DIM; ++b) {
    float Eavg = bandEnergySum[b] / (float)nFrames;
    featOut[b] = log10f(Eavg + 1e-12f);
  }
}


// ==========================================
// Loop
//  -> 1초마다 한 번씩 분류
// ==========================================
void loop() {
  if (!secReady) {
    // 아직 1초가 모이지 않음
    return;
  }

  // 1초 버퍼를 로컬로 복사 (인터럽트와 충돌 방지)
  int16_t localSec[SEC_SAMPLES];

  noInterrupts();
  memcpy(localSec, secBuffer, sizeof(secBuffer));
  secReady = false;
  interrupts();

  // 1) 1초 구간에서 feature 계산
  float feat[FEAT_DIM];
  compute_features_1sec(localSec, feat);

  // 2) SVM으로 분류
  float score_silence = compute_score(feat, svm_w_silence, svm_mu_silence, svm_sigma_silence, svm_bias_silence);
  float score_speaking = compute_score(feat, svm_w_speaking, svm_mu_speaking, svm_sigma_speaking, svm_bias_speaking);
  float score_snoring = compute_score(feat, svm_w_snoring, svm_mu_snoring, svm_sigma_snoring, svm_bias_snoring);

  int bestClass = CLASS_SILENCE;
  float bestScore = score_silence;
  if (score_speaking > bestScore) {
    bestScore = score_speaking;
    bestClass = CLASS_SPEAKING;
  }
  if (score_snoring > bestScore) {
    bestScore = score_snoring;
    bestClass = CLASS_SNORING;
  }

  int label = bestClass; // svm_predict(feat);


  // 3) 결과 출력
  Serial.print("feat = [");
  for (int i = 0; i < FEAT_DIM; ++i) {
    Serial.print(feat[i], 4);
    if (i < FEAT_DIM - 1) Serial.print(", ");
  }
  Serial.print("]  ->  ");

  if (label == CLASS_SILENCE) {
    Serial.println("Silence");
  } else if (label == CLASS_SNORING) {
    Serial.println("Snoring");
  } else if (label == CLASS_SPEAKING) {
    Serial.println("Speaking");
  } else {
    Serial.println("Unknown class");
  }

}