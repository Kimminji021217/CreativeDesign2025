#include <PDM.h>
	// Nano 33 BLE 계열 내장 마이크(PDM)에서 샘플을 받기 위한 라이브러리
#include <arduinoFFT.h>
	// FFT 계산용
	// Fast Fourier Transform (고속 푸리에 변환): 시간 영역의 신호를 주파수 영역의 성분으로 변환해 분석하는 알고리즘

// 실시간 오디오 입력 + FFT 특징 추출

// ==========================================
// 설정값
// ==========================================

#define FS           16000        // FS(Sampling Frequency)=16000: 1초에 16000개 샘플
#define FFT_SAMPLES  1024         // FFT_SAMPLES=1054: 한 프레임 FFT 길이 (MATLAB과 동일하게)
#define FEAT_DIM     5            // FEAT(Feature)_DIM(Dimension)=5: 5개 밴드 에너지 특징
#define SEC_SAMPLES  16000        // SEC_SAMPLES=16000:1초 당 샘플 수(버퍼 크기) (FS * 1초)
	// 버퍼(Buffer): 데이터를 한 곳에서 다른 곳으로 전송 시 처리 속도 차이 완화를 위해 데이터를 일시적으로 저장하는 임시 기억 영역
#define FRAME_LEN    FFT_SAMPLES  // FRAME_LEN=1024 (MATLAB의 frameLen=Nfft, hop=Nfft와 대응)
#define HOP_SAMPLES  FFT_SAMPLES  // HOP_SAMPLES=1024
	// 홉(Hop): 다음 프레임으로 얼마나 이동하느냐 (프레임을 자를 때, 시작점을 몇 칸 이동?)
	// 프레임: 오디오 전체를 작은 조각으로 나눈 것
		// 1초 = 16000 샘플 (1024개씩 잘라서 FFT)

// 크기가 큰 프레임이지만 그만큼 이동하여 이전 프레임과 샘플을 공유하지 않음

// ==========================================
// SVM 파라미터 (MATLAB에서 출력한 값 붙여넣기)
// ==========================================

// 처음 아두이노에서 받는 입력값은 1초 소리에서 뽑은 5개 주파수 대역의 log 에너지
	// ex. feat = [-1.32, -0.85, -2.10, -2.45, -3.01]
	// 사람의 소리는 저주파 쪽에 에너지가 몰려 있기 때문에 
	// 0~300Hz 대역 에너지는 상대적으로 크고, 2400~4000Hz 대역은 거의 0에 가까운 것과 같이
	// 주파수 대역마다 에너지 크기의 차이가 큼

#include "svm_model_silence.h"
#include "svm_model_speaking.h"
#include "svm_model_snoring.h"

// 헤더 내부

// svm_w_*     : i번째 특징의 가중치 (값이 클 경우 그 특징을 가중하여 처리)
// svm_mu_*	   : i번째 특징의 평균
// svm_sigma_* : i번째 특징의 표준편차 (표준화 시 값이 평균에서 얼마나 떨어져 있는가를 표준편차 기준으로 측정)
// svm_bias_*  : 결정 기준선을 어디에 둘지 정하는 값 (모든 판단 기준이 원점을 지나야 함)

const int CLASS_SILENCE  = 0;   // ClassNames(0)
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

// 1초 샘플 버퍼
int16_t secBuffer[SEC_SAMPLES];			// secBuffer: 1초짜리 샘플을 모아두는 메인 버퍼
volatile int secSamplesCollected = 0;	// secSamplesCollected: 지금까지 모은 샘플 수
volatile bool secReady = false;			// secReady: 1초가 꽉 찼다는 신호
	// volatile인 이유: 콜백에서 바뀌기 때문에 컴파일러 최적화로 값이 꼬이지 않게

// FFT 버퍼
double vReal[FFT_SAMPLES];		// vReal/vImag는 FFT 입력/연산 공간
double vImag[FFT_SAMPLES];
double hannWindow[FFT_SAMPLES];
	// hannWindow: FFT 처리 전 신호의 양 끝을 부드럽게 0으로 만들어주는 가중치 곱셈
		// FFT를 자른 후 입력 시 신호 끊김
		// 이 때, 값이 튄다고 인식한 FFT가 없는 고주파 생성 (spectral leakage)
	// hannWindow는 미리 계산해두고 매 프레임 곱
	// vReal[i] = xScaled * hannWindow[i]
	// Hann window (MATLAB의 hann(Nfft,'periodic')와 대응)
	
// 주파수 누설(spectral leakage) 감소와 에너지 분포 안정

ArduinoFFT<double> FFT  = ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, FS);

// ==========================================
// Hann window 초기화
// ==========================================
void initHannWindow() {
  // MATLAB: win = hann(Nfft,'periodic');
  for (int n = 0; n < FFT_SAMPLES; ++n) {
    hannWindow[n] = 0.5 * (1.0 - cos(2.0 * PI * n / (FFT_SAMPLES - 1)));
	// cos(): 부드러운 곡선
	// 1 - cos(): 중앙이 가장 큼
	// 0.5: 전체 크기 조정
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

// 특징 feat를 표준화
	// feat: 소리에서 뽑아낸 최종 특징 벡터


// ==========================================
// PDM 콜백: 마이크 데이터 수신
//  -> 1초(16000 샘플) 버퍼 채우기
// ==========================================
void onPDMdata() {
  int bytesAvailable = PDM.available();
  if (bytesAvailable <= 0) return;

  int samples = bytesAvailable / 2;  // int16_t 샘플 개수로 변환
  static int16_t pdmBuffer[256];     // 임시 버퍼

  if (samples > (int)(sizeof(pdmBuffer)/sizeof(pdmBuffer[0]))) {
    samples = sizeof(pdmBuffer)/sizeof(pdmBuffer[0]);
  }

  PDM.read(pdmBuffer, samples * 2);

  for (int i = 0; i < samples; ++i) {
    if (secReady) continue;		// 이전 데이터가 아직 처리 중이면 버림

    if (secSamplesCollected < SEC_SAMPLES) {
      secBuffer[secSamplesCollected++] = pdmBuffer[i];	// 임시 버퍼에서 읽은 데이터 복사 후 누적
      if (secSamplesCollected >= SEC_SAMPLES) {
        secReady = true;          // 1초 구간 준비 완료 (16000개 수집 완료)
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
  while (!Serial) { ; }  // PC 연결 대기

  Serial.println("Initializing PDM...");

  // Hannwindow 초기화
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
	// 1초 안을 1024-sample 프레임으로 나눠서
	// 각 프레임 FFT -> band 에너지 -> 프레임 평균 -> log10
// ==========================================
void compute_features_1sec(const int16_t *x, float featOut[FEAT_DIM]) {	
							// x: 1초짜리 raw 오디오, featOut: 최종 log 5개의 에너지
  const int N_HALF   = FFT_SAMPLES / 2;		// FFT_SAMPLES / 2: 512 (실수 신호 FFT의 스펙트럼은 좌우대칭이므로 유효한 절반 사용)
  const int nFrames  = (SEC_SAMPLES - FRAME_LEN) / HOP_SAMPLES + 1;

  float bandEnergySum[FEAT_DIM];
  for (int b = 0; b < FEAT_DIM; ++b) {	// b번째 주파수 대역 에너지를 프레임 전체에 대해 누적한 합
    bandEnergySum[b] = 0.0f;
  }

  float freqRes = (float)FS / (float)FFT_SAMPLES;	// freqRes: 주파수 해상도
  
  // band 구간을 k범위로 바꾸기 위해 필요한 계산

  for (int n = 0; n < nFrames; ++n) {
    int startIdx = n * HOP_SAMPLES;

    // 1) frame 추출 + [-1,1] 스케일 + Hann window
    for (int i = 0; i < FRAME_LEN; ++i) {
      // MATLAB의 x = double(x_int16)/32768; 과 유사한 수치로 변형
      float xScaled = (float)x[startIdx + i] / 32768.0f;
      vReal[i] = (double)xScaled * hannWindow[i];
      vImag[i] = 0.0;
    }

    // 2) FFT 수행
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
	
	// vReal[k]: k번째 주파수 성분의 크기

    // 3) one-sided power spectrum
    static float P[FFT_SAMPLES / 2 + 1];
		// 매 프레임마다 큰 배열을 생성하는 것이 아닌 static으로 할당해 재사용
    for (int k = 0; k <= N_HALF; ++k) {
      double mag = vReal[k];
      P[k] = (float)(mag * mag);	// 에너지는 보통 진폭의 제곱에 비례
    }

    // 4) band 에너지 합산
    for (int b = 0; b < FEAT_DIM; ++b) {
      float f1 = bandEdgesHz[b][0];
      float f2 = bandEdgesHz[b][1];

      int k1 = (int)ceilf(f1 / freqRes);	// Hz -> FFT bin 변환
      int k2 = (int)floorf(f2 / freqRes);

      if (k1 < 0)      k1 = 0;				// 범위 보정
      if (k2 > N_HALF) k2 = N_HALF;
      if (k2 < k1)     k2 = k1;

      float E = 0.0f;						// 에너지 합산
      for (int k = k1; k <= k2; ++k) {
        E += P[k];
      }
      if (E <= 0.0f) E = 1e-12f;

      bandEnergySum[b] += E;
    }
  }

  // 5) 프레임 평균 + log10 (최종 feat)
  for (int b = 0; b < FEAT_DIM; ++b) {
    float Eavg = bandEnergySum[b] / (float)nFrames;
		// 프레임마다 차이나는 에너지를 안정화
    featOut[b] = log10f(Eavg + 1e-12f);
		// 에너지의 범위가 크기 때문에 log 사용 (SVM 사용시 편리)
  }
}


// ==========================================
// Loop
//  -> 1초마다 한 번씩 분류
// ==========================================
void loop() {
  if (!secReady) {
    // 1초 입력 대기
    return;
  }

  // 1초 버퍼를 로컬로 복사 (인터럽트와 충돌 방지)
  int16_t localSec[SEC_SAMPLES];

  noInterrupts();
  memcpy(localSec, secBuffer, sizeof(secBuffer));
  secReady = false;
  interrupts();
  
	// secBuffer는 콜백을 계속 사용하는 버퍼이므로 데이터가 섞일 가능성 존재
	// interrupt 금지 -> 복사 -> secReady -> interrupt 허용
  
  // loop는 완성된 1초 데이터(localSec)로 안정적 계산 가능

  // 1) 1초 구간에서 feature 계산
  float feat[FEAT_DIM];
  compute_features_1sec(localSec, feat);	// feat 생성 완료

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

  // delay(100);
}
