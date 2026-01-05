#include <PDM.h>
#include <arduinoFFT.h>
#include <ArduinoBLE.h>

// ==========================================
// 설정값
// ==========================================
#define FS           16000
#define FFT_SAMPLES  1024
#define FEAT_DIM     5
#define SEC_SAMPLES  16000
#define FRAME_LEN    FFT_SAMPLES
#define HOP_SAMPLES  FFT_SAMPLES

// ==========================================
// SVM 파라미터
// ==========================================
#include "svm_model_silence.h"
#include "svm_model_speaking.h"
#include "svm_model_snoring.h"

const int CLASS_SILENCE = 0;
const int CLASS_SPEAKING = 1;
const int CLASS_SNORING  = 2;

// ==========================================
// Band edges
// ==========================================
const float bandEdgesHz[FEAT_DIM][2] = {
  {0.0f, 300.0f}, {300.0f, 600.0f}, {600.0f, 1200.0f},
  {1200.0f, 2400.0f}, {2400.0f, 4000.0f}
};

// ==========================================
// 버퍼 및 FFT
// ==========================================
int16_t secBuffer[SEC_SAMPLES];
volatile int secSamplesCollected = 0;
volatile bool secReady = false;

double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];
double hannWindow[FFT_SAMPLES];

ArduinoFFT<double> FFT(vReal, vImag, FFT_SAMPLES, FS);

// ==========================================
// BLE UUID
// ==========================================
const char* SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214";
const char* CHAR_UUID    = "19B10001-E8F2-537E-4F6C-D104768A1214";

BLEService soundService(SERVICE_UUID);
BLEStringCharacteristic resultChar(CHAR_UUID, BLERead | BLENotify, 16);


// ==========================================
// Hann window
// ==========================================
void initHannWindow() {
  for (int n = 0; n < FFT_SAMPLES; ++n) {
    hannWindow[n] = 0.5 * (1.0 - cos(2.0 * PI * n / (FFT_SAMPLES - 1)));
  }
}

// ==========================================
// SVM Score
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
// PDM 콜백
// ==========================================
void onPDMdata() {
  int bytesAvailable = PDM.available();
  if (bytesAvailable <= 0) return;

  int samples = bytesAvailable / 2;
  static int16_t pdmBuffer[256];

  if (samples > 256) samples = 256;
  PDM.read(pdmBuffer, samples * 2);

  for (int i = 0; i < samples; i++) {
    if (secReady) continue;

    if (secSamplesCollected < SEC_SAMPLES) {
      secBuffer[secSamplesCollected++] = pdmBuffer[i];
      if (secSamplesCollected >= SEC_SAMPLES) {
        secReady = true;
        secSamplesCollected = 0;
      }
    }
  }
}


// ==========================================
// Feature 계산
// ==========================================
void compute_features_1sec(const int16_t *x, float featOut[FEAT_DIM]) {
  const int N_HALF = FFT_SAMPLES / 2;
  const int nFrames = (SEC_SAMPLES - FRAME_LEN) / HOP_SAMPLES + 1;

  float bandEnergySum[FEAT_DIM] = {0,};

  float freqRes = (float)FS / (float)FFT_SAMPLES;

  for (int n = 0; n < nFrames; ++n) {

    int startIdx = n * HOP_SAMPLES;

    for (int i = 0; i < FRAME_LEN; ++i) {
      float xScaled = (float)x[startIdx + i] / 32768.0f;
      vReal[i] = (double)xScaled * hannWindow[i];
      vImag[i] = 0.0;
    }

    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();

    static float P[FFT_SAMPLES / 2 + 1];
    for (int k = 0; k <= N_HALF; ++k) {
      P[k] = (float)(vReal[k] * vReal[k]);
    }

    for (int b = 0; b < FEAT_DIM; ++b) {
      float f1 = bandEdgesHz[b][0];
      float f2 = bandEdgesHz[b][1];

      int k1 = (int)ceilf(f1 / freqRes);
      int k2 = (int)floorf(f2 / freqRes);

      if (k1 < 0) k1 = 0;
      if (k2 > N_HALF) k2 = N_HALF;
      if (k2 < k1) k2 = k1;

      float E = 0.0f;
      for (int k = k1; k <= k2; ++k)
        E += P[k];

      if (E <= 0.0f) E = 1e-12f;

      bandEnergySum[b] += E;
    }
  }

  for (int b = 0; b < FEAT_DIM; ++b) {
    float Eavg = bandEnergySum[b] / nFrames;
    featOut[b] = log10f(Eavg + 1e-12f);
  }
}


// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Type 'start' to enable BLE advertising...");

  // ===== start 명령 대기 =====
  while (true) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input.equalsIgnoreCase("start")) {
        Serial.println("START command received!");
        break;
      }
    }
  }

  // ===== BLE 시작 =====
  if (!BLE.begin()) {
    Serial.println("BLE Start Failed!");
    while (1);
  }

  Serial.print("My BLE MAC Address: ");
  Serial.println(BLE.address());

  BLE.setLocalName("SnoreMonitor");
  BLE.setAdvertisedService(soundService);

  soundService.addCharacteristic(resultChar);
  BLE.addService(soundService);

  resultChar.writeValue("READY");
  delay(800);  
  BLE.advertise();

  Serial.println("BLE advertising started");
  Serial.println("Waiting for BLE connection...");

  // ===== PDM 초기화 =====
  initHannWindow();
  PDM.onReceive(onPDMdata);

  if (!PDM.begin(1, FS)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  PDM.setGain(24);
}


// ======================================================
// LOOP
// ======================================================
void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());
    Serial.println(">>> BLE 연결 성공! <<<");

    // ★★★ 5초 대기 후 코골이 감지 시작 ★★★
    delay(5000);

    while (central.connected()) {

      if (!secReady) continue;

      int16_t localSec[SEC_SAMPLES];

      noInterrupts();
      memcpy(localSec, secBuffer, sizeof(secBuffer));
      secReady = false;
      interrupts();

      float feat[FEAT_DIM];
      compute_features_1sec(localSec, feat);

      float s_sil = compute_score(feat, svm_w_silence, svm_mu_silence, svm_sigma_silence, svm_bias_silence);
      float s_spk = compute_score(feat, svm_w_speaking, svm_mu_speaking, svm_sigma_speaking, svm_bias_speaking);
      float s_snr = compute_score(feat, svm_w_snoring, svm_mu_snoring, svm_sigma_snoring, svm_bias_snoring);

      int label = CLASS_SILENCE;
      float best = s_sil;

      if (s_spk > best) { best = s_spk; label = CLASS_SPEAKING; }
      if (s_snr > best) { best = s_snr; label = CLASS_SNORING; }

      String msg;

      if (label == CLASS_SILENCE) msg = "Silence";
      else if (label == CLASS_SPEAKING) msg = "Speaking";
      else msg = "Snoring";

      Serial.print("Result: ");
      Serial.println(msg);

      resultChar.writeValue(msg);   // BLE 알림 전송
    }

    Serial.println("Disconnected.");
  }
}
