#include <PDM.h>

#define FS           16000
#define SEC_SAMPLES  16000   // 1초 = 16000 샘플

int16_t secBuffer[SEC_SAMPLES];
volatile int secSamplesCollected = 0;
volatile bool secReady = false;
volatile bool recording = false;

void onPDMdata() {
  if (!recording) {
    // 녹음 중이 아닐 때는 버림
    int bytesAvailable = PDM.available();
    if (bytesAvailable > 0) {
      
      int16_t dummy[256];
      int samples = bytesAvailable / 2;
      if (samples > (int)(sizeof(dummy)/sizeof(dummy[0]))) {
        samples = sizeof(dummy)/sizeof(dummy[0]);
      }
      PDM.read(dummy, samples * 2);
    }
    return;
  }

  int bytesAvailable = PDM.available();
  if (bytesAvailable <= 0) return;

  int16_t pdmBuffer[256];
  int samples = bytesAvailable / 2;
  if (samples > (int)(sizeof(pdmBuffer)/sizeof(pdmBuffer[0]))) {
    samples = sizeof(pdmBuffer)/sizeof(pdmBuffer[0]);
  }

  PDM.read(pdmBuffer, samples * 2);

  for (int i = 0; i < samples; ++i) {
    if (!recording || secReady) {
      // 이미 다 채웠으면 무시
      continue;
    }
    if (secSamplesCollected < SEC_SAMPLES) {
      secBuffer[secSamplesCollected++] = pdmBuffer[i];
      if (secSamplesCollected >= SEC_SAMPLES) {
        secReady = true;
        recording = false;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {;}

  Serial.println("Nano PDM 1-second recorder. Send 'r' to record 1s.");

  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, FS)) {  // mono, 16kHz
    Serial.println("Failed to start PDM!");
    while (1);
  }
  PDM.setGain(24);
}

void loop() {
  // PC에서 'r' 명령을 받으면 1초 녹음 시작
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') {
      secSamplesCollected = 0;
      secReady = false;
      recording = true;
      // 안내 메시지 (디버깅용, 필요 없으면 지워도 됨)
      Serial.println("RECORDING_1SEC");
    }
  }

  // 1초 녹음이 끝나면 버퍼를 그대로 바이너리로 전송
  if (secReady) {
    // 헤더로 간단한 태그 전송 (동기용)
    Serial.write('B'); Serial.write('E'); Serial.write('G'); Serial.write('N');

    // int16 16000개 = 32000 byte
    Serial.write((uint8_t*)secBuffer, sizeof(secBuffer));
    Serial.write('E'); Serial.write('N'); Serial.write('D'); Serial.write('!');

    secReady = false;
    // 이후 다시 'r'을 받으면 또 녹음 가능
  }
}
