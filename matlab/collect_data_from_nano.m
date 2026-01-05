%% ===== 0. 공통 설정 =====
clear; clc;

% ------------- (1) 현재 녹음할 클래스 지정 -------------
% 'speaking', 'snoring', 'silence' 등으로 바꿔 사용
cur_class = "snoring";    

baseDir   = "data";   % 기본 데이터 폴더
saveDir   = fullfile(baseDir, cur_class + "_nano");  % 예: data/speaking_nano

if ~exist(saveDir, "dir")
    mkdir(saveDir);
end

% ------------- (2) 시리얼 포트 설정 -------------
port = "COM7";        % PC 기준 포트 번호로 변경
baud = 115200;

fs   = 16000;
Ns   = 16000;         % 1초

% ===== 1. 시리얼 포트 열기 =====
if ~exist("s","var") || ~isvalid(s)
    s = serialport(port, baud);
    configureTerminator(s, "LF");
    s.Timeout = 5;
    flush(s);
end

%% ===== 2. 1초 녹음 요청 =====
writeline(s, "r");   % 아두이노에게 'r' 전송 → 1초 녹음 시작
disp('Recording 1 second on Nano...');

pause(1.2);          % 1초 + 약간 여유

% ===== 3. 'BEGN' 헤더까지 읽어 흘려보내기 =====
headerStr = 'BEGN';                 % 아두이노 쪽에서 보낸 헤더와 동일하게
buf = uint8([]);

tic;
while true
    if s.NumBytesAvailable > 0
        newBytes = read(s, s.NumBytesAvailable, "uint8");
        buf = [buf; newBytes]; %#ok<AGROW>
        idx = strfind(char(buf), headerStr);   % 행벡터로 캐스팅
        if ~isempty(idx)
            % 헤더 이후부터 남기고 나머지는 버림
            buf = buf(idx(end) + length(headerStr):end);
            break;
        end
    end
    if toc > 5
        error('Timeout: BEGN header was not found');
    end
end

disp('BEGN header found, reading 1-second int16 data...');

% ===== 4. int16 16000개 읽기 =====
bytesNeeded = Ns * 2;

if numel(buf) >= bytesNeeded
    rawBytes = buf(1:bytesNeeded);
else
    rawBytes = [buf; read(s, bytesNeeded - numel(buf), "uint8")];
end

x_int16 = typecast(rawBytes, 'int16');   % uint8 → int16 (리틀엔디언)

% ---- 여기서 [-1, 1]로 정규화 (wav와 동일 스케일) ----
x = double(x_int16) / 32768;            % 아두이노에서도 동일 연산 수행

% ===== 5. 파형/스펙트로그램 확인 (옵션) =====
t = (0:Ns-1)/fs;

figure;
subplot(2,1,1);
plot(t, x);
xlabel('Time [s]'); ylabel('Amplitude');
title("Nano PDM 1-second capture (" + cur_class + ")"); grid on;

subplot(2,1,2);
spectrogram(x, hamming(256), 128, 512, fs, 'yaxis');
ylim([0 8]); title('Spectrogram'); colorbar;

% ===== 6. wav 파일로 자동 저장 =====

% 현재 클래스 폴더에 몇 개 저장되어 있는지 확인
existing = dir(fullfile(saveDir, '*.wav'));
nextIdx  = numel(existing) + 1;

fname = sprintf('%s_%03d.wav', cur_class, nextIdx);  % speaking_001.wav 같은 형식
outPath = fullfile(saveDir, fname);

% --- (A) 들어보고 싶은 경우 정규화해서 재생 ---
x_play = x / (max(abs(x)) + eps);    % 듣기용
sound(x_play, fs);

audiowrite(outPath, x, fs);

fprintf('Saved: %s\n', outPath);

%%
