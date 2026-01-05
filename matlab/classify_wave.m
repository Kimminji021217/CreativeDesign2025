%% ========== 설정 ==========
clear; clc;

silenceDir = 'data/silence_nano';
speakingDir = 'data/speaking_nano';
snoringDir  = 'data/snoring_nano';

targetFs = 16000;       % 16 kHz
Nfft     = 1024;        % FFT 길이 (Arduino에서도 동일하게 쓸 예정)

% 5개 주파수 대역 [Hz]
bandEdges = [ ...
      0    300;
    300    600;
    600   1200;
   1200   2400;
   2400   4000];

NBANDS = size(bandEdges, 1);

rng(0);   % 재현성 확보용

%% ========== 1. 데이터 로딩 + 특징 추출 ==========
fprintf('=== Loading and featurizing audio files ===\n');

features  = [];   % [N x NBANDS]
labels    = [];   % [N x 1], 0 = speaking, 1 = snoring
fileNames = {};

% ----- silence (label = 0) -----
siFiles = dir(fullfile(silenceDir, '*.wav'));
for k = 1:numel(siFiles)
    file = fullfile(silenceDir, siFiles(k).name);
    [feat, ok] = extract_band_feature(file, targetFs, bandEdges, Nfft);
    if ~ok, continue;
    end
    features  = [features; feat(:)'];   % 1 x NBANDS
    labels    = [labels; 0];
    fileNames{end+1,1} = file;
end

% ----- speaking (label = 1) -----
spFiles = dir(fullfile(speakingDir, '*.wav'));
for k = 1:numel(spFiles)
    file = fullfile(speakingDir, spFiles(k).name);
    [feat, ok] = extract_band_feature(file, targetFs, bandEdges, Nfft);
    if ~ok, continue;
    end
    features  = [features; feat(:)'];   % 1 x NBANDS
    labels    = [labels; 1];
    fileNames{end+1,1} = file;
end

% ----- snoring (label = 2) -----
snFiles = dir(fullfile(snoringDir, '*.wav'));
for k = 1:numel(snFiles)
    file = fullfile(snoringDir, snFiles(k).name);
    [feat, ok] = extract_band_feature(file, targetFs, bandEdges, Nfft);
    if ~ok, continue;
    end
    features  = [features; feat(:)'];
    labels    = [labels; 2];
    fileNames{end+1,1} = file;
end

%% ========== 2. 3-fold stratified cross-validation ==========
classNames = {'silence','speaking','snoring'};
for targetClass = 0:2
    labels_binary = (labels == targetClass);

    Mdl = fitcsvm(features, labels_binary, ...
        'KernelFunction','linear', ...
        'Standardize',true, ...
        'ClassNames',[0 1]);

    CM     = compact(Mdl);
    w      = CM.Beta;
    b      = CM.Bias;
    mu     = CM.Mu;
    sigma  = CM.Sigma;

    headerFile = sprintf('svm_model_%s.h', classNames{targetClass+1});
    fid = fopen(headerFile, 'w');

    fprintf(fid, '// %s vs rest\n', classNames{targetClass+1});
    fprintf(fid, '#ifndef SVM_MODEL_%s_H\n', upper(classNames{targetClass+1}));
    fprintf(fid, '#define SVM_MODEL_%s_H\n\n', upper(classNames{targetClass+1}));
    fprintf(fid, '#define FEAT_DIM %d\n\n', NBANDS);

    fprintf(fid, 'static const float svm_w[%d]     = {', NBANDS);
    fprintf(fid, ' %.8ff,', w);
    fprintf(fid, ' };\n');

    fprintf(fid, 'static const float svm_mu[%d]    = {', NBANDS);
    fprintf(fid, ' %.8ff,', mu);
    fprintf(fid, ' };\n');

    fprintf(fid, 'static const float svm_sigma[%d] = {', NBANDS);
    fprintf(fid, ' %.8ff,', sigma);
    fprintf(fid, ' };\n');

    fprintf(fid, 'static const float svm_bias = %.8ff;\n\n', b);
    fprintf(fid, '#endif // SVM_MODEL_%s_H\n', upper(classNames{targetClass+1}));

    fclose(fid);
end
fprintf('\nAll done. Three .h files generated for each class vs rest.\n');
%% ========== 보조 함수: 5-band 에너지 특징 추출 ==========
function [feat, ok] = extract_band_feature(file, targetFs, bandEdges, Nfft)
    ok = false;
    feat = [];
    try
        [x, fs] = audioread(file);
    catch ME
        warning('Failed to read %s: %s', file, ME.message);
        return;
    end

    if isempty(x)
        warning('Empty file: %s', file);
        return;
    end

    % 스테레오 -> 모노
    if size(x,2) > 1
        x = mean(x, 2);
    end

    % 정규화 (선택)
    % x = x ./ (max(abs(x)) + eps);

    % resample to targetFs
    if fs ~= targetFs
        try
            x = resample(x, targetFs, fs);
        catch
            warning('Resample failed for %s. Skipping.', file);
            return;
        end
        fs = targetFs;
    end


    % 길이가 Nfft보다 짧으면 zero padding, 길면 앞부분만 사용
    if numel(x) < Nfft
        x = [x; zeros(Nfft - numel(x), 1)];
    else
        x = x(1:Nfft);
    end

    % 간단한 window (Hann)
    win = hann(Nfft, 'periodic');
    xw  = x(:) .* win;

    % FFT 및 one-sided power spectrum
    X = fft(xw, Nfft);
    P = abs(X(1:Nfft/2+1)).^2;       % power
    f = (0:Nfft/2) * fs / Nfft;      % freq axis [Hz]

    NBANDS = size(bandEdges, 1);
    feat   = zeros(NBANDS, 1);

    for b = 1:NBANDS
        f1 = bandEdges(b,1);
        f2 = bandEdges(b,2);
        idx = (f >= f1) & (f < f2);
        if ~any(idx)
            E = eps;
        else
            E = sum(P(idx));
        end
        feat(b) = log10(E + eps);
    end

    ok = true;
end
