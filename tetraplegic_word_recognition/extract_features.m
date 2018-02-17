function framefrequencies = extract_features(sound)
    Fs = 8000;
    framesize = 125;
    overlap = 50;
    D = 6; % Number of frequencies stored from each signal frame

    frames = buffer(sound, framesize, overlap);
    % Buffer an entire signal vector into framesize-sample frames,
    % each frame overlapping the previous one by overlap samples.
    w = hamming(framesize);
    % Return a framesize-point symmetric Hamming window in a column vector
    [~, N] = size(frames);
    % returns the number of rows and columns(N) in frames as separate output variables. 
    framefrequencies = zeros(D, N);

    i = 1;
    NFFT = 2^nextpow2(framesize); %returns the first NFFT such that 2.^NFFT >= abs(framesize).  It is
%   often useful for finding the nearest power of two sequence length for FFT operations.
%   This function is useful for optimizing FFT operations,
%   which are most efficient when sequence length is an exact power of two.
    for frame = frames
        x = frame .* w;
        plot(frame./max(abs(frame)))
%         hold on
%         figure(1)
%         plot(w, 'r')
%         plot(x, 'c')
%         legend('Speech signal', 'Hamming window')
%         xlabel('time')
        
        X = fft(x, NFFT)/framesize; %is the N-point FFT, padded with zeros if x has less
%   than NFFT points and truncated if it has more.
        f = Fs/2*linspace(0, 1, NFFT/2 + 1); %generates NFFT/2 + 1 points between 0 and 1.
%         hold off
%         figure(2)
%         plot(f, abs(X(1:(NFFT/2 + 1))), 'r')
%         xlabel('F [Hz]')
%         ylabel('|X(F)|')
        
        
        % Finding local maxima in single-sided amplitude spectrum
        [~, peaklocations] = findpeaks(abs(X(1:(NFFT/2 + 1))), 'SORTSTR', 'descend');

        if isempty(peaklocations)
            peaklocations = ones(D, 1);
        elseif length(peaklocations) < D
            peaklocations = padarray(peaklocations, D - length(peaklocations), 1, 'post');
        end

        framefrequencies(:, i) = f(peaklocations(1:D))';
        i = i + 1;
    end
end
