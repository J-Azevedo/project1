clc
clear
close all

[audio_signals, word_labels] = load_audio_from_folder('audio');

display(sprintf('Loaded a total of %d audio signals for the following words:', length(audio_signals)))
display(unique(word_labels))

vocabulary = Vocabulary;

%   MCR = CROSSVAL('mcr',X,Y,'Predfun', PREDFUN) returns MCR, a scalar
%   containing a 5-fold cross-validation estimate of misclassification
%   rate (the proportion of misclassified samples) for the function PREDFUN
%   with the matrix X as predictor values and vector Y as class labels.
%   PREDFUN should use XTRAIN and YTRAIN to fit a classification model and
%   return YFIT as the predicted class labels for XTEST. CROSSVAL then
%   computes the number of misclassifications between YFIT and the
%   corresponding response test set, and returns the overall
%   misclassification rate across all test sets.
%   'Kfold'        The number of folds K for K-fold cross-validation.
crossval('mcr', audio_signals', word_labels', 'predfun', @vocabulary.train_test, 'kfold', 5);


% predicted_word_labels = vocabulary.train_test(audio_signals', word_labels', audio_signals');
% 
% misses = 0;
% for i = 1:length(word_labels)
%     fact  = word_labels(i);
%     guess = predicted_word_labels(i);
%     if ~isequal(fact, guess)
%         misses = misses + 1;
%         display(sprintf('Miss %d: Predicted %s, but was %s.', misses, char(guess), char(fact)))
%     end
% end
% 
% mcr = misses / length(word_labels)