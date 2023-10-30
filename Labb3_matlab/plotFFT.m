function [f,Y] = plotFFT(x,y,fs)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
N = length(y);
%subplot(2,1,1);
%plot(x,y,'color',"#D95319")
%grid;
%title('Time-domain')
X=fft(y);
f=0:(fs/N):fs/2;

M = length(f);
Y = abs(X(1:M)/(N/2));

%subplot(2,1,2);
plot(f,Y,'color',"#77AC30")
grid;
title('Frequency-domain')
hold off
end