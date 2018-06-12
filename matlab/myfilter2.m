% 1st-order Butterworth - fc 10 Hz at fs 1000 Hz
nmax = 100;
n = (1:nmax);
fs = 500;
f = 10;
t = n / fs;
x = sin(2*pi*f*n/fs);

% real coefficients (butter(1,0.02)
a0 = 0.0305; 
a1 = a0;
b0 = 1;
b1 = -0.9391;
x1 = 0; y1 = 0;
for i = 1:nmax
    x0 = x(i);
    y0 = a0 * x0 + a1 * x1 - b1 * y1;
    y(i) = y0;
    x1 = x0; y1 = y0;
end;

% 14 bits fraction
a0 = 500; 
a1 = a0;
b0 = 16384;
b1 = -15386;
x1 = 0; y1 = 0;
for i = 1:nmax
    x0 = x(i);
    y0 = (a0 * x0 + a1 * x1 - b1 * y1) / 16384;
    y(i) = y0;
    x1 = x0; y1 = y0;
end;

% 14 bits fraction, approximated to powers of 2
% a0 = 256 + 128 + 64 + 32; % 480
% a0 = 256 + 128 + 64; % 448
a0 = 256 + 128; % 384
a1 = a0;
b0 = 16384;
% b1 = -(8192 + 4096 + 2048 + 1024); % 15360
% b1 = -(8192 + 4096 + 2048); % 14336
b1 = -(8192 + 4096); % 12288
x1 = 0; y1 = 0;
for i = 1:nmax
    x0 = x(i);
    y0 = (a0 * x0 + a1 * x1 - b1 * y1) / 16384;
    z(i) = y0;
    x1 = x0; y1 = y0;
end;
figure ; plot(t,[x',y',z']), legend('x', 'y', 'z');
