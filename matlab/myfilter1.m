% 1st-order Butterworth - fc 25 Hz at fs 500 Hz
nmax = 100;
n = (1:nmax);
fs = 500;
f = 10;
t = n / fs;
x = sin(2*pi*f*n/fs);

% real coefficients (butter(1,0.1)
a0 = 0.1367; 
a1 = a0;
b0 = 1;
b1 = -0.7265;
x1 = 0; y1 = 0;
for i = 1:nmax
    x0 = x(i);
    y0 = a0 * x0 + a1 * x1 - b1 * y1;
    y(i) = y0;
    x1 = x0; y1 = y0;
end;

% 14 bits fraction
a0 = 2239; 
a1 = a0;
b0 = 16384;
b1 = -11903;
x1 = 0; y1 = 0;
for i = 1:nmax
    x0 = x(i);
    y0 = (a0 * x0 + a1 * x1 - b1 * y1) / 16384;
    y(i) = y0;
    x1 = x0; y1 = y0;
end;

% 14 bits fraction, approximated to powers of 2
a0 = 2048 + 128 + 64; % 2240
a1 = a0;
b0 = 16384;
b1 = -(8192 + 2048 + 1024); % 11264
x1 = 0; y1 = 0;
for i = 1:nmax
    x0 = x(i);
    y0 = (a0 * x0 + a1 * x1 - b1 * y1) / 16384;
    z(i) = y0;
    x1 = x0; y1 = y0;
end;
figure ; plot(t,[x',y',z']), legend('x', 'y', 'z');
