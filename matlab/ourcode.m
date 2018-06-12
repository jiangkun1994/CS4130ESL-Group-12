clear all;
close all;
% load data
t = csvread('logfile.csv', 0, 0, [0,0,1920,0]);
sax = csvread('logfile.csv', 0, 16, [0,16,1920,16]);
say = csvread('logfile.csv', 0, 17, [0,17,1920,17]);
saz = csvread('logfile.csv', 0, 18, [0,18,1920,18]);
p = csvread('logfile.csv', 0, 13, [0,13,1920,13]);
q = csvread('logfile.csv', 0, 14, [0,14,1920,14]);
r = csvread('logfile.csv', 0, 15, [0,15,1920,15]);

% sample freq: t is in us
	fs = length(t)*1000000/(t(length(t))-t(1));
	n = length(t);
% map acc to angles (simplified) 
    phi = say;
    theta = sax;
    
%--------------------------------------------------------------------------    
% filter phi to reduce scale to show movement
%--------------------------------------------------------------------------    
% filter 2nd order 10Hz 
    [b,a] = butter(2,10/(fs/2));
    phi2 = filter(b,a,phi);

%--------------------------------------------------------------------------    
% kalman filter to remove dynamic offset in p using phi2
%--------------------------------------------------------------------------    
    p2phi = 0.0081;
    p_bias(1:n) = 0;
    phi_kalman(1:n) = 0;
    C1 = 256;
    C2 = 1000000;
    for i=(2:n)
        p_kalman(i) = p(i-1) - p_bias(i-1);
        phi_kalman(i) = phi_kalman(i-1) + p_kalman(i) * p2phi;
        phi_error(i) = phi_kalman(i) - phi2(i);
        phi_kalman(i) = phi_kalman(i) - phi_error(i) / C1;
        p_bias(i) = p_bias(i-1) + (phi_error(i)/p2phi) / C2;
    end;
    figure; plot(t,phi2,t,p,t,phi_kalman), legend('phi2', 'p', 'phi\_kalman');
    figure; plot(t,p_bias'), legend('p_bias');
    
    
    
% compare phi from kalman with phi from p
    phi_p(1:n) = 0;
    phi_p(1) = 0;
    for i=(2:n) 
        phi_p(i) = phi_p(i-1) + p(i-1) * p2phi; 
    end;
    figure; plot(t,phi_p,t,phi_kalman), legend('phi_p', 'phi\_kalman');