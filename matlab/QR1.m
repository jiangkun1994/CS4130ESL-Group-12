%--------------------------------------------------------------------------
% QR1 -- 
%--------------------------------------------------------------------------
    clear all;
    close all;
% load data
	load logfile.dat;
	t = logfile(:,1);
	sax = logfile(:,2);
	say = logfile(:,3);
	saz = logfile(:,4);
    p = logfile(:,5);
	q = logfile(:,6);
	r = logfile(:,7);
% sample freq: t is in us
	fs = length(t)*1000000/(t(length(t))-t(1));
	n = length(t);
% map acc to angles (simplified) 
    phi = say;
    theta = sax;
    
%--------------------------------------------------------------------------    
% do fft on phi
%--------------------------------------------------------------------------    
	f = (0:n-1)*(fs/n);
	fftphi = fft(phi);
	Pphi = fftphi.*conj(fftphi)/n;
	figure; plot(f(1:floor(n/2)),Pphi(1:floor(n/2)));    
      
%--------------------------------------------------------------------------    
% show p and phi, integrate p to phi to show movement vs noise
%--------------------------------------------------------------------------    
% p2phi is given
    p2phi = 0.0081;
% compute phi from p
    phi_p(1:n) = 0;
    phi_p(1) = 0;
    for i=(2:n) phi_p(i) = phi_p(i-1) + p(i-1) * p2phi; end;
    figure; plot(t,[phi,p,phi_p']);

%--------------------------------------------------------------------------    
% filter phi to reduce scale to show movement
%--------------------------------------------------------------------------    
% filter 2nd order 10Hz 
    [b,a] = butter(2,10/(fs/2));
    % a = [1.000000000000000, -1.910582703318998, 0.914412856345180];
    % b = [0.000957538256546,  0.001915076513091, 0.000957538256546];
    phi2 = filter(b,a,phi);
    p2 = filter(b,a,p);
    figure; plot(t,[phi2,p,phi_p']);

% compute phi from p2
    phi_p2(1:n) = 0;
    phi_p2(1) = 0;
    for i=(2:n) phi_p2(i) = phi_p2(i-1) + p2(i-1) * p2phi; end;
    figure; plot(t,[phi2,p2,phi_p2']);
% show that phi_p2 equals phi_p (so no need to filter p)
    figure; plot(t,[phi_p',phi_p2']);

    
