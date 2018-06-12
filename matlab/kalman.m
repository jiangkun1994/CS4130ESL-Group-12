%--------------------------------------------------------------------------
% kalman -- test inertia-only Kalman 
%--------------------------------------------------------------------------
% synthesize signals sp and sphi
	n = 20000; % 5 ms: 100s trace
    t = 1:n;
% make impulse function
    %p = 10 * ((mod(t,1000) == 0) .* square(t/1000));
% ordinary sine wave
    p = 0.1 * sin(t/1000);
% use that to synthesize sphi and add some noise
    p2phi = 0.023;
    sphi(1) = 0;
    for i=(2:n) sphi(i) = sphi(i-1) + p(i-1) * p2phi; end;
    sphi = sphi + 1.0 * random('Normal',0,1,1,n);
% add some bias to p + some noise to create sp    
    sp = p + 0.01 + 0.1 * random('Normal',0,1,1,n);
    
%--------------------------------------------------------------------------
% now we synthesized sp and sphi which represent QR movement 
%--------------------------------------------------------------------------    
% show integrated angle phi from sp and compare to real phi (sphi)
    phi_p(1) = 0;
    for i=(2:n) phi_p(i) = phi_p(i-1) + sp(i-1) * p2phi; end;
    figure; plot(t,sphi,t,sp,t,phi_p), legend('sphi', 'sp', 'phi_p');
    
%--------------------------------------------------------------------------
% now apply kalman filtering to cope with sp bias and sphi noise
%--------------------------------------------------------------------------
    p_bias(1:n) = 0;
    phi_kalman(1:n) = 0;
    C1 = 128;  
    C2 = 1000000; 
    for i=(2:n)
        p_kalman(i) = sp(i-1) - p_bias(i-1);
        phi_kalman(i) = phi_kalman(i-1) + p_kalman(i) * p2phi;
        phi_error(i) = phi_kalman(i) - sphi(i);
        phi_kalman(i) = phi_kalman(i) - phi_error(i) / C1;
        p_bias(i) = p_bias(i-1) + (phi_error(i)/p2phi) / C2;
    end;
    figure; plot(t,sphi,t,sp,t,phi_kalman), legend('sphi', 'sp', 'phi\_kalman');
    figure; plot(t,p_bias');
    % figure; plot(t,sphi,t,phi_p,t,phi_kalman);
