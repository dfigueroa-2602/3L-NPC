function fitness = LQR_Search(Ts,n_h,w_vec,R_mode,sw,A_aug,B_aug,Ad,Bd,Ard,Brd,Ha,beta_c,VDC,w,A_ref,Tsim,T_settling)
% Function that injects the given swarm into the lqr function, then call
% the simulation function.
% Version 0.2
% Dave Figueroa
% January 2026

    nx  = size(Bd,1); 
    nu  = size(Bd,2);
    nxr = size(Brd,1);

    states_per_h = nxr / n_h;

    % ---- split particle sw ----
    idx_x_end = nx/2;
    idx_u_end = idx_x_end + nu/2;

    sw_x  = sw(1:idx_x_end);                  % plant states
    sw_u  = sw(idx_x_end+1:idx_u_end);        % inputs
    sw_hr = sw(idx_u_end+1:idx_u_end+n_h);    % 1 scalar per harmonic

    % ---- plant part of Q, as before (log scale, duplicated for +- states) ----
    Qx = repelem(sw_x,2);           % length nx
    Qu = repelem(sw_u,2);           % length nu

    % ---- resonant part of Q with Bryson scaling ----
    Qr_vec = zeros(nxr,1);
    offset = 0;
    for i = 1:n_h
        qi  = 10.^sw_hr(i);   % optimization variable in log10 scale
        wi2 = w_vec(i)^2;
        Qr_block = qi * wi2 * ones(states_per_h,1);  % same for all states of this harmonic
        Qr_vec(offset+1:offset+states_per_h) = Qr_block;
        offset = offset + states_per_h;
    end

    if isrow(Qu)
        Qu = Qu';
    end

    % ---- assemble total Q ----
    Q_diag = [10.^Qx; 10.^Qu; Qr_vec];
    Q      = diag(Q_diag);

    if R_mode == 1
        Ru = repelem(sw(end - nu/2 + 1:end),2);
        R = diag((10.^Ru)');
    else
        R = diag(repelem(1,nu));
    end

    if any(R(:) < 0,1) || any(Q(:) < 0,1)
        disp('One of the elements of the Q and R matrixes is negative!')
        fitness = 1e6; return;
    end
    
    try
        [K,~,~] = dlqr(A_aug, B_aug, Q, R);
    catch
        disp('Error in the LQR')
        fitness = 1e6; return;
    end

    Kx = K(:,1:nx);
    Ku = K(:,(nx + 1):(nx + nu));
    Kr = K(:,(nx + nu + 1):end);

    Acl = A_aug - B_aug*K;
    spec = max(abs(eig(Acl)));
    if spec >= 1
        fitness = 1e6;
        disp('One of the particles is too close of the instable area!')
        return;
    end

    try
        fitness = Sim_ResSystem(Tsim,Ts,VDC,w,A_ref,T_settling,beta_c,Ad,Bd,Ard,Brd,Ha,Kx,Kr,Ku);
    catch ME
        fprintf('\nSimulation failed: %s\n', ME.message)
        disp(getReport(ME, 'extended'))
        fitness = 1e6;
        return;
    end
end