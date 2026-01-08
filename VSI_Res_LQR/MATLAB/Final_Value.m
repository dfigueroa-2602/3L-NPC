function [Kx, Kr, Ku] = Final_Value(b_swarm,w,w_vec,Ad_aug,Bd_aug,nx,nu,nxr,n_h,R_mode)
% Function that gives the gains from the best swarm
% Version 0.2
% Dave Figueroa
% January 2026
    
    sw_best = squeeze(b_swarm);

    states_per_h = nxr / n_h;

    % ---- split particle sw ----
    idx_x_end = nx/2;
    idx_u_end = idx_x_end + nu/2;

    sw_x  = sw_best(1:idx_x_end);                  % plant states
    sw_u  = sw_best(idx_x_end+1:idx_u_end);        % inputs
    sw_hr = sw_best(idx_u_end+1:idx_u_end+n_h);    % 1 scalar per harmonic

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
        Ru = repelem(sw_best(end - nu/2 + 1:end),2);
        R = diag((10.^Ru)');
    else
        R = diag(repelem(1,nu));
    end
    
   [K,~,~] = dlqr(Ad_aug, Bd_aug, Q, R);

    Kx = K(:,1:nx);
    Ku = K(:,(nx + 1):(nx + nu));
    Kr = K(:,(nx + nu + 1):end);
end