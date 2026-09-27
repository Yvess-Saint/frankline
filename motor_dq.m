function [omega_r, Te, ids, iqs] = motor_dq(Vd, Vq, Rs, Rr, Ls, Lr, Lm, p, J, B, omega_s, fs, N)
    dt = 1/fs;

    % State variables: [ids; iqs; idr; iqr; omega_r]
    x = zeros(5, N);
    x(:,1) = [0; 0; 0; 0; 0];

    % Inductance matrix
    L = [Ls 0 Lm 0;
         0 Ls 0 Lm;
         Lm 0 Lr 0;
         0 Lm 0 Lr];
    L_inv = inv(L);

    for k = 1:N-1
        ids_k = x(1,k); iqs_k = x(2,k);
        idr_k = x(3,k); iqr_k = x(4,k);
        wr_k  = x(5,k);

        % Flux linkages
        psi_ds = Ls*ids_k + Lm*idr_k;
        psi_qs = Ls*iqs_k + Lm*iqr_k;
        psi_dr = Lr*idr_k + Lm*ids_k;
        psi_qr = Lr*iqr_k + Lm*iqs_k;

        % Voltage equations
        v = [Vd(k); Vq(k); 0; 0];
        R = diag([Rs Rs Rr Rr]);
        omega_matrix = [0 -omega_s 0 0;
                        omega_s 0 0 0;
                        0 0 0 -(omega_s - wr_k);
                        0 0 (omega_s - wr_k) 0];

        di = L_inv * (v - R*[ids_k; iqs_k; idr_k; iqr_k] - omega_matrix*[psi_ds; psi_qs; psi_dr; psi_qr]);

        % Torque
        Te_k = 1.5 * p * Lm * (iqs_k*idr_k - ids_k*iqr_k);

        % Mechanical
        dwr = (Te_k - B*wr_k) / J;

        % Update states (Euler integration)
        x(1,k+1) = ids_k + dt*di(1);
        x(2,k+1) = iqs_k + dt*di(2);
        x(3,k+1) = idr_k + dt*di(3);
        x(4,k+1) = iqr_k + dt*di(4);
        x(5,k+1) = wr_k  + dt*dwr;
    end

    ids = x(1,:);
    iqs = x(2,:);
    omega_r = x(5,:);
    Te = 1.5 * p * Lm * (x(2,:).*x(3,:) - x(1,:).*x(4,:));
end
