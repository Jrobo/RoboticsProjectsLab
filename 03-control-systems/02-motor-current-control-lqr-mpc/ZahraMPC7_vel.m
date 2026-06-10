function [du, dx] = ZahraMPC7_vel(Ad, Bd, Q_LQ, R_LQ, S, N, du_min, du_max, umin, umax, U_bar_dis, xmin, xmax, xref, x, u_prev, input)
    m = size(Bd, 2);
    n = size(Ad, 1);

    % Q and R matrices for Open-Loop MPC (with Q1)
    Qsig = blkdiag(kron(eye(N-1),Q_LQ),S);
    Rsig = kron(eye(N),R_LQ);

    % Build incremental input matrices
    % U = [Δu_0; Δu_1; ...; Δu_{N-1}]
    % u_k = u_prev + sum_{i=0}^{k} Δu_i
    % Build the mapping from ΔU to U
    T = tril(ones(N)); % Lower triangular matrix for cumulative sum

    % A matrix for prediction
    Asig = Ad;
    for i = 2:N
        Asig = [Asig; Ad^i];
    end

    % B matrix for prediction
    Bsig = [];
    for i = 1:N
        temp = zeros(size(Bd,1)*(i-1),size(Bd,2));
        for j = 0:N-i
            temp = [temp; Ad^(j)*Bd];
        end
        Bsig = [Bsig temp];
    end

    % Map ΔU to U
    % U = T * ΔU + 1_N ⊗ u_prev
    % So, x_pred = Asig*x + Bsig*(T*ΔU + kron(ones(N,1),u_prev))
    %           = (Asig*x + Bsig*kron(ones(N,1),u_prev)) + Bsig*T*ΔU

    % Update cost function for ΔU
    H = (Bsig*T)'*Qsig*(Bsig*T) + Rsig;
    F = Asig'*Qsig*Bsig*T;
    f = (x'*F)';

    % Input increment constraints (Δu)
    lb_du = repmat(du_min, N, 1);
    ub_du = repmat(du_max, N, 1);

    % Absolute input constraints (u)
    % U = T*ΔU + kron(ones(N,1),u_prev)
    lb_u = repmat(umin-U_bar_dis, N, 1) - kron(ones(N,1),u_prev);
    ub_u = repmat(umax-U_bar_dis, N, 1) - kron(ones(N,1),u_prev);

    % State constraints
    lb_x = repmat(xmin-xref, N, 1);
    ub_x = repmat(xmax-xref, N, 1);

    % Combine constraints for quadprog: A*ΔU <= b
    % State constraints: Bsig*T*ΔU <= ub_x - Asig*x - Bsig*kron(ones(N,1),u_prev)
    A_x = [ Bsig*T; -Bsig*T ];
    b_x = [ ub_x - Asig*x - Bsig*kron(ones(N,1),u_prev);
           -(lb_x - Asig*x - Bsig*kron(ones(N,1),u_prev)) ];

    % Input constraints: T*ΔU <= ub_u, -T*ΔU <= -lb_u
    A_u = [ T; -T ];
    b_u = [ ub_u; -lb_u ];

    % Increment constraints: I*ΔU <= ub_du, -I*ΔU <= -lb_du
    A_du = [ eye(N*m); -eye(N*m) ];
    b_du = [ ub_du; -lb_du ];

    % Combine all constraints
    A_cons = [A_x; A_u; A_du];
    b_cons = [b_x; b_u; b_du];

    options = optimset('Algorithm', 'interior-point-convex','Diagnostics','off', ...
        'Display','off');

    % Solve the quadratic programming problem
    [DU, fval, exitflag] = quadprog(H, f, A_cons, b_cons, [], [], [], [], [], options);

    if exitflag == -2
        [DU, fval, exitflag] = quadprog(H, f, [], [], [], [], [], [], [], options);
    end

    % Get the optimal input increment (the receding horizon principle is applied)
    du = DU(1:m);

    % --- Velocity form for state update ---
    u = u_prev + du;
    dx = (Ad - eye(n)) * x + Bd * u;
end
