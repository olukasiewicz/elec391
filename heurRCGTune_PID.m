function KOUT = heurRCGTune_PID(KIN, Ts_target, Tr_target, OSu_target, Ess_target, p, G, H)
% heurRCGTune_PID()
% Heuristic tuning for PID with derivative filter pole p.
%
% Inputs:
%   KIN: struct with fields K, Kp, Ki, Kd (normalized gains if K=1)
%   Ts_target: settling time target (seconds)  -> constraint: Ts <= Ts_target
%   Tr_target: rise time target (seconds)      -> (soft preference term)
%   OSu_target: overshoot target (%)           -> constraint: OSu <= OSu_target
%   Ess_target: steady-state error target (%)  -> constraint: Ess <= Ess_target
%   p: derivative filter pole (negative number, e.g. -600)
%   G: forward path (plant TF)
%   H: feedback path (usually 1)
%
% Output:
%   KOUT = [Kp_best, Ki_best, Kd_best]  (ACTUAL gains including KIN.K)

    warning('off');
    s = tf('s');

    % --- Step response helper ---
    function [Ts, OSu, Ess, Tr, valid] = getStep(Kp, Ki, Kd)
        try
            % PID with derivative filter pole p (p must be negative)
            D = Kp + Ki/s + Kd * (-p*s/(s - p));

            % Closed-loop
            T = feedback(G*D, H);

            info = stepinfo(T, 'RiseTimeLimits', [0 1]);
            Ts  = info.SettlingTime;
            Tr  = info.RiseTime;
            OSu = info.Overshoot;
            Ess = abs(1 - dcgain(T)) * 100;   % percent

            valid = all(isfinite([Ts, Tr, OSu, Ess]));
        catch
            Ts = Inf; Tr = Inf; OSu = Inf; Ess = Inf;
            valid = false;
        end
    end

    % --- Violation-only cost (constraints) + soft preference terms ---
    function cost = costFunction(x)
        Kp = x(1);
        Ki = x(2);
        Kd = x(3);

        % IMPORTANT: capture Tr (don't throw it away)
        [Ts, OSu, Ess, Tr, valid] = getStep(Kp, Ki, Kd);

        if ~valid
            cost = Inf;
            return;
        end

        % --- Constraint violations (must be <= targets) ---
        vTs  = max(0, Ts  - Ts_target);
        vOSu = max(0, OSu - OSu_target);
        vEss = max(0, Ess - Ess_target);

        cost = 0;

        % HARD penalties
        if vTs > 0
            cost = cost + 100*(vTs/Ts_target)^2;
        end

        if vOSu > 0
            cost = cost + 100*(vOSu/OSu_target)^2;
        end

        if vEss > 0
            cost = cost + 10*(vEss/max(1,Ess_target))^2;
        end

        % --- Soft performance preference (also improves inside feasible region) ---
        % Guard against divide-by-zero if Tr_target is 0
        Tr_den = max(Tr_target, 1e-6);

        cost = cost + 0.2*(Ts/Ts_target)^2 ...
                     + 0.2*(OSu/max(1,OSu_target))^2 ...
                     + 0.5*(Tr/Tr_den)^2;
    end

    % --- Initial guess (ACTUAL gains) ---
    Kp0 = KIN.Kp * KIN.K;
    Ki0 = KIN.Ki * KIN.K;
    Kd0 = KIN.Kd * KIN.K;

    x0 = [3*Kp0, Ki0, Kd0];

    % --- Adaptive bounds ---
    lb = [0, 0, 0];

    base = abs(x0);
    base(base < 1e-9) = 1;

    % Force meaningful exploration for Kd even if seed ~0
    base(3) = max(base(3), 0.1*base(1));

    ub = 500*base + 1;

    % --- Cap Kp ---
    Kp_max = 15;
    
    % Ensure initial guess respects the cap
    x0(1) = min(x0(1), Kp_max);
    
    % Ensure upper bound respects the cap
    ub(1) = min(ub(1), Kp_max);

    % --- Optimizer settings ---
    options = optimoptions('fmincon', ...
        'Display', 'iter', ...
        'Algorithm', 'sqp', ...
        'MaxIterations', 500);

    best = fmincon(@costFunction, x0, [], [], [], [], lb, ub, [], options);

    KOUT = best;

    warning('on');

    % Printout
    fprintf('==========================================\n');
    fprintf('Found PID Parameters (actual gains):\n');
    fprintf('Kp = %.6g\nKi = %.6g\nKd = %.6g\n', best(1), best(2), best(3));

    [Ts, OSu, Ess, Tr, ~] = getStep(best(1), best(2), best(3));
    fprintf('Step response: Ts = %.4f s (target %.4f), ', Ts, Ts_target);
    fprintf('Tr = %.4f s (target %.4f), ', Tr, Tr_target);
    fprintf('OSu = %.4f%% (target %.4f), ', OSu, OSu_target);
    fprintf('Ess = %.4f%% (target %.4f)\n', Ess, Ess_target);
end