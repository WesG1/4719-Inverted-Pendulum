%% ECE 4719 Inverted Pendulum Project - design + nonlinear simulation
% Cart-pendulum: state-feedback + integral action + full-order observer.
clear; clc; close all;

g = 9.81; Lp = 0.5; alpha0 = 0.5; amax = 0.5; r = 1;

%% I. Linearized model  (sin(th)~th, cos(th)~1)
% x = [theta; theta_dot; s; s_dot], u = a, w = alpha, y = [theta; s]
%   theta_ddot = (g/L) theta - (1/L) a + alpha
%   s_ddot     = a
A = [0 1 0 0; g/Lp 0 0 0; 0 0 0 1; 0 0 0 0];
B = [0; -1/Lp; 0; 1];
E = [0; 1; 0; 0];                 % disturbance input matrix
C = [1 0 0 0; 0 0 1 0];

disp('Open-loop eigenvalues:'); disp(eig(A));          % +/-4.4294, 0, 0 -> unstable
fprintf('rank(ctrb) = %d, rank(obsv) = %d (n = 4)\n', rank(ctrb(A,B)), rank(obsv(A,C)));

%% Integral action on cart position: xi_dot = r - s
Aa = [A, zeros(4,1); -C(2,:), 0];
Ba = [B; 0];
fprintf('rank(ctrb augmented) = %d (n = 5)\n', rank(ctrb(Aa,Ba)));

%% Pole selection  (choose design = 1 for the hinted poles, 2 for the tuned poles)
design = 2;
if design == 1
    pc = [-1.8 -1.81 -1.82 -1.83 -1.84];
else
    pc = [-1.2 -1.3 -1.4 -5 -5.1];      % tuned: more margin to the saturation limit
end
po = [-100 -100.1 -100.2 -100.3];       % observer poles

Kf = place(Aa, Ba, pc);                 % u = -Kf*[x; xi]
K  = Kf(1:4);                           % state-feedback gain
Ki = -Kf(5);                            % u = -K*xhat + Ki*xi   (as in the block diagram)
Lo = place(A', C', po)';                % observer gain (4x2)

fprintf('K  = '); disp(K); fprintf('Ki = %.4f\n', Ki); disp('Lo ='); disp(Lo);

%% II. Nonlinear simulation (with saturation on a(t))
tf = 10;
[t, X, a] = simulate(alpha0, tf, A, B, C, K, Ki, Lo, g, Lp, amax, r);

fprintf('max s = %.5f m (overshoot = %.2e), s(10) = %.5f m\n', max(X(:,3)), max(X(:,3))-r, X(end,3));
fprintf('max |theta| = %.4f rad, max |a| = %.4f m/s^2\n', max(abs(X(:,1))), max(abs(a)));
idx = find(abs(X(:,3)-r) > 0.02, 1, 'last');
fprintf('2%% settling time = %.2f s\n', t(idx+1));

figure('Position',[100 100 600 700]);
subplot(3,1,1); plot(t,X(:,3),'LineWidth',1.5); hold on; yline(r,'--'); grid on;
ylabel('s(t) [m]'); title('Fig. 1  Cart position');
subplot(3,1,2); plot(t,X(:,1),'LineWidth',1.5); grid on;
ylabel('\theta(t) [rad]'); title('Fig. 2  Pendulum angle');
subplot(3,1,3); plot(t,a,'LineWidth',1.5); grid on;
ylabel('a(t) [m/s^2]'); xlabel('t [s]'); title('Fig. 3  Control input (saturated)');

%% Step 7: largest disturbance that keeps the closed loop stable
alphas = 0.5:0.01:1.5; lastGood = NaN;
for al = alphas
    [~, Xs] = simulate(al, 25, A, B, C, K, Ki, Lo, g, Lp, amax, r);
    ok = all(isfinite(Xs(:))) && max(abs(Xs(:,1))) < 1 && abs(Xs(end,3)-r) < 0.01;
    if ok, lastGood = al; else, break; end
end
fprintf('Largest alpha with stable response (0.01 resolution): %.2f rad/s^2\n', lastGood);

%% ---- local function ----
function [t, X, a] = simulate(alpha, tf, A, B, C, K, Ki, Lo, g, Lp, amax, r)
    % y = [x(4); xhat(4); xi]
    opts = odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',0.01);
    [t, Y] = ode45(@(t,y) rhs(y), [0 tf], zeros(9,1), opts);
    X = Y(:,1:4);
    a = zeros(size(t));
    for k = 1:numel(t)
        a(k) = min(max(-K*Y(k,5:8)' + Ki*Y(k,9), -amax), amax);
    end
    function dy = rhs(y)
        x = y(1:4); xh = y(5:8); xi = y(9);
        u  = min(max(-K*xh + Ki*xi, -amax), amax);        % saturation
        dx = [x(2); (g*sin(x(1)) - u*cos(x(1)))/Lp + alpha; x(4); u];  % nonlinear plant
        dxh = A*xh + B*u + Lo*(C*x - C*xh);                % observer (linear model)
        dy = [dx; dxh; r - x(3)];
    end
end
