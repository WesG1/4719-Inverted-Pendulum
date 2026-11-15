%% build_pendulum_model.m
% Builds the Simulink model of the cart-pendulum + state feedback + integral
% action + observer, runs it, and sweeps the disturbance alpha.
% Standalone: computes its own gains (same design as pendulum_project.m).
% NOTE: written to follow the project block diagram; check the first run
% against Figs. 1-3 from pendulum_project.m.
clear; clc; close all;

%% Parameters and design (base workspace; the model reads these by name)
g = 9.81; Lp = 0.5; alpha0 = 0.5; amax = 0.5; r = 1;
A = [0 1 0 0; g/Lp 0 0 0; 0 0 0 1; 0 0 0 0];
B = [0; -1/Lp; 0; 1];
C = [1 0 0 0; 0 0 1 0];
Aa = [A, zeros(4,1); -C(2,:), 0];  Ba = [B; 0];
pc = [-1.2 -1.3 -1.4 -5 -5.1];          % tuned controller poles
po = [-100 -100.1 -100.2 -100.3];       % observer poles
Kf = place(Aa, Ba, pc);
K  = Kf(1:4);  Ki = -Kf(5);             % u = -K*xhat + Ki*xi
Lo = place(A', C', po)';

mdl = 'CartPendulumControl';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl); open_system(mdl);

pos = @(x,y,w,h) [x y x+w y+h];

%% ---- Subsystem: CartPendulum  (inputs: alpha, a ; outputs: theta, s) ----
cp = [mdl '/CartPendulum'];
add_block('simulink/Ports & Subsystems/Subsystem', cp, 'Position', pos(420,60,110,70));
delete_line(cp, 'In1/1', 'Out1/1');
delete_block([cp '/In1']); delete_block([cp '/Out1']);

add_block('simulink/Sources/In1',  [cp '/alpha'], 'Position', pos(30, 30,30,14));   % port 1
add_block('simulink/Sources/In1',  [cp '/a'],     'Position', pos(30,100,30,14));   % port 2
add_block('simulink/Math Operations/Product', [cp '/aCos'], 'Position', pos(150,95,35,40));
add_block('simulink/Math Operations/Trigonometric Function', [cp '/cosT'], ...
    'Operator','cos', 'Position', pos(150,170,50,30));
add_block('simulink/Math Operations/Trigonometric Function', [cp '/sinT'], ...
    'Operator','sin', 'Position', pos(150,230,50,30));
add_block('simulink/Math Operations/Gain', [cp '/g'], 'Gain','g', 'Position', pos(240,230,40,30));
add_block('simulink/Math Operations/Sum', [cp '/SumTrig'], 'Inputs','|+-', ...
    'Position', pos(320,150,30,40));                         % g*sin(th) - a*cos(th)
add_block('simulink/Math Operations/Gain', [cp '/invL'], 'Gain','1/Lp', 'Position', pos(390,155,45,30));
add_block('simulink/Math Operations/Sum', [cp '/SumAlpha'], 'Inputs','|++', ...
    'Position', pos(480,40,30,40));                          % + alpha
add_block('simulink/Continuous/Integrator', [cp '/IntThetaDot'], 'Position', pos(550,45,35,30));
add_block('simulink/Continuous/Integrator', [cp '/IntTheta'],    'Position', pos(620,45,35,30));
add_block('simulink/Continuous/Integrator', [cp '/IntV'], 'Position', pos(250,330,35,30));
add_block('simulink/Continuous/Integrator', [cp '/IntS'], 'Position', pos(330,330,35,30));
add_block('simulink/Sinks/Out1', [cp '/theta'], 'Position', pos(720,50,30,14));  % port 1
add_block('simulink/Sinks/Out1', [cp '/s'],     'Position', pos(430,335,30,14)); % port 2

L = @(a,b) add_line(cp, a, b, 'autorouting','on');
L('a/1','aCos/1');          L('cosT/1','aCos/2');
L('sinT/1','g/1');          L('g/1','SumTrig/1');
L('aCos/1','SumTrig/2');    L('SumTrig/1','invL/1');
L('invL/1','SumAlpha/1');   L('alpha/1','SumAlpha/2');
L('SumAlpha/1','IntThetaDot/1'); L('IntThetaDot/1','IntTheta/1');
L('IntTheta/1','sinT/1');   L('IntTheta/1','cosT/1');   L('IntTheta/1','theta/1');
L('a/1','IntV/1');          L('IntV/1','IntS/1');        L('IntS/1','s/1');

%% ---- Subsystem: Observer  (inputs: u, y=[theta;s] ; output: xhat) ----
ob = [mdl '/Observer'];
add_block('simulink/Ports & Subsystems/Subsystem', ob, 'Position', pos(420,300,110,70));
delete_line(ob, 'In1/1', 'Out1/1');
delete_block([ob '/In1']); delete_block([ob '/Out1']);
add_block('simulink/Sources/In1', [ob '/u'], 'Position', pos(30,40,30,14));   % port 1
add_block('simulink/Sources/In1', [ob '/y'], 'Position', pos(30,90,30,14));   % port 2
add_block('simulink/Signal Routing/Mux', [ob '/Mux'], 'Inputs','[1 2]', 'Position', pos(120,35,10,70));  % u (1) + y (2)
add_block('simulink/Continuous/State-Space', [ob '/ObsSS'], ...
    'A','A-Lo*C', 'B','[B Lo]', 'C','eye(4)', 'D','zeros(4,3)', ...
    'Position', pos(190,40,90,60));
add_block('simulink/Sinks/Out1', [ob '/xhat'], 'Position', pos(350,62,30,14));
add_line(ob,'u/1','Mux/1','autorouting','on');
add_line(ob,'y/1','Mux/2','autorouting','on');
add_line(ob,'Mux/1','ObsSS/1','autorouting','on');
add_line(ob,'ObsSS/1','xhat/1','autorouting','on');

%% ---- Top level: reference, integral action, state feedback, saturation ----
add_block('simulink/Sources/Step', [mdl '/Reference'], 'Time','0', 'Before','0', 'After','r', ...
    'Position', pos(30,60,40,30));
add_block('simulink/Math Operations/Sum', [mdl '/ErrSum'], 'Inputs','|+-', 'Position', pos(110,55,30,40));
add_block('simulink/Continuous/Integrator', [mdl '/Integrator'], 'Position', pos(170,60,35,30));
add_block('simulink/Math Operations/Gain', [mdl '/KiGain'], 'Gain','Ki', 'Position', pos(240,60,40,30));
add_block('simulink/Math Operations/Sum', [mdl '/CtrlSum'], 'Inputs','|++', 'Position', pos(320,55,30,40));
add_block('simulink/Discontinuities/Saturation', [mdl '/Saturation'], ...
    'UpperLimit','amax', 'LowerLimit','-amax', 'Position', pos(380,60,40,30));
add_block('simulink/Sources/Constant', [mdl '/alpha'], 'Value','alpha0', 'Position', pos(330,10,40,25));
add_block('simulink/Signal Routing/Mux', [mdl '/yMux'], 'Inputs','[1 1]', 'Position', pos(600,150,10,50));  % theta, s
add_block('simulink/Math Operations/Gain', [mdl '/negK'], 'Gain','-K', ...
    'Multiplication','Matrix(K*u)', 'Position', pos(300,320,50,30));
add_block('simulink/Sinks/Out1', [mdl '/theta_out'], 'Position', pos(700,70,30,14));
add_block('simulink/Sinks/Out1', [mdl '/s_out'],     'Position', pos(700,110,30,14));
add_block('simulink/Sinks/Out1', [mdl '/a_out'],     'Position', pos(480,200,30,14));

T = @(a,b) add_line(mdl, a, b, 'autorouting','on');
T('Reference/1','ErrSum/1');   T('ErrSum/1','Integrator/1');
T('Integrator/1','KiGain/1');  T('KiGain/1','CtrlSum/1');
T('negK/1','CtrlSum/2');       T('CtrlSum/1','Saturation/1');
T('alpha/1','CartPendulum/1'); T('Saturation/1','CartPendulum/2');
T('Saturation/1','Observer/1');T('Saturation/1','a_out/1');
T('CartPendulum/1','yMux/1');  T('CartPendulum/2','yMux/2');
T('yMux/1','Observer/2');      T('Observer/1','negK/1');
T('CartPendulum/2','ErrSum/2');
T('CartPendulum/1','theta_out/1'); T('CartPendulum/2','s_out/1');

%% ---- Solver settings ----
set_param(mdl, 'StopTime','10', 'Solver','ode45', 'SolverType','Variable-step', ...
    'MaxStep','0.01', 'RelTol','1e-8', 'AbsTol','1e-10', ...
    'SaveOutput','on', 'SaveFormat','Array', 'ReturnWorkspaceOutputs','on');
save_system(mdl);

%% ---- Run nominal case ----
out = sim(mdl);
t = out.tout;  y = out.yout;           % columns: theta, s, a
th = y(:,1); s = y(:,2); a = y(:,3);
fprintf('max s = %.5f m, s(10) = %.5f m, max|theta| = %.4f rad, max|a| = %.4f\n', ...
    max(s), s(end), max(abs(th)), max(abs(a)));
idx = find(abs(s-r) > 0.02, 1, 'last');
fprintf('2%% settling time = %.2f s\n', t(idx+1));

figure('Position',[100 100 600 700]);
subplot(3,1,1); plot(t,s,'LineWidth',1.5); yline(r,'--'); grid on;
ylabel('s(t) [m]'); title('Fig. 1  Cart position');
subplot(3,1,2); plot(t,th,'LineWidth',1.5); grid on;
ylabel('\theta(t) [rad]'); title('Fig. 2  Pendulum angle');
subplot(3,1,3); plot(t,a,'LineWidth',1.5); grid on;
ylabel('a(t) [m/s^2]'); xlabel('t [s]'); title('Fig. 3  Control input (saturated)');

%% ---- Step 7: disturbance sweep ----
set_param(mdl, 'StopTime','25');
lastGood = NaN;
for al = 0.5:0.01:1.5
    set_param([mdl '/alpha'], 'Value', num2str(al));
    try
        o = sim(mdl);
        yy = o.yout;
        ok = all(isfinite(yy(:))) && max(abs(yy(:,1))) < 1 && abs(yy(end,2)-r) < 0.01;
    catch
        ok = false;                    % solver blew up -> unstable
    end
    if ok, lastGood = al; else, break; end
end
fprintf('Largest alpha with stable response: %.2f rad/s^2\n', lastGood);
set_param([mdl '/alpha'], 'Value', 'alpha0');
set_param(mdl, 'StopTime','10');
save_system(mdl);
