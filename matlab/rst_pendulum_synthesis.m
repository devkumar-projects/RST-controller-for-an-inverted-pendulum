clear; clc; close all;

%% ============================================================
%  INITIAL RST : beta = 1, gamma = 10
%  ============================================================

%% Physical parameters
l      = 0.35;
g      = 9.81;
alpha  = sqrt(g/l);
beta   = 1;
gamma  = 10;
omega0 = 1;

%% Closed-loop polynomial
Abf = poly([-alpha; -alpha; -beta; -gamma; -beta-i*omega0; -beta+i*omega0]);

c1 = Abf(2); c2 = Abf(3); c3 = Abf(4);
c4 = Abf(5); c5 = Abf(6); c6 = Abf(7);

fprintf('\n========== INITIAL RST (beta=1, gamma=10) ==========\n')
fprintf('Coefficients of Abf :\n')
fprintf('  c1 = %.4f\n  c2 = %.4f\n  c3 = %.4f\n', c1, c2, c3)
fprintf('  c4 = %.4f\n  c5 = %.4f\n  c6 = %.4f\n', c4, c5, c6)

%% Polynomials R and S
sigma1 =  c1;
r0     =  (omega0^2 - alpha^2 - c2) / alpha^2;
r1     =  (c1*(omega0^2 - alpha^2) - c3) / alpha^2;
r2     = -(alpha^2*omega0^2 + c4) / alpha^2;
r3     = -(alpha^2*c1*omega0^2 + c5) / alpha^2;
r4     = -c6 / alpha^2;

fprintf('\nCoefficients of polynomial R(s) :\n')
fprintf('  Sigma1 = %.4f\n', sigma1)
fprintf('  r0 = %.4f\n  r1 = %.4f\n  r2 = %.4f\n', r0, r1, r2)
fprintf('  r3 = %.4f\n  r4 = %.4f\n', r3, r4)

%% Polynomial T(s)
t2 = r4 / (gamma*beta);
M  = [1-gamma*beta,  -(gamma+beta);
     -(gamma+beta),   gamma*beta-1];
Y  = [r0-r2+r4-(gamma*beta-1)*t2;
      r3-r1-(gamma+beta)*t2];
Sol = M \ Y;
t0  = Sol(1);
t1  = Sol(2);

fprintf('\nCoefficients of polynomial T(s) :\n')
fprintf('  t0 = %.4f\n  t1 = %.4f\n  t2 = %.4f\n', t0, t1, t2)

%% Controller analysis
R = [r0, r1, r2, r3, r4];
S = conv([1 0 1 0], [1 sigma1]);
T = conv([1 beta+gamma beta*gamma], [t0 t1 t2]);
A = [1, 0, -alpha^2];
B = -alpha^2;

num_BO = conv(B, R);
den_BO = conv(S, A);

[Gm, Pm, Wcg, Wcp] = margin(num_BO, den_BO);
[re, im, ~] = nyquist(tf(num_BO, den_BO));
re = squeeze(re); im = squeeze(im);
Mm = min(sqrt((re+1).^2 + im.^2));

fprintf('\nStability margins :\n')
fprintf('  Gain margin    : %.4f dB\n', 20*log10(Gm))
fprintf('  Phase margin   : %.4f deg\n', Pm)
fprintf('  Modulus margin : %.4f\n', Mm)
fprintf('  Gain crossover frequency  : %.4f rad/s\n', Wcg)
fprintf('  Phase crossover frequency : %.4f rad/s\n', Wcp)

%% Frequency plots — initial RST
figure('Name', 'Bode initial RST', 'NumberTitle', 'off');
margin(num_BO, den_BO);
title('Open-loop Bode diagram — initial RST (\beta=1, \gamma=10)')
grid on;

figure('Name', 'Nyquist initial RST', 'NumberTitle', 'off');
nyquist(num_BO, den_BO);
title('Open-loop Nyquist diagram — initial RST (\beta=1, \gamma=10)')
axis([-2 0.5 -2 2])
grid on;
hold on;
theta_circle = linspace(0, 2*pi, 200);
plot(-1 + Mm*cos(theta_circle), Mm*sin(theta_circle), 'r--', 'LineWidth', 1.5)
plot(-1, 0, 'r+', 'MarkerSize', 12, 'LineWidth', 2)
legend('Open-loop Nyquist', sprintf('Modulus margin circle M_m = %.3f', Mm), 'Critical point -1', 'Location', 'best')
hold off;

%% Simulink simulation — initial RST
H = poly([-2 -2 -1 -1]);
sim('simu_rst_pendule')

figure('Name', 'Time response initial RST', 'NumberTitle', 'off');
subplot(4,1,1);
plot(t, ymc, 'g', 'LineWidth', 1.5); hold on;
plot(t, ym,  'r', 'LineWidth', 1.2);
legend('Reference y_{mc}', 'Output y_m', 'Location', 'best')
xlabel('Time (s)'); ylabel('Position (V)');
title('Reference tracking — initial RST (\beta=1, \gamma=10)')
ylim([-3 3])
grid on;

subplot(4,1,2);
plot(t, ym - ymc, 'c', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Error (V)');
title('Tracking error e(t) = y_m(t) - y_{mc}(t)')
grid on;

subplot(4,1,3);
plot(t, u, 'm', 'LineWidth', 1.2); hold on;
yline( 5, 'r--', '+5 V limit');
yline(-5, 'r--', '-5 V limit');
xlabel('Time (s)'); ylabel('Control u (V)');
title('Control signal x_m(t)')
ylim([-7 7])
grid on;

subplot(4,1,4);
plot(t, Theta, 'Color', [1 0.5 0], 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Angle \theta (deg)');
title('Rod angle \theta(t)')
grid on;


%% ============================================================
%  OPTIMISED RST : beta = 0.5, gamma = 30
%  ============================================================

beta_opt  = 0.5;
gamma_opt = 30;

%% Closed-loop polynomial
Abf_o = poly([-alpha; -alpha; -beta_opt; -gamma_opt; ...
              -beta_opt-i*omega0; -beta_opt+i*omega0]);

c1_o = Abf_o(2); c2_o = Abf_o(3); c3_o = Abf_o(4);
c4_o = Abf_o(5); c5_o = Abf_o(6); c6_o = Abf_o(7);

fprintf('\n========== OPTIMISED RST (beta=0.5, gamma=30) ==========\n')
fprintf('Coefficients of Abf :\n')
fprintf('  c1 = %.4f\n  c2 = %.4f\n  c3 = %.4f\n', c1_o, c2_o, c3_o)
fprintf('  c4 = %.4f\n  c5 = %.4f\n  c6 = %.4f\n', c4_o, c5_o, c6_o)

%% Polynomials R and S
sigma1_o =  c1_o;
r0_o     =  (omega0^2 - alpha^2 - c2_o) / alpha^2;
r1_o     =  (c1_o*(omega0^2 - alpha^2) - c3_o) / alpha^2;
r2_o     = -(alpha^2*omega0^2 + c4_o) / alpha^2;
r3_o     = -(alpha^2*c1_o*omega0^2 + c5_o) / alpha^2;
r4_o     = -c6_o / alpha^2;

fprintf('\nCoefficients of polynomial R(s) :\n')
fprintf('  Sigma1 = %.4f\n', sigma1_o)
fprintf('  r0 = %.4f\n  r1 = %.4f\n  r2 = %.4f\n', r0_o, r1_o, r2_o)
fprintf('  r3 = %.4f\n  r4 = %.4f\n', r3_o, r4_o)

%% Polynomial T(s)
t2_o = r4_o / (gamma_opt*beta_opt);
M_o  = [1-gamma_opt*beta_opt,  -(gamma_opt+beta_opt);
       -(gamma_opt+beta_opt),   gamma_opt*beta_opt-1];
Y_o  = [r0_o-r2_o+r4_o-(gamma_opt*beta_opt-1)*t2_o;
        r3_o-r1_o-(gamma_opt+beta_opt)*t2_o];
Sol_o = M_o \ Y_o;
t0_o  = Sol_o(1);
t1_o  = Sol_o(2);

fprintf('\nCoefficients of polynomial T(s) :\n')
fprintf('  t0 = %.4f\n  t1 = %.4f\n  t2 = %.4f\n', t0_o, t1_o, t2_o)

%% Controller analysis
R_o = [r0_o, r1_o, r2_o, r3_o, r4_o];
S_o = conv([1 0 1 0], [1 sigma1_o]);
T_o = conv([1 beta_opt+gamma_opt beta_opt*gamma_opt], [t0_o t1_o t2_o]);
A_o = [1, 0, -alpha^2];
B_o = -alpha^2;

num_BO_o = conv(B_o, R_o);
den_BO_o = conv(S_o, A_o);

[Gm_o, Pm_o, Wcg_o, Wcp_o] = margin(num_BO_o, den_BO_o);
[re_o, im_o, ~] = nyquist(tf(num_BO_o, den_BO_o));
re_o = squeeze(re_o); im_o = squeeze(im_o);
Mm_o = min(sqrt((re_o+1).^2 + im_o.^2));

fprintf('\nStability margins :\n')
fprintf('  Gain margin    : %.4f dB\n', 20*log10(Gm_o))
fprintf('  Phase margin   : %.4f deg\n', Pm_o)
fprintf('  Modulus margin : %.4f\n', Mm_o)
fprintf('  Gain crossover frequency  : %.4f rad/s\n', Wcg_o)
fprintf('  Phase crossover frequency : %.4f rad/s\n', Wcp_o)

%% Frequency plots — optimised RST
figure('Name', 'Bode optimised RST', 'NumberTitle', 'off');
margin(num_BO_o, den_BO_o);
title('Open-loop Bode diagram — optimised RST (\beta=0.5, \gamma=30)')
grid on;

figure('Name', 'Nyquist optimised RST', 'NumberTitle', 'off');
nyquist(num_BO_o, den_BO_o);
title('Open-loop Nyquist diagram — optimised RST (\beta=0.5, \gamma=30)')
axis([-2 0.5 -2 2])
grid on;
hold on;
theta_circle = linspace(0, 2*pi, 200);
plot(-1 + Mm_o*cos(theta_circle), Mm_o*sin(theta_circle), 'r--', 'LineWidth', 1.5)
plot(-1, 0, 'r+', 'MarkerSize', 12, 'LineWidth', 2)
legend('Open-loop Nyquist', sprintf('Modulus margin circle M_m = %.3f', Mm_o), 'Critical point -1', 'Location', 'best')
hold off;

%% Simulink simulation — optimised RST
beta   = beta_opt;
gamma  = gamma_opt;
sigma1 = sigma1_o;
r0 = r0_o; r1 = r1_o; r2 = r2_o; r3 = r3_o; r4 = r4_o;
t0 = t0_o; t1 = t1_o; t2 = t2_o;
R  = R_o;
S  = S_o;
T  = T_o;
H  = poly([-2 -2 -1 -1]);

sim('simu_rst_pendule')

figure('Name', 'Time response optimised RST', 'NumberTitle', 'off');
subplot(4,1,1);
plot(t, ymc, 'g', 'LineWidth', 1.5); hold on;
plot(t, ym,  'r', 'LineWidth', 1.2);
legend('Reference y_{mc}', 'Output y_m', 'Location', 'best')
xlabel('Time (s)'); ylabel('Position (V)');
title('Reference tracking — optimised RST (\beta=0.5, \gamma=30)')
ylim([-3 3])
grid on;

subplot(4,1,2);
plot(t, ym - ymc, 'c', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Error (V)');
title('Tracking error e(t) = y_m(t) - y_{mc}(t)')
grid on;

subplot(4,1,3);
plot(t, u, 'm', 'LineWidth', 1.2); hold on;
yline( 5, 'r--', '+5 V limit');
yline(-5, 'r--', '-5 V limit');
xlabel('Time (s)'); ylabel('Control u (V)');
title('Control signal x_m(t)')
ylim([-7 7])
grid on;

subplot(4,1,4);
plot(t, Theta, 'Color', [1 0.5 0], 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Angle \theta (deg)');
title('Rod angle \theta(t)')
grid on;


%% Robustness test : variation of rod length L

L_values = [0.16, 0.23, 0.28, 0.35];
colors   = {'r', 'b', 'm', 'c'};

l_original = l;

results_t   = cell(length(L_values), 1);
results_ymc = cell(length(L_values), 1);
results_ym  = cell(length(L_values), 1);
results_u   = cell(length(L_values), 1);
results_th  = cell(length(L_values), 1);

fprintf('\n========== ROBUSTNESS TEST — Rod length variation ==========\n')
fprintf('(Optimised RST controller maintained : beta=%.2f, gamma=%.2f)\n\n', beta_opt, gamma_opt)
fprintf('%-10s %-10s %-15s %-15s %-15s %-12s\n',...
        'L (m)', 'alpha', 'Gm (dB)', 'Pm (deg)', 'Mm', '|u|max (V)')

for k = 1:length(L_values)
    l = L_values(k);
    alpha_k = sqrt(g/l);

    A_k = [1, 0, -alpha_k^2];
    B_k = -alpha_k^2;
    num_BO_k = conv(B_k, R_o);
    den_BO_k = conv(S_o, A_k);

    [Gm_k, Pm_k, ~, ~] = margin(num_BO_k, den_BO_k);
    [re_k, im_k, ~] = nyquist(tf(num_BO_k, den_BO_k));
    re_k = squeeze(re_k); im_k = squeeze(im_k);
    Mm_k = min(sqrt((re_k+1).^2 + im_k.^2));

    sim('simu_rst_pendule')
    results_t{k}   = t;
    results_ymc{k} = ymc;
    results_ym{k}  = ym;
    results_u{k}   = u;
    results_th{k}  = Theta;

    fprintf('%-10.2f %-10.4f %-15.4f %-15.4f %-15.4f %-12.4f\n',...
            l, alpha_k, 20*log10(Gm_k), Pm_k, Mm_k, max(abs(u)))
end

l = l_original;

labels_ym = arrayfun(@(L) sprintf('y_m (L=%.2f m)', L), L_values, 'UniformOutput', false);
labels_th = arrayfun(@(L) sprintf('\\theta (L=%.2f m)', L), L_values, 'UniformOutput', false);
labels_u  = arrayfun(@(L) sprintf('u (L=%.2f m)', L), L_values, 'UniformOutput', false);

figure('Name', 'Robustness test — rod length variation', 'NumberTitle', 'off');

subplot(3,1,1);
plot(results_t{1}, results_ymc{1}, 'g', 'LineWidth', 1.8); hold on;
for k = 1:length(L_values)
    plot(results_t{k}, results_ym{k}, colors{k}, 'LineWidth', 1.2);
end
legend([{'Reference y_{mc}'}, labels_ym], 'Location', 'best');
xlabel('Time (s)'); ylabel('Position (V)');
title('Reference tracking — Robustness to rod length variation')
ylim([-3 3])
grid on;

subplot(3,1,2);
hold on;
for k = 1:length(L_values)
    plot(results_t{k}, results_th{k}, colors{k}, 'LineWidth', 1.2);
end
legend(labels_th, 'Location', 'best');
xlabel('Time (s)'); ylabel('Angle \theta (deg)');
title('Rod angle \theta(t)')
grid on;

subplot(3,1,3);
hold on;
for k = 1:length(L_values)
    plot(results_t{k}, results_u{k}, colors{k}, 'LineWidth', 1.2);
end
yline( 5, 'k--', '+5 V limit');
yline(-5, 'k--', '-5 V limit');
legend(labels_u, 'Location', 'best');
xlabel('Time (s)'); ylabel('Control u (V)');
title('Control signal x_m(t)')
ylim([-7 7])
grid on;
