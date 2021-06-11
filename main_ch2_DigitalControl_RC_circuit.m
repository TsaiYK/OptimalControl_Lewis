% The script is to demonstrate the example for optimal control
% The purpose is to find the feedback control strategy explicitly: u(x)
% Case: discrete-time-dynamic-system optimization
% Problem definition: 
% Dynamic system: x_{k+1} = a_k+b*u_k, k = 0,1,...,N-1 (scalar dynamics)
% We modified a little bit of the cost function: the goal is to make the
% state, x, go to zero.
% This is a digital control of a RC circuit system. The system equation:
% dx = -1/tau*x+1/tau*u, where tau = 1/R/C, R is the resistance, and C is
% the capacitance
% Cost function J_0 = l(x_0,u_0) + l(x_1,u_1) + ... + l(x_{N-1},u_{N-1}) + 
% Jf(x_N), where l(xk,uk) = 1/2*(x_k^2+u_k^2), Jf(x_N) = 1/2*x_N^2, and N
% is the time horizon (=1)
% Reference: Lewis et al. 2012

clear;
clc;
close all

%% Parameters
tau = 5; % tau = 1/R/C
ac = -1/tau;
bc = 1/tau;
N = 10;

T = 0.5; % sampling period time
tf = T*N;
k = 0:N;
t_step = 1e-3;
t = 0:t_step:tf;

a = exp(ac*T);
b = integral(@(t) exp(ac*t)*bc,0,T);
qN = 1;
x0 = -10:0.1:10; % samples of state for showing the control strategy
u_max = 0.5;

%% Case of free final state
slope = -a*b*qN/(1+b^2*qN);
u0 = slope*x0;
u_opt = min(u_max, max(-u_max, u0));

figure
plot(x0,u_opt,'LineWidth',2); hold on
xlabel('$\xi$ (V)','FontSize', 10, 'Interpreter', 'Latex')
ylabel('$u^*$ (V)','FontSize', 10, 'Interpreter', 'Latex')
title('Feedback control strategy')

%% Dynamic evaluation
x = zeros(1,length(k));
x(1) = 10;
for i = 1:length(k)-1
    u_tmp = slope*x(i);
    u(i) = min(u_max, max(-u_max, u_tmp));
    x(i+1) = a*x(i)+b*u(i);
end
j = 1;
for i = 1:T/t_step:length(t)-1
    u_continuous(i:i+T/t_step-1) = repmat(u(j),1,T/t_step);
    j = j+1;
end

figure
subplot(2,1,1)
plot(k*T,x,'LineWidth',2); hold on
ylabel('$\xi$ (V)', 'FontSize', 10, 'Interpreter', 'Latex')

subplot(2,1,2)
% plot(k(1:end-1)*T,u,'LineWidth',2); hold on
plot(t(1:end-1),u_continuous,'LineWidth',2); hold on
ylabel('$u$ (V)', 'FontSize', 10, 'Interpreter', 'Latex')
xlabel('$t$ (sec)', 'FontSize', 10, 'Interpreter', 'Latex')

Psi = 1/2*(sum(x(1:end-1).^2)+sum(u.^2))+1/2*qN*x(end)^2

