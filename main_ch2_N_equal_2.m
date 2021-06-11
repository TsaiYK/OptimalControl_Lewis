% The script is to demonstrate the example for optimal control
% The purpose is to find the feedback control strategy explicitly: u(x)
% Case: discrete-time-dynamic-system optimization
% Problem definition: 
% Dynamic system: x_{k+1} = a_k+b*u_k, k = 0,1,...,N-1 (scalar dynamics)
% We modified a little bit of the cost function: the goal is to make the
% state, x, go to zero.
% Cost function J_0 = l(x_0,u_0) + l(x_1,u_1) + ... + l(x_{N-1},u_{N-1}) + 
% Jf(x_N), where l(xk,uk) = 1/2*(x_k^2+u_k^2), Jf(x_N) = 1/2*x_N^2, and N
% is the time horizon (=2)
% Reference: Lewis et al. 2012

clear;clc;
% close all

%% Parameters
a = 1;
b = 1;
qN = 0.1;
N = 100;
x0 = -10:0.1:10; % samples of state for showing the control strategy
t = 0:N;
u_max = 1;

%% Case of free final state
Lambda = 1+a^2*qN/(1+b^2*qN);
K = [-a*b*Lambda/(1+b^2*Lambda);-a*b*qN/(1+b^2*qN)*a/(1+b^2*Lambda)];
u_policy = K*x0;
u0 = u_policy(1,:);
u_opt = min(u_max, max(-u_max, u0));

figure
plot(x0,u_opt,'.','LineWidth',1.5)
xlabel('$x$','Interpreter','Latex')
ylabel('$u^*$','Interpreter','Latex')
title('Feedback control strategy')

%% Dynamic evaluation
x = zeros(1,length(t));
x(1) = 10;
for i = 1:length(t)-1
    u_tmp = K(1)*x(i);
    u(i) = min(u_max, max(-u_max, u_tmp));
    x(i+1) = a*x(i)+b*u(i);
end

figure
subplot(2,1,1)
plot(t,x,'.','LineWidth',1.5)
ylabel('$x$','Interpreter','Latex')

subplot(2,1,2)
plot(t(1:end-1),u,'.','LineWidth',1.5)
ylabel('$u$','Interpreter','Latex')
xlabel('$k$','Interpreter','Latex')

Psi = 1/2*(sum(x(1:end-1).^2)+sum(u.^2))+1/2*qN*x(end)^2

