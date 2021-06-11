% The script is to demonstrate the example for optimal control
% The purpose is to find the feedback control strategy explicitly: u(x)
% Case: discrete-time-dynamic-system optimization
% Problem definition: 
% Dynamic system: x_{k+1} = a_k+b*u_k, k = 0,1,...,N-1 (scalar dynamics)
% We modified a little bit of the cost function: the goal is to make the
% state, x, go to zero.
% Cost function J_0 = l(x_0,u_0) + l(x_1,u_1) + ... + l(x_{N-1},u_{N-1}) + 
% Jf(x_N), where l(xk,uk) = 1/2*(x_k^2+u_k^2), Jf(x_N) = 1/2*x_N^2, and N
% is the time horizon (=1)
% Reference: Lewis et al. 2012

clear;
clc;
close all

%% Parameters
a = 1;
b = 1;
qN = 1;
N = 10;
x0 = -10:0.1:10; % samples of state for showing the control strategy
t = 0:N;
u_max = 2;

%% Objective space
u = -2:0.01:2;

[X,Y] = meshgrid(x0,u);
X = X'; Y = Y';
J = 1/2*(1+a^2)*X.^2+1/2*(1+b^2)*Y.^2+a*b*X.*Y;

[J_min,index_min] = min(J,[],2);
u_opt = u(index_min);

figure
surf(X,Y,J); hold on
shading flat
plot3(x0,u_opt,J_min,'k-','LineWidth',2)
xlabel('$\xi_0$', 'FontSize', 15,'Interpreter','Latex')
ylabel('$u$', 'FontSize', 15,'Interpreter','Latex')
zlabel('$J$', 'FontSize', 15,'Interpreter','Latex')
title('Objective space', 'FontSize', 15)


