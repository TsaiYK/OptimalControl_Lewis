% The script is to demonstrate the example for optimal control
% The purpose is to find the feedback control strategy explicitly: u(x)
% Case: discrete-time-dynamic-system optimization
% Problem definition: 
% Dynamic system: x_{k+1} = a_k+b*u_k, k = 0,1,...,N-1 (scalar dynamics)
% We modified a little bit of the cost function: the goal is to make the
% state, x, go to zero.
% Cost function J_0 = l(x_0,u_0) + l(x_1,u_1) + ... + l(x_{N-1},u_{N-1}) + 
% Jf(x_N), where l(xk,uk) = 1/2*(x_k^2+u_k^2), Jf(x_N) = 1/2*x_N^2, and N
% is the time horizon whose value is chosen by users
% Reference: Lewis et al. 2012, Bertsekas 2000

clear;
clc;
close all

%% Parameters
para.a = 1;
para.b = 1;
para.qN = 1;
N = 5;
u0 = zeros(1,N); % initial design variable, u = [u_0,u_1,...,u_{N-1}]'
xi0 = -3:0.1:3; % samples of state for showing the control strategy
u_max = 1;

%% Gradient-based optimizer
lb = -u_max*ones(1,N);
ub = u_max*ones(1,N);
options = optimoptions('fmincon','Algorithm','sqp','Display','off');
for i = 1:length(xi0)
    fun = @(u) cost_func(u,xi0(i),para);
    [u_opt(i,:),fval(i),exitflag{i},output{i}] = fmincon(fun,u0,[],[],[],[],...
        lb,ub,[],options);
end
u_max = 1;

%% Plots of control strategies
for i = 1:N
    figure
    plot(xi0,u_opt(:,i)','LineWidth',1.5); grid on
    axis([xi0(1),xi0(end),-u_max,u_max])
    xlabel('$\xi_0$', 'FontSize', 15, 'Interpreter', 'Latex')
    ylabel(['$u^*$[',num2str(i),']'], 'FontSize', 15, 'Interpreter', 'Latex')
end

%% Gain calculation (slopes)
xi_step = xi0(2)-xi0(1);
slope = (u_opt((length(xi0)-1)/2+1,:)-u_opt((length(xi0)-1)/2+2,:))/xi_step;
