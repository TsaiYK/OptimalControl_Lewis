% The script is to demonstrate the example for optimal control
% By the case: discrete-timer optimization
% Problem definition: 
% Dynamic system: x_{k+1} = a_k+b*u_k, k = 0,1,...,N-1 (scalar dynamics)
% Cost function J = r/2*(u_0^2+...+u_{N-1}^2)
% For two cases: (a) fixed final state (b) free final state
% Both have analytical solutions
% Reference: Lewis et al. 2012

clear;clc;close all

%% Parameters
a = 0.99;
b = 0.1;
N = 100;
x0 = 0; % initial state
rN = 10; % final state
t = 0:N;

%% Case of fixed final state
% x = zeros(1,N);
% x(1) = x0;
% u_opt = zeros(1,N-1);
% for k = 1:N-1
%     u_opt(k) = (1-a^2)/b/(1-a^(2*N))*(rN-a^N*x0)*a^(N-k-1);
%     x(k+1) = a*x(k)+b*u_opt(k);
% end

% figure
% plot(1:N,x)
% figure
% plot(1:N-1,u_opt)

%% Case of free final state
r = [0,0.1,0.5,1,10];
x = zeros(length(r),N);
x(:,1) = repmat(x0,length(r),1);
u_opt = zeros(length(r),N-1);
for i = 1:length(r)
    Lambda = b^2*(1-a^(2*N))/r(i)/(1-a^2);
    for k = 1:N
        if r(i)==0
            u_opt(i,k) = (1-a^2)/b/(1-a^(2*N))*(rN-a^N*x0)*a^(N-k-1);
        else
            u_opt(i,k) = b/r(i)/(1+Lambda)*(rN-a^N*x0)*a^(N-k-1);
        end
        x(i,k+1) = a*x(i,k)+b*u_opt(i,k);
    end
    figure(1)
    plot(t,x(i,:)); hold on
    figure(2)
    plot(t(1:end-1),u_opt(i,:)); hold on
    LegendInfo{i} = ['r=',num2str(r(i))];
end
figure(1)
legend(LegendInfo);
xlabel('$k$','interpreter','latex'); ylabel('$x^*$','interpreter','latex')
figure(2)
legend(LegendInfo);
xlabel('$k$','interpreter','latex'); ylabel('$u^*$','interpreter','latex')



