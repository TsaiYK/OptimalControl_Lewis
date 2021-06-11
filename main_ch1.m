% The script is to demonstrate the example for optimal control
% By the case: static optimization
% Reference: Lewis et al. 2012

clear
clc
close all

% Define the weight matrix Q
% if Q is positive definite, u_opt is minimum
% if Q is negative definite, u_opt is maximum
% if Q is nondefinite, u_opt is a saddle point
Q = [1,1;1,2];
detQ = det(Q)
eig_Q = eig(Q);

for i = 1:rank(Q)
  if eig_Q(i) <= 0 
    flag = 1;
  end
end
if flag == 1
  disp('the matrix is not positive definite')
else
  disp('the matrix is positive definite')
end
% [~,p] = chol(Q)
S = [0;1];

u1 = linspace(-5,5,100);
u2 = linspace(-5,5,100);

for i = 1:length(u1)
    for j = 1:length(u2)
        u = [u1(i);u2(j)];
        L(i,j) = 1/2*u'*Q*u+S'*u;
    end
end

u_opt = -inv(Q)*S;


[X,Y] = meshgrid(u1,u2);
X = X';
Y = Y';
figure
surf(X,Y,L); hold on
plot3(u_opt(1),u_opt(2),min(min(L)),'r*');
xlabel('u_1'); ylabel('u_2'); zlabel('L')

figure
contour(X,Y,L); hold on
plot(u_opt(1),u_opt(2),'r*');
xlabel('u_1'); ylabel('u_2');
colorbar


