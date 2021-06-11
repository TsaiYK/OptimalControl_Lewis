function J_sum = cost_func(u,xi0,para)
xi(1) = xi0;
J = [];
for i = 1:length(u)
    J = [J,1/2*(xi(i)^2+u(i)^2)];
    xi(i+1) = para.a*xi(i)+para.b*u(i);
end
J = [J,1/2*para.qN*xi(end)];
J_sum = sum(J);