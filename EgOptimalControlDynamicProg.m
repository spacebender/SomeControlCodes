%{
...
OPtimal COntrol example from DE Kirk Pg:84 Chapter Dynamic Programing 
Created om 30/Jan/2020 16:10

...
%}
clearvars;clc;close all
A = [0.9974 0.0539; -0.1078 1.1591];
B = [0.0013;0.0539];
H = zeros(2,2);
Q = [0.25 0.00;0.00 0.05];
R = 0.05;

P(1:2,1:2,197) = H;
x(:,1) = [2;1];
for k = 2:198
   
    F(199-k,:) = -inv([R + B'*P(1:2,1:2,k-1)*B])*B'*P(1:2,1:2,k-1)*A;
    u(k-1) =F(199-k,:)*x(:,k-1);% Note there is some trouble with 'u'
    P(1:2,1:2,k) = [A + B*F(199-k,:)]'*P(1:2,1:2,k-1)*[A + B*F(199-k,:)]...
        +F(199-k,:)'*R*F(199-k,:) + Q;
    x(:,k) = A*x(:,k-1) + B*u(k-1);
end
% x(:,1) = [2;1];
% for k = 198:-1:1
%    
%     F(199-k,:) = -inv([R + B'*P(1:2,1:2,k-1)*B])*B'*P(1:2,1:2,k-1)*A;
%     u(199-k) =F(199-k,:)*x(:,199-k);% Note there is some trouble with 'u'
%     P(1:2,1:2,k) = [A + B*F(199-k,:)]'*P(1:2,1:2,k-1)*[A + B*F(199-k,:)]...
%         +F(199-k,:)'*R*F(199-k,:) + Q;
%     x(:,k) = A*x(:,k-1) + B*u(199-k);
% end
J = 0.5*(sum(0.25*x(1,:).^2)+sum(0.05*x(2,:).^2)+sum(0.05*u.^2));
fprintf('The value of cost is :%f\n',J)
figure
plot((1:197),u)
hold on
plot((1:198),x(2,:))
plot((1:198),x(1,:))
grid on
axis tight
title('optimal trajectory')
% legend('\it{u^*}','\it{x_1}^*','\it{x_2}^*')
legend('$u^*$','${x_1}^*$','${x_2}^*$','interpreter','latex')

figure
plot([1:197],F)
grid on
axis tight
title('Feedback Gain Coefficients')
% legend('\it{f_1}','\it{f_2}','Location','northwest')
legend('$f_1$','$f_2$','interpreter','latex','Location','northwest')