%{ 
...
This is an example from "Fundamentals of spacecraft attitude detreminationa
nd COntrol" 
Example 7.1 Pg 292 (Sec 7.1 Attitude Control Regulation Case)

created on  01/Feb/2020 17:34:02
...
%}
clc;clearvars;close all
% Set the default Axis Parameters.
textSize = 14;
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'DefaultAxesFontSize', textSize)
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultTextFontSize', textSize)

% Set all the Parameters and the initial condtions
J = [10000 0 0;...
     0 9000 0;...
     0 0 12000];
qc = [0;cos(1);0;sin(1)];
q0 = [0.6853 0.6953 0.1531 0.1531];
w0 = deg2rad([0.5300 0.5300 0.0530]);
Kp = 50;
Kd = 500;
tspan = [0 300];
Ic = [w0 q0];

[t,x] = ode45(@(t,w) dynamicEqSatellite(t,w,qc,J,Kp,Kd) ,tspan, Ic); 

% Extract and CAlculate all the required VAlues 
q1 = x(:,4); q2 = x(:,5); q3 = x(:,6); q4 = x(:,7);

qc1 = qc(1); qc2 = qc(2); qc3 = qc(3); qc4 = qc(4);
qcmat = [qc4 qc3 -qc2 -qc1;...
         -qc3 qc4 qc1 -qc2;...
         qc2 -qc1 qc4 -qc3];
delq = [qcmat*[q1 q2 q3 q4]';([q1 q2 q3 q4]*qc)'];
        
del_q1 = delq(1,:); 
del_q2 = delq(2,:); 
del_q3 = delq(3,:); 
del_q4 = delq(4,:);


w1 = x(:,1) ; w2 = x(:,2); w3 = x(:,3);

L = (-Kp*sign(del_q4).*[del_q1;del_q2;del_q3])' - Kd*[w1 w2 w3];

% plot the Figures
 figure()
 subplot(3,1,1)
 plot(t,L(:,1))
 ylabel('$L_1$','interpreter','latex')
 ylim([-40,20])
 grid on
 
 subplot(3,1,2)
 plot(t,L(:,2))
 ylim([-40,20])
 grid on
 ylabel('$L_2$','interpreter','latex')
 
 subplot(3,1,3)
 plot(t,L(:,3))
 ylim([-10,5])
 grid on
 ylabel('$L_3$','interpreter','latex')
 xlabel('$time(t)$','interpreter','latex')
 sgtitle('Controller Torques')
 
 
 figure()
 
 subplot(4,1,1)
 plot(t,del_q1)
 ylim([-0.2,0.8])
 grid on
 ylabel('$\delta_{q1}$','interpreter','latex')
 
 subplot(4,1,2)
 plot(t,del_q2)
 ylim([-0.2,0.8])
 grid on
 ylabel('$\delta_{q2}$','interpreter','latex')
 
 subplot(4,1,3)
 plot(t,del_q3)
 ylim([-0.2,0.2])
 grid on
 ylabel('$\delta_{q3}$','interpreter','latex')
 
 subplot(4,1,4)
 plot(t,del_q4)
 ylim([-0.2,1.2])
  grid on
 ylabel('$\delta_{q4}$','interpreter','latex')
 xlabel('$time(t)$','interpreter','latex')
 sgtitle('Quaternion Errors')

 
 figure()

 subplot(3,1,1)
 plot(t,x(:,1))
 ylabel('$w_1$','interpreter','latex')
 grid on
 
 subplot(3,1,2)
 plot(t,x(:,2))
 grid on
 ylabel('$w_2$','interpreter','latex')
 
 subplot(3,1,3)
 plot(t,x(:,3))
 grid on
 ylabel('$w_3$','interpreter','latex')
 xlabel('$time(t)$','interpreter','latex')
 sgtitle('$\omega$ values','interpreter','latex')
 
 
 figure()
 subplot(4,1,1)
 plot(t,x(:,4))
 grid on
 ylabel('$q_1$','interpreter','latex')
 
 subplot(4,1,2)
 plot(t,x(:,5))
 grid on
 ylabel('$q_2$','interpreter','latex')
 
 subplot(4,1,3)
 plot(t,x(:,6))
 grid on
 ylabel('$q_3$','interpreter','latex')
 
 subplot(4,1,4)
 plot(t,x(:,7))
 grid on
 ylabel('$q_4$','interpreter','latex')
 xlabel('$time(t)$','interpreter','latex')
 sgtitle('Quaternions')