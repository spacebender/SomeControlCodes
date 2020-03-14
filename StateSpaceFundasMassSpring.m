%{ 
This is a continuuing example from 'Linear STate Space Systems'
by Robert L.Willams II and Douglas A Lawrence.
The example is Mass Spring Damper Systems both the cases are presented 

There are four structures "sys1, sys2,Par,Extras"

sys1 - All the important stuff related to sys1
sys2 - All the important stuff related to sys2
Par - Parameters of MAss Spring Damper
Extras - Intermediate values

This file is created on 18/01/2020 - 19/01/2020 23:5135
 by Karthi
%}
 

clearvars; clc; close all
fig = 0;
% Mass Spring Damper system
Par.k1 = 400;
Par.k2 = 200;
Par.c1 = 20;
Par.c2 = 10;
Par.m1 = 40;
Par.m2 = 20;
%% Case a MIMO
sys1.A = [0 1 0 0;-(Par.k1 + Par.k2)/Par.m1 -(Par.c1 + Par.c2)/Par.m1 Par.k2/Par.m1 Par.c2/Par.m1;...
    0 0 0 1;Par.k2/Par.m2 Par.c2/Par.m2 -Par.k2/Par.m2 -Par.c2/Par.m2];
sys1.B = [0 0;1/Par.m1 0;0 0;0 1/Par.m2];
sys1.C = [1 0 0 0;0 0 1 0];
sys1.D = [0 0 ;0 0];
%% Case b SISO
sys2.A = sys1.A;
sys2.B = sys1.B(:,2);
sys2.C = sys1.C(1,:);
sys2.D = 0;
%% %%%%%%%%%%%%%%%%%%%%%% CHAPTER 2  EXCERSICE %%%%%%%%%%%%%%%%%%%%%%
%% Simulate the open loop response using lsim
% form the state space for both the cases
sys1.sys = ss(sys1.A,sys1.B,sys1.C,sys1.D);
sys1.systf = tf(sys1.sys);
sys2.sys = ss(sys2.A,sys2.B,sys2.C,sys2.D);
sys2.systf = tf(sys2.sys);
%% case a open loop response
sys1.IC = [0;0;0;0];
sys1.tspan = linspace(0,40,1000);
sys1.U = [20*ones(size(sys1.tspan));10*ones(size(sys1.tspan))];
[sys1.y,sys1.t,sys1.x] = lsim(sys1.sys,sys1.U,sys1.tspan,sys1.IC);
[sys1.eigvec,sys1.eigval] = eig(sys1.A);




%% case b open loop response

sys2.IC = [0.1;0;0.2;0];
sys2.tspan = linspace(0,40,1000);
sys2.U = [zeros(size(sys1.tspan))];
[sys2.y,sys2.t,sys2.x] = lsim(sys2.sys,sys2.U,sys2.tspan,sys2.IC);
[sys2.eigvec,sys2.eigval] = eig(sys2.A);
% Figures


%% Coordinate transformation
sys1.T_dcf = sys1.eigvec;
sys1.A_dcf = inv(sys1.T_dcf)*sys1.A*sys1.T_dcf;
sys1.B_dcf = inv(sys1.T_dcf)*sys1.B;
sys1.C_dcf = sys1.C*sys1.T_dcf;
sys1.D_dcf = sys1.D;
[sys1.Canon,sys1.Tcan] = canon(sys1.sys,'modal');
sys2.T_dcf = sys2.eigvec;
sys2.A_dcf = inv(sys2.T_dcf)*sys2.A*sys2.T_dcf;
sys2.B_dcf = inv(sys2.T_dcf)*sys2.B;
sys2.C_dcf = sys2.C*sys2.T_dcf;
sys2.D_dcf = sys2.D;
[sys2.Canon,sys2.Tcan] = canon(sys2.sys,'modal');

%% %%%%%%%%%%%%%%%%%%%%%% CHAPTER 3 EXCERSICE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Controllability matrix=ces
% case a
sys1.OCF = canon(sys1.sys,'companion');
% CCF
sys1.CCF.A = sys1.OCF.A';
sys1.CCF.B = sys1.OCF.C';
sys1.CCF.C = sys1.OCF.B';
sys1.CCF.D = sys1.OCF.D;

sys2.OCF = canon(sys2.sys,'companion');
sys2.CCF.A = sys2.OCF.A';
sys2.CCF.B = sys2.OCF.C';
sys2.CCF.C = sys2.OCF.B';
sys2.CCF.D = sys2.OCF.D;

%% %%%%%%%%%%%%%%%%%%%%%%%%% CHAPTER 7 State feedback%%%%%%%%%%%%%%%%%%%%%%%
%% 
% case a
% required overshoot 5% and required setting time 2s
% requitre setting tome 2s

func = @(zeta) exp(-(zeta*pi)/sqrt(1-zeta^2))-0.05;
Extras.guess = 0.1;   % just to solve for zeta
Extras.zeta = fzero(func,Extras.guess); % zeta
Extras.omega_n = 4/(Extras.zeta*2); % omega
sys1.reqTF = tf([(Extras.omega_n)^2],[1 2*Extras.zeta*Extras.omega_n Extras.omega_n^2]);
sys1.reqTFstepinfo = stepinfo(sys1.reqTF);

% find the required eigen value 
[Extras.Areq,Extras.Breq,Extras.Creq,Extras.Dreq] = tf2ss([(Extras.omega_n)^2],...
    [1 2*Extras.zeta*Extras.omega_n Extras.omega_n^2]);% state space matrices of req system
sys1.reqSS = ss(Extras.Areq,Extras.Breq,Extras.Creq,Extras.Dreq);
[sys1.reqeigvec,sys1.reqeigval] = eig(sys1.reqSS.A);

sys1.reqeigvalue(1,1) = sys1.reqeigval(1,1);
sys1.reqeigvalue(2,1) = sys1.reqeigval(2,2);
sys1.reqeigvalue(3,1) = -20;%10*real(sys1.reqeigval(1,1));
sys1.reqeigvalue(4,1) = -21;%10*real(sys1.reqeigval(1,1))+1;

% polyn = poly(sys1.reqeigvalue);
% sys1.req4thorderTF = 



sys1.K = place(sys1.A,sys1.B,sys1.reqeigvalue);
Extras.A1_cl = sys1.A - sys1.B*sys1.K; % closed loop matrix for controller
sys1.A_cl = Extras.A1_cl;
sys1.B_cl = sys1.B;
sys1.C_cl = sys1.C;
sys1.D_cl = sys1.D;
sys1.sys_cl = ss(sys1.A_cl,sys1.B_cl,sys1.C_cl,sys1.D_cl);

% simulate with the same Initial COndition
[sys1.y_cl,sys1.t_cl,sys1.x_cl] = lsim(sys1.sys_cl,sys1.U,sys1.tspan,sys1.IC);

% case b
% characteristic polynomial with fourth order ITAE
Extras.pol = [1 2.1*Extras.omega_n 3.4*Extras.omega_n^2 2.7*Extras.omega_n^3   Extras.omega_n^4];
sys2.reqeigvalue = roots(Extras.pol);
sys2.K = place(sys2.A,sys2.B,sys2.reqeigvalue);

Extras.A2_cl = sys2.A - sys2.B*sys2.K;
sys2.A_cl = Extras.A2_cl;
sys2.B_cl = sys2.B;
sys2.C_cl = sys2.C;
sys2.D_cl = sys2.D;
sys2.sys_cl = ss(sys2.A_cl,sys2.B_cl,sys2.C_cl,sys2.D_cl);

% simulate with the same Initial COndition
[sys2.y_cl,sys2.t_cl,sys2.x_cl] = lsim(sys2.sys_cl,sys2.U,sys2.tspan,sys2.IC);

%% %%%%%%%%%%%%%%%%%%%%%%CHAPTER 8 - OBSERVERS AND OBSERVER BASED COMPENSATOR %%%%%%%%%%%%%%%%%%%%%%
%% 
% Get the observer eigen values 
% case a
sys1.reqObsEigenValue = [10*sys1.reqeigvalue];
sys1.L = place(sys1.A',sys1.C',sys1.reqObsEigenValue)';

Extras.A1_hat = sys1.A - sys1.L*sys1.C;
sys1.A_ObsBasedcl = [Extras.A1_cl sys1.B*sys1.K; zeros(size(sys1.A)) Extras.A1_hat];
sys1.B_ObsBasedcl = [sys1.B;zeros(size(sys1.B))];
sys1.C_ObsBasedcl = [sys1.C zeros(size(sys1.C))];
sys1.D_ObsBasedcl = sys1.D;
sys1.sys_ObsBasedcl = ss(sys1.A_ObsBasedcl,sys1.B_ObsBasedcl,sys1.C_ObsBasedcl,sys1.D_ObsBasedcl);

sys1.ObsBasedIC = [0;0;0;0;0.5;0;1;0];
[sys1.y_ObsBasedcl,sys1.t_ObsBasedcl,sys1.x_ObsBasedcl] = lsim(sys1.sys_ObsBasedcl,...
    sys1.U,sys1.tspan,sys1.ObsBasedIC);


% case b
sys2.reqObsEigenValue = [10*sys2.reqeigvalue];
sys2.L = place(sys2.A',sys2.C',sys2.reqObsEigenValue)';

Extras.A2_hat = sys2.A - sys2.L*sys2.C;
sys2.A_ObsBasedcl = [Extras.A2_cl sys2.B*sys2.K; zeros(size(sys2.A)) Extras.A2_hat];
sys2.B_ObsBasedcl = [sys2.B;zeros(size(sys2.B))];
sys2.C_ObsBasedcl = [sys2.C zeros(size(sys2.C))];
sys2.D_ObsBasedcl = sys2.D;
sys2.sys_ObsBasedcl = ss(sys2.A_ObsBasedcl,sys2.B_ObsBasedcl,sys2.C_ObsBasedcl,sys2.D_ObsBasedcl);

sys2.ObsBasedIC = [0;0;0;0;0.5;0;1;0];
[sys2.y_ObsBasedcl,sys2.t_ObsBasedcl,sys2.x_ObsBasedcl] = lsim(sys2.sys_ObsBasedcl,...
    sys2.U,sys1.tspan,sys2.ObsBasedIC);


%% Simulation for Case a
fig = fig +1;
figure(fig)
subplot(2,1,1)
plot(sys1.t,sys1.y(:,1))
hold on
plot(sys1.t_cl,sys1.y_cl(:,1))
plot(sys1.t_ObsBasedcl,sys1.y_ObsBasedcl(:,1))
ylabel('\it{y}_1')
grid on
subplot(2,1,2)
plot(sys1.t,sys1.y(:,2))
hold on
plot(sys1.t_cl,sys1.y_cl(:,2))
plot(sys1.t_ObsBasedcl,sys1.y_ObsBasedcl(:,2))
xlabel('\it{time}')
ylabel('\it{y}_2')
grid on
sgtitle('step Response case a')

hold off

%% Simulation for Case b

fig = fig +1;
figure(fig)
subplot(4,1,1)
plot(sys2.t,sys2.x(:,1))
hold on
plot(sys2.t_cl,sys2.x_cl(:,1))
plot(sys2.t_ObsBasedcl,sys2.x_ObsBasedcl(:,1))
ylabel('\it{x}_1')
grid on
subplot(4,1,2)
plot(sys2.t,sys2.x(:,2))
hold on
plot(sys2.t_cl,sys2.x_cl(:,2))
plot(sys2.t_ObsBasedcl,sys2.x_ObsBasedcl(:,2))
ylabel('\it{x}_2')
grid on
subplot(4,1,3)
plot(sys2.t,sys2.x(:,3))
hold on
plot(sys2.t_cl,sys2.x_cl(:,1))
plot(sys2.t_ObsBasedcl,sys2.x_ObsBasedcl(:,1))
grid on
ylabel('\it{x}_3')
subplot(4,1,4)
plot(sys2.t,sys2.x(:,4))
hold on
plot(sys2.t_cl,sys2.x_cl(:,4))
plot(sys2.t_ObsBasedcl,sys2.x_ObsBasedcl(:,4))
grid on
ylabel('\it{x}_4')
xlabel('\it{time}')
sgtitle('step Response case b')





































