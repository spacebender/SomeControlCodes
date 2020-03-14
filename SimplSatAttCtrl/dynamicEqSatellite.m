%{ 
...
This is an example from "Fundamentals of spacecraft attitude detreminationa
nd COntrol" 
Example 7.1 Pg 292 (Sec 7.1 Attitude Control Regulation Case)

created on  01/Feb/2020 17:15:59
...
%}
function [wdot] = dynamicEqSatellite(t,w,qc,J,Kp,Kd)
% define the variables
w1 = w(1);w2 = w(2); w3 = w(3); 
q1 = w(4); q2 = w(5); q3 = w(6); q4 = w(7);
qc1 = qc(1); qc2 = qc(2); qc3 = qc(3); qc4 = qc(4);

% define omegaCross and qc matrix
w_X = [0 -w3 w2;...
       w3 0 -w1;...
       -w2 w1 0];

qcmat = [qc4 qc3 -qc2 -qc1;...
         -qc3 qc4 qc1 -qc2;...
         qc2 -qc1 qc4 -qc3];
% define delq matrix    
delq = [qcmat*[q1;q2;q3;q4] ;...
        [q1 q2 q3 q4]*[qc1;qc2;qc3;qc4]];
del_q1 = delq(1); del_q2 = delq(2); del_q3 = delq(3); del_q4 = delq(4);

% define q matrix

mat_q = [q4 -q3 q2;...
        q3 q4 -q1;...
        -q2 q1 q4;...
        -q1 -q2 -q3];
%define L

L = -Kp*sign(del_q4)*[del_q1;del_q2;del_q3] - Kd*[w1;w2;w3];
  

% dynamic equations
    
w_dot = J\w_X*J*[w1;w2;w3] + J\L;
qdot = 0.5*mat_q*[w1; w2; w3];

wdot = [w_dot;qdot];