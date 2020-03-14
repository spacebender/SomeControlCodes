%{
...
How to do a lagrange Interpolation?
It is used to find the polynomial approximation of the data points .
If we have 'n' data points then we can approximate by 'n-1' th degree
polynomial

Created on 07/02/2020 18:07 By Karthi 
...
%}
clearvars;clc

x = [-2 0 2];% 
y = [4 2 8];
% x = [0 0.5 1 1.5 2];
% y = [0 0.19 0.26 0.29 0.31];
% x = [1 3 4 6];
% y = [7 53 157 857];



[xVal,Yval,prod] = LagrangeInterpol(x,y);
plot(xVal,Yval,xVal,prod)
grid on 
xlabel('\it{x values}')
ylabel('\it{Interpolated y values}')

           
