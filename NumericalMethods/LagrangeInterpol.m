%{
...
How to do a lagrange Interpolation

It is used to find the polynomial approximation of the data points .
If we have 'n' data points then we can approximate by 'n-1' th degree
polynomial

Inputs:x and y data points
Outputs: Interpolated y values and xdata points 

Supply only x and y data points the function Interpolates for
xStart:0.01:xEnd

Created on 07/02/2020 18:35 By Karthi 
...
%}
function [xx,sum,prod] = LagrangeInterpol(x,y)
xStart = x(1);
xEnd = x(end);
xx = xStart:0.01:xEnd;
k = length(xx);
sum = 0;
for j = 1:length(y)
    pr = 1;
    for i = 1:length(x)
        if i~=j
            pr = pr.*(xx(1:k)-x(i))./(x(j)-x(i));
        end
    end
    prod = y(j)*pr;
    sum = sum + prod;
end