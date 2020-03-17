%{
...

Created on 10/3/2020 21:00
Problems from "Adaptive Control" by KJ Astrom 

Q1 :Investigate the open loop and Closed loop response of the followig
system for different values of 'a'.Note Similar closed loop behaviour, despite 
different open loop behaviour.

                 1                 1
  G_o(s) =  -------------  = -----------------
            (s + 1)(s + a)   s^2 + (a + 1)s + a

A closed loop system for a feedback gain 'K' is given by

G_cl = 1/(1+GK), then for K = 1


                       1                          
            -----------------------      
                 (s + 1)(s + a)                      1
G_cl(s) = ----------------------------- = -------------------------
                        1                 s^2 + (a + 1)s + a + 1
              1 + -------------
                  (s + 1)(s + a)

Syntax
-------
sys = tf([num],[den])
step(sys) or stepplot(sys)  - Step response
bode(sys)                   - Frequency Response

to create a feedback either "feedback(G,K)" from matlab can be used or the
above can be used in general syntax
...
%}


clearvars; clc; close all
%% Open Loop Step ,Impulse and Frequency response
% ------------------------------------------

% Step Response
% -------------
figure()
for a = -0.01:0.01:0.01
sys = tf(1,[1 (a+1) a]);
step(sys)
xlim([0 300])
ylim([0 300])
hold on 
grid on
end
title('Open Loop Step Response')
Legend = cell(3,1);
Legend{1} = 'a = -0.01';
Legend{2} = 'a = 0';
Legend{3} = 'a = 0.01';

legend(Legend,'Location','southeast')
set(gcf,'PaperPosition',[0 0 5 5]);
set(gcf,'PaperSize',[5 5])
saveas(gcf, 'Q1_OpenLoop_Step', 'pdf');


% Impulse Response
% -------------
figure()
for a = -0.01:0.01:0.01
    
sys = tf(1,[1 (a+1) a]);
impulse(sys)
hold on 
grid on
end
title('Open Loop Impulse Response')
Legend = cell(3,1);
Legend{1} = 'a = -0.01';
Legend{2} = 'a = 0';
Legend{3} = 'a = 0.01';

legend(Legend,'Location','southeast')
set(gcf,'PaperPosition',[0 0 5 5]);
set(gcf,'PaperSize',[5 5])
saveas(gcf, 'Q1_OpenLoop_Impulse', 'pdf');

% Frequency Response
% -------------
figure()
for a = -0.01:0.01:0.01
sys = tf(1,[1 (a+1) a]);
bode(sys)
grid on
hold on 
end
title('Open Loop Frequency Response')

Legend = cell(3,1);
Legend{1} = 'a = -0.01';
Legend{2} = 'a = 0';
Legend{3} = 'a = 0.01';

legend(Legend,'Location','southeast')
set(gcf,'PaperPosition',[0 0 5 5]);
set(gcf,'PaperSize',[5 5])
saveas(gcf, 'Q1_OpenLoop_Bode', 'pdf');


%% Closed Loop Step Impulseand Frequency response
% ------------------------------------------
K = 1 ;                      % unity feedback

% Step Response
% -------------
figure()
for a = -0.01:0.01:0.01
sys = tf(1,[1 (a+1) a]);
feed_sys = feedback(sys,K);    % (or)  feed_sys = tf(1,[1 (a+1) (a+1)]);
stepplot(feed_sys)

grid on 
hold on
end
title('Closed Loop Step Response')


Legend = cell(3,1);
Legend{1} = 'a = -0.01';
Legend{2} = 'a = 0';
Legend{3} = 'a = 0.01';

legend(Legend,'Location','southeast')
set(gcf,'PaperPosition',[0 0 5 5]);
set(gcf,'PaperSize',[5 5])
saveas(gcf, 'Q1_FeedBack_Step', 'pdf');

% Impulse Response
% -------------
figure()
for a = -0.01:0.01:0.01
sys = tf(1,[1 (a+1) a]);
feed_sys = feedback(sys,K);    % (or)  feed_sys = tf(1,[1 (a+1) (a+1)]);
impulse(feed_sys)

hold on 
grid on
end
Legend = cell(3,1);
Legend{1} = 'a = -0.01';
Legend{2} = 'a = 0';
Legend{3} = 'a = 0.01';

legend(Legend,'Location','northeast')
set(gcf,'PaperPosition',[0 0 5 5]);
set(gcf,'PaperSize',[5 5])
saveas(gcf, 'Q1_Feedback_Impulse', 'pdf');

% Frequency Response
% -------------
figure()
for a = -0.01:0.01:0.01
sys = tf(1,[1 (a+1) a]);
feed_sys = feedback(sys,K);    % (or)  feed_sys = tf(1,[1 (a+1) (a+1)]);
bode(feed_sys)

grid on 
hold on
end
title('Closed Loop frequency Response')


Legend = cell(3,1);
Legend{1} = 'a = -0.01';
Legend{2} = 'a = 0';
Legend{3} = 'a = 0.01';

legend(Legend,'Location','southeast')
set(gcf,'PaperPosition',[0 0 5 5]);
set(gcf,'PaperSize',[5 5])
saveas(gcf, 'Q1_FeedBack_Bode', 'pdf');





%{
...


Created on 10/3/2020 22:00
Problem from "Adaptive Control" by KJ Astrom 


Q2 :Investigate the open loop and Closed loop response of the followig
system for different values of 'T'.Note Similar open loop behaviour, 
despite different closed loop behaviour.

                 400(1 - Ts)                         400(1 - Ts)
G_o(s)  =  -----------------------    =  ------------------------------------
            (s + 1)(s + 20)(1 + Ts)       Ts^3 + s^2(1 + 21T) + s(21 + 20T) + 20


A closed loop system for a feedback gain 'K' is given by

G_cl = 1/(1+GK), then for K = 1


                  400(1 - Ts)                          
              -----------------------      
            (s + 1)(s + 20)(1 + Ts)                 400(1 - Ts)
G_cl(s) =  ----------------------------- = ------------------------------------
                     400(1 - Ts)         Ts^3 + s^2(1 + 21T) + s(21 - 380T) + 420
           1 + -----------------------
               (s + 1)(s + 20)(1 + Ts)

Syntax
-------
sys = tf([num],[den])
step(sys) or stepplot(sys)  - Step response
bode(sys)                   - Frequency Response
...
%}


clearvars; clc; close all
%% Define the Open Loop system for different 'T' values
% ---------------------------------------------------------
Constants.T = 0:0.015:0.03; % from the refernce book "Adaptive Control" by Karl J Astrom
OL.sys1 = tf([-Constants.T(1) 1],[Constants.T(1) (1 + 21*Constants.T(1)) (21 +Constants.T(1)*20) 20]);
OL.sys2 = tf([-Constants.T(2) 1],[Constants.T(2) (1 + 21*Constants.T(2)) (21 +Constants.T(2)*20) 20]);
OL.sys3 = tf([-Constants.T(3) 1],[Constants.T(3) (1 + 21*Constants.T(3)) (21 +Constants.T(3)*20) 20]);



%% Define the Closed Loop system for different 'T' values
% ---------------------------------------------------------
% feed back gain 
Constants.K = 1 ;% unity feedback

% We can use this  
% ----------------
% feedback_sys1 = feedback(OL_sys1,K);
% feedback_sys2 = feedback(OL_sys2,K);
% feedback_sys3 = feedback(OL_sys3,K);

% (or)

feedback.sys1 =  tf([-Constants.T(1) 1],[Constants.T(1) (1 + 21*Constants.T(1)) (21 - Constants.T(1)*380) 420]);
feedback.sys2 =  tf([-Constants.T(2) 1],[Constants.T(2) (1 + 21*Constants.T(2)) (21 - Constants.T(2)*380) 420]);
feedback.sys3 =  tf([-Constants.T(3) 1],[Constants.T(3) (1 + 21*Constants.T(3)) (21 - Constants.T(3)*380) 420]);




%% -------------------------------------------------Plot Section --------------------------------------------------------

% Plot the open loop step and frequency response
figure()
step(OL.sys1,OL.sys2,OL.sys3) % step
hold on 
grid on
legend('OL_sys1(T = 0)','OL_sys2(T = 0.015)','OL_sys3(T = 0.03)','Location','southeast')
set(gcf,'PaperPosition',[0 0 5 5]);
set(gcf,'PaperSize',[5 5])
saveas(gcf, 'Q2_OpenLoop_Step', 'pdf');

figure()
bode(OL.sys1,OL.sys2,OL.sys3) % Frequency
hold on 
grid on
legend('OL_sys1(T = 0)','OL_sys2(T = 0.015)','OL_sys3(T = 0.03)','Location','northeast')
set(gcf,'PaperPosition',[0 0 5 5]);
set(gcf,'PaperSize',[5 5])
saveas(gcf, 'Q2_OpenLoop_Bode', 'pdf');

% Plot the closed loop step and frequency response
figure()
step(feedback.sys1,feedback.sys2,feedback.sys3)  % step
xlim([0 1])
hold on 
grid on
legend('feedback_sys1(T = 0)','feedback_sys2(T = 0.015)','feedback_sys3(T = 0.03)','Location','southeast')
set(gcf,'PaperPosition',[0 0 5 5]);
set(gcf,'PaperSize',[5 5])
saveas(gcf, 'Q2_FeedBack_Step', 'pdf');

figure()
bode(feedback.sys1,feedback.sys2,feedback.sys3)  % Frequency
hold on 
grid on
legend('feedback_sys1(T = 0)','feedback_sys2(T = 0.015)','feedback_sys3(T = 0.03)','Location','northeast')
set(gcf,'PaperPosition',[0 0 5 5]);
set(gcf,'PaperSize',[5 5])
saveas(gcf, 'Q2_FeedBack_Bode', 'pdf');