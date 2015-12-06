% -----------------------------------
% E lecture 12
% -----------------------------------
clear all; clc; close all;
%% ----------------------------------
% Creating an open-loop system model
% -----------------------------------
Jb = 400;
Jp = 1000;
k = 10;
b = 5;
s = tf('s');
hb1 = 1/(Jb*s);
hb2 = 1/s;
hp1 = 1/(Jp*s);
hp2 = 1/s;
sat0 = append(ss(hb1), ss(hb2), k, b, ss(hp1), ss(hp2));

% Connecting
inputs = [1];
outputs = [1 2 5 6];    %theta b dot, theta b, theta p dot, theta p
Q = [1, -3, -4;...
     2, 1, 0;...
     3, 2, -6;...
     4, 1, -5;...
     5, 3, 4;...
     6, 5, 0];
 sat1=connect(sat0,Q,inputs,outputs)
 % -----------------------------
 % What is the problem?
 % -----------------------------
 t=0:100;
 [Y]=step(sat1,t);
 plot(t,Y);legend('bdot','b','pdot','p')
 Y(end,:)
 max(abs(Y(:,2)-Y(:,4)))
 
 % --------------------------------
 % Initial tuning with bode
 % --------------------------------
 Hp = minreal(tf([0 0 0 1]*sat1))
 handlefig1=figure('Name','Bode 1')
 bode(Hp,{0.001 1})
 % Look at all the negative phase (lag)
 Kp=1;
 Kd_Kp=50;
%  Kd=Kd_Kp*Kp;
 GPD=Kp*(1+Kd_Kp*s);
 HpHc=(GPD*Hp);
 figure()
 [a,b]=bode(HpHc,{0.001 1});
 bode(HpHc,{0.001 1});
 % -------- tuning --------
  Kp=10^(-12.3/20);
 Kd_Kp=50;
%  Kd=Kd_Kp*Kp;
 GPD=Kp*(1+Kd_Kp*s);
 HpHc=(GPD*Hp);
 figure()
 [a,b]=bode(HpHc,{0.001 1});
 bode(HpHc,{0.001 1});
 % check for closed loop
 sat2=feedback(HpHc,1);
 figure()
 subplot(1,2,1)
 step(sat2)
 subplot(1,2,2)
 bode(sat2,{0.001 1})
 
 %% ------------------------------------
 % Looking at notch filters
 % -------------------------------------
 clear all; clc; close all;
 s=tf('s');
 zeta1=0.0;
 zeta2=0.7;
 omega_no=10*2*pi;
 Num=1+2*zeta1*s/omega_no+(s/omega_no)^2;
 Den=1+2*zeta2*s/omega_no+(s/omega_no)^2;
 H_no=Num/Den
 %%%%%%%%%%%%%%% PZmap %%%%%%%%%%%%%%%
%  pzmap(H_no)
 % Staircase input
 t=0:0.001:0.5;
 u= t-mod(t,0.1);
 plot(t,u)
 % Staircase
 figure()
 subplot(3,1,1)
 lsim(H_no,u,t)
 % Bode
 subplot(3,1,2)
 bode(H_no,10^-2:0.001:1000)
 % nyquist
 subplot(3,1,3)
 nyquist(H_no)
 %% Open-loop Bode of system with notch filter
 clear all; close all; clc
  s=tf('s');
  %%%%%%%%%%%%%%  NOtch %%%%%%%%%%%%%%%
 zeta1=0.05;
 zeta2=0.7;
 omega_no=0.19;
 Num=1+2*zeta1*s/omega_no+(s/omega_no)^2;
 Den=1+2*zeta2*s/omega_no+(s/omega_no)^2;
 H_no=Num/Den
 %%%%%%%%%%%%%%%%%%% Satellite %%%%%%%%%%%%%
 Jb = 400;
Jp = 1000;
k = 10;
b = 5;
s = tf('s');
hb1 = 1/(Jb*s);
hb2 = 1/s;
hp1 = 1/(Jp*s);
hp2 = 1/s;
sat0 = append(ss(hb1), ss(hb2), k, b, ss(hp1), ss(hp2));

% Connecting
inputs = [1];
outputs = [1 2 5 6];    %theta b dot, theta b, theta p dot, theta p
Q = [1, -3, -4;...
     2, 1, 0;...
     3, 2, -6;...
     4, 1, -5;...
     5, 3, 4;...
     6, 5, 0];
 sat1=connect(sat0,Q,inputs,outputs)
 %%%%%%%%%%%%%%%% PD %%%%%%%%%%%%%%%%%%%
Kp=1;
Kptune=1.63;
Kd_Kp=50;
GPD=Kp*(1+Kd_Kp*s);
GPD2=Kptune*(1+Kd_Kp*s);
%%%%%%%%%%%%%%%%%% Open loop %%%%%%%%%%%%%%%%%
Hp = minreal(tf([0 0 0 1]*sat1))
HpHcHn=(GPD*Hp*H_no);
HpHcHn2=(GPD2*Hp*H_no);
figure()
[a,b]=bode(HpHcHn);
subplot(1,2,1)
margin(HpHcHn)
subplot(1,2,2)
margin(HpHcHn2)

% Final check
sat3=feedback(HpHcHn2,1);
t=0:0.01:250;
ramp=t;
 figure()
 subplot(1,3,3)
 yr=lsim(sat3,ramp,t);
 lsim(sat3,ramp,t)
 subplot(1,3,2)
 y=step(sat3,t);
 step(sat3,t)
 subplot(1,3,1)
 [mag,phs]=bode(sat3);
 bode(sat3)
 bandwidth(sat3)
 % assume we have the transfer function for the satellite
Hp
% and the transfer function for the notch filter
H_no
% the feedback parameters are:
Kp = 1.63
tau = 50
s = tf('s')
% create the feedback
Hc = feedback(Kp*H_no*Hp,(1+tau*s))
t=0:0.01:250;
ramp=t;
 figure()
 subplot(1,3,3)
 yr=lsim(Hc,ramp,t);
 lsim(Hc,ramp,t)
 subplot(1,3,2)
 y=step(Hc,t);
 step(Hc,t)
 subplot(1,3,1)
 [mag,phs]=bode(Hc);
 bode(Hc)
figure()
pzmap(Hc)