% ======================== Practice Exam =============================
clear all; clc;close all;
s=tf('s');
H3=s/(s^2+s+16);
pzmap(H3)
y3=step(H3,0:0.01:250);
plot(y3)
%% open loop response
clear all;close all;clc;
s=tf('s');
H=(0.5*(2*s+1))/(s^2*(s^2+0.4*s+4));
[Gm,Pm,Wg,Wp] = margin(H);
Gmdb=20*log10(Gm);
K=10^((Gmdb-5)/20)
[MAG,PHASE]=bode(feedback(H*K,1));
max(20*log10(MAG))
%% State space
clear all;clc;close all;
M1=11;
M2=1000;
K1=46000;
K2=8000;
B=2600;
a=[0,   1,   0,  0;...
  -(K1+K2)/M1, -B/M1, K2/M1, B/M1;...
  0, 0, 0, 1;...
  K2/M2, B/M2, -K2/M2, -B/M2];
b=[0; K1; 0; 0];
c= [0, 0, 1, 0;...
    K2/M2, B/M2, -K2/M2, -B/M2];
d=[0;0];
SYS=ss(a,b,c,d)
%% Root locus
clear all;clc;close all;
s=tf('s');
K=1;
Kr=0.8;
inside=feedback(K*20/((s+1)*(s+4)),Kr);
zpk(inside)
inside2=inside*1/s;
total=feedback(inside2,1);
total.num{1}
total.den{1}
%

% Settling time
t=0:0.001:50;
y=step(total,t);
lsiminfo(y,t,'SettlingTimeThreshold',0.05)