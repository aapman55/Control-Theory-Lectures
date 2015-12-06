% Practice try 2
%% recognise
clear all;clc;close all
s=tf('s');
H3=s/(s^2+s+16);
step(H3)
%% open loop frequency response
clear all;clc;close all;
s=tf('s');
H=(0.5*(2*s+1))/(s^2*(s^2+0.4*s+4));
% open loop
margin(H)
% change gain margin
sisotool(H)
% Last one
[Gm,Pm] =bode(feedback(0.855*H,1))
%% Mass damper
close all;clc;clear all;
% constants
M2=1000;
M1=10;
K1=46000;
K2=8000;
B=2400;
a=[0,    1,   0,   0;...
   -K2/M2,-B/M2,K2/M2,B/M2;...
   0,0,0,1;...
   K2/M1,B/M1,(-K1-K2)/M1,-B/M1];
b=[0;0;0;K1/M1];
c=[1,0,0,0;
     -K2/M2,-B/M2,K2/M2,B/M2];
d=[0;0];
sys=ss(a,b,c,d)
%% Root locus
clear all;clc;close all;
s=tf('s');
K=1;
Kr=0.9;
H=20/((s+1)*(s+4));
Inner=feedback(K*H,Kr);
Inner_outer=feedback(Inner*1/s,1);
Error=feedback(1,Inner*1/s);
Error.num{1}
Error.den{1}
Inner_outer.num{1}
Inner_outer.den{1}
%% Settling
t=0:0.01:50;
y=step(Inner_outer,t);
lsiminfo(y,t,'SettlingTimeTreshold',0.05)
% Tuning
rltool(K*H/s*(s*Kr+1))