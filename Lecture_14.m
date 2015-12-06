%% %%%%%%%%%%%%%%%% E Lecture 14 %%%%%%%%%%%%%%%%%%%
clear all; close all; clc
%% ================== Introduction ===================
close all;clc;
s=tf('s');
F=1/2*(s+1/2)/((s-3/4)*(s^2+s+1));
figure()
nyquist(F)
H=2*(s+1/2)/((s-3/4)*(s^2+s+1));
zpk(H)
figure()
nyquist(H+1)
H1=zpk(feedback(H,1))

%% ================  Tuning with Bode  ===================
close all;clear all; clc;
s=tf('s');
J=5000;
l1=1;
l2=1.5;
D=200;
T=10000;
K=[1, 0.1, 0.01];
for i=1:3
  H1(i)=K(i)*(1+3*s);
  H2(i)=4/(4+3*s+s^2);
  H3(i)=(l2*T/J)/(s^2-D*l1/J);
  sys(i)=H1(i)*H2(i)*H3(i);
  a=subplot(1,3,i)
  margin(sys(i))
end
Ho=sys(1);
%% ==================== Tuning with Nyquist =====================
pole(Ho)        % Check for poles, one unstable pole
nyquist(sys)
% Check closed loop with K=0.045
figure()
step(feedback(Ho*0.045,1))
theta=step(feedback(Ho*0.045,1));
theta(end)
figure()
bode(Ho*0.045)
%% =========== Rocket differential equation ================
Hsys=Ho;
freq1=0.005; %tau=200   Lag
freq2=0.05;     % tau =20   Lead
Glag=((s+freq2)/(s+freq1));
sisotool(Glag*Hsys)
