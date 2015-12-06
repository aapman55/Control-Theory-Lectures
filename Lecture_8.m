% Lecture 8
%hi
% Hallo
%% --------------------------------------
% Dominant poles and step response
% ---------------------------------------
clear all;clc;close all;
s=tf('s');                              % Define la place variable
G=(6205)/(s*(s^2+13*s+1281));           % set transfer function
zpk(G)                                  % display in nice format
H_eq=feedback(G,1);                     % Closed loop transfer function
zpk(H_eq)                               % display in nice format
tau=1/5;                                % Set Tau
omega=sqrt(1241);                       % Set omega
zeta=8/(2*omega);                       % set Zeta
H1=(1/tau)/(1/tau+s);                   % Set H1
H2=omega^2/(omega^2+2*zeta*omega*s+s^2);% Set H2
% Check H1H2
H1*H2
% Determine step output
t=0:0.01:2.5;
y=step(H_eq,t);
y1=step(H1,t);
y2=step(H2,t);
plot(t,y,t,y1,t,y2,'Linewidth',2)
legend('Total','5/(s+5)','1241/(s^2+8s+1241)')
% Integral
D1=trapz(t,abs(y-y1));
D2=trapz(t,abs(y-y2));
%% -----------------------------------------------------
% Simple operations with state-space systems
% ------------------------------------------------------
% In series
s1 = rss(3, 1, 2)
s2 = rss(2, 3, 1)
stotal = s2 * s1        % Why s2*s1 and not reverse?
% Parallel
s1 = rss(3, 1, 2)
s2 = rss(2, 1, 2)
stotal = s1 + s2
% feedback loops
a = [  0       1      0   ; ...
      -0.0071 -0.111  0.12; ...
       0       0.07  -0.3];
b = [  0 ;    -0.095; 0.072];
c = [ 1    0    0; ...  % theta
      0    1    0; ...  % q
      0    0    1; ...  % alpha
      1    0   -1]  ;    % gamma = theta - alpha
d = zeros(4, 1);
sys2 = ss(a, b, c, d)
k = [ 0  -0.67 0  0];
sclosed = feedback(sys2, k)
% check eigenvalues
eig(sys2.A)
eig(sclosed.A)
%% -------------------------------------
% Yaw response of an aircraft
% --------------------------------------
clear all,clc,close all,format;
A=[-0.2,0.06,0,-1;...
    0,0,1,0;...
    -17,0,-3.8,1;...
    9.4,0,-0.4,-0.6];
B=[-0.01,0.06;...
    0,0;...
    -32,5.4;...
    2.6,-7];
C=[0,0,0,1];
D=[0];
sys=ss(A,B(:,2),C,D); % need only rudder input and output is r
t=0:0.001:20;
step(sys,t);
%% -------------------------------------
% Yaw response of an aircraft 2
% --------------------------------------
clear all,clc,close all,format;
Kd=-0.5;
A=[-0.2,0.06,0,-1;...
    0,0,1,0;...
    -17,0,-3.8,1;...
    9.4,0,-0.4,-0.6];
B=[-0.01,0.06;...
    0,0;...
    -32,5.4;...
    2.6,-7];
C=diag(ones(4,1));
D=zeros(4,2);
SYS=ss(A,B,C,D)
K=[0,0,0,0;...
    0,0,0,Kd]
SYSclosed=feedback(SYS,K)
%% ------------------------------------
% Control with state space systems 2
% -------------------------------------
clear all,clc,close all,format;
Kd=-0.5;
A=[-0.2,0.06,0,-1;...
    0,0,1,0;...
    -17,0,-3.8,1;...
    9.4,0,-0.4,-0.6];
B=[-0.01,0.06;...
    0,0;...
    -32,5.4;...
    2.6,-7];
C=[0,0,0,1];
D=[0,0];
SYS=ss(A,B,C,D);
K1=[0;Kd];
SYSclosed=feedback(SYS,K1);
K2=[0; Kd * ss(-0.5, 0.5, -1, 1)];
SYSclosed2=feedback(SYS,K2);
% actual question
t=0.1:0.1:20;
input=[[ones(1,10),zeros(1,190)]',[zeros(200,1)]];
lsim(SYS,SYSclosed,SYSclosed2,input,t)
[Y1,T1,X1]=lsim(SYS,input,t);
[Y2,T2,X2]=lsim(SYSclosed,input,t);
[Y3,T3,X3]=lsim(SYSclosed2,input,t);
% zeta SYS
-eig(SYS.A);real(ans(2))/abs(ans(2))        % Better use command damp()
% zeta SYSclosed
-eig(SYSclosed.A);real(ans(2))/abs(ans(2))  % Better use command damp()
%% -------------------------------------
% wash out filter test
% --------------------------------------
t = 0:0.1:10;
wash = ss(-0.5, 0.5, -1, 1)
y = step(wash, t);
plot(t, y)
%% -------------------------------------
% constructing system models
% --------------------------------------
clear all,clc,close all;
s = tf('s');
h1 = 1/s^2;
hs = [h1; ...         % attitude out
      h1*s];           % differentiated attitude = velocity out
ssat=ss(hs);
% enter the transfer function
h2 = 40/(s^2 + 12*s + 40);
hm = [ h2; ...      % attitude (measured)
       h2*s ];       % rotational velocity (measured)
% convert to state-space
ssen = ss(hm);
% controller
Ks = 0.5;
% merge
sys = append(ssat, ssen, Ks);
% connect
Q = [ 1  5  0; ...    % connecting the input of the satellite
      3 -3 -4; ...    % connecting the sensor to the controller (feedback)
      2  1  0];       % input of sensor gets satellite attitude
                      % connection matrix is now complete
inputs = [ 3 ];
outputs = [5 1 2];
sysc = connect(sys, Q, inputs, outputs)
% check
hc = feedback(Ks * h1, h2 * (1 + s))
pole(hc)
eig(sysc.A)
%% ------------------------------------
% Roll control of an aircraft
% -------------------------------------
clear all,clc,close all;
Ka=4;
tau_a=1;
Kap=0.6;
s = tf('s');
autopilot=Kap;
actuator=9/(s^2+5*s+9);
roll_dyna=Ka/((tau_a*s+1)*s);
% normal way
SYS=feedback(ss(autopilot*actuator*roll_dyna),1);
% assembly way
sautopilot=ss(autopilot);
sactuator=ss(actuator);
sroll_dyna=ss(roll_dyna);
sys=append(sautopilot,sactuator,sroll_dyna,1);
input=[1];
output=[1,2,3];
Q=[4,3;1,-4;2,1;3,2];
sysc=connect(sys,Q,input,output);
pole(SYS)
eig(sysc)
%% ---------------------------------------
% Control of the Moller skycar
% ----------------------------------------
clear all,clc,close all
K=0.6;
s=tf('s');
H1=(K*(4*s^2+2*s+1))/(s*(1+0.1*s));
H2=1/(s^2*(s^2+s+4));
sH1=ss(H1);
sH2=ss(H2);
sys=append(sH1,sH2);
Q=[1,-2;2,1];
input=[1];
output=[2,1];
sysc=connect(sys,Q,input,output);
% Normal way
SYS=feedback(ss(H1*H2),1);
eig(sysc)
pole(SYS)
sysc.a
sysc.b
sysc.c
sysc.d