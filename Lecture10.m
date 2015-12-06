% Lecture 10
%% ----------------------------------
% Attitude control, take 1
% -----------------------------------
zeta_sp=2/sqrt(13);         % Define zeta_sp
omega_sp=sqrt(13);          % Define omega_sp
Kq=-24;                     % Define Kq
T_theta2=1.4;               % Define time constant
s=tf('s');                  % Define la place
%Transfer funtion
H=(Kq*(1+T_theta2*s))/...
    (s*(s^2+2*zeta_sp*omega_sp*s+omega_sp^2));
rltool(-H)
% Found Ktheta=-0.437
sys=feedback(-0.437*H,1)
damp(sys)
%% -----------------------------------
% Rate feedback
% ------------------------------------
zeta_sp=2/sqrt(13);         % Define zeta_sp
omega_sp=sqrt(13);          % Define omega_sp
Kq=-24;                     % Define Kq
T_theta2=1.4;               % Define time constant
s=tf('s');                  % Define la place
%Transfer funtion
H=(Kq*(1+T_theta2*s))/...
    ((s^2+2*zeta_sp*omega_sp*s+omega_sp^2));
rltool(-H)

Kr=-0.089445729;
sys=feedback(H,Kr);
damp(sys)
%% -----------------------------------
% getting the new system
% ------------------------------------
clear all;clc;close all;
VTAS=160;                   % m/s
zeta_sp=2/sqrt(13);         % Define zeta_sp
omega_sp=sqrt(13);          % Define omega_sp
Kq=-24;                     % Define Kq
T_theta2=1.4;               % Define time constant
s=tf('s');                  % Define la place

Hq=Kq*(1+T_theta2*s)/(s^2+2*zeta_sp*omega_sp*s+omega_sp^2);
Ht=1/s*Hq;
Hg=Hq/(s*(1+T_theta2*s));
Hh=(Hq*VTAS)/(s^2*(1+T_theta2*s));

sys=[Hq;Ht;Hg;Hh];
SYS=minreal(ss((sys)));

% closed loop
Kr=-0.089;
feedback_matrix=Kr*[1,0,0,0];
SYS2=feedback(SYS,feedback_matrix);
step(SYS(1,:),SYS2(1,:));

% -------------------------------------
% Attitude control
% -------------------------------------
Hth = tf([0 1 0 0]*SYS2);
SYS3=minreal(Hth,0.1);
rltool(-SYS3);

% Found K=-0.478
K_theta=-0.47; % Lamp uses 0.47
SYS4=feedback(K_theta*SYS3,1);
damp(SYS4) % Look for non real poles

% -----------------------------------
% Making the theta feedback controller
% -----------------------------------
SYS5=feedback(SYS2*K_theta,[0,1,0,0])
% step(SYS5(2,:))
t = 0:0.05:20;
y = lsim(SYS5, ones(size(t)), t);
plot(t, y(:,2))
% -------------------------------------
% attitude hold controller
% -------------------------------------
t=0:0.01:20;
y=step(SYS5(3,:),t);
t(sum(y<0.95))
% --------------------------------------
% Attitude Controller
% --------------------------------------
% assuming that SYS5 is the system with theta and q feedback, and its
% fourth output is the altitude:
H = tf([0 0 0 1]*SYS5)
% remember that .* is element-wise multiplication, set these small s^3
% etc. coefficients really to zero
% H.num{1} = [0 0 0 0 1] .* H.num{1}
H.den{1}=H.den{1}.*[1 1 1 1 0];
rltool(H)
Kh=1.2E5;
SYS6=feedback(Kh*SYS5,[0 0 0 1]);
step(SYS6(4,:),0:0.01:20)