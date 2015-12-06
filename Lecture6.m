% Lecture 6
clear all, clc, close all
%% State Space systems in Matlab
% -------------------
% setup matrices
% -------------------
a = [  0       1      0   ; ...
      -0.0071 -0.111  0.12; ...
       0       0.07  -0.3];
b = [  0 ;    -0.095; 0.072];
c = [  1       0     0    ];
d = [  0     ] ;

%-------------------------------------
% State vector x= [theta, q ,alfa]^T
% Input u=[delta_e]
% ------------------------------------

% -----------------------------------
% setup the sate space system
% -----------------------------------
sys=ss(a,b,c,d)

% ------------------------------------
% set new c matrix and d-matrix
% ------------------------------------
newc = [ 1    0    0; ...  % theta
         0    1    0; ...  % q
         0    0    1; ...  % alpha
         1    0   -1]      % gamma = theta - alpha
newd = zeros(4, 1);
sys2 = ss(a, b, newc, newd)

%% A simple state-space system
% Differential equation of a mass spring system
% mx"+cx'+kx=F(t)
clear all, close all, clc, format rat
% -----------------
% set constants
% -----------------
m=4.5;  % mass
b=9;    % damper constant
k=60;   % spring constant
A=[0 1;-k/m -b/m]
B=[0;1/m]
C=[1 0;0 1;-k/m -b/m]
D=[0;0;1/m]

%% Testing the system
% Differential equation of a mass spring system
% mx"+cx'+kx=F(t)
% Now F=0
clear all, close all, clc, format rat
% -----------------
% set constants
% -----------------
m=4.5;  % mass
b=9;    % damper constant
k=60;   % spring constant
A=[0 1;-k/m -b/m]
B=[0;1/m]
C=[1 0;0 1;-k/m -b/m]
D=[0;0;1/m]

SYS=ss(A,[],C,[]);
y0=[0 1];
t=0:0.001:10;
initial(SYS,y0)
%% Resonance
% Differential equation of a mass spring system
% mx"+cx'+kx=F(t)
% Now F=sin(omega*t)
clear all, close all, clc
% -----------------
% set constants
% -----------------
m=4.5;              % mass
b=9;                % damper constant
k=60;               % spring constant
t=0:0.001:20;       % time
omega1=3.5118846;   % frequency 1 in rad/s
omega2=3.3665016;   % frequency 2 in rad/s
A=[0 1;-k/m -b/m]
B=[0;1/m]
C=[1 0;0 1;-k/m -b/m]
D=[0;0;1/m]
SYS=ss(A,B,C,D)
u1=sin(omega1*t);
u2=sin(omega2*t);
y1=lsim(SYS,u1,t);
y2=lsim(SYS,u2,t);
plot(t,y1(:,1),t,(y2(:,1)))
legend(['\omega_1 = ',num2str(omega1)],['\omega_2 = ',num2str(omega2)])
%% Testing the system (change m,k or b)
% Differential equation of a mass spring system
% mx"+cx'+kx=F(t)
% Now F=0
clear all, close all, clc, format rat
% -----------------
% set constants
% -----------------
m=4.5;  % mass
b=9;    % damper constant
k=60;   % spring constant
A=[0 1;-k/m -b/m]
B=[0;1/m]
C=[1 0;0 1;-k/m -b/m]
D=[0;0;1/m]
t=0:0.01:20;
SYS=ss(A,[],C,[]);
% changed constants
m2=4.5;  % mass
b2=9;    % damper constant
k2=2*60;   % spring constant
A=[0 1;-k2/m2 -b2/m2]
B=[0;1/m2]
C=[1 0;0 1;-k2/m2 -b2/m2]
D=[0;0;1/m2]
t=0:0.01:20;
SYS2=ss(A,[],C,[]);
y0=[0 1];
t=0:0.001:10;
[y1 dummy1 dummy2]=initial(SYS,y0,t);
[y2 dummy1 dummy2]=initial(SYS2,y0,t);
plot(t,y1(:,1),t,y2(:,1))
%% -------------------------------------------------------------- 
% Converting between representations
% ---------------------------------------------------------------
% can you recognise this one? ...
a = [  0       1      0   ; ...
      -0.0071 -0.111  0.12; ...
       0       0.07  -0.3]
b = [  0 ;    -0.095; 0.072]
c = [  1       0     0    ]
d = [  0     ]
sys=ss(a, b, c, d)
% convert to transfer function
h = tf(sys)

% create a random system. Look up rss to see its definition
sys = rss(3, 2, 3)
% convert to tf
h = tf(sys)
% see what we got
class(h)
h11 = tf(h.num{1}, h.den{1})  % Remember the curly braces!
% select the first row of transfer functions (output 1)
[1 0] * h
h11 = [1 0] * h * [0; 1; 0]
% The nice thing about this method is that it also functions for a state-space system.
%Apply these multiplications and you get a single-input, single output state-space system,
%that is equivalent to h11. I suggest you try and check this:
h11b = tf([1 0] * sys * [0; 1; 0])
h11 - h11b
%% -----------------------------------------------------
% Converting a state space system to a transfer function
% ------------------------------------------------------
clear all;clc;close all;
A = [ -1.2   0    0; ...
       0.4   -1.2   -5.0; ...
       0      1      0]
B = [ 0.3;  0;  0]
C = [ 0     1   0; ...
      0.1   0   1]
D= zeros(2,1)
SYS=ss(A,B,C,D)
h=tf(SYS)
zpk(h)
roots(h.den{1})
%% ----------------------------
% Converting a transfer function to a state-space system
% -----------------------------
clear all; clc ; close all
s=tf('s')
b0 = 0.2;
b1 = 0.1;
b2 = 0.5;
a0 = 2.3;
a1 = 6.3;
a2 = 3.6;
a3 = 1.0;
numerator=b0+b1*s+b2*s^2;
denominator=a0+a1*s+a2*s^2+a3*s^3;
H=numerator/denominator;
h=[H;H*s];
out=ss(h)
%% SS from diagram
k1=3.8;
k2=1.2;
k3=2.5;
k4=0.6;
A=[0,-k2*k4,-k3*k4; 1, -1, 0;0 ,1, 0];
B=[k4, 1;0, 0;0, 0];
C=[k1, 0, 1];
D=[0, 0];
sys=ss(A,B,C,D)
initial(sys,[1 1 0])