% Worksheet 5
clear all,clc,close all;
% Set Variables
Ky=1;
Kphi=1;
V=10;
l=15;
% set Matrices
A=[0 1/V; -Kphi*Ky -Kphi];
B=[0;Kphi*Ky];
C=[-Kphi*Ky -Kphi;0 1;1 0];
D=[Kphi*Ky;0;0];
% Make system
sys=ss(A,B,C,D)
impulse(sys)