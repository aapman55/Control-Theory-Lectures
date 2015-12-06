%% second order system
m = 2
k = 20
b = 5
s = tf('s', 0)
h = 1/m/(s^2 + b/m*s + k/m)
t = 0:0.001:10;
% % natural frequency and damping coefficient
% w_n = sqrt(k/m)
% z = b/(2*m*w_n)
y=step(h,t);
plot(t,y)
% settling time
setl=y>1.05*y(end) | y<0.95*y(end)
%% 
K=2.5;
s=tf('s');
auto=K/(s+10);
ac=9/(s*(s+1.4));
H=feedback(auto*ac,1);
t=0:0.01:30;
y=step(H,t);
idx = y > y(end)*1.01 | y < y(end)*0.99;
tx=t(idx);tx(end)+0.01
