close all
clear vars
clc

states_0=[2;4;7;10;2;0;20*pi/180;15*pi/180;30*pi/180;2*pi/180;1*pi/180;0];
t0=0;
tf=15;
h=0.01;
t=t0:h:tf;
m=15;
Forces=[10 ; 5 ; 9];
Moments=[10 ; 20 ; 5];
I=[1 -2 -1 ; -2 5 -3 ; -1 -3 0.1];

[Xe,Ye,Ze,u,v,w,phai,theta,psi,p,q,r]= RBDsolver(states_0,Forces,Moments,m,I,t0,tf,h);

position_6DOF=out.position_vec_e.data;
orientation_6DOF=out.orientation_vec.data;
velocity_6DOF=out.velocity_vec.data;
angular_rate_6DOF=out.angular_rate_vec.data;

figure
subplot(3,1,1)
plot(t,Xe,'LineWidth',1.5)
hold on
plot(t,position_6DOF(:,1),'LineWidth',1.3,'color','k','linestyle','--')
title('Position Vector using RBDsolver & 6DOF (Euler Angles) \bf')
xlabel('t (sec) \bf')
ylabel('x (m) \bf')
legend('RBDsolver','6DOF (Euler Angles)','Location','northwest')
subplot(3,1,2)
plot(t,Ye,'LineWidth',1.5)
hold on
plot(t,position_6DOF(:,2),'LineWidth',1.3,'color','k','linestyle','--')
xlabel('t (sec) \bf')
ylabel('y (m) \bf')
legend('RBDsolver','6DOF (Euler Angles)','Location','northwest')
subplot(3,1,3)
plot(t,Ze,'LineWidth',1.5)
hold on
plot(t,position_6DOF(:,3),'LineWidth',1.3,'color','k','linestyle','--')
xlabel('t (sec) \bf')
ylabel('z (m) \bf')
legend('RBDsolver','6DOF (Euler Angles)','Location','northwest')

figure
subplot(3,1,1)
plot(t,u,'LineWidth',1.5)
hold on
plot(t,velocity_6DOF(:,1),'LineWidth',1.3,'color','k','linestyle','--')
title('Velocity Vector using RBDsolver & 6DOF (Euler Angles) \bf')
xlabel('t (sec) \bf')
ylabel('u (m/s) \bf')
legend('RBDsolver','6DOF (Euler Angles)','Location','northwest')
subplot(3,1,2)
plot(t,v,'LineWidth',1.5)
hold on
plot(t,velocity_6DOF(:,2),'LineWidth',1.3,'color','k','linestyle','--')
xlabel('t (sec) \bf')
ylabel('v (m/s) \bf')
legend('RBDsolver','6DOF (Euler Angles)','Location','northwest')
subplot(3,1,3)
plot(t,w,'LineWidth',1.5)
hold on
plot(t,velocity_6DOF(:,3),'LineWidth',1.3,'color','k','linestyle','--')
xlabel('t (sec) \bf')
ylabel('w (m/s) \bf')
legend('RBDsolver','6DOF (Euler Angles)','Location','northwest')

figure
subplot(3,1,1)
plot(t,phai,'LineWidth',1.5)
hold on
plot(t,orientation_6DOF(:,1),'LineWidth',1.3,'color','k','linestyle','--')
title('Orientation Vector using RBDsolver & 6DOF (Euler Angles) \bf')
xlabel('t (sec) \bf')
ylabel('\phi (rad) \bf')
legend('RBDsolver','6DOF (Euler Angles)','Location','northwest')
subplot(3,1,2)
plot(t,theta,'LineWidth',1.5)
hold on
plot(t,orientation_6DOF(:,2),'LineWidth',1.3,'color','k','linestyle','--')
xlabel('t (sec) \bf')
ylabel('\theta (rad) \bf')
legend('RBDsolver','6DOF (Euler Angles)','Location','northwest')
subplot(3,1,3)
plot(t,psi,'LineWidth',1.5)
hold on
plot(t,orientation_6DOF(:,3),'LineWidth',1.3,'color','k','linestyle','--')
xlabel('t (sec) \bf')
ylabel('\psi (rad) \bf')
legend('RBDsolver','6DOF (Euler Angles)','Location','northwest')

figure
subplot(3,1,1)
plot(t,p,'LineWidth',1.5)
hold on
plot(t,angular_rate_6DOF(:,1),'LineWidth',1.3,'color','k','linestyle','--')
title('Angular Rates Vector using RBDsolver & 6DOF (Euler Angles) \bf')
xlabel('t (sec) \bf')
ylabel('p (rad/s) \bf')
legend('RBDsolver','6DOF (Euler Angles)','Location','southwest')
subplot(3,1,2)
plot(t,q,'LineWidth',1.5)
hold on
plot(t,angular_rate_6DOF(:,2),'LineWidth',1.3,'color','k','linestyle','--')
xlabel('t (sec) \bf')
ylabel('q (rad/sec) \bf')
legend('RBDsolver','6DOF (Euler Angles)','Location','southwest')
subplot(3,1,3)
plot(t,r,'LineWidth',1.5)
hold on
plot(t,angular_rate_6DOF(:,3),'LineWidth',1.3,'color','k','linestyle','--')
xlabel('t (sec) \bf')
ylabel('r (rad/sec) \bf')
legend('RBDsolver','6DOF (Euler Angles)','Location','southwest')