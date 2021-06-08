% Firstly, change del_e or del_t in non_linear simulator and run the simulator in simulink
% Secondly, run the "ReadExcelModified.m" file
% Eventually, change the value of Delta_e or Delta_t in excel sheet
% Don't forget to make sure that the time vector in this file as the same
% as simulink
clc
close all

%% Inputs
delta_e=25*pi/180;
delta_t=10000;
t=0:0.01:100;

%% Non Linear States From Simulink
u_non_linear=out.u.data;
w_non_linear=out.w.data;
q_non_linear=out.q.data;
theta_non_linear=out.theta.data;

%% Longitudinal EOM
%% Full Linear Model
A_Long=...
    [Xu                     , Xw                      , -w_0                         , -g*cos(theta_0) ; 
    Zu/(1-Zw_dot)           , Zw/(1-Zw_dot)           , (Zq+u_0)/(1-Zw_dot)           , (-g*sin(theta_0))/(1-Zw_dot);
    Mu+Mw_dot*Zu/(1-Zw_dot) , Mw+Mw_dot*Zw/(1-Zw_dot) , Mq+Mw_dot*(Zq+u_0)/(1-Zw_dot) , -Mw_dot*g*sin(theta_0)/(1-Zw_dot);
    0                       , 0                       , 1                             , 0];
B_Long=...
    [Xdel_e                          , Xdel_t ; 
    Zdel_e/(1-Zw_dot)               , Zdel_t/(1-Zw_dot) ;
    Mdel_e+Mw_dot*Zdel_e/(1-Zw_dot) , Mdel_t+Mw_dot*Zdel_t/(1-Zw_dot) ;
    0                               , 0];
C_Long=eye(4);
D_Long=zeros(4,2);

sys_long=ss(A_Long,B_Long,C_Long,D_Long);
states_long=step(sys_long,t);
%Input Delta Elevator
u_full_linearized_e=delta_e*states_long(:,1,1)+u_0;
w_full_linearized_e=delta_e*states_long(:,2,1)+w_0;
q_full_linearized_e=delta_e*states_long(:,3,1)+q_0;
theta_full_linearized_e=delta_e*states_long(:,4,1)+theta_0;
%Input Delta Thrust
u_full_linearized_t=delta_t*states_long(:,1,2)+u_0;
w_full_linearized_t=delta_t*states_long(:,2,2)+w_0;
q_full_linearized_t=delta_t*states_long(:,3,2)+q_0;
theta_full_linearized_t=delta_t*states_long(:,4,2)+theta_0;

%% Short Period Approximation
A_SP=A_Long(2:3,2:3);  
B_SP=B_Long(2:3,:);  
C_SP=eye(2);
D_SP=zeros(2,2);
sys_sp=ss(A_SP,B_SP,C_SP,D_SP);
states_sp=step(sys_sp,t);
%Input Delta Elevator
w_sp_e=delta_e*states_sp(:,1,1)+w_0;
q_sp_e=delta_e*states_sp(:,2,1)+q_0;
%Input Delta Thrust
w_sp_t=delta_t*states_sp(:,1,2)+w_0;
q_sp_t=delta_t*states_sp(:,2,2)+q_0;

%% Long Period Approximation
A_LP=[Xu            ,    -g*cos(theta_0);
      -Zu/(Zq+u_0)  ,    g*sin(theta_0)/(Zq+u_0)];
  
B_LP=[Xdel_e            ,   Xdel_t;
      -Zdel_e/(Zq+u_0)  ,  -Zdel_t/(Zq+u_0)];
  
C_LP=eye(2);
D_LP=zeros(2,2);

sys_lp=ss(A_LP,B_LP,C_LP,D_LP);
states_lp=step(sys_lp,t);
%Input Delta Elevator
u_lp_e=delta_e*states_lp(:,1,1)+u_0;
theta_lp_e=delta_e*states_lp(:,2,1)+theta_0;
%Input Delta Thrust
u_lp_t=delta_t*states_lp(:,1,2)+u_0;
theta_lp_t=delta_t*states_lp(:,2,2)+theta_0;

%% Transfer Functions & Root Locus & Bode Plot
%Input Delta Elevator
tf_full=tf(sys_long);
tf_sp=tf(sys_sp);
tf_lp=tf(sys_lp);

figure
rlocus(tf_full(1,1))
title('Root Locus u/\delta_e (Full Linear Model)')
figure
rlocus(tf_lp(1,1))
title('Root Locus u/\delta_e (Long Period Approximation)')


figure
rlocus(tf_full(2,1))
title('Root Locus w/\delta_e (Full Linear Model)')
figure
rlocus(tf_sp(1,1))
title('Root Locus w/\delta_e (Short Period Approximation)')

figure
rlocus(tf_full(3,1))
title('Root Locus q/\delta_e (Full Linear Model)')
figure
rlocus(tf_sp(2,1))
title('Root Locus q/\delta_e (Short Period Approximation)')

figure
rlocus(tf_full(4,1))
title('Root Locus \theta/\delta_e (Full Linear Model)')
figure
rlocus(tf_lp(2,1))
title('Root Locus \theta/\delta_e (Long Period Approximation)')

figure
bode(tf_full(1,1))
title('Bode Plot u/\delta_e (Full Linear Model)')
figure
bode(tf_lp(1,1))
title('Bode Plot u/\delta_e (Long Period Approximation)')

figure
bode(tf_full(2,1))
title('Bode Plot w/\delta_e (Full Linear Model)')
figure
bode(tf_sp(1,1))
title('Bode Plot w/\delta_e (Short Period Approximation)')

figure
bode(tf_full(3,1))
title('Bode Plot q/\delta_e (Full Linear Model)')
figure
bode(tf_sp(2,1))
title('Bode Plot q/\delta_e (Short Period Approximation)')


figure
bode(tf_full(4,1))
title('Bode Plot \theta/\delta_e (Full Linear Model)')
figure
bode(tf_lp(2,1))
title('Bode Plot \theta/\delta_e (Long Period Approximation)')

%Input Delta Thrust
figure
rlocus(tf_full(1,2))
title('Root Locus u/\delta_t (Full Linear Model)')
figure
rlocus(tf_lp(1,2))
title('Root Locus u/\delta_t (Long Period Approximation)')

figure
rlocus(tf_full(2,2))
title('Root Locus w/\delta_t (Full Linear Model)')
figure
rlocus(tf_sp(1,2))
title('Root Locus w/\delta_t (Short Period Approximation)')

figure
rlocus(tf_full(3,2))
title('Root Locus q/\delta_t (Full Linear Model)')
figure
rlocus(tf_sp(2,2))
title('Root Locus q/\delta_t (Short Period Approximation)')

figure
rlocus(tf_full(4,2))
title('Root Locus \theta/\delta_t (Full Linear Model)')
figure
rlocus(tf_lp(2,2))
title('Root Locus \theta/\delta_t (Long Period Approximation)')

figure
bode(tf_full(1,2))
title('Bode Plot u/\delta_t (Full Linear Model)')
figure
bode(tf_lp(1,2))
title('Bode Plot u/\delta_t (Long Period Approximation)')

figure
bode(tf_full(2,2))
title('Bode Plot w/\delta_t (Full Linear Model)')
figure
bode(tf_sp(1,2))
title('Bode Plot w/\delta_t (Short Period Approximation)')

figure
bode(tf_full(3,2))
title('Bode Plot q/\delta_t (Full Linear Model)')
figure
bode(tf_sp(2,2))
title('Bode Plot q/\delta_t (Short Period Approximation)')


figure
bode(tf_full(4,2))
title('Bode Plot \theta/\delta_t (Full Linear Model)')
figure
bode(tf_lp(2,2))
title('Bode Plot \theta/\delta_t (Long Period Approximation)')

%% Plotting
figure
plot(t,u_full_linearized_t,'LineWidth',1.5)
hold on
plot(t,u_non_linear,'LineWidth',1.5,'color','k','linestyle','--')
hold on
plot(t,u_lp_t,'LineWidth',1.5,'color','r')
title(['u vs t for \delta_t= ',num2str(delta_t),' lb'])
legend('u (Full Linear Model)','u (Non Linear Model)','u (Long Period App)','Location','best')
xlabel('t (s)')
ylabel('u (ft/s)')

figure
plot(t,w_full_linearized_t,'LineWidth',1.5)
hold on
plot(t,w_non_linear,'LineWidth',1.5,'color','k','linestyle','--')
hold on
plot(t,w_sp_t,'LineWidth',1.5,'color','r')
title(['w vs t for \delta_t= ',num2str(delta_t),' lb'])
legend('w (Full Linear Model)','w (Non Linear Model)','w (Short Period App)','Location','best')
xlabel('t (s)')
ylabel('w (ft/s)')

figure
plot(t,q_full_linearized_t*180/pi,'LineWidth',1.5)
hold on
plot(t,q_non_linear,'LineWidth',1.5,'color','k','linestyle','--')
hold on
plot(t,q_sp_t*180/pi,'LineWidth',1.5,'color','r')
title(['q vs t for \delta_t= ',num2str(delta_t),' lb'])
legend('q (Full Linear Model)','q (Non Linear Model)','q (Short Period App)','Location','best')
xlabel('t (s)')
ylabel('q (deg/s)')

figure
plot(t,theta_full_linearized_t*180/pi,'LineWidth',1.5)
hold on
plot(t,theta_non_linear,'LineWidth',1.5,'color','k','linestyle','--')
hold on
plot(t,theta_lp_t*180/pi,'LineWidth',1.5,'color','r')
title(['\theta vs t for \delta_t= ',num2str(delta_t),' lb'])
legend('\theta (Full Linear Model','\theta (Non Linear Model)','\theta (Long Period App)','Location','best')
xlabel('t (s)')
ylabel('\theta (deg)')