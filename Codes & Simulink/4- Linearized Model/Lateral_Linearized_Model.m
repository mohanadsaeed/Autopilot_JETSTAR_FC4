% Firstly, change del_e or del_t in non_linear simulator and run the simulator in simulink
% Secondly, run the "ReadExcelModified.m" file
% Eventually, change the value of Delta_e or Delta_t in excel sheet
% Don't forget to make sure that the time vector in this file as the same
% as simulink
clc
close all

%% Inputs
delta_a=25*pi/180;
delta_r=25*pi/180;
t=0:0.01:100;

%% Non Linear States From Simulink
v_non_linear=out.v.data;
p_non_linear=out.p.data;
r_non_linear=out.r.data;
phi_non_linear=out.phi.data;
psi_non_linear=out.psi.data;

%% Longitudinal EOM
%% Full Linear Model
Yp=0;
Yr=0;
A_Lat=...
    [Yv        ,   Yp+w_0    ,   Yr-u_0         ,   g*cos(theta_0)   ,   0;
     Lv_dash   ,   Lp_dash   ,   Lr_dash        ,   0                ,   0;
     Nv_dash   ,   Np_dash   ,   Nr_dash        ,   0                ,   0;
     0         ,   1         ,   tan(theta_0)   ,   0                ,   0;
     0         ,   0         ,   sec(theta_0)   ,   0                ,   0];
B_Lat=...
    [0             ,    Ydel_r;
     Ldel_a_dash   ,    Ldel_r_dash;
     Ndel_a_dash   ,    Ndel_r_dash;
     0             ,    0;
     0             ,    0];
C_Lat=eye(5);
D_Lat=zeros(5,2);

sys_lat=ss(A_Lat,B_Lat,C_Lat,D_Lat);
states_lat=step(sys_lat,t);
%Input Delta Aileron
v_full_linearized_a=delta_a*states_lat(:,1,1)+v_0;
p_full_linearized_a=delta_a*states_lat(:,2,1)+p_0;
r_full_linearized_a=delta_a*states_lat(:,3,1)+r_0;
phi_full_linearized_a=delta_a*states_lat(:,4,1)+phi_0;
psi_full_linearized_a=delta_a*states_lat(:,5,1)+psi_0;
%Input Delta Rudder
v_full_linearized_r=delta_r*states_lat(:,1,2)+v_0;
p_full_linearized_r=delta_r*states_lat(:,2,2)+p_0;
r_full_linearized_r=delta_r*states_lat(:,3,2)+r_0;
phi_full_linearized_r=delta_r*states_lat(:,4,2)+phi_0;
psi_full_linearized_r=delta_r*states_lat(:,5,2)+psi_0;

%% 3-DOF Spiral Approximation
A_spiral=A_Lat(2:4,2:4);  
B_spiral=B_Lat(2:4,2);  
C_spiral=eye(3);
D_spiral=zeros(3,1);
sys_spiral=ss(A_spiral,B_spiral,C_spiral,D_spiral);
states_spiral=step(sys_spiral,t);
%Input Delta Rudder
p_spiral_r=delta_r*states_spiral(:,1)+p_0;
r_spiral_r=delta_r*states_spiral(:,2)+r_0;
phi_spiral_r=delta_r*states_spiral(:,3)+phi_0;

%% 3-DOF Dutch-Roll Approximation
A_3dof=...
    [Yv        ,   Yp+w_0   ,   Yr-u_0  ;
     Lv_dash   ,   Lp_dash  ,   0       ;
     Nv_dash   ,   0        ,   Nr_dash];
B_3dof=...
    [0             ,   Ydel_r;
     Ldel_a_dash   ,   Ldel_r_dash;
     Ndel_a_dash   ,   Ndel_r_dash];
C_3dof=eye(3);
D_3dof=zeros(3,2);
sys_3dof=ss(A_3dof,B_3dof,C_3dof,D_3dof);
states_3dof=step(sys_3dof,t);
%Input Delta Aileron
v_3dof_a=delta_a*states_3dof(:,1)+v_0;
p_3dof_a=delta_a*states_3dof(:,2)+p_0;
r_3dof_a=delta_a*states_3dof(:,3)+r_0;
%Input Delta Rudder
v_3dof_r=delta_r*states_3dof(:,1)+v_0;
p_3dof_r=delta_r*states_3dof(:,2)+p_0;
r_3dof_r=delta_r*states_3dof(:,3)+r_0;

%% 2-DOF Dutch-Roll Approximation
A_2dof=...
    [Yv         ,   Yr-u_0;
     Nv_dash   ,   Nr_dash];
B_2dof=...
    [0             ,   Ydel_r;
     Ndel_a_dash   ,   Ndel_r_dash];
  
C_2dof=eye(2);
D_2dof=zeros(2,2);

sys_2dof=ss(A_2dof,B_2dof,C_2dof,D_2dof);
states_2dof=step(sys_2dof,t);
%Input Delta Aileron
v_2dof_a=delta_a*states_2dof(:,1,1)+v_0;
r_2dof_a=delta_a*states_2dof(:,2,1)+r_0;
%Input Delta Rudder
v_2dof_r=delta_r*states_2dof(:,1,1)+v_0;
r_2dof_r=delta_r*states_2dof(:,2,1)+r_0;

%% 1-DOF Roll Approximation
A_1dof=Lp_dash;
B_1dof=Ldel_a_dash;
C_1dof=1;
D_1dof=0;
sys_1dof=ss(A_1dof,B_1dof,C_1dof,D_1dof);
states_1dof=step(sys_1dof,t);
%Input Delta Aileron
p_1dof_a=delta_a*states_1dof(:,1)+p_0;

%% %% Transfer Functions & Root Locus & Bode Plot
tf_full=tf(sys_lat);
tf_spiral=tf(sys_spiral);
tf_3dof=tf(sys_3dof);
tf_2dof=tf(sys_2dof);
tf_1dof=tf(sys_1dof);

%Input Delta Aileron
figure
rlocus(tf_full(1,1));
title('Root Locus v/\delta_a (Full Linear Model)')
figure
rlocus(tf_3dof(1,1));
title('Root Locus v/\delta_a (3-DOF Dutch Roll Approximation)')
figure
rlocus(tf_2dof(1,1));
title('Root Locus v/\delta_a (2-DOF Dutch Roll Approximation)')

figure
rlocus(tf_full(2,1));
title('Root Locus p/\delta_a (Full Linear Model)')
figure
rlocus(tf_3dof(2,1));
title('Root Locus p/\delta_a (3-DOF Dutch Roll Approximation)')
figure 
rlocus(tf_1dof(1,1));
title('Root Locus p/\delta_a (1-DOF Roll Approximation)')

figure
rlocus(tf_full(3,1));
title('Root Locus r/\delta_a (Full Linear Model)')
figure
rlocus(tf_3dof(3,1));
title('Root Locus r/\delta_a (3-DOF Dutch Roll Approximation)')
figure
rlocus(tf_2dof(2,1));
title('Root Locus r/\delta_a (2-DOF Dutch Roll Approximation)')

figure
rlocus(tf_full(4,1));
title('Root Locus \phi/\delta_a (Full Linear Model)')

figure
rlocus(tf_full(5,1));
title('Root Locus \psi/\delta_a (Full Linear Model)')


figure
bode(tf_full(1,1));
title('Bode Plot v/\delta_a (Full Linear Model)')
figure
bode(tf_3dof(1,1));
title('Bode Plot v/\delta_a (3-DOF Dutch Roll Approximation)')
figure
bode(tf_2dof(1,1));
title('Bode Plot v/\delta_a (2-DOF Dutch Roll Approximation)')

figure
bode(tf_full(2,1));
title('Bode Plot p/\delta_a (Full Linear Model)')
figure
bode(tf_3dof(2,1));
title('Bode Plot p/\delta_a (3-DOF Dutch Roll Approximation)')
figure
bode(tf_1dof(1,1));
title('Bode Plot p/\delta_a (1-DOF Roll Approximation)')

figure
bode(tf_full(3,1));
title('Bode Plot r/\delta_a (Full Linear Model)')
figure
bode(tf_3dof(3,1));
title('Bode Plot r/\delta_a (3-DOF Dutch Roll Approximation)')
figure
bode(tf_2dof(2,1));
title('Bode Plot For Transfer Function r/\delta_a (2-DOF Dutch Roll Approximation)')

figure
bode(tf_full(4,1));
title('Bode Plot \phi/\delta_a (Full Linear Model)')

figure
bode(tf_full(5,1));
title('Bode Plot \psi/\delta_a (Full Linear Model)')

%Input Delta Rudder
figure
rlocus(tf_full(1,2));
title('Root Locus v/\delta_r (Full Linear Model)')
figure
rlocus(tf_3dof(1,2));
title('Root Locus v/\delta_r (3-DOF Dutch Roll Approximation)')
figure
rlocus(tf_2dof(1,2));
title('Root Locus v/\delta_r (2-DOF Dutch Roll Approximation)')

figure
rlocus(tf_full(2,2));
title('Root Locus p/\delta_r (Full Linear Model)')
figure
rlocus(tf_spiral(1,1));
title('Root Locus p/\delta_r (3-DOF Spiral Approximation)')
figure
rlocus(tf_3dof(2,2));
title('Root Locus p/\delta_r (3-DOF Dutch Roll Approximation)')

figure
rlocus(tf_full(3,2));
title('Root Locus r/\delta_r (Full Linear Model)')
figure
rlocus(tf_spiral(2,1));
title('Root Locus r/\delta_r (3-DOF Spiral Approximation)')
figure
rlocus(tf_3dof(3,2));
title('Root Locus r/\delta_r (3-DOF Dutch Roll Approximation)')
figure
rlocus(tf_2dof(2,2));
title('Root Locus r/\delta_r (2-DOF Dutch Roll Approximation)')

figure
rlocus(tf_full(4,2));
title('Root Locus \phi/\delta_r (Full Linear Model)')
figure
rlocus(tf_spiral(3,1));
title('Root Locus \phi/\delta_r (3-DOF Spiral Approximation)')

figure
rlocus(tf_full(5,2));
title('Root Locus \psi/\delta_r (Full Linear Model)')


figure
bode(tf_full(1,2));
title('Bode Plot v/\delta_r (Full Linear Model)')
figure
bode(tf_3dof(1,2));
title('Bode Plot v/\delta_r (3-DOF Dutch Roll Model)')
figure
bode(tf_2dof(1,2));
title('Bode Plot v/\delta_r (2-DOF Dutch Roll Model)')

figure
bode(tf_full(2,2));
title('Bode Plot p/\delta_r (Full Linear Model)')
figure
bode(tf_spiral(1,1));
title('Bode Plot p/\delta_r (3-DOF Spiral Model)')
figure
bode(tf_3dof(2,2));
title('Bode Plot p/\delta_r (3-DOF Dutch Roll Approximation)')

figure
bode(tf_full(3,2));
title('Bode Plot r/\delta_r (Full Linear Model)')
figure
bode(tf_spiral(2,1));
title('Bode Plot r/\delta_r (3-DOF Spiral Approximation)')
figure
bode(tf_3dof(3,2));
title('Bode Plot r/\delta_r (3-DOF Dutch Roll Approximation)')
figure
bode(tf_2dof(2,2));
title('Bode Plot r/\delta_r (2-DOF Dutch Roll Approximation)')

figure
bode(tf_full(4,2));
title('Bode Plot \phi/\delta_r (Full Linear Model)')
figure
bode(tf_spiral(3,1));
title('Bode Plot \phi/\delta_r (3-DOF Spiral Approximation)')

figure
bode(tf_full(5,2));
title('Bode Plot \psi/\delta_r (Full Linear Model)')

%% Plotting
figure
plot(t,v_full_linearized_r,'LineWidth',1.5)
hold on
plot(t,v_non_linear,'LineWidth',1.5,'color','k','linestyle','--')
hold on
plot(t,v_3dof_r,'LineWidth',1.5,'color','r')
hold on
plot(t,v_2dof_r,'LineWidth',1.5,'color','m')
title(['v vs t for \delta_r= ',num2str(delta_r*180/pi),'°'])
legend('v (Full Linear Model)','v (Non Linear Model)','v (3-DOF Dutch-Roll App)','v (2-DOF Dutch-Roll App)','Location','best')
xlabel('t (s)')
ylabel('v (ft/s)')

figure
plot(t,p_full_linearized_r*180/pi,'LineWidth',1.5)
hold on
plot(t,p_non_linear,'LineWidth',1.5,'color','k','linestyle','--')
hold on
plot(t,p_spiral_r,'LineWidth',1.5,'color','r')
hold on
plot(t,p_3dof_r*180/pi,'LineWidth',1.5,'color','m')
title(['p vs t for \delta_r= ',num2str(delta_r*180/pi),'°'])
legend('p (Full Linear Model)','p (Non Linear Model)','p (3-DOF Spiral App)','p (3-DOF Dutch-Roll App)','Location','best')
xlabel('t (s)')
ylabel('p (deg/s)')

figure
plot(t,r_full_linearized_r*180/pi,'LineWidth',1.5)
hold on
plot(t,r_non_linear,'LineWidth',1.5,'color','k','linestyle','--')
hold on
plot(t,r_spiral_r*180/pi,'LineWidth',1.5,'color','r')
hold on
plot(t,r_3dof_a*180/pi,'LineWidth',1.5,'color','g')
hold on
plot(t,r_2dof_a*180/pi,'LineWidth',1.5,'color','m')
title(['r vs t for \delta_r= ',num2str(delta_r*180/pi),'°'])
legend('r (Full Linear Model)','r (Non Linear Model)','r (3-DOF Spiral App)','r (3-DOF Dutch-Roll App)','r (2-DOF Dutch-Roll App)','Location','best')
xlabel('t (s)')
ylabel('r (deg/s)')

figure
plot(t,phi_full_linearized_r*180/pi,'LineWidth',1.5)
hold on
plot(t,phi_non_linear,'LineWidth',1.5,'color','k','linestyle','--')
hold on
plot(t,phi_spiral_r*180/pi,'LineWidth',1.5,'color','r')
title(['\phi vs t for \delta_r= ',num2str(delta_r*180/pi),'°'])
legend('\phi (Full Linear Model','\phi (Non Linear Model)','\phi (3-DOF Spiral App)','Location','best')
xlabel('t (s)')
ylabel('\phi (deg)')

figure
plot(t,psi_full_linearized_r*180/pi,'LineWidth',1.5)
hold on
plot(t,psi_non_linear,'LineWidth',1.5,'color','k','linestyle','--')
title(['\psi vs t for \delta_r= ',num2str(delta_r*180/pi),'°'])
legend('\psi (Full Linear Model','\psi (Non Linear Model)','Location','best')
xlabel('t (s)')
ylabel('\psi (deg)')