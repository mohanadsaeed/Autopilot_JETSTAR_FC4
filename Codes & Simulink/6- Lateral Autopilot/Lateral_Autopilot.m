clc
close all
clear all

run('Lateral_Linearized_Matrices')

%Servo Transfer Function
servo=tf(10,[1 10]);
%Integrator Transfer Function
integrator=tf(1,[1 0]);
%Differentiator Transfer Function
differentiator=tf([1 0],1);

%Lateral Transfer Functions
Lat_ss=ss(A_Lat,B_Lat,C_Lat,D_Lat);
Lat_TF=tf(Lat_ss);

%Aileron Transfer Funtions
v_da=Lat_TF(1,1);
p_da=Lat_TF(2,1);
r_da=Lat_TF(3,1);
phi_da=Lat_TF(4,1);
psi_da=Lat_TF(5,1);

%Rudder Transfer Funciton
v_dr=Lat_TF(1,2);
p_dr=Lat_TF(2,2);
r_dr=Lat_TF(3,2);
phi_dr=Lat_TF(4,2);
psi_dr=Lat_TF(5,2);

%% Lateral Autopilot
%% Yaw Damping Controller
OL_r_rcom=minreal(servo*r_dr);
%Washout circuit (compensator)
Yawdamping_comp = tf(0.38326*[1 0],[1 0.7]);
 
%% Roll Controller
%Calculated from SISO Tool after Yaw Damper Designing
%New TF after adding Yaw Damper
Lat_yawdamped_tf = feedback(servo*Lat_TF,Yawdamping_comp,2,3,1);
phi_da_yawdamped = Lat_yawdamped_tf(4,1);
OL_phi_phicom = minreal(phi_da_yawdamped);
