clc
close all
clear all

run('Longitudinal_Linearized_Matrices');

%Servo Transfer Function
servo=tf(10,[1 10]);
%Integrator Transfer Function
integrator=tf(1,[1 0]);
%Differentiator Transfer Function
differentiator=tf([1 0],1);
%Engine Time Lag Transfer Function
engine_timelag=tf(0.1,[1 0.1]);

%Longitudinal Transfer Functions
Long_ss=ss(A_Long,B_Long,C_Long,D_Long);
Long_TF=tf(Long_ss);

%Elevator Transfer Funtions
u_de=Long_TF(1,1);
w_de=Long_TF(2,1);
q_de=Long_TF(3,1);
theta_de=Long_TF(4,1);
wdot_de=u_de*differentiator;
az_de=wdot_de-u_0*q_de;
az_theta=az_de/theta_de;
alpha_de=w_de/u_0;
gamma_de=theta_de-alpha_de;
gamma_theta=minreal(gamma_de/theta_de);
h_theta=minreal(-integrator*(minreal(w_de/theta_de-u_0)));

%Thrust Transfer Funciton
u_dth=Long_TF(1,2);
w_dth=Long_TF(2,2);
q_dth=Long_TF(3,2);
theta_dth=Long_TF(4,2);

%% Longitudinal Autopilot
%% Pitch Control (Theta Theta_comm Open Loop TF)
OL_theta_thetacom=minreal(-servo*theta_de);
 
%% Altittude Control (h h_comm Open Loop TF)
 %Calculated from SISO Tool after Pitch Controller Designing
load('CL_theta_thetacom.mat'); 
OL_h_hcom=minreal(CL_theta_thetacom*h_theta);

%% Velocity Control (u u_comm Open Loop TF)
OL_u_ucom=minreal(servo*engine_timelag*u_dth);