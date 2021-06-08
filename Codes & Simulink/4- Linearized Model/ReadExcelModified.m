% the aerodynamic forces and moments can be expressed as a function of all the motion variables [Nelson] page 63
% clc
% clear all
% close all
%  Excel Sheets Data
% global aircraft_derivatives_dimensions 
filename_density_L = 'excelsheet_data_GPO4'; %%put here the location of your excel sheet

aircraft_data=xlsread(filename_density_L,'B2:B61');%% here B2:B61 means read the excel sheet from cell B2 to cell B61

%%in the next step we will read from the vector(aircraft_data) but take care of the order the values in excel sheet is arranged

% Time vector parameters
dt = aircraft_data(1);    tfinal = aircraft_data(2); lengths=tfinal/dt+1;
time_V = (0:dt:tfinal)';

% initial conditions
s0 = aircraft_data(4:15);
sdot0 = zeros(12,1);
Vto = sqrt(s0(1)^2 + s0(2)^2 + s0(3)^2);    % Vto

% control actions values
% da = aircraft_data(57);
% dr = aircraft_data(58);
% de = aircraft_data(59);
% dth = aircraft_data(60);
dc = [ aircraft_data(57:59) * pi/180 ; aircraft_data(60)];

% gravity, mass % inertia
m = aircraft_data(51);
g = aircraft_data(52);
Ixx = aircraft_data(53);
Iyy = aircraft_data(54);
Izz = aircraft_data(55);
Ixz = aircraft_data(56);    Ixy=0;  Iyz=0;
I = [Ixx , -Ixy , -Ixz ;...
    -Ixy , Iyy , -Iyz ;...
    -Ixz , -Iyz , Izz];
invI=inv(I);

% stability derivatives Longitudinal motion
SD_Long = aircraft_data(21:36);
SD_Long_final = SD_Long;


% stability derivatives Lateral motion
SD_Lat_dash = aircraft_data(37:50);
 
% Delta of Control Surface
del_a=dc(1);            del_e=dc(3);    
del_r=dc(2);            del_t=dc(4);

% Initial Conditions
u_0=s0(1);   v_0=s0(2);   w_0=s0(3);       
p_0=s0(4);   q_0=s0(5);   r_0=s0(6);
wdot_0=0;
phi_0=s0(7);    theta_0=s0(8);  psi_0=s0(9);

% Derivatives
Xu=SD_Long_final(1);    Zu=SD_Long_final(2);    Mu=SD_Long_final(3);
Xw=SD_Long_final(4);    Zw=SD_Long_final(5);    Mw=SD_Long_final(6);
Zw_dot=SD_Long_final(7);Zq=SD_Long_final(8);    Mw_dot=SD_Long_final(9);
Mq=SD_Long_final(10);   Xdel_e=SD_Long_final(11);Zdel_e=SD_Long_final(12);
Mdel_e=SD_Long_final(13); Xdel_t=SD_Long_final(14);Zdel_t=SD_Long_final(15);
Mdel_t=SD_Long_final(16);Yv=SD_Lat_dash(1);     YB=SD_Lat_dash(2);
LB_dash=SD_Lat_dash(3); NB_dash=SD_Lat_dash(4); Lp_dash=SD_Lat_dash(5);
Np_dash=SD_Lat_dash(6); Lr_dash=SD_Lat_dash(7);
Nr_dash=SD_Lat_dash(8); Ydel_a_star=SD_Lat_dash(9);
Ydel_r_star=SD_Lat_dash(10);    Ldel_a_dash=SD_Lat_dash(11);
Ndel_a_dash=SD_Lat_dash(12);    Ldel_r_dash=SD_Lat_dash(13);
Ndel_r_dash=SD_Lat_dash(14);
SD_dash=[LB_dash ; Lp_dash ; Lr_dash ; Ldel_a_dash ; Ldel_r_dash ; 
        NB_dash ; Np_dash ; Nr_dash ; Ndel_a_dash ; Ndel_r_dash];
Matrix_SD_dash_SD=...
        [1 0 0 0 0 -Ixz/Ixx 0 0 0 0 ; 
        0 1 0 0 0 0 -Ixz/Ixx 0 0 0 ; 
        0 0 1 0 0 0 0 -Ixz/Ixx 0  0 ; 
        0 0 0 1 0 0 0 0 -Ixz/Ixx 0 ; 
        0 0 0 0 1 0 0 0 0 -Ixz/Ixx ; 
        -Ixz/Izz 0 0 0 0 1 0 0 0 0 ; 
        0 -Ixz/Izz 0 0 0 0 1 0 0 0 ; 
        0 0 -Ixz/Izz 0 0 0 0 1 0 0 ; 
        0 0 0 -Ixz/Izz 0 0 0 0 1 0 ;  
        0 0 0 0 -Ixz/Izz 0 0 0 0 1];

SD_Lat=Matrix_SD_dash_SD*SD_dash;
    
LB=SD_Lat(1);   Lp=SD_Lat(2);   Lr=SD_Lat(3);   Ldel_a=SD_Lat(4);
Ldel_r=SD_Lat(5);
NB=SD_Lat(6);   Np=SD_Lat(7);   Nr=SD_Lat(8);   Ndel_a=SD_Lat(9);   
Ndel_r=SD_Lat(10);

Ydel_r=Ydel_r_star*Vto;
Lv=LB/Vto;
Nv=NB/Vto;
Lv_dash=LB_dash/Vto;
Nv_dash=NB_dash/Vto;
