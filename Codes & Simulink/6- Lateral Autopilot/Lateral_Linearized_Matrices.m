clc

run('ReadExcelModified')

%% Lateral EOM
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

%% 2-DOF Dutch-Roll Approximation
A_2dof=...
    [Yv         ,   Yr-u_0;
     Nv_dash   ,   Nr_dash];
B_2dof=...
    [0             ,   Ydel_r;
     Ndel_a_dash   ,   Ndel_r_dash];
  
C_2dof=eye(2);
D_2dof=zeros(2,2);

%% 1-DOF Roll Approximation
A_1dof=Lp_dash;
B_1dof=Ldel_a_dash;
C_1dof=1;
D_1dof=0;