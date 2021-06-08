clc

run('ReadExcelModified')

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

%% Short Period Approximation
A_SP=A_Long(2:3,2:3);  
B_SP=B_Long(2:3,:);  
C_SP=eye(2);
D_SP=zeros(2,2);

%% Long Period Approximation
A_LP=[Xu            ,    -g*cos(theta_0);
      -Zu/(Zq+u_0)  ,    g*sin(theta_0)/(Zq+u_0)];
  
B_LP=[Xdel_e            ,   Xdel_t;
      -Zdel_e/(Zq+u_0)  ,  -Zdel_t/(Zq+u_0)];
  
C_LP=eye(2);
D_LP=zeros(2,2);
