clc
clear vars
close all
% Equations
dydt=@(t,y) [(sin(t)+cos(y(1))+sin(y(2)));(cos(t)+sin(y(2)))];
% Initial Conditions
y10=-1;
y20=1;
y0=[y10;y20];
% Time span
t0=0;
tf=20;
n=500;
h=(tf-t0)/n;
tspan=t0:h:tf;
% Solve using ODE45
tic
[t,y]=ode45(dydt,tspan,y0);
timeode45=toc
% Solve using RK1
rk=1;
tic
[t1,y1]=rk1_4(dydt,tspan,y0,h,rk);
time1=toc
% Solve using RK2
rk=2;
tic
[t2,y2]=rk1_4(dydt,tspan,y0,h,rk);
time2=toc
% Solve using RK3
rk=3;
tic
[t3,y3]=rk1_4(dydt,tspan,y0,h,rk);
time3=toc
% Solve using RK4
rk=4;
tic
[t4,y4]=rk1_4(dydt,tspan,y0,h,rk);
time4=toc

% Plotting the RK Solution vs ODE45 Solution For y1
figure;
subplot(2,1,1)
plot(t1,y1(:,1),'LineWidth',1.3);
hold on
plot(tspan,y(:,1),'LineWidth',1.3,'color','k','linestyle','--');
title('RK1 vs ODE45 Solution For y_1')
legend('y_1 (ODE45)','y_1 (RK1)','Location','southeast')
legend('boxoff')
xlabel('t')
ylabel('y_1')

subplot(2,1,2)
plot(t2,y2(:,1),'LineWidth',1.3);
hold on
plot(tspan,y(:,1),'LineWidth',1.3,'color','k','linestyle','--');
title('RK2 vs ODE45 Solution For y_1')
legend('y_1 (ODE45)','y_1 (RK2)','Location','southeast')
legend('boxoff')
xlabel('t')
ylabel('y_1')

figure;
subplot(2,1,1)
plot(t3,y3(:,1),'LineWidth',1.3);
hold on
plot(tspan,y(:,1),'LineWidth',1.3,'color','k','linestyle','--');
title('RK3 vs ODE45 Solution For y_1')
legend('y_1 (ODE45)','y_1 (RK3)','Location','southeast')
legend('boxoff')
xlabel('t')
ylabel('y_1')

subplot(2,1,2)
plot(t4,y4(:,1),'LineWidth',1.3);
hold on
plot(tspan,y(:,1),'LineWidth',1.3,'color','k','linestyle','--');
title('RK4 vs ODE45 Solution For y_1')
legend('y_1 (ODE45)','y_1 (RK4)','Location','southeast')
legend('boxoff')
xlabel('t')
ylabel('y_1')

% Plotting the RK Solution vs ODE45 Solution For y2
figure;
subplot(2,1,1)
plot(t1,y1(:,2));
hold on
plot(tspan,y(:,2),'LineWidth',1.3,'color','k','linestyle','--');
title('RK1 vs ODE45 Solution For y_2')
legend('y_2 (ODE45)','y_2 (RK1)','Location','southeast')
legend('boxoff')
xlabel('t')
ylabel('y_2')

subplot(2,1,2)
plot(t2,y2(:,2),'LineWidth',1.3);
hold on
plot(tspan,y(:,2),'LineWidth',1.3,'color','k','linestyle','--');
title('RK2 vs ODE45 Solution For y_2')
legend('y_2 (ODE45)','y_2 (RK2)','Location','southeast')
legend('boxoff')
xlabel('t')
ylabel('y_2')

figure;
subplot(2,1,1)
plot(t3,y3(:,2),'LineWidth',1.3);
hold on
plot(tspan,y(:,2),'LineWidth',1.3,'color','k','linestyle','--');
title('RK3 vs ODE45 Solution For y_2')
legend('y_2 (ODE45)','y_2 (RK3)','Location','southeast')
legend('boxoff')
xlabel('t')
ylabel('y_2')

subplot(2,1,2)
plot(t4,y4(:,2),'LineWidth',1.3);
hold on
plot(tspan,y(:,2),'LineWidth',1.3,'color','k','linestyle','--');
title('RK4 vs ODE45 Solution For y_2')
legend('y_2 (ODE45)','y_2 (RK4)','Location','southeast')
legend('boxoff')
xlabel('t')
ylabel('y_2')

% Calculation the maximum error
error1=abs(y1(1:n+1,1)-y(:,1));
error1_max=max(error1(20:n+1));
error1_max_percentage=error1_max/y(find(error1==error1_max))*100

error2=abs(y2(1:n+1,1)-y(:,1));
error2_max=max(error2);
error2_max_percentage=error2_max/y(find(error2==error2_max))*100

error3=abs(y3(1:n+1,1)-y(:,1));
error3_max=max(error3);
error3_max_percentage=error3_max/y(find(error3==error3_max))*100

error4=abs(y4(1:n+1,1)-y(:,1));
error4_max=max(error4);
error4_max_percentage=error4_max/y(find(error4==error4_max))*100


