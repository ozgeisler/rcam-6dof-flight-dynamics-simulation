clc
clear 
close all

load("XU_data.mat")
load("Aerodynamic_Force_Coefficients.mat")
load("AerodynamicMoments.mat")
load("EngineForce_Moment_Gravitational.mat")
load("Xdotvalues.mat")
% Extracting the data
dataX = simX.signals.values;  % 743x9
dataU = simU.signals.values;  % 743x5
t = simX.time';                % 743x1

% Calculate CL,CD and Cy for each timestep
CL_sim = zeros(1,743);
CD_sim = zeros(1,743);
CY_sim = zeros(1,743);
MAcg_b_sim = zeros(3,743);
FE_b_sim   = zeros(3,743);
MEcg_b_sim = zeros(3,743);
Fg_b_sim   = zeros(3,743);
XDOT_sim = zeros(9,743);

for i = 1:743
    X = dataX(i,:);
    U = dataU(i,:);
    [CL_sim(i), CD_sim(i), CY_sim(i)] = RCAM_model_A(X, U);
    [MAcg_b_sim(:,i)] = RCAM_model_B(X,U);
    [FE_b_sim(:,i),MEcg_b_sim(:,i),Fg_b_sim(:,i)] = RCAM_Model_C(X,U);
    [XDOT_sim(:,i)] = RCAM_Model_D(X,U);
end

% Plot
figure

subplot(3,1,1)
plot(t, CL_sim, 'b', t, CL, 'r--')
xlabel('Time (s)'), ylabel('CL')
legend('Simulink','Reference')
title('CL Comparison')

subplot(3,1,2)
plot(t, CD_sim, 'b', t, CD, 'r--')
xlabel('Time (s)'), ylabel('CD')
legend('Simulink','Reference')
title('CD Comparison')

subplot(3,1,3)
plot(t, CY_sim, 'b', t, CY, 'r--')
xlabel('Time (s)'), ylabel('CY')
legend('Simulink','Reference')
title('CY Comparison')

figure
subplot(3,1,1)
plot(t, MAcg_b_sim(1,:), 'b', t, MAcg_b(1,:), 'r--')
xlabel('Time (s)'), ylabel('L')
legend('Simulink','Reference')
title('Roll Moment Comparison')

subplot(3,1,2)
plot(t, MAcg_b_sim(2,:), 'b', t, MAcg_b(2,:), 'r--')
xlabel('Time (s)'), ylabel('M')
legend('Simulink','Reference')
title('Pitch Moment Comparison')

subplot(3,1,3)
plot(t, MAcg_b_sim(3,:), 'b', t, MAcg_b(3,:), 'r--')
xlabel('Time (s)'), ylabel('N')
legend('Simulink','Reference')
title('Yaw Moment Comparison')

figure
subplot(2,1,1)
plot(t,FE_b_sim(1,:),'b',t,FE_b(1,:),'r--')
xlabel('Time (s)')
ylabel('Engine Forces')
legend('Simulink','Reference')
title('Engine/Thrust Forces  Comparision in Fb_x')

subplot(2,1,2)
plot(t,MEcg_b_sim(2,:),'b',t,MEcg_b(2,:),'r--')
xlabel('Time (s)')
ylabel('Engine Moment')
legend('Simulink','Reference')
title('Engine Moment Comparision in CoG in Fb_z')

figure
subplot(3,1,1)
plot(t,Fg_b_sim(1,:),'b',t,Fg_b(1,:),'r--')
xlabel('Time (s)')
ylabel('Gravitational Force')
legend('Simulink','Reference')
title('Gravitational Force Comparision in Fb_x')

subplot(3,1,2)
plot(t,Fg_b_sim(2,:),'b',t,Fg_b(2,:),'r--')
xlabel('Time (s)')
ylabel('Gravitational Force')
legend('Simulink','Reference')
title('Gravitational Force Comparision in Fb_y')

subplot(3,1,3)
plot(t,Fg_b_sim(3,:),'b',t,Fg_b(3,:),'r--')
xlabel('Time (s)')
ylabel('Gravitational Force')
legend('Simulink','Reference')
title('Gravitational Force Comparision in Fb_z')

figure
subplot(3,1,1)
plot(t,XDOT_sim(1,:),'b',t,xdot(1,:),'r--')
xlabel('Time(s)')
ylabel('udot')
legend('Simulink','Reference')
title(' Body Velocity in U Changes Comparision')

subplot(3,1,2)
plot(t,XDOT_sim(2,:),'b',t,xdot(2,:),'r--')
xlabel('Time(s)')
ylabel('ydot')
legend('Simulink','Reference')
title(' Body Velocity in y Changes Comparision')

subplot(3,1,3)
plot(t,XDOT_sim(3,:),'b',t,xdot(3,:),'r--')
xlabel('Time(s)')
ylabel('wdot')
legend('Simulink','Reference')
title(' Body Velocity in w Changes Comparision')


figure
subplot(3,1,1)
plot(t,XDOT_sim(4,:),'b',t,xdot(4,:),'r--')
xlabel('Time(s)')
ylabel('pdot')
legend('Simulink','Reference')
title('Change in P Angular Velocity Comparision')

subplot(3,1,2)
plot(t,XDOT_sim(5,:),'b',t,xdot(5,:),'r--')
xlabel('Time(s)')
ylabel('qdot')
legend('Simulink','Reference')
title('Change in q Angular Velocity Comparision')

subplot(3,1,3)
plot(t,XDOT_sim(6,:),'b',t,xdot(6,:),'r--')
xlabel('Time(s)')
ylabel('rdot')
legend('Simulink','Reference')
title('Change in r Angular Velocity Comparision')

figure
subplot(3,1,1)
plot(t,XDOT_sim(7,:),'b',t,xdot(7,:),'r--')
xlabel('Time(s)')
ylabel('phidot')
legend('Simulink','Reference')
title('Change in Bank Euler Angle in Comparison')

subplot(3,1,2)
plot(t,XDOT_sim(8,:),'b',t,xdot(8,:),'r--')
xlabel('Time(s)')
ylabel('thetadot')
legend('Simulink','Reference')
title('Change in Pitch Euler Angle in Comparision')

subplot(3,1,3)
plot(t,XDOT_sim(9,:),'b',t,xdot(9,:),'r--')
xlabel('Time(s)')
ylabel('psidot')
legend('Simulink','Reference')
title('Change in Yaw Euler Angle Comparision')


