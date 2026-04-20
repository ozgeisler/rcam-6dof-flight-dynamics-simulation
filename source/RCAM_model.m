clc
clear
close all

load("XU_data.mat")

x0 = simX.signals.values(1,:)';
t  = simX.time';
dataX = simX.signals.values;
dataU = simU.signals.values;
simU_data = [t, dataU];

out = sim("RCAM_MODEL_full.slx", ...
    'StopTime', num2str(t(end)), ...
    'SolverType', 'Fixed-step', ...
    'FixedStep', '0.05');

X_sim = squeeze(out.X_sim)';   % 743x9
size(X_sim)   % to check

X_sim = squeeze(out.X_sim)';   % 743x9

stateNames = {'u (m/s)','v (m/s)','w (m/s)','p (rad/s)','q (rad/s)','r (rad/s)','phi (rad)','theta (rad)','psi (rad)'};

figure;
for i = 1:9
    subplot(3,3,i)
    plot(t, X_sim(:,i), 'b', t, dataX(:,i), 'r--')
    xlabel('Time (s)')
    ylabel(stateNames{i})
    legend('Simulated','Reference')
    title(stateNames{i})
end
sgtitle('State Trajectory Comparison')