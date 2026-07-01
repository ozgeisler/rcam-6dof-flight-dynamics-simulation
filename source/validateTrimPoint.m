clc
clear all
close 

temp = load('trim_values_straight_level.mat')

XStar = temp.XStar;
UStar = temp.UStar;

out = sim("RCAM_Aircraft.slx");

t = out.simX.time;
X = out.simX.signals.values;

figure 
for k=1:9
    subplot(5,2,k)
    plot(t,X(:,k))
    ylabel(['x_',num2str(k)])
    grid on 
end


