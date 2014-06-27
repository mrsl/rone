%%% Battlebot Manipulator Servo test Data
clear all
close all

% for the servo on a 35.75 mm arm with weights attached. 
mom_arm = 35.75; %millimeters
mass = [0 50 55 60 65 70 75 80 85 90 95 100]'; %grams
current = [.0049 .0529 .0546 .0591 .0624 .0660 .0694 .0745 .0790 ...
    .0840 .0848 .0851]'; %amps
force = mass./1000.*9.80; %newtons
torque = force.*(mom_arm/1000); %netwon*meters

figure
plot(force,current,'o-','markersize',10,'linewidth',2);
hold on
title('Test on Servo Alone with 35.75mm Moment Arm','fontsize',18,...
    'interpreter','latex')
xlabel('Force(N)','fontsize',16,'interpreter','latex')
ylabel('Current(A)','fontsize',16,'interpreter','latex')

% for the servo and the claw aparatus
mass2 = mass;
force2 = mass2./1000.*9.80;
current2 = [.0051 .0052 .0079 .0093 .0103 .0151  .0052 .020 .0213 .0291 ...
    .0292 .0171]';

figure
plot(force2, current2,'o-','markersize',10,'linewidth',2)
hold on
title('Test on Battlebot Aparatus','fontsize',18,...
    'interpreter','latex')
xlabel('Force(N)','fontsize',16,'interpreter','latex')
ylabel('Current(A)','fontsize',16,'interpreter','latex')
axis([0 1 0 0.03])