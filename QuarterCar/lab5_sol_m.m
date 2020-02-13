%% Lab 2 Master File
% The purpose of the master file is to enter system parameters, call the
% differential equation solver, and finally post-process and plot results.

%% Clean Up Workspace
clear all
close all
clc

%% Input System Parameters

global Rw Lw Tm M bt R Gr Cr g Cd ro Af Uino Kp Ki vi

Rw = 0.01; %ohms, Armature winding resistance
Lw = 0.015; %H, Armature winding inductance
Tm = 1.718; % weber ,Transduction coefficient
M = 2200; %kg, Vehical mass
bt = 0.05; %Nms/rad, Drive shaft friction
R = 0.2; %m, Wheel radius
Gr = 5; %Gear ratio
Cr = 0.006; %Rolling resistance coefficient
g = 9.81; %m/s^2,acceleration due to gravity
Cd = 0.32; %Drag Coefficient
ro = 1.21; %kg/m^3, Air density
Af = 2.05; %m^2, Vehical Frontal Area
Kp =200;
Ki = 250;
vi = 0;

%% Define Initial Conditions
% The initial condition values for each system state variable are defined.
% Expressions for displacements are non-zero and determined by solvnig
% state equations when dP values are set to zero

 P3o = (Rw*Lw)/(Gr*Tm)*((Gr^2*bt)/Rw^2*vi+M*g*Cr+0.5*ro*Af*Cd*vi^2);
 P9o = vi*M;
 Uino = (Rw/Lw)*P3o+20*(Tm*Gr)/(R);

 initial = [P3o,P9o,0,0,0];
% [Flux linkage (P3), Momentum of car (P9), Reference displacment, vehical
% displacement, power]
%% Setup Time Array
% The timestep was given in problem specification

tspan = 0:0.05:300;
%% Call ode45() Ordinary Differential Equation Solver

[t,s]=ode45(@lab5_sol_e,tspan,initial);

P3=s(:,1);  % Flux linkage
P9=s(:,2);  % Momentum of car
dref = s(:,3); %reference displacement
d = s(:,4); %displacement of the car
power = s(:,5); %power input

%% Obtain derivatives
for i=1:length(t)
    [ds] = lab5_sol_e(t(i),s(i,:));
end
%% Outputs

%This makes an array with the command velocity values 
vref=zeros(length(t),1);
for i=1:length(t)
 vref(i) = LA92Oracle(t(i));
end

%These lines compute rise time, settling time and %O.S. for our 
%unit step input 
tr = t(max(find(P9 < M*0.9))) - t(max(find(P9 < M*0.1)));
ts = t(max(find(P9 > M*1.02 | P9 < M*.98)));
OS = max((P9/M)/vref);

%Calculates engery effiency for the LA92 cycle.
EE = max(power)/max(d);

%outputs plot of velocity
figure('Name','displacement','NumberTitle','off','Color','white')
plot(t,(P9/M),'k','LineWidth',2);grid on
title('Velocity of the Car')
ylabel('Vel (m/s)')
xlabel('Time (s)')

%outputs vehicle velocity against reference
figure('Name','displacement1','NumberTitle','off','Color','white')
plot(t,[(P9/M),vref],'LineWidth',2);grid on
title('Velocity vs. Reference')
legend('Vehicle','Reference')
ylabel('Vel (m/s)')
xlabel('Time (s)')
axis([32,54,4,8])

%% Bode Plots

%statespace matricies
A = [[-Rw/Lw, -(Tm*Gr)/(R*M)];[Tm*Gr/(R*Lw),-Gr^2*bt/(R^2*M)]];
B = [1;0];
C = [0,1/M];
D = [0];

%creating TF variables
s  = tf('s');
I = eye(2);

%Computing the transfer function
tfvu = C*(s*I - A)^-1*B+D;

%Plotting the Bode Diagram
bode(tfvu)








