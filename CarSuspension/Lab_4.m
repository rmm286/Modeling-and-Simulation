%% Lab 3 Master File
% The purpose of the master file is to enter system parameters, call the
% differential equation solver, and finally post-process and plot results.
%% Clean Up Workspace
clear all
close all
clc

%% Input System Parameters
global g m_us m_s k_s b_s k_t T m_a k_a b_a U X_i slope_i b_c

g=9.8;
m_tot=3000./2.2; %Total vehicle mass
msmus=5; %Sprung to unsprung mass ratio
m_us=m_tot/(1+msmus); %unsprung mass
m_s=m_tot-m_us; %Sprung mass
w_s=2*pi*1.2; %Suspension frequency
k_s=m_s*w_s^2; %Suspension stiffness
zeta_s=.1; zeta_c=.7; %Damping ratio for passive and active system. These will be varied.
b_s=2*zeta_s*w_s*m_s; %Suspension damping constant
b_c=2*zeta_c*w_s*m_s; %Effective damping constant for control
w_wh=2*pi*8; %Wheel hop frequency
k_t = m_us*w_wh^2; %Tire stiffness
R_w=.005; %Winding resistance, Ohm CAN BE IN RANGE 0.001< R <0.01
T=1; %Nm/A Coupling constant CAN BE IN RANGE 1< T <10
m_a=.02*m_s; %Actuator mass, kg
w_a=2*pi*5; %Actuator frequency
k_a= m_a*w_a^2; %Actuator stiffness
b_a=2*.1*w_a*m_a; %Actuator damping
U=40*.46; %m/s Trial vehicle velocity

%Flow source
rng('default');
delta_x=.5; Length=500; %This defines a length of a road in m
X_i=0:delta_x:Length; %This establishes a length vector
n_pts=fix(Length/delta_x);
slope_raw=randn(n_pts+1,1); %This generates uniformly distributed random number that we interpret as the road slope at each delta_x
slope_i=.005*(slope_raw-mean(slope_raw)); %This is the slope
%vector that has any average value removed. This makes the road zero mean slope.
%The scaling number at the front makes the passive vehicle
%without control have a sprung mass acceleration that is
%reasonable. I experimented to determine this number.


%% Define initial Conditions
% The initial condition values for each system state variable are defined.

qto= (m_s+m_a+m_us)*g/k_t; %m, tire flex (disp)
puso=0;%N-s, un spring momentum
pso=0; %N-s, sprung mass displacement
qso=(m_s +m_a)*g/k_s; %m, suspension displacement
qao= m_a*g/k_a; %m, actuator displacement
pao=0; %N-s, car body momentum

initial = [qto;puso;pso;qso;qao;pao];
% [tire flex, un sprung momentum, sprung mass displacement, suspension displacement,actuator disp, car body momentum]

%% Setup Time Array
% The timestep was given in problem specification
tspan = 0:0.0001:5;

%% Call ode45() Ordinary Differential Equation Solver

[t,s]=ode45(@Lab4_fxn,tspan,initial); %call ode45

qt=s(:,1); %m, tire flex (disp)
pus=s(:,2); %N-s, un spring momentum
ps=s(:,3); %N-s, sprung mass displacement
qs=s(:,4); %m, suspension displacement
qa=s(:,5); %m, actuator disp
pa=s(:,6); %N-s, car body momentum

% obtain derivatives and additional outputs
for i=1:length(t)
    [ds(i,:), ext(i,:)] = Lab4_fxn(t(i),s(i,:));
end

dqt=ds(:,1); %m, tire flex (disp)
dpus=ds(:,2); %N-s, un spring momentum
dps=ds(:,3); %N-s, sprung mass displacement
dqs=ds(:,4); %m, suspension displacement
dqa=ds(:,5); %m, actuator disp
dpa=ds(:,6); %N-s, car body momentum

Vin = ext(:,1); %input velocity
ic = ext(:,2); %input current

Psacc = dps/m_s; %acceleration of the spring mass
Powerinput = R_w*ic.*sign(ic) + ic.*(pa*(T/m_a) - ps*(T/m_s)); %input power of the actuator
Powerloss = R_w*ic.*sign(ic);

%% Outputs

figure('Name','Actuator Disp','NumberTitle','off','Color','white')
 subplot(2,1,1)
 plot(tspan, qa,'k','LineWidth',1);grid on;
 title('Actuator Displacement')
 ylabel('Distance (m)')
 xlabel('Time (s)')
 subplot(2,1,2)
 plot(tspan, Psacc, 'k','LineWidth',1);grid on;
 title('Accl. of Sprung Mass')
 ylabel('Acc (m/s^2)')
 xlabel('Time (s)')
 sgtitle('Active Control - High Coupling Constant')

 figure('Name','Power Input','NumberTitle','off','Color','white')
 subplot(3,1,1)
 plot(tspan, ic,'k','LineWidth',1);grid on;
 title('Input Current')
 ylabel('Current (A)')
 xlabel('Time (s)')
 subplot(3,1,2)
 plot(tspan, Powerinput/1000, 'k','LineWidth',1);grid on;
 title('Input Power')
 ylabel('Power (kW)')
 xlabel('Time (s)')
 subplot(3,1,3)
 plot(tspan, Powerloss, 'k','LineWidth',1);grid on;
 title('Power Loss')
 ylabel('Power (W)')
 xlabel('Time (s)')
 sgtitle('Low Coupling Constant')







