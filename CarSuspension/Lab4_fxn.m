function [ds, ext] = Lab4_fxn(t,s)
%% input system parameters
global g m_us m_s k_s b_s k_t T m_a k_a b_a U X_i slope_i b_c

%% rename state variables

qt=s(1); %m, tire flex (disp)
pus=s(2);%N-s, un sprung momentum
ps=s(3); %N-s, sprung mass displacement
qs=s(4); %m, suspension displacement
qa =s(5); 
pa=s(6); %N-s, car body momentum

%% input specifications
% Here we define the inputs into our system. First we calculate our effort
% sources. Then we calculate the velocity profile of the triangular bump.

%Flow source
X = U*t; %pos of vehicle
slope = interp1(X_i,slope_i,X);
Vi = U*slope; 

%% equations of motion
% These are the dynamic equations describing our state variables given in
% the form s'=f(s,inputs).
ic = (b_c/T)*(ps/m_s);
dqt = Vi - pus/m_us; 
dpus = qt*k_t - m_us*g - qs*k_s - b_s*(pus/m_us - ps/m_s);
dps = qs*k_s + b_s*(pus/m_us - ps/m_s) - m_s*g- qa*k_a - b_a*(ps/m_s - pa/m_a) -T*ic;
dqs = pus/m_us - ps/m_s;
dqa = ps/m_s - pa/m_a;
dpa = qa*k_a + b_a*(ps/m_s - pa/m_a) + T*ic - m_a*g; 

%% stacking up the derivatives for output as a vector
ds=[dqt; dpus; dps; dqs; dqa; dpa];
ext(1)= Vi;  % displacement of input force
ext(2) = ic; %current input