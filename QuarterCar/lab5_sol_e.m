function [ds,ext] = lab5_sol_e(t,s)
%% input system parameters

global Rw Lw Tm M bt R Gr Cr g Cd ro Af Ki Kp

%% rename state variables

P3=s(1);  % Flux linkage
P9=s(2);  % Momentum of car
dref = s(3); %reference displacment
d = s(4); %vehical displacement
power = s(5);
%% input specifications
% Here we define the inputs into our system.
% These input sources are defined graphically in the problem statement. 
% The graphical information has been translated into equations.


T1=0.5;           % s, controller start

if t<T1; vref=0; 
else vref=LA92Oracle(t); 
end

Uin = Kp*(vref-P9/M)+Ki*(dref-d);

%% equations of motion
% These are the dynamic equations describing our state variables given in
% the form s'=f(s,inputs).
dP3 = Uin - (Rw/Lw)*P3-(Tm*Gr)/(R*M)*P9;
dP9 = (Gr*Tm)/(Rw*Lw)*P3 - (Gr^2*bt)/(Rw^2*M)*P9 - M*g*Cr*((P9/M)/(abs((P9/M))+0.000001))-0.5*ro*Af*Cd*(1/M)*P9*abs(P9/M);
ddref = vref;
dd =  P9/M;
dpower = Uin;
%% stacking up the derivatives for output as a vector
ds=[dP3;dP9;ddref;dd;dpower];

end

