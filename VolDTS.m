function [V] = VolDTS(Dt)

% V_rotor = VolRotorDisk(Dt);
% V_duct = VolDuct(Dt,ri,re,theta_diff);
% 
% V = V_rotor + V_duct; %(m^3)

% Data from CAD Models
Dttemp = [2:1:5]; %m
Voltemp = [0.27,0.66,1.31,2.31]; % m^3

VolEqn = polyfit(Dttemp,Voltemp,3); % Eqn to estimate vol of DTS depending on Dt based
V = polyval(VolEqn,Dt); % m3

end