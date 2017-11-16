
function [E_Prop,time_tot,P_Turbine,P_AUV] = AUVPropulsion(Turb,Hull,Wing,velocities)

global rho_seawater ShuttleDistance Efficiencies;

Dt          = Turb(1);
ri          = Turb(2);
re          = Turb(3);
theta_diff  = Turb(4);
Cd_Duct     = Turb(5);
Cd_Turbine  = Turb(6);
Cd_DTS_Moving = Turb(7);
Cd_DTS_Opr  = Turb(8);
Cp_DTS      = Turb(9);

Dh          = Hull(1);
Lh          = Hull(2);
frac        = Hull(3);
Cd_Hull     = Hull(4);
WS          = Hull(5);
Thickness   = Hull(6);
Vol_Hull    = Hull(7);
IVol_Hull   = Hull(8);

LbyD = Lh./Dh;

Ar = 0.25*pi*Dt^2; % turbine rotor swept area (m^2)
Ae = re*Ar; % duct exit cross section area (m^2)
Ai = ri*Ar; % duct inlet cross section area (m^2)

Cd_wing = Wing(15);
Aeff_wing = Wing(10);

v_rated = velocities(1); % m/s
v_AUV = velocities(2); % m/s
v_current = velocities(3); % m/s

% resultant and advance velocities
beta = rad2deg(asin(v_current/v_AUV));  % degrees. AUV resultant velocity is perpendicular to the direction of current.
v_resultant = v_AUV*cos(deg2rad(beta)); % m/s
v_advance = sqrt((v_AUV*cos(deg2rad(beta)))^2 + (v_AUV*sin(deg2rad(beta)) + v_current)^2); % m/s 

% Drag Forces
F_drag_DTS = Cd_DTS_Moving*0.5*rho_seawater*Ae.*v_advance^2 ; % N
F_drag_Hull = Cd_Hull*0.5*rho_seawater*WS.*v_advance^2 ; % N
F_drag_Wing = Cd_wing*rho_seawater*Aeff_wing.*v_advance^2; % N
F_drag_total = 2*F_drag_DTS + F_drag_Hull + F_drag_Wing ; % N, Total drag force on the AUV Generator 

time = (ShuttleDistance/(v_resultant))./3600; % hours, Range travel time

P_Prop = (F_drag_total*v_advance)./(Efficiencies.Hull*Efficiencies.BehindHullProp*Efficiencies.Shaft*Efficiencies.Motor)  ; % (W) Propulsive Power ; % W
E_Prop = 0.001*P_Prop*time; % kWh, Energy input to the propeller

time_tot = time; % hours

%%%%%%%%%% Rated Power of AUV Generator %%%%%%%%%
[P_Turbine,P_AUV] = AUVPowEstimator(Ae,Cp_DTS,v_rated); % kW 
% This would now give the Total Power Rating of the AUV. The energy that
% goes in to the batteries in the storage bay.

end