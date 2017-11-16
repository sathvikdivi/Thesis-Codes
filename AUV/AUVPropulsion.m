
function [EPropTot,time_tot_days,P_Turbine,P_AUV] = AUVPropulsion(Turb,Hull,Wing,velocities)
% Function to compute the propulsion energy for a given configuration of
% the AUV Generator. 

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

%%%%%%%%%% Rated Power of AUV Generator %%%%%%%%%
[P_Turbine,P_AUV] = AUVPowEstimator(Ae,Cp_DTS,v_rated); % kW 
% This would now give the Total Power Rating of the AUV. The energy that
% goes in to the batteries in the storage bay.

%%%%%%%%%%% Upstream %%%%%%%%%%%%%%%%

% Upstream Case
v_advance_us = v_AUV + v_current ; % (m/s)
v_rel_us = v_AUV - v_current ; % (m/s)  
time_us = (ShuttleDistance./v_rel_us)/3600 ; %(hours)

% Drag Forces
F_drag_wing_us = Cd_wing*0.5*rho_seawater*Aeff_wing.*v_advance_us^2; % drag force on the winged portion (N)
F_drag_Hull_us = Cd_Hull*0.5*rho_seawater*WS*v_advance_us.^2 ; % (N)
F_drag_DTS_us = Cd_DTS_Moving*0.5*rho_seawater*Ae.*v_advance_us.^2; % (N)
F_drag_total_us = 2*F_drag_DTS_us + F_drag_Hull_us + F_drag_wing_us; % (N) Total Drag force on AUV 

% Propulsive Power & Energy - Power & Energy delivered to the propeller
P_Prop_us =  0.001*(F_drag_total_us.*v_advance_us)./(Efficiencies.Hull*Efficiencies.BehindHullProp*Efficiencies.Shaft*Efficiencies.Motor) ; % (kW) Power Output from Propeller Motor 
E_Prop_us = P_Prop_us.*time_us ; % kWh Energy output from Propeller Motor

%%%%%%%%%% Downstream %%%%%%%%%%%%%%%%

% Downstream Case
v_advance_ds = v_AUV - v_current ; % (m/s)
v_rel_ds = v_AUV + v_current ; % (m/s)
time_ds = (ShuttleDistance./v_rel_ds)/3600 ; %(hours)

% Drag Forces
F_drag_wing_ds = Cd_wing*0.5*rho_seawater*Aeff_wing.*v_advance_ds.^2; % drag force on the winged portion (N)
F_drag_Hull_ds = Cd_Hull*0.5*rho_seawater*WS.*v_advance_ds.^2 ; % (N)
F_drag_DTS_ds = Cd_Hull*0.5*rho_seawater*Ae.*v_advance_ds.^2; % (N)
F_drag_total_ds = 2*F_drag_DTS_ds + F_drag_Hull_ds + F_drag_wing_ds ; % (N) Total Drag force on AUV 

% Propulsive Power & Energy - Power & Energy delivered to the propeller
P_Prop_ds = (F_drag_total_ds.*v_advance_ds)./(Efficiencies.Hull*Efficiencies.BehindHullProp*Efficiencies.Shaft*Efficiencies.Motor)  ; % (W) Propulsive Power 
E_Prop_ds = 0.001*P_Prop_ds.*time_ds ; % kWh Energy input to the propeller

%%%%%%%%%% Total travel times and Energy %%%%%%%%%%%%%% 

time_tot = time_us + time_ds ; % (hours) time for the onward & return trip
time_us_days = round(time_us/24,2); % (days) upstream
time_ds_days = round(time_ds/24,2); % (days) downstream
time_tot_days = round(time_tot/24,2); % (days)

EPropTot = (E_Prop_us + E_Prop_ds)*0.001; % (MWh)


end