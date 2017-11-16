
function [Wt_Batteries_rm] = BallastProps(Turb,Hull,Wing,velocities) 

global rho_seawater Buffers Batteries;

% Calculate the rough estimates for weight of the total system and the
% displaced volume - Written by STandon & and later revised by SDivi
Dh          = Hull(1);
Lh          = Hull(2);
frac        = Hull(3);
Cd_hull     = Hull(4);
WS          = Hull(5);
Thickness   = Hull(6);
Vol_Hull    = Hull(7);
IVol_Hull   = Hull(8);

Dt          = Turb(1); % Turbine Diameter, m
ri          = Turb(2); % inlet area ratio
re          = Turb(3); % exit area ratio
theta_diff  = Turb(4); % diffuser angle
Cd_Duct     = Turb(5); % drag on duct while in operation
Cd_Turbine  = Turb(6); % thrust coefficient for turbine
Cd_DTS_Moving = Turb(7); % drag coeff for DTS when moving
Cd_DTS_Opr  = Turb(8); % drag coeff for DTS when harvesting
Cp_DTS      = Turb(9); % power coeff for DTS 

v_rated     = velocities(1);  % Rated Velocity (m/s)
v_AUV       = velocities(2); % AUV Cruise Velocity (m/s)
v_current   = velocities(3); % Current Velocity (m/s)

% Battery spec: 200 Whr/ kg; 300 Whr/ L; Density: 1200 kg/ m3
% Assumption: Storage Volume is the entire volume. Batteries occupy a part 
% of this volume. Once we do this, we work backwards seeking the balance
% point and thereby zero in on the number of batteries that can be fit
% inside the hull for  a given set of AUV Generator parameters.
Vol_Storage = IVol_Hull; 
Vol_Batteries = Vol_Storage./Buffers.VolumeBuffer; % m^3
Wt_Batteries = (1000*Vol_Batteries*Batteries.VolDensity)./Batteries.GravDensity; % kg 

Vol_DTS         = 2*VolDTS(Dt); % m^3 
Vol_Wing        = Wing(12); % m^3

Wt_DTS          = 2*wtDTS(Dt); % kg
Wt_Hull         = WtHull([Dh,Lh,Lh./Dh,frac,Thickness]); % kg      
Wt_Wing         = Wing(14); % kg

Vol_Tot = round(Vol_Hull + Vol_DTS + Vol_Wing,2) ; %(m3) volume of water displaced by the AUV Generator
Wt_Tot  = round((Wt_Batteries + Buffers.WeightBuffer*(Wt_Hull + Wt_DTS + Wt_Wing)),2) ; %(metric tons)
        
Wt_Extra        = Wt_Tot - rho_seawater*Vol_Tot; % kg , excess weight  that needs to be balanced 
Wt_Batteries_rm      = -Wt_Extra + Wt_Batteries; % kg, % batteries remaining after balancing. 
% If it is negative , then it means the design parameters are infeasible.
Wt_Batteries_pr      = (Wt_Batteries_rm*100)./Wt_Batteries; % Percent battery (by weight) remaining

E_Batteries_rm       = 200*Wt_Batteries_rm*0.001 ; % kWh

end
