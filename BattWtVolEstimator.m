function [WtBattReqd,VolBattReqd] = BattWtVolEstimator(Etot)

% Assuming Battery Wt density = 250 wh/kg and vol density = 300wh/l
% E is in kWh

global Batteries;

WtBattReqd = (Etot./Batteries.GravDensity)*1000 ; % Kg
VolBattReqd = (Etot./Batteries.VolDensity) ; % m^3

end