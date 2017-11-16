
function [P_Turbine,P_AUV] = AUVPowEstimator(Ae,Cp,vrated)
    
    global rho_seawater Efficiencies; % kg/m^3
    
    P_Turbine = 0.001*Cp*0.5*rho_seawater*Ae*vrated^3; % kW
    P_AUV = 2*Efficiencies.GearBox*Efficiencies.Generator*P_Turbine; % kW

end