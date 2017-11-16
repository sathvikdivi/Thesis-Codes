
function [Vol_Wing_Out,Vol_Wing_Net,Wt_Wing] = WingWeightVolEstimator(ChordLength_Out,ChordLength_In,ChordThick_Out,ChordThick_In,bref,beff)
    
    global rho_wing CB_wing rho_foam;
    
    Vol_Wing_Out =  (CB_wing*bref*ChordThick_Out*ChordLength_Out); % m^3
    Vol_Wing_In = (CB_wing*bref*ChordThick_In*ChordLength_In); % m^3
    Vol_Wing_Net = Vol_Wing_Out - Vol_Wing_In ; % m^3
    Wt_Wing = rho_wing*Vol_Wing_Net +  rho_foam*Vol_Wing_In ; % kg
    
end