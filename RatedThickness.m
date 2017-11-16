function [Thickness] = RatedThickness(D,RatedPressure)
    
    global Buffers Aluminium;
     
    Sigma = Aluminium.YieldStrength ; 
    SF = Buffers.StructuralBuffer ; 
    r = 0.5*D;
    Thickness = (RatedPressure*r)./(SF*Sigma); %m

end