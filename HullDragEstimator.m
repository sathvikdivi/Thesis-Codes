
function [Cds] = HullDragEstimator(velocity,WS,V,L,D,K2)

global nu ;

Re = L*velocity/nu;  % Reynolds Number
Cf = 0.075./(log10((Re - 2)^2)) ; % skin friction coefficient
formfac = 1 + 0.5*(D/L) + 3*(D/L)^3 ; % form factor

Ca = 0.0004; % assumption, roughness correlation coefficient
Cr = 0.00789/(L/D - K2); % pressure difference coefficient

Cds = Cf*formfac + Ca + Cr + L*D*0.0001; % drag coefficient

end