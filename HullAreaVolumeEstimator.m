
function [WS,V,K1,K2] = HullAreaVolumeEstimator(nf,na,La,Lf,Lpmb,L,D)

[Cpf,Cwsf,Cpa,Cwsa] = HullCoeffEstimator(nf,na);

K1 = (L - Lpmb - Lf*Cpf - La*Cpa)/D;
K2 = (L - Lpmb - Lf*Cwsf - La*Cwsa)/D;

WS = pi*D*(Lpmb + Lf*Cwsf + La*Cwsa); % m^2
V = 0.25*pi*(D^2)*(Lpmb + La*Cpa + Lf*Cpf); % m^3

end