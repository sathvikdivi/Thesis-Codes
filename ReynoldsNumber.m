
function [Fd_wing,Cd_wing,beff,bref,Aeff_wing,Aref_wing,ChordLength_In,ChordLength_Out,ChordThick_In,ChordThick_Out,deltaChord] = ReynoldsNumber(v,Dt) 

% nu = 1.004e-6;
% Dt = [1:1:5]';
% Dh = 0.15.*Dt;
% v = [1,1.5,2,2.5,3,3.5];
% t = 21; %
% rho = 1025;
% RHull = 0.625; 

% for i = 1:length(v)
%     Re(:,i) = v(i).*c/nu ; 
% end
% 
% Re = Re*1e-6;
% Re = round(Re,2);
global rho_seawater ;

Cd = [8.5,7.77,7.37,7.05,6.92,6.73;7.38,6.94,6.67,6.44,6.38,6.24;...
     6.94,6.55,6.38,6.23,6.12,6.03;6.68,6.39,6.22,6.04,6.02,5.91;...
     6.45,6.23,6.04,5.99,5.92,5.87];

TurbDias = [1:1:5]'; 

if (v >= 1 && v < 1.5 )
    r = 1;
elseif (v >= 1.5 && v < 2 )
    r = 2;
elseif (v >= 2 && v < 2.5)
    r = 3;
elseif (v >= 2.5 && v < 3.0)
    r = 4;
elseif (v >= 3.0 && v < 3.5)
    r = 5;
elseif (v >= 3.5 && v < 4)
    r = 6;
end

if (Dt == 1)
    c = 1;
elseif (Dt == 2)
    c = 2;
elseif (Dt == 3)
    c = 3;
elseif (Dt == 4)
    c = 4;
elseif (Dt == 5)
    c = 5;
else
    c = 0;
end

P = polyfit(TurbDias,Cd(:,r),4); % polynomial equation for drag 

t = 21 ; % 21% thickness , NACA 21
Dh = 0.15*Dt; % hub diameter, m
ChordThick_In = Dh; % m, inner thickness of the chord
ChordLength_Out = Dh./0.17; % m, outer length of the chord
deltaChord = 0.02*ChordLength_Out; % thickness is 2% of the chord length
ChordLength_In = (ChordLength_Out - 2*deltaChord); % chord length, m 
ChordThick_Out = ChordThick_In + 2*deltaChord ;% chord outer thickness, m

bref = 2*1.5*Dt; % total reference wing length (considering both the turbine systems) 
% (portion of the wing that is outside the hull, considering the portion of the wing inside the duct), m
beff = 2*Dt; % m, total effective wing length (considering both the turbine systems), 
% (portion of the wing that is outside the hull and duct, only this portion is considered for drag calculation)
% bwing =  bref + Dh ; % m, total wing length (considering both the turbine systems)

% Aplanform_wing = ChordLength_Out.*bwing ; % Planform area (m^2)
Aeff_wing = ChordLength_Out.*beff ; % Effective wing area that is in contact with the fluid body (m^2)
Aref_wing = ChordLength_Out.*bref; % Reference wing area that is outside the hull and the inner portion of the duct is also considered (m^2)

if c == 0
    Cd_wing = 0.001*polyval(P,Dt); % drag coefficient for the wing
else 
    Cd_wing = 0.001*Cd(c,r); % drag coefficient for the wing
end

Fd_wing = Cd_wing*0.5*rho_seawater*Aeff_wing*v^2 ; % N


end

