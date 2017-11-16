
function [Des,Des_Props_Hull,Des_Props_Wings,Hull_Coords] = FACaseII(Dt,Dh,Lh)


data55v5 = csvread('C:\users\sdivi\Documents\AUV\Propulsion\ShipPropulsionEnergyEstimates55v5_HarvaldCSV.csv',1,0);
[DesTemp,Des_Props_Hull,Des_Props_Wing,HullCoords] = AUVCase(Lh,Dh,Dt);


ECS = data55v5(1:26,5); % ship cargo
EPS = data55v5(1:26,14); % ship propulsion
P = polyfit(ECS,EPS,3); % polynomial that gives EPS as a function of ECS

ShipDes(:,1) = DesTemp(:,1); % Turb Dia
ShipDes(:,2) = DesTemp(:,2); % Hull Dia
ShipDes(:,3) = DesTemp(:,3); % Hull Length 
ShipDes(:,4) = DesTemp(:,5); % P_AUV
ShipDes(:,5) = DesTemp(:,6); % AUV Cargo Energy % MWh
ShipDes(:,6) = DesTemp(:,7); % AUV Propulsion Energy % MWh
ShipDes(:,7) = DesTemp(:,8); % AUV Surplus Energy % MWh
ShipDes(:,8) = polyval(P,ShipDes(:,7)); % Ship Propulsion Energy, % MWh 
ShipDes(:,9) = ShipDes(:,7) - ShipDes(:,8);  % Ship Surplus Energy, % MWh, This is the energy
% dumped on mainland
ShipDes(:,10) = DesTemp(:,9); % GS Time in days

Des = ShipDes ;
Des_Props_Hull = Des_Props_Hull ;
Des_Props_Wings = Des_Props_Wing;
Hull_Coords = HullCoords;

end