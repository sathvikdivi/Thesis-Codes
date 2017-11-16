clear
clc
close all

rho = 1025;
Ehouseday = 30; %kWh

data55v5 = csvread('C:\users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\ShipPropulsionEnergyEstimates55v5_Harvald_CSV_rev2.csv',1,0);
data55v8 = csvread('C:\users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\ShipPropulsionEnergyEstimates55v8_Harvald_CSV_rev2.csv',1,0);

ECS = data55v5(1:26,5); % ship cargo
EPS = data55v5(1:26,14); % ship propulsion
P = polyfit(ECS,EPS,3); % polynomial that gives EPS as a function of ECS

lengths = 6;

for i = 1:length(lengths)
    [temp1,temp2,temp3] = Propcharts_ShipCase(lengths(i));
    Des = temp1;
    Des_Props_Hull = temp2;
    Des_Props_Wing = temp3;
end

TurbDia = [1:0.5:5];
HullDia = [0.5:0.5:5];

for i = 1:length(lengths)
    ShipDes(:,1) = Des(:,1); % Dt
    ShipDes(:,2) = Des(:,2); % Dh
    ShipDes(:,3) = Des(:,7); % AUV Surplus = Ship Cargo
    ShipDes(:,4) = polyval(P,ShipDes(:,3));  % Ship Propulsion
    ShipDes(:,5) = ShipDes(:,3) - ShipDes(:,4); % Ship Surplus
end

pos = find(ShipDes(:,2) == 3);
TestData = ShipDes(pos,:);
AUVTestData1 = Des(pos,:);
AUVTestData2 = Des_Props_Hull(pos,:);
AUVTestData3 = Des_Props_Wing(pos,:);

v_ship = 5;
E_ship = data55v5(1:26,14);
k_ship = E_ship./(v_ship)^2;
E_shipnew = k_ship.*1.5^2;


% figure(1)
% plot(TestData(:,1),TestData(:,3),'LineWidth',2);
% xlabel('Dt')
% ylabel('Ship Cargo')
% 
% figure(2)
% plot(TestData(:,1),TestData(:,4),'LineWidth',2);
% xlabel('Dt')
% ylabel('Ship Prop')
% 
% figure(3)
% plot(TestData(:,1),TestData(:,5),'LineWidth',2);
% xlabel('Dt')
% ylabel('Ship Surp')
% 
% 
% figure(4)
% plot(AUVTestData1(:,1),AUVTestData1(:,5),'LineWidth',2);
% xlabel('Dt')
% ylabel('AUV Cargo')
% 
% figure(5)
% plot(AUVTestData1(:,1),AUVTestData1(:,6),'LineWidth',2);
% xlabel('Dt')
% ylabel('AUV Prop')
% 
% figure(6)
% plot(AUVTestData1(:,1),AUVTestData1(:,7),'LineWidth',2);
% xlabel('Dt')
% ylabel('AUV Surp')



