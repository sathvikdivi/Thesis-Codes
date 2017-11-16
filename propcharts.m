clear
clc
close all

lengths = [6:2:10];

% Global Constants
global nu rho_seawater Aluminium Buffers rateddepth g CB_wing rho_wing Batteries ShuttleDistance Efficiencies rho_foam;
nu = 1.004e-6; % kinetmatic viscocity of water
rho_seawater = 1025; % density of sea water
rateddepth = 500; % m, max operating depth
g = 9.81;  % accln due to gravity
ShuttleDistance = 250000; % m

% Hull Material
Name = 'Aluminium 7075 T6';
YieldStrength = 503e6;
UltimateStrength = 570e6;
ElasticityModulus = 70e9;
Density = 2900;
Aluminium.Name = Name;
Aluminium.Density = Density;
Aluminium.YieldStrength = YieldStrength;
Aluminium.UltimateStrength = UltimateStrength;
Aluminium.ElasticityModulus = ElasticityModulus;

% Efficiencies
Efficiencies.GearBox = 0.95; % Gear box efficiency
Efficiencies.Generator = 0.9; % Generator efficiency
Efficiencies.Hull = 1; % Hull efficiency
Efficiencies.BehindHullProp = 0.76; % Behind Hull Propeller Efficiency
Efficiencies.Shaft = 0.98; % Shaft Efficiency
Efficiencies.Motor = 0.95; % Motor Efficiency

% Batteries
Batteries.GravDensity = 100; % wh/kg
Batteries.VolDensity = 120; % wh/L

% Wing
CB_wing = 0.667; % area block coefficient for wing cross section. Obtained after sketching models in Solidworks.
% the above value of Cb_wing works only for NACA 0021 foil 
rho_wing = 2100 ; % density of the wing material (GFRP Composite)
rho_foam = 62; % density of polyurethane foam (stuffed inside the remaining spaces in the wing volume.)

% Buffers
StructuralBuffer = 1.5;
WeightBuffer = 1.2;
VolumeBuffer = 1.3;
Buffers.StructuralBuffer = StructuralBuffer;
Buffers.WeightBuffer = WeightBuffer;
Buffers.VolumeBuffer = VolumeBuffer;

% Velocities
v_rated = 2; % GS Rated speed for energy harvesting
v_AUV = 1.5; % AUV Cruise Speed
v_current = 0.2; % current speeds encountered by AUV while journeying between mainland and GS

% Ducted Turbine Systems
ri = 1.07; % inlet area ratio
re = 1.47; % exit area ratio
theta_diff = 20; % diffuser exit angle
Cd_Duct = 0.2; % drag coeff for duct
Cd_Turbine = 0.69; % drag coeff for turbine
Cd_DTS_Moving = 0.30; % drag coeff for DTS when moving
Cd_DTS_Opr = 0.61; % drag coeff for DTS when operating
Cp_DTS = 0.5; % power coeff for DTS

% Hull
PercentArray = [0.2,0.4,0.6]; % sectional percentage coefficients [mid, front, aft]
nf = 2.5; % front section exponential
na = 2; % aft section exponential


for w = 1:length(lengths)

Des = [];
Des_Props_Hull = [];
Des_Props_Wing = [];
Weights = [];
Volumes = [];

LD = lengths(w) ;

TurbDia = [1:0.5:5];
HullDia = [0.5:0.5:5];

for i = 1:length(TurbDia)
    for j = 1:length(HullDia)
        [x,y,SectionalLengths] = HullForm(HullDia(j)*LD,HullDia(j),PercentArray,na,nf);
        Lpmb = SectionalLengths(1);
        Lf = SectionalLengths(2);
        La = SectionalLengths(3);
        [WS_Hull,Vol_Hull,K1,K2] = HullAreaVolumeEstimator(nf,na,La,Lf,Lpmb,HullDia(j)*LD,HullDia(j));
        Cd_Hull = HullDragEstimator(v_AUV,WS_Hull,Vol_Hull,HullDia(j)*LD,HullDia(j),K2);
        frac = Vol_Hull./(0.25*pi*HullDia(j)^2*HullDia(j)*LD);
        RatedPressure = rho_seawater*g*rateddepth; % Pa
        Thickness = RatedThickness(HullDia(j),RatedPressure); % m 
        ID = HullDia(j) - 2*Thickness; % m, inner hull dia
        IL = HullDia(j)*LD - 2*Thickness;  % m, inner hull length
        [Ix,Iy,ISectionalLengths] = HullForm(IL,ID,PercentArray,na,nf);
        ILpmb = ISectionalLengths(1);
        ILf = ISectionalLengths(2);
        ILa = ISectionalLengths(3);
        [IWS,IVol_Hull,IK1,IK2] = HullAreaVolumeEstimator(nf,na,ILa,ILf,ILpmb,IL,ID);
        [Fd_wing,Cd_wing,beff,bref,Aeff_wing,Aref_wing,ChordLength_In,ChordLength_Out,ChordThick_In,ChordThick_Out,deltaChord] = ReynoldsNumber(v_AUV,TurbDia(i));
        bwing = bref + HullDia(j);
        Aplanform_wing = bwing*ChordLength_Out;
        [Vol_Wing_Out,Vol_Wing_Net,Wt_Wing] = WingWeightVolEstimator(ChordLength_Out,ChordLength_In,ChordThick_Out,ChordThick_In,bref,beff); % gives the volume and weight for the wing, m^3 and kg
        Wing(1) = bwing;
        Wing(2) = bref;
        Wing(3) = beff;
        Wing(4) = ChordThick_In;
        Wing(5) = ChordThick_Out;
        Wing(6) = ChordLength_In;
        Wing(7) = ChordLength_Out;
        Wing(8) = Aplanform_wing;
        Wing(9) = Aref_wing;
        Wing(10) = Aeff_wing;
        Wing(11) = Vol_Wing_Out;
        Wing(12) = Vol_Wing_Out - Vol_Wing_Net;
        Wing(13) = Vol_Wing_Net ;
        Wing(14) = Wt_Wing;
        Wing(15) = Cd_wing;
        Wing(16) = Fd_wing;
        [Wt_Batteries_rm] = BallastProps([TurbDia(i),ri,re,theta_diff,Cd_Duct,Cd_Turbine,Cd_DTS_Moving,Cd_DTS_Opr,Cp_DTS],[HullDia(j),HullDia(j)*LD,frac,Cd_Hull,WS_Hull,Thickness,Vol_Hull,IVol_Hull],Wing,[v_rated,v_AUV,v_current]); % kg
        [EPropTot,time_tot_days,P_Turbine,P_AUV] = AUVPropulsion([TurbDia(i),ri,re,theta_diff,Cd_Duct,Cd_Turbine,Cd_DTS_Moving,Cd_DTS_Opr,Cp_DTS],[HullDia(j),HullDia(j)*LD,frac,Cd_Hull,WS_Hull,Thickness,Vol_Hull,IVol_Hull],Wing,[v_rated,v_AUV,v_current]); % MWh, days, kW, kW
        E_Cargo = 0.001*0.001*Wt_Batteries_rm * Batteries.GravDensity; % MWh
        t_opr_days = ((E_Cargo./P_AUV)*1000)./24; % days
        E_Surplus = E_Cargo - EPropTot ; % MWh
        Vol_Storage = ((E_Cargo*1000*1000/300)*0.001)./Buffers.VolumeBuffer; % m^3
        Vol_Ballast = IVol_Hull - Vol_Storage; % m^3
        Des_Props_Hull = vertcat(Des_Props_Hull,[TurbDia(i),HullDia(j),frac,Cd_Hull,WS_Hull,Vol_Hull,IVol_Hull,Thickness,Vol_Storage,Vol_Ballast]);
        Des_Props_Wing = vertcat(Des_Props_Wing,[TurbDia(i),HullDia(j),Cd_wing,Fd_wing,bwing,bref,beff,ChordLength_Out,ChordThick_Out,ChordLength_In,ChordThick_In,Vol_Wing_Out,Wing(12),Wt_Wing,Aplanform_wing,Aref_wing,Aeff_wing]);
        Des = vertcat(Des,[TurbDia(i),HullDia(j),Wt_Batteries_rm,P_AUV,E_Cargo,EPropTot,E_Surplus,t_opr_days]);
    end
end

% Need to check once from here
[m,n] = size(Des);

lines = {'-','--','-.'};
clrs = [0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 0.5 0.5;1 0.5 0;0.5 0.5 0.5;0.5 0.5 1];

figure(1)
for i = 1:length(TurbDia)
    pos = find(Des(:,1) == TurbDia(i));
    plot(Des(pos,2),0.001*Des(pos,3),'Color',clrs(i,:),'LineWidth',1.75);
    hold on
end
for z = 1:length(TurbDia)
    leg(z) = cellstr(['D_{t} = ',num2str(TurbDia(z))]);
end
grid on
xlabel('Hull Diameter (m)');
ylabel('Battery Weight (Tons)')
xlim([min(HullDia) max(HullDia)]);
caption = [{'Batteries vs Hull Diameter'},{['(L/D = ',num2str(LD),')']}];
title(caption)
legend(leg,'Location','Bestoutside')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case I\plots\';
filename = ['BattWtvsHullDia_LD',num2str(LD),'_v2'];
saveas(gcf,fullfile(fpath,filename),'jpeg');


figure(2)
for i = 1:length(TurbDia)
    pos = find(Des(:,1) == TurbDia(i));
    plot(Des(pos,2),Des(pos,5),'Color',clrs(i,:),'LineWidth',1.75);
    hold on
end
for z = 1:length(TurbDia)
    leg(z) = cellstr(['D_{t} = ',num2str(TurbDia(z))]);
end
grid on
xlabel('Hull Diameter (m)');
ylabel('Cargo Energy (MWh)');
xlim([min(HullDia) max(HullDia)]);
caption = [{'Cargo Energy vs Hull Diameter'},{['(L/D = ',num2str(LD),')']}];
title(caption)
legend(leg,'Location','Bestoutside')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case I\plots\';
filename = ['CargoEnergyvsHullDia_LD',num2str(LD),'_v2'];
saveas(gcf,fullfile(fpath,filename),'jpeg');


figure(3)
for i = 1:length(TurbDia)
    pos = find(Des(:,1) == TurbDia(i));
    plot(Des(pos,2),Des(pos,6),'Color',clrs(i,:),'LineWidth',1.75);
    hold on
end
for z = 1:length(TurbDia)
    leg(z) = cellstr(['D_{t} = ',num2str(TurbDia(z))]);
end
grid on
xlabel('Hull Diameter (m)');
ylabel('Propulsion Energy (MWh)');
xlim([min(HullDia) max(HullDia)]);
caption = [{'Propulsion Energy vs Hull Diameter'},{['(L/D = ',num2str(LD),')']}];
title(caption);
legend(leg,'Location','Bestoutside')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case I\plots\';
filename = ['PropEnergyvsHullDia_LD',num2str(LD),'_v2'];
saveas(gcf,fullfile(fpath,filename),'jpeg');


figure(4)
for i = 1:length(TurbDia)
    pos = find(Des(:,1) == TurbDia(i));
    plot(Des(pos,2),Des(pos,7),'Color',clrs(i,:),'LineWidth',1.75);
    hold on
end
for z = 1:length(TurbDia)
    leg(z) = cellstr(['D_{t} = ',num2str(TurbDia(z))]);
end
grid on
xlabel('Hull Diameter (m)');
ylabel('Deliverable Energy (MWh)');
xlim([min(HullDia) max(HullDia)]);
caption = [{'Deliverable Energy vs Hull Diameter '},{['(L/D = ',num2str(LD),')']}];
title(caption)
legend(leg,'Location','Bestoutside')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case I\plots\';
filename = ['DelvEnergyvsHullDia_LD',num2str(LD),'_v2'];
saveas(gcf,fullfile(fpath,filename),'jpeg');


figure(5)
for i = 1:length(TurbDia)
    pos = find(Des(:,1) == TurbDia(i));
    plot(Des(pos,2),Des(pos,8),'Color',clrs(i,:),'LineWidth',1.75);
    hold on
end
for z = 1:length(TurbDia)
    leg(z) = cellstr(['D_{t} = ',num2str(TurbDia(z))]);
end
grid on
xlabel('Hull Diameter (m)');
ylabel('GS Time(days)');
xlim([min(HullDia) max(HullDia)]);
ylim([0 max(Des(:,8)*1.1)]);
caption = [{'GS Operating Time vs Hull Diameter '},{['(L/D = ',num2str(LD),')']}];
title(caption)
legend(leg,'Location','Bestoutside')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case I\plots\';
filename = ['GSTimevsHullDia_LD',num2str(LD),'_v2'];
saveas(gcf,fullfile(fpath,filename),'jpeg');

clear leg


figure(6)
for i = 1:length(HullDia)
    pos = find(Des(:,2) == HullDia(i));
    plot(Des(pos,1),Des(pos,5),'Color',clrs(i,:),'LineWidth',1.75);
    hold on
end
for z = 1:length(HullDia)
    leg(z) = cellstr(['D_{h} = ',num2str(HullDia(z))]);
end
grid on
xlabel('Turbine Diameter(m)');
ylabel('Cargo Energy(MWh)');
xlim([min(TurbDia) max(TurbDia)]);
caption = [{'Cargo Energy vs Turbine Diameter '},{['(L/D = ',num2str(LD),')']}];
title(caption)
legend(leg,'Location','Bestoutside')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case I\plots\';
filename = ['CargoEnergyvsTurbDia_LD',num2str(LD),'_v2'];
saveas(gcf,fullfile(fpath,filename),'jpeg');


figure(7)
for i = 1:length(HullDia)
    pos = find(Des(:,2) == HullDia(i));
    plot(Des(pos,1),Des(pos,6),'Color',clrs(i,:),'LineWidth',1.75);
    hold on
end
for z = 1:length(HullDia)
    leg(z) = cellstr(['D_{h} = ',num2str(HullDia(z))]);
end
grid on
xlabel('Turbine Diameter (m)');
ylabel('Propulsion Energy (MWh)');
xlim([min(TurbDia) max(TurbDia)]);
caption = [{'Propulsion Energy vs Turbine Diameter '},{['(L/D = ',num2str(LD),')']}];
title(caption)
legend(leg,'Location','Bestoutside')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case I\plots\';
filename = ['PropEnergyvsTurbDia_LD',num2str(LD),'_v2'];
saveas(gcf,fullfile(fpath,filename),'jpeg');


figure(8)
for i = 1:length(HullDia)
    pos = find(Des(:,2) == HullDia(i));
    plot(Des(pos,1),Des(pos,7),'Color',clrs(i,:),'LineWidth',1.75);
    hold on
end
for z = 1:length(HullDia)
    leg(z) = cellstr(['D_{h} = ',num2str(HullDia(z))]);
end
grid on
xlabel('Turbine Diameter (m)');
ylabel('Deliverable Energy (MWh)');
xlim([min(TurbDia) max(TurbDia)]);
yLimits = ylim;
ylim([0 yLimits(2)*1.2]);
caption = [{'Deliverable Energy vs Turbine Diameter'},{['(L/D = ',num2str(LD),')']}];
title(caption)
legend(leg,'Location','Bestoutside')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case I\plots\';
filename = ['DelvEnergyvsTurbDia_LD',num2str(LD),'_v2'];
saveas(gcf,fullfile(fpath,filename),'jpeg');
xlim([1 5]);

figure(9)
for i = 1:length(HullDia)
    pos = find(Des(:,2) == HullDia(i));
    plot(Des(pos,1),Des(pos,8),'Color',clrs(i,:),'LineWidth',1.75);
    hold on
end
for z = 1:length(HullDia)
    leg(z) = cellstr(['D_{h} = ',num2str(HullDia(z))]);
end
grid on
xlabel('Turbine Diameter (m)');
ylabel('GS Time (days)');
xlim([min(TurbDia) max(TurbDia)]);
ylim([0 max(Des(:,8))*1.2])
caption = [{'GS Operating Time vs Turbine Diameter'},{['(L/D = ',num2str(LD),')']}];
title(caption)
legend(leg,'Location','Bestoutside')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case I\plots\';
filename = ['GSTimevsTurbDia_LD',num2str(LD),'_v2'];
saveas(gcf,fullfile(fpath,filename),'jpeg');

tempxmax = max(Des(:,8));
clear leg;


figure(10)
for i = 1:length(TurbDia)
    pos = find(Des(:,1) == TurbDia(i));
    tempx = Des(pos,8);
    tempy = Des(pos,7);
    tempy = vertcat(tempy,tempy(end));
    tempx = vertcat(tempx,tempxmax);
    plot(tempx,tempy,'Color',clrs(i,:),'LineWidth',1.75);
    hold on
end
grid on
for z = 1:length(TurbDia)
    leg(z) = cellstr(['D_{t} = ',num2str(TurbDia(z))]);
end
xlabel('GS Operating Time (Days)');
ylabel('Deliverable Energy (MWh)');
caption = [{'Deliverable Energy vs GS Operating Time'},{['(L/D = ',num2str(LD),')']}];
title(caption)
legend(leg,'Location','Bestoutside')
% XLim = xlim;
% YLim = ylim;
% axis([0 2000 0 140]);
xlim([0 tempxmax]);
ylim([0 tempy(end)+20]);
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case I\plots\';
filename = ['DelvEnergyvsGSTime_LD',num2str(LD),'_v2'];
saveas(gcf,fullfile(fpath,filename),'jpeg');

clear leg

figure(11)
for i = 2:length(HullDia)
    pos = find(Des(:,2) == HullDia(i));
    HD = (Des(pos,7).*1000)./30; 
    GST = Des(pos,8);
    plot(Des(pos,1),HD./abs(GST),'Color',clrs(i,:),'LineWidth',1.75);
    leg(i-1) = cellstr(['D_{h} = ',num2str(HullDia(i))]);
    hold on
end

grid on
xlabel('Turbine Diameter (m)');
ylabel('No. of Houses Powered');
xlim([min(TurbDia) max(TurbDia)]);
ylim([-10 100]);
caption = [{'No. of Houses powered for GS Time'}, {['vs Turbine Diameter ',' (L/D = ',num2str(LD),')']}];
title(caption)
legend(leg,'Location','Bestoutside')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case I\plots\';
filename = ['NumberofHousesvsTurbDia_LD',num2str(LD),'_v2'];
saveas(gcf,fullfile(fpath,filename),'jpeg');

clear leg

close all

end