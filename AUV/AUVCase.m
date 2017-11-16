

function [AUVDesTable] = AUVCase(HullLength,HullDia,TurbDia)

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
v_AUV = 3; % AUV Cruise Speed
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


Des = [];
Des_Props_Hull = [];
Des_Props_Wing = [];
Weights = [];
Volumes = [];

LD = HullLength./HullDia ;


[x,y,SectionalLengths] = HullForm(HullDia*LD,HullDia,PercentArray,na,nf);
Lpmb = SectionalLengths(1);
Lf = SectionalLengths(2);
La = SectionalLengths(3);
[WS_Hull,Vol_Hull,K1,K2] = HullAreaVolumeEstimator(nf,na,La,Lf,Lpmb,HullDia*LD,HullDia);
Cd_Hull = HullDragEstimator(v_AUV,WS_Hull,Vol_Hull,HullDia*LD,HullDia,K2);
frac = Vol_Hull./(0.25*pi*HullDia^2*HullDia*LD);
RatedPressure = rho_seawater*g*rateddepth; % Pa
Thickness = RatedThickness(HullDia,RatedPressure); % m 
ID = HullDia - 2*Thickness; % m, inner hull dia
IL = HullDia*LD - 2*Thickness;  % m, inner hull length
[Ix,Iy,ISectionalLengths] = HullForm(IL,ID,PercentArray,na,nf);
ILpmb = ISectionalLengths(1);
ILf = ISectionalLengths(2);
ILa = ISectionalLengths(3);
[IWS,IVol_Hull,IK1,IK2] = HullAreaVolumeEstimator(nf,na,ILa,ILf,ILpmb,IL,ID);
[Fd_wing,Cd_wing,beff,bref,Aeff_wing,Aref_wing,ChordLength_In,ChordLength_Out,ChordThick_In,ChordThick_Out,deltaChord] = ReynoldsNumber(v_AUV,TurbDia);
bwing = bref + HullDia;
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
[Wt_Batteries_rm] = BallastProps([TurbDia,ri,re,theta_diff,Cd_Duct,Cd_Turbine,Cd_DTS_Moving,Cd_DTS_Opr,Cp_DTS],[HullDia,HullDia*LD,frac,Cd_Hull,WS_Hull,Thickness,Vol_Hull,IVol_Hull],Wing,[v_rated,v_AUV,v_current]); % kg
[EPropTot,time_tot_days,P_Turbine,P_AUV] = AUVPropulsion([TurbDia,ri,re,theta_diff,Cd_Duct,Cd_Turbine,Cd_DTS_Moving,Cd_DTS_Opr,Cp_DTS],[HullDia,HullDia*LD,frac,Cd_Hull,WS_Hull,Thickness,Vol_Hull,IVol_Hull],Wing,[v_rated,v_AUV,v_current]); % MWh, days, kW, kW
E_Cargo = 0.001*0.001*Wt_Batteries_rm * Batteries.GravDensity; % MWh
t_opr_days = ((E_Cargo./P_AUV)*1000)./24; % days
E_Surplus = E_Cargo - EPropTot ; % MWh
Vol_Storage = ((E_Cargo*1000*1000/300)*0.001)./Buffers.VolumeBuffer; % m^3
Vol_Ballast = IVol_Hull - Vol_Storage; % m^3
Des_Props_Hull = vertcat(Des_Props_Hull,[TurbDia,HullDia,frac,Cd_Hull,WS_Hull,Vol_Hull,IVol_Hull,Thickness,Vol_Storage,Vol_Ballast]);
Des_Props_Wing = vertcat(Des_Props_Wing,[TurbDia,HullDia,Cd_wing,Fd_wing,bwing,bref,beff,ChordLength_Out,ChordThick_Out,ChordLength_In,ChordThick_In,Vol_Wing_Out,Wing(12),Wt_Wing,Aplanform_wing,Aref_wing,Aeff_wing]);
Des = vertcat(Des,[TurbDia,HullDia,HullLength,Wt_Batteries_rm,P_AUV,E_Cargo,EPropTot,E_Surplus,t_opr_days]);
AUVDesTable = Des;
end
