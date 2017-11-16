
function [Des,HullCoord,Des_Wt,Des_Vol] = FACaseIII(Dt,Dh,L)

% Global Constants
global nu rho_seawater Aluminium Buffers rateddepth g CB_wing rho_wing Batteries ShuttleDistance Efficiencies rho_foam;
nu = 1.004e-6; % kinetmatic viscocity of water
rho_seawater = 1025; % density of sea water
rateddepth = 500; % m, max operating depth
g = 9.81;  % accln due to gravity
ShuttleDistance = 60000; % m

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
EnergyBuffer = 1.2;
Buffers.StructuralBuffer = StructuralBuffer;
Buffers.WeightBuffer = WeightBuffer;
Buffers.VolumeBuffer = VolumeBuffer;
Buffers.EnergyBuffer = EnergyBuffer ;

% Velocities
v_rated = 2; % GS Rated speed for energy harvesting
v_AUV = 1.5; % AUV Cruise Speed
v_current = 0.5; % current speeds encountered by AUV while moving

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
Des_Wt = [];
Des_Vol = [];
Weights = [];
Volumes = [];
LD = L./Dh ;

        [x,y,SectionLengths] = HullForm(L,Dh,PercentArray,na,nf);
        HullCoord = [x,y];
        Lpmb = SectionLengths(1);
        Lf = SectionLengths(2);
        La = SectionLengths(3);
        [WS,Vol_Hull,K1,K2] = HullAreaVolumeEstimator(nf,na,La,Lf,Lpmb,L,Dh);
        frac = Vol_Hull./(0.25*pi*Dh^2*L); % block coefficient for the hull
        RatedPressure = rho_seawater*g*rateddepth; 
        thickness_hull = RatedThickness(Dh,RatedPressure);
        [Wt_Hull,HullMat] = WtHull([Dh,L,LD,frac,thickness_hull]);
        IDh = HullMat(1);
        IL = HullMat(2);
        ILD = HullMat(3);
        IVol_Hull = HullMat(4);
        [Fd_wing,Cd_wing,beff,bref,Aeff_wing,Aref_wing,ChordLength_In,ChordLength_Out,ChordThick_In,ChordThick_Out,deltaChord] = ReynoldsNumber(v_AUV,Dt);
        bwing = bref + Dh;
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
        beta = rad2deg(asin(v_current/v_AUV));  % degrees. AUV resultant velocity is perpendicular to the direction of current.
        v_resultant = v_AUV*cos(deg2rad(beta)); % m/s
        v_advance = sqrt((v_AUV*cos(deg2rad(beta)))^2 + (v_AUV*sin(deg2rad(beta)) + v_current)^2); % m/s 
        Cd_Hull = HullDragEstimator(v_AUV,WS,Vol_Hull,L,Dh,K2);
        [E_Prop,time_travel,P_Turbine,P_AUV] = AUVPropulsion([Dt,ri,re,theta_diff,Cd_Duct,Cd_Turbine,Cd_DTS_Moving,Cd_DTS_Opr,Cp_DTS],[Dh,L,frac,Cd_Hull,WS,thickness_hull,Vol_Hull,IVol_Hull],Wing,[v_rated,v_AUV,v_current]);
        E_Hotel = (Buffers.EnergyBuffer - 1)*E_Prop;
        E_Total = E_Hotel + E_Prop; % kWh
        [Wt_Batt,Vol_Batt] = BattWtVolEstimator(E_Total);
        Vol_Storage =  Vol_Batt./Buffers.VolumeBuffer ;
        Wt_DTS = 2*wtDTS(Dt); % kg
        Vol_DTS = 2*VolDTS(Dt); % m^3
        Wt_Total = Buffers.WeightBuffer*(Wt_DTS + Wt_Wing + Wt_Batt + Wt_Hull); % kg
        Vol_Total = Vol_DTS + Vol_Wing_Out + Vol_Hull ; % m^3
        Wt_DisplacedWater = rho_seawater*Vol_Total; % kg
        Delta_Wt = Wt_DisplacedWater - Wt_Total ; % kg
        Delta_Vol = IVol_Hull - Vol_Storage ; % m^3 
        Des_Wt = vertcat(Des_Wt,[Dt,Dh,L,LD,thickness_hull,Wt_Batt,Wt_DTS,Wt_Wing,Wt_Hull,Wt_Total,Wt_DisplacedWater,Delta_Wt,rho_seawater*Delta_Vol]);
        Des_Vol = vertcat(Des_Vol,[Dt,Dh,L,LD,thickness_hull,Vol_Batt,Vol_Storage,Vol_DTS,Vol_Wing_Out,Vol_Hull,IVol_Hull,Vol_Total,Delta_Vol]);
        Des_Wing = [bwing,beff,bref,ChordLength_Out,ChordThick_Out,ChordLength_In,ChordThick_In,Aplanform_wing,Aref_wing,Aeff_wing,CB_wing];
        time_opr_days = ((E_Total./P_AUV)./24); % days
        Des = vertcat(Des,[Dt,Dh,L,P_AUV,0.001*P_AUV*24,0.001*E_Total,0.001*E_Prop,time_opr_days]);
        
end