
clear
clc
close all

Al = struct('Name','Aluminium 7075 T6','YieldStrength',500e6,'UltimateStrength',570e6,'ElasticityModulus',71.7e9,'Density',2810,'SafetyFactor',1.5);
Buffers = struct();

Dt = 3 ; % m
At = 0.25*pi*Dt^2 ; % m^2
CD_Duct = 0.2;
CD_Turbine = 0.45;
CD_DTS = 0.59;
CD_DTS_Moving = 0.35; 
Cp = 0.5;

LD = 8;
D = 1.25;
L = LD*D;
v_AUV = 3;
v_AUVMax = 4;
v_rated = 1.8; 
v_current = 0.5;
distance = 60000;
PropulsiveEfficiency = 0.75 ;
ShaftEfficiency = 0.98;
GeneratorEfficiency = 0.9;
Percentages = [0.2,0.4,0.6];
frontexponent = 2.5;
aftexponent = 2;

global rho_seawater nu LiBattGravDensity  LiBattVolDensity;
rho_seawater = 1025;
nu = 1.004e-6; 
LiBattGravDensity = 250; % wh/kg
LiBattVolDensity = 300; % wh/L


SF = 1.5;
energySF = 1.2;
volumeSF = 1.5;
weightSF = 1.2;
Buffers.StructuralSafetyFactor = SF;
Buffers.EnergySafetyFactor = energySF;
Buffers.CabinVolumeSafetyFactor = volumeSF;
Buffers.WeightSafetyFactor = weightSF;
RatedDepth = 200;


nf = frontexponent;    
na = aftexponent;    

[xtot,ytot,Lengths] = HullForm(L,D,Percentages,na,nf);
Lpmb = Lengths(1);
Lf = Lengths(2);
La = Lengths(3);

clrs = [0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 0.5 0.5;1 0.5 0;0.5 0.5 0.5;0.5 0.5 1];

% fpath = 'C:\Users\sdivi\Documents\AUV\Design Parameters';
% filename = ['AUVBodyHullFormCoordinates_',num2str(na),'na_',num2str(nf*10),'npf.csv'];
% Data = array2table([xtot,ytot]);
% writetable(Data,fullfile(fpath,filename));

[WS,V,K1,K2] = HullAreaVolumeEstimator(nf,na,La,Lf,Lpmb,L,D);
Cds = HullDragEstimator(v_AUV,WS,V,L,D,K2);

Turbine = struct('Name','Ducted Turbine System','RotorDiameter',Dt,'InletAreaRatio',1.07,'ExitAreaRatio',1.47,'DiffuserAngle',20,'RotorArea',At,'DuctDrag',CD_Duct,'TurbineDrag',CD_Turbine,'OperatingDrag',CD_DTS,'MovingDrag',CD_DTS_Moving,'PowCoeff',Cp);
Hull = struct('Name','Hull','HullDiameter',D,'Length',L,'LDRatio',LD,'AftLength',La,'FrontLength',Lf,'ParallelMidBodyLength',Lpmb,'WettedArea',WS,'EnclosedVolume',V,'DragCoeff',Cds);
Misc = struct('Name','Miscellanous','AUVCruiseSpeed',v_AUV,'AUVMaxSpeed',v_AUVMax,'GSRatedSpeed',v_rated,'GSDeepCurrentSpeed',v_current,'GSRange',distance,'PropulsiveEfficiency',PropulsiveEfficiency,'ShaftEfficiency',ShaftEfficiency,'GeneratedEfficiency',GeneratorEfficiency,'RatedDepth',RatedDepth);

[E_Prop,PropPower,t_tot] = AUVPropulsion(Turbine,Hull,Misc);

AUVCharacs = struct();
AUVCharacs.PropellerPower = PropPower; % kW
AUVCharacs.PropulsiveEfficiency = Misc.PropulsiveEfficiency;
AUVCharacs.ShaftEfficiency = Misc.ShaftEfficiency ; 
AUVCharacs.GeneratorEfficiency = GeneratorEfficiency;
AUVCharacs.PropulsionEnergy = E_Prop ; % kWh
AUVCharacs.GSRangeTime = t_tot ; % hours
AUVCharacs.GSRangeDistance = Misc.GSRange ; % m

Misc.RatedPressure = rho_seawater*9.81*Misc.RatedDepth ; % Pa
AUVCharacs.RatedPressure = Misc.RatedPressure ; % Pa 
AUVCharacs.RatedDepth = Misc.RatedDepth ;  % m

Hull.Thickness = round(RatedThickness(Hull,Misc,Al),3);

AUVCharacs.HullThickness = Hull.Thickness;

ID = D - 2*AUVCharacs.HullThickness ; 
IL = L - 2*AUVCharacs.HullThickness ;

[xItot,yItot,ILengths] = HullForm(IL,ID,Percentages,na,nf);

ILpmb = ILengths(1);
ILf = ILengths(2);
ILa = ILengths(3);

[wstemp,IV,IK1,IK2] = HullAreaVolumeEstimator(nf,na,ILa,ILf,ILpmb,IL,ID);

Hull.BodyWeight = Al.Density*(V - IV); % kg
AUVCharacs.BodyWeight = Hull.BodyWeight ;  

Hull.Volume = V; 

Turbine.Weight = wtDTS(Turbine.RotorDiameter); % kg
Turbine.Volume = VolDTS(Turbine.RotorDiameter); % kg 

HotelLoad = (Buffers.EnergySafetyFactor - 1) * AUVCharacs.PropulsionEnergy ; % kWh
TotalEnergy = AUVCharacs.PropulsionEnergy + HotelLoad ; % kWh
AUVCharacs.OnboardStoredEnergy = TotalEnergy ; % kWh 

[BattWt,BattVol] = BattWtVolEstimator(AUVCharacs.OnboardStoredEnergy);
AUVCharacs.BatteryWeight = BattWt ; % kg
AUVCharacs.BatteryVolume = BattVol ; % m^3

CabinVolume = Buffers.CabinVolumeSafetyFactor * AUVCharacs.BatteryVolume ; % m^3
AUVCharacs.CabinVolume = CabinVolume ; % m^3
AUVCharacs.HullInnerVolume = IV; % m^3

TotalWeight = Buffers.WeightSafetyFactor*(AUVCharacs.BodyWeight + AUVCharacs.BatteryWeight + 2*Turbine.Weight); % kg
AUVCharacs.TotalWeight = TotalWeight;
DisplacedVolume = 2*Turbine.Volume + Hull.Volume; % m^3
DisplacedMass = rho_seawater * DisplacedVolume ; % kg 

ExcessWeight = DisplacedMass - AUVCharacs.TotalWeight ; %kg
ExcessVolume = AUVCharacs.HullInnerVolume - AUVCharacs.CabinVolume ; % m^3

[Turbine.RatedPower,TotTurbPow] = AUVPowEstimator(Turbine.RotorArea*Turbine.ExitAreaRatio,Turbine.PowCoeff,Misc.GSRatedSpeed) ; % kW, kW

MaxBallastWeight = ExcessVolume * rho_seawater ; % kg

AUVCharacs.TurbineRatedPower = Turbine.RatedPower; %kW
AUVCharacs.ActualPower = TotTurbPow*AUVCharacs.GeneratorEfficiency; % kW

Overall = struct();
TurbineSystems = struct();
HullSystems = struct();
Propulsion = struct();
Ballast = struct();

Overall.AUVGeneratorPower = AUVCharacs.ActualPower;
Overall.TurbineDiameter = Turbine.RotorDiameter; 
Overall.HullDiameter = Hull.HullDiameter;
Overall.HullLength = Hull.Length;
Overall.TotalWeight = AUVCharacs.TotalWeight;
Overall.ReserveEnergy = AUVCharacs.OnboardStoredEnergy ;
Overall.CabinVolume = AUVCharacs.CabinVolume ; 
Overall.BallastVolume = ExcessVolume  ;

TurbineSystems.TurbinePowerRating = Turbine.RatedPower;
TurbineSystems.Weight = Turbine.Weight;
TurbineSystems.RotorDiameter  = Turbine.RotorDiameter;
TurbineSystems.Volume = Turbine.Volume ; 
TurbineSystems.DuctInletAreaRatio = Turbine.InletAreaRatio ; 
TurbineSystems.DuctExitAreaRatio = Turbine.ExitAreaRatio ; 
TurbineSystems.DuctExitAngle = Turbine.DiffuserAngle;
TurbineSystems.PowerCoefficient = Turbine.PowCoeff ; 
TurbineSystems.TurbineDrag = Turbine.TurbineDrag;
TurbineSystems.DuctDrag = Turbine.DuctDrag ; 
TurbineSystems.OperatingDrag = Turbine.OperatingDrag;
TurbineSystems.MovingDrag = Turbine.MovingDrag;
TurbineSystems.ThrustForce = 0.001*Turbine.OperatingDrag * 0.5*rho_seawater*(Turbine.RotorArea*Turbine.ExitAreaRatio)*Misc.GSRatedSpeed^2; % kN

HullSystems.HullDiameter = Hull.HullDiameter;
HullSystems.Length = Hull.Length;
HullSystems.L2DRatio = Hull.LDRatio;
HullSystems.FrontLength = Hull.FrontLength;
HullSystems.AftLength = Hull.AftLength; 
HullSystems.MidLength = Hull.ParallelMidBodyLength;
HullSystems.WettedArea = Hull.WettedArea;
HullSystems.TotalOuterVolume = Hull.EnclosedVolume;
HullSystems.HullDrag = Hull.DragCoeff ;
HullSystems.BodyThickness = Hull.Thickness;
HullSystems.Weight = Hull.BodyWeight ; 


Generator.GeneratorPowerMin = 0.9 * TurbineSystems.TurbinePowerRating ;
Generator.GeneratorPowerMax = 1.1 * TurbineSystems.TurbinePowerRating;
Generator.GenratorEfficiency = Misc.GeneratedEfficiency ; 

Propulsion.PropellerPower = AUVCharacs.PropellerPower ; 
Propulsion.PropulsiveEfficiency = AUVCharacs.PropulsiveEfficiency;
Propulsion.ShaftPower = AUVCharacs.PropellerPower/AUVCharacs.ShaftEfficiency;
Propulsion.ShaftEfficiency = AUVCharacs.ShaftEfficiency ;
Propulsion.PropulsionEnergy = AUVCharacs.PropulsionEnergy; 

Ballast.MaxAvailableVolume = ExcessVolume ;
Ballast.MaxWeight = MaxBallastWeight;
Ballast.MinRequiredWeight = ExcessWeight; 

AUV = {Overall,TurbineSystems,HullSystems,Generator,Propulsion,Ballast}; 




