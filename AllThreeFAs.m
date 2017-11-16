
clear
clc

% Constants
global rho_seawater nu LiBattGravDensity  LiBattVolDensity Buffers;
rho_seawater = 1025;
nu = 1.004e-6; 
LiBattGravDensity = 250; % wh/kg
LiBattVolDensity = 300; % wh/L

% Fixed Inputs
v_rated = 2; % rated speed 
v_AUV = 3; % cruise speed
v_AUVMax = 4; % max AUV speed 
v_current = 0.2; % current speed
% Dt = 3; 
% % HullDias = [0.5:0.5:5];
% HullDias = 1.25;
% LD = 10.7./HullDias;
distance = [250000,60000,60000];
MatName = 'SS'; 
RatedDepth = 200; % m, max operating depth
SF = 1.5; % safety factor for material failure
energySF = 1.2; % hotel load energy buffer
volumeSF = 1.3; % volume buffer 
weightSF = 1.2; % weight buffer
Buffers.StructuralSafetyFactor = SF;
Buffers.EnergySafetyFactor = energySF;
Buffers.CabinVolumeSafetyFactor = volumeSF;
Buffers.WeightSafetyFactor = weightSF;
PropulsiveEfficiency = 0.76 ; %  \eta_h * \eta_B (hull eff * behind hull propeller efficiency)
ShaftEfficiency = 0.98; % shaft efficiency
MotorEfficiency = 0.95; % motor efficiency
GeneratorEfficiency = 0.9;  % generator efficiency
GenGearBoxEfficiency = 0.95; % gearbox efficiency

% Other Subssytem Fixed Parameters
ri = 1.07; % inlet area ratio
re = 1.47; % exit area ratio
theta_diff = 20; % diffuser angle
Cp = 0.5; % power coefficient for dts
Cd_duct = 0.2; % drag coefficient for duct
Cd_turbine = 0.69; % turbine drag coefficient
Cd_dtsmoving = 0.30; % drag coefficient for moving dts 
Cd_dtsoperating = 0.61; % drag coefficient for operating dts

% cases = [1,2,3]; 
cases = 3;

path = 'C:\users\sdivi\Documents\AUV\Design Parameters';
file = 'FACase3_Designs';

Designs = csvread(fullfile(path,[file,'.csv']),1,0,[1 0 10 8]);
[m,n] = size(Designs);

for i = 1:m
    Dt = Designs(i,1);
    HullDias = Designs(i,2);
    HullLength = Designs(i,3);
    LD = Designs(i,4);
    Des{i} = [Dt,HullDias,HullLength,LD];
end

HullMatProp = HullMaterialProps(MatName);

for i = 1:m
%     if (cases(i) == 2)
%         CaseName = 'Case 2';
%     elseif (cases(i) == 3);
        CaseName = 'Case3';
        nf = 2.5;
        na = 2;
        Percentages = [0.2,0.4,0.6];
        RangeDistance = distance(3);
        cd('C:\users\sdivi\Documents\AUV\Matlab\Three Cases FA\Case 3\')
        for j = 1:length(HullDias)
            Dh = HullDias(j);
            L = LD*Dh;
            [xtot,ytot,Lengths] = HullForm(L,Dh,Percentages,na,nf);
            Lpmb = Lengths(1);
            Lf = Lengths(2);
            La = Lengths(3);

            [WS,V,K1,K2] = HullAreaVolumeEstimator(nf,na,La,Lf,Lpmb,L,Dh);
            Cds = HullDragEstimator(v_AUV,WS,V,L,Dh,K2);
            
            At = 0.25*pi*Dt^2;
            Turbine = struct('Name','Ducted Turbine System','RotorDiameter',Dt,'InletAreaRatio',1.07,'ExitAreaRatio',1.47,'DiffuserAngle',20,'RotorArea',At,'DuctDrag',Cd_duct,'TurbineDrag',Cd_turbine,'OperatingDrag',Cd_dtsoperating,'MovingDrag',Cd_dtsmoving,'PowCoeff',Cp);
            Hull = struct('Name','Hull','HullDiameter',Dh,'Length',L,'LDRatio',LD,'AftLength',La,'FrontLength',Lf,'ParallelMidBodyLength',Lpmb,'WettedArea',WS,'EnclosedVolume',V,'DragCoeff',Cds);
            Misc = struct('Name','Miscellanous','AUVCruiseSpeed',v_AUV,'AUVMaxSpeed',v_AUVMax,'GSRatedSpeed',v_rated,'GSDeepCurrentSpeed',v_current,'GSRange',RangeDistance,'PropulsiveEfficiency',PropulsiveEfficiency,'ShaftEfficiency',ShaftEfficiency,'GeneratedEfficiency',GeneratorEfficiency,'RatedDepth',RatedDepth);

            [E_Prop,PropPower,t_tot,Wing] = AUVPropulsion(Turbine,Hull,Misc);

            AUVCharacs = struct();
            AUVCharacs.PropellerPower = PropPower; % kW
            AUVCharacs.PropulsiveEfficiency = Misc.PropulsiveEfficiency;
            AUVCharacs.ShaftEfficiency = Misc.ShaftEfficiency ; 
            AUVCharacs.GeneratorEfficiency = GeneratorEfficiency;
            AUVCharacs.GenGearBoxEfficiency = GenGearBoxEfficiency;
            AUVCharacs.PropulsionEnergy = E_Prop ; % kWh
            AUVCharacs.GSRangeTime = t_tot ; % hours
            AUVCharacs.GSRangeDistance = Misc.GSRange ; % m

            Misc.RatedPressure = rho_seawater*9.81*Misc.RatedDepth ; % Pa
            AUVCharacs.RatedPressure = Misc.RatedPressure ; % Pa 
            AUVCharacs.RatedDepth = Misc.RatedDepth ;  % m

            Hull.Thickness = round(RatedThickness(Hull,Misc,HullMatProp),3);
            
            if (Hull.Thickness == 0)
                Hull.Thickness = round(RatedThickness(Hull,Misc,HullMatProp),4);
            end
            
            AUVCharacs.HullThickness = Hull.Thickness;
    
            ID = Dh - 2*AUVCharacs.HullThickness ; 
            IL = L - 2*AUVCharacs.HullThickness ;

            [xItot,yItot,ILengths] = HullForm(IL,ID,Percentages,na,nf);

            ILpmb = ILengths(1);
            ILf = ILengths(2);
            ILa = ILengths(3);

            [wstemp,IV,IK1,IK2] = HullAreaVolumeEstimator(nf,na,ILa,ILf,ILpmb,IL,ID);

            Hull.BodyWeight = HullMatProp.Density*(V - IV); % kg
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
            AUVCharacs.ActualPower = TotTurbPow*AUVCharacs.GeneratorEfficiency*AUVCharacs.GenGearBoxEfficiency; % kW

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
            Overall.Displacement = DisplacedMass;

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
            
            Battery.Weight = AUVCharacs.BatteryWeight ;
            Battery.Volume = AUVCharacs.BatteryVolume;
            Battery.Energy = AUVCharacs.OnboardStoredEnergy;
            
            Wing = Wing;
            
            Characs(j,:) = [Dh,L,Overall.AUVGeneratorPower,Battery.Energy,Overall.TotalWeight,Overall.Displacement,Propulsion.PropellerPower];
            Days = [1:5:100]';
            EnergyGenPrototype(:,j) = 0.001*Overall.AUVGeneratorPower*24.*Days;  
            HouseDays(:,j) =1000.*(EnergyGenPrototype(:,j)./30);  
%         end
    end
end

% 
% 
% figure(1)
% plot(Days,EnergyGenPrototype,'LineWidth',1.75);
% grid on
% xlabel('Days of Operation')
% ylabel('Energy (MWh)')
% title('Energy sent to the Mainland')
% set(gca, 'FontName', 'Calibri');
% set(gca, 'FontSize', 18);   
% set(gcf, 'Color', [1, 1, 1])
% fpath = 'C:\Users\sdivi\Documents\AUV\Thesis\Pictures';
% filename = ['AUVGenPrototypeTheseus_EnergyvsGSTime'];
% saveas(gcf,fullfile(fpath,filename),'jpeg');
% 
% figure(2)
% plot(Days,HouseDays,'LineWidth',1.75);
% grid on
% xlabel('Days of Operation')
% ylabel('House-Days')
% title('AUV Generator Impact Index')
% set(gca, 'FontName', 'Calibri');
% set(gca, 'FontSize', 18);   
% set(gcf, 'Color', [1, 1, 1])
% fpath = 'C:\Users\sdivi\Documents\AUV\Thesis\Pictures';
% filename = ['AUVGenPrototypeTheseus_HouseDaysvsGSTime'];
% saveas(gcf,fullfile(fpath,filename),'jpeg');

