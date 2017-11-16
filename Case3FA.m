
close all
clear
clc

lengths = [6:2:10];

% Constants
global rho_seawater nu Aluminium Buffers rateddepth g CB_wing rho_wing Batteries ShuttleDistance Efficiencies rho_foam;
rho_seawater = 1025; % density of sea water
nu = 1.004e-6; % kinetmatic viscocity of water
rateddepth = 500; % m, max operating depth
g = 9.81; % accln due to gravity
ShuttleDistance = 60000; % 60 Km. travel locally around the Gulf Stream

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


for j = 1:m
    
            Dt = Des{j}(1);
            Dh = Des{j}(2);
            L = Des{j}(3);
            LD = Des{j}(4);
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

            [E_Prop,PropPower,t_tot,WingTemp] = AUVPropulsion(Turbine,Hull,Misc);
    
            Wing.ChordLength = WingTemp.ChordLength;
            Wing.ChordThickness = WingTemp.ChordThickness;
            Wing.ChordLengthIn = WingTemp.ChordLengthIn ;
            Wing.Length = WingTemp.Length;
            Wing.TotalLength = WingTemp.TotalLength;
            Wing.PlanformArea = WingTemp.PlanformArea ;
            Wing.Density = WingMatProp.Density;
            Wing.Material = WingMatProp.Material;
            Wing = WingWeightVolEstimator(Wing);
    
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
            Turbine.Volume = VolDTS(Turbine.RotorDiameter); % m^3

            HotelLoad = (Buffers.EnergySafetyFactor - 1) * AUVCharacs.PropulsionEnergy ; % kWh
            TotalEnergy = AUVCharacs.PropulsionEnergy + HotelLoad ; % kWh
            AUVCharacs.OnboardStoredEnergy = TotalEnergy ; % kWh 

            [BattWt,BattVol] = BattWtVolEstimator(AUVCharacs.OnboardStoredEnergy);
            AUVCharacs.BatteryWeight = BattWt ; % kg
            AUVCharacs.BatteryVolume = BattVol ; % m^3

            CabinVolume = Buffers.CabinVolumeSafetyFactor * AUVCharacs.BatteryVolume ; % m^3
            AUVCharacs.CabinVolume = CabinVolume ; % m^3
            AUVCharacs.HullInnerVolume = IV; % m^3

            TotalWeight = Buffers.WeightSafetyFactor*(AUVCharacs.BodyWeight + AUVCharacs.BatteryWeight + 2*Turbine.Weight + 2*Wing.Weight); % kg
            AUVCharacs.TotalWeight = TotalWeight;
            DisplacedVolume = 2*Turbine.Volume + Hull.Volume + 2*Wing.DisplacedVolume; % m^3
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
            
            Characs(j,:) = [Dt,Dh,L,Overall.AUVGeneratorPower,Battery.Energy,Overall.TotalWeight,Overall.Displacement,Propulsion.PropellerPower];
            Days = [1:5:100]';
            EnergyGenPrototype(:,j) = 0.001*Overall.AUVGeneratorPower*24.*Days;  
            HouseDays(:,j) =1000.*(EnergyGenPrototype(:,j)./30);  
            HousesPerDay(:,j) = HouseDays(:,j)./Days ;
            XYCoords{j} = vertcat([0,0],[xtot,ytot]);
            XYCoords{j} = vertcat(XYCoords{j},[0,0]);
            
            
            AUV{j} = struct('Overall',Overall,'TurbineSystems',TurbineSystems,'HullSystems',HullSystems,'Wing',Wing,'Generator',Generator,'Propulsion',Propulsion,'Ballast',Ballast,'Battery',Battery);
            
end


clrs = [0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 0.5 0.5;1 0.5 0;0.5 0.5 0.5;0.5 0.5 1];

% figure(1)
% 
% for z = 1:m
%     plot(Days,EnergyGenPrototype(:,z),'Color',clrs(z,:),'LineWidth',1.75);
%     hold on
% %     leg(z) = cellstr(['Des ',num2str(z),' (Turbine Dia = ',num2str(Characs(z,1)),')']);
%     leg(z) = cellstr(['Turbine Dia = ',num2str(Characs(z,1))]);
% end
% grid on
% xlabel('Days of Operation')
% ylabel('Energy (MWh)')
% title('Energy sent to the Mainland')
% legend(leg,'Location','northwest')
% set(gca, 'FontName', 'Calibri');
% set(gca, 'FontSize', 18);   
% set(gcf, 'Color', [1, 1, 1])
% % fpath = 'C:\Users\sdivi\Documents\AUV\Thesis\Pictures';
% % filename = ['Case3FA_EnergyvsGSTime'];
% % saveas(gcf,fullfile(fpath,filename),'jpeg');
% % fpath = 'C:\Users\sdivi\Documents\AUV\Thesis\Pictures';
% % filename = ['PrototypeCase_EnergyvsGSTime'];
% % saveas(gcf,fullfile(fpath,filename),'jpeg');
% 
% figure(2)
% for z = 1:m
%     HPD(z) = HousesPerDay(1,z);
%     Cx(z) = Characs(z,1);
% end
% bar(Cx,HPD,0.6);
% grid on
% xlabel('Turbine Diameter')
% ylabel('Number of Houses Powered Per Day')
% title('Number of Houses Powered Per Day vs Turbine Size')
% % for z = 1:m
% %     HPD(z) = [HousesPerDay(1,z)]; 
% % %     plot(Characs(:,2),HousesPerDay(:,z),'Color',clrs(z,:),'LineWidth',1.75);
% % %     hold on
% % %     leg(z) = cellstr(['Des ',num2str(z),' (Dt = ',num2str(Characs(z,1)),')']);
% % end
% % Cx = Characs(:,1)';
% % bar(Cx,HPD)
% % % grid on
% % % xlabel('Days of Operation')
% % % ylabel('House-Days')
% % % title('AUV Generator Impact Index')
% % % legend(leg,'Location','northeastoutside')
% set(gca, 'FontName', 'Calibri');
% set(gca, 'FontSize', 18);   
% set(gcf, 'Color', [1, 1, 1])
% % fpath = 'C:\Users\sdivi\Documents\AUV\Thesis\Pictures';
% % filename = ['Case3FA_HousesPerDaysvsTurbDia'];
% % saveas(gcf,fullfile(fpath,filename),'jpeg');
% 
% figure(3)
% % k = 2:2:m;
% k = m;
% for z = 1:length(k)
%     plot(XYCoords{k(z)}(:,1),XYCoords{k(z)}(:,2),'Color',clrs(z,:),'LineWidth',1.75);
%     hold on
%     leg(z) = cellstr(['Turbine Dia = ',num2str(Characs(k(z),1))]);  
% end
% grid on
% xlabel('Hull Length (m)')
% ylabel('Hull Diameter (m)')
% title('AUV Size')
% legend(leg,'Location','northwest')
% set(gca, 'FontName', 'Calibri');
% set(gca, 'FontSize', 18);   
% set(gcf, 'Color', [1, 1, 1])
% axis equal
% fpath = 'C:\Users\sdivi\Documents\AUV\Thesis\Pictures';
% filename = ['Case3FA_HullSizes'];
% saveas(gcf,fullfile(fpath,filename),'jpeg');
% fpath = 'C:\Users\sdivi\Documents\AUV\Thesis\Pictures';
% filename = ['PrototypeCase_HullSize'];
% saveas(gcf,fullfile(fpath,filename),'jpeg');
