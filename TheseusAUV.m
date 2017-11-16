% Code to calculate parameters if the AUV design was that of Theseus AUV

clear
clc

Dt = [0.5:0.5:5];
Dh = 1.28;
Lh = 10.8;
LbyD = Lh/Dh;

v_rated = 2;
v_AUV = 3;
v_current = 0.2;

ri = 1.07;
re = 1.47;
theta_diff = 20;
Cp = 0.5;
Cd = 0.2;
C_B = 0.7;
Ct = 0.45;
Cd_hull = 0.25;

for i = 1:length(Dt)
    Turb = [Dt(i),ri,re,theta_diff,Cd,Ct];
    Hull = [Dh,Lh,C_B,Cd_hull];
    wt_cargo_rm(i) = BallastProps(Turb,Hull,v_rated);
    [EProp_Theseus(i),ESurp_Theseus(i),E_cargo_Theseus(i),time_opr_days_Theseus(i)] = PropEnergyTot([Dt(i),Dh,LbyD],[ri,re,theta_diff,Cp,Cd,Ct],[v_rated,v_AUV,v_current]); 
end


EnConHouse = 0.001*30*time_opr_days_Theseus ; % MWh
ImpIndex1 = ESurp_Theseus./EnConHouse ; 
ImpIndex2 = 1000*(ESurp_Theseus./30) ;
Data = [Dt',E_cargo_Theseus',EProp_Theseus',ESurp_Theseus',time_opr_days_Theseus',EnConHouse',ImpIndex1',ImpIndex2'];
Data = round(Data,2);
Data = array2table(Data);

X = ['TheseusAUVGenerator_v',num2str(v_rated),'_DesignCases'];
fpath = 'C:\Users\sdivi\Documents\AUV\Design Parameters';
filename = [X,'.csv'];
writetable(Data,fullfile(fpath,filename));

