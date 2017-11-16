clear
clc

Ship_L = 25; % ship length (m)
E_Surp_Tgt = 17; % MWh


Dt = [0.5:0.5:5];
Dh = [0.5:0.5:5];
LbyD = [6:1:10];
GraveYard_Wt = [];
GraveYard_Energy = [];
GraveYard_Time = [];
Stage1Des = [];
Stage2Des = [];
Stage3Des = [];
Stage4Des = [];

v_rated = 2;
v_AUV = 3;
v_current = 0.2;

ri = 1.07;
re = 1.47;
theta_diff = 20;
Cp = 0.5;
Cd = 0.2;
Ct = 0.45;
C_B = 0.7;
Cd_hull = 0.25;

for k = 1:length(LbyD)
   for i = 1:length(Dt)
       for j = 1:length(Dh)
           Turb = [Dt(i),ri,re,theta_diff,Cd,Ct];
           Hull = [Dh(j),Dh(j)*LbyD(k),C_B,Cd_hull];
           wt_cargo_rm{k}(i,j) = BallastProps(Turb,Hull,v_rated); % kg
           if wt_cargo_rm{k}(i,j) <= 0
               GraveYard_Wt = vertcat(GraveYard_Wt,[Dt(i),Dh(j),LbyD(k)]);
           else
               Stage1Des = vertcat(Stage1Des,[Dt(i),Dh(j),LbyD(k)]);
           end
        end
   end   
end

[m,n] = size(Stage1Des);

for i = 1:m 
   
    Des = Stage1Des(i,:);
    Dt = Des(1);
    Dh = Des(2);
    LbyD = Des(3);
    wt_cargobatt(i) = BallastProps([Dt,ri,re,theta_diff,Cd,Ct],[Dh,Dh*LbyD,C_B,Cd_hull],v_rated); % kg
    E_cargo(i) = 250*wt_cargobatt(i)*0.001*0.001 ; % MWh   
    P_AUV(i) = AUVPowRating(Dt,re,Cp,v_rated); %kW
    E_Prop(i) = 0.001*GeneratorAUVProp([Dt,Dh,LbyD],[ri,re,theta_diff,Cp,Cd],[v_AUV,v_current]); % MWh
    E_Surp(i) = E_cargo(i) - E_Prop(i); %MWh
    topr_days(i) = (1000*(E_cargo(i)./P_AUV(i)))/24;
    Stage2Des = vertcat(Stage2Des,[Des,P_AUV(i),E_cargo(i),E_Prop(i),E_Surp(i),topr_days(i)]);
    
end

[m2,n2] = size(Stage2Des);

for i = 1:m2
    Des = Stage2Des(i,:);
    if Des(6) >= 91
        GraveYard_Time = vertcat(GraveYard_Time,Des(1),Des(2),Des(3));
    else
        Stage3Des = vertcat(Stage3Des,Des);
    end
end



[m3,n3] = size(Stage3Des);

for i = 1:m3
    Des = Stage3Des(i,:);
    if Des(7) < round(E_Surp_Tgt,0)
        GraveYard_Energy = vertcat(GraveYard_Energy,Des(1),Des(2),Des(3));
    else
        Stage4Des = vertcat(Stage4Des,Des);
    end
end

Stage4Des = round(Stage4Des,2);
temp_designs = array2table(Stage4Des);
Designs = sortrows(temp_designs,'Stage4Des6','descend');


X = ['ShipAUV_GDC_vrated',num2str(v_rated),'_Esurp',num2str(E_Surp_Tgt)];
fpath = 'C:\Users\sdivi\Documents\AUV\Design Parameters';
filename = [X,'.csv'];
writetable(Designs,fullfile(fpath,filename));

