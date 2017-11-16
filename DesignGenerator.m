function [Designs] = DesignGenerator(v1,v2,v3)

Dt = [0.5:0.5:5];
Dh = [0.5:0.5:5];
LbyD = [6:1:10];
GraveYard_Wt = [];
GraveYard_Energy = [];
GraveYard_Time = [];
Stage1Des = [];
Stage2Des = [];
Stage3Des = [];

v_rated = v1;
v_AUV = v2;
v_current = v3;

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
           wt_cargo_rm{k}(i,j) = BallastProps(Turb,Hull,v1); % kg
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
    wt_cargobatt(i) = BallastProps([Dt,ri,re,theta_diff,Cd,Ct],[Dh,Dh*LbyD,C_B,Cd_hull],v1); % kg
    E_cargo(i) = 250*wt_cargobatt(i)*0.001*0.001 ; % MWh
    [EProp(i),ESurp(i),E_cargocheck(i),time_opr_days(i)] = PropEnergyTot([Dt,Dh,LbyD],[ri,re,theta_diff,Cp,Cd,Ct],[v_rated,v_AUV,v_current]);
    P_AUV(i) = AUVPowRating(Dt,re,Cp,v_rated);
    if ESurp(i) <= 1
        GraveYard_Energy = vertcat(GraveYard_Energy,[Dt,Dh,LbyD]);
    else
        Stage2Des = vertcat(Stage2Des,[Dt,Dh,LbyD,P_AUV(i),E_cargo(i),time_opr_days(i),EProp(i),ESurp(i)]); 
    end
end

[m2,n2] = size(Stage2Des);

for i = 1:m2
    Des = Stage2Des(i,:);
    if Des(6) >= 366
        GraveYard_Time = vertcat(GraveYard_Time,Des(1),Des(2),Des(3));
    else
        Stage3Des = vertcat(Stage3Des,Des);
    end
end

EnConHouse = 0.001*30*Stage3Des(:,6); % MWh
ImpInd1 = Stage3Des(:,8)./EnConHouse ; % Number of Houses powered for the same time that the AUV spends in the GS
ImpInd2 = (Stage3Des(:,8)*1000)./30 ; % Number of Houses powered for a day or number of days one house can be powered for

TempDesigns = [Stage3Des,ImpInd1,ImpInd2];
TempDesigns(:,11) = TempDesigns(:,10)./TempDesigns(:,6);

[mt,nt] = size(TempDesigns);

Designs = [];
GraveYard_final = [];

for i = 1:mt
    Des = TempDesigns(i,:);
    if Des(6) > 91
        GraveYard_final = vertcat(GraveYard_final,[Des(1),Des(2),Des(3)]);
    else
        Designs = vertcat(Designs,Des);
    end
end

Designs = round(Designs,2);

Designs = array2table(Designs);

end



