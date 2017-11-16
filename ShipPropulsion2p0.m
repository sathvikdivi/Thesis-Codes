clear
clc
close all

rho = 1025;
Ehouseday = 30; %kWh

data55v5 = csvread('C:\users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\ShipPropulsionEnergyEstimates55v5_Harvald_CSV_rev2.csv',1,0);
data55v8 = csvread('C:\users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\ShipPropulsionEnergyEstimates55v8_Harvald_CSV_rev2.csv',1,0);

figure(1)
plot(data55v5(:,4)./1000,data55v5(:,6),'Linewidth',1.25)
hold on
plot(data55v8(:,4)./1000,data55v8(:,6),'Linewidth',1.25)
xlabel('Cargo Weight(tons)')
ylabel('Power(KW)')
title('Ship Brake Power vs Cargo Weight')
grid on
leg = [{'Speed - 5m/s'},{'Speed - 8m/s'}];
legend(leg,'Location','northwest')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 13);   
set(gcf, 'Color', [1, 1, 1]);
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\plots\';
filename = ['ShipBrakePowervsCargoWeight'];
saveas(gcf,fullfile(fpath,filename),'jpeg');

deltav5 = (data55v5(:,1)./rho).*1000 ;
deltav8 = (data55v8(:,1)./rho).*1000 ;
L = round((deltav5./0.01238).^(1/3),0) ;
L8 = round((deltav8./0.01238).^(1/3),0);

figure(2)
plot(data55v5(1:17,3),L(1:17),'Linewidth',1.25)
xlabel('Cargo Weight(tons)')
ylabel('Ship Length (m)')
title('Ship Size vs Cargo Weight')
grid on
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 13);   
set(gcf, 'Color', [1, 1, 1]);
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\plots\';
filename = ['ShipSizevsCargoWeight'];
saveas(gcf,fullfile(fpath,filename),'jpeg');

figure(3)
plot(data55v5(1:20,5),L(1:20),'Linewidth',1.25)
xlabel('Cargo Energy(MWh)')
ylabel('Length (m)')
title('Ship Size vs Cargo Energy')
grid on
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 13);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Thesis\Pictures';
filename = ['ShipSizevsCargoEnergy'];% 
saveas(gcf,fullfile(fpath,filename),'jpeg');
axis([0 60 0 40])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\plots\';
filename = ['ShipSizevsCargoEnergy_zoomed'];
saveas(gcf,fullfile(fpath,filename),'jpeg');
NoH = (data55v5(1:20,5)./30)*1000 ;

figure(4)
plot(NoH(1:20),L(1:20),'Linewidth',1.25);
ax = [{'No. of Houses'},{'(Alt: No. of Days)'}];
xlabel(ax)
ylabel('Ship Length(m)')
a = [{'Ship Size vs Number of Houses whose energy consumption per day can be met'},{'(Alt: Ship Size vs Number of days a house can be powered for)'}];
title(a)
grid on
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 13);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\plots\';
filename = ['ShipSizevsHouses'];
saveas(gcf,fullfile(fpath,filename),'jpeg');

figure(5)
plot(data55v5(1:26,5),data55v5(1:26,12),'Linewidth',1.25)
hold on
plot(data55v5(1:26,5),data55v5(1:26,13),'Linewidth',1.25)
plot(data55v5(1:26,5),data55v5(1:26,14),'Linewidth',1.25)
plot(data55v5(1:26,5),data55v5(1:26,15),'Linewidth',1.25)
xlabel('Cargo Energy(MWh)')
ylabel('Propulsion Energy(MWh)')
title('Propulsion Energy vs Cargo Energy - Ship speed 5m/s')
grid on
leg = [{'Stillwater'},{'1mps Current'},{'2mps Current'},{'3mps Current'}];
legend(leg,'Location','southeast');
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 13);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\plots\';
filename = ['PropEnergyvsCargoEnergyv5'];
saveas(gcf,fullfile(fpath,filename),'jpeg');
axis([0 200 0 10])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\plots\';
filename = ['PropEnergyvsCargoEnergyv5_zoomed'];
saveas(gcf,fullfile(fpath,filename),'jpeg');

figure(6)
plot(data55v8(1:17,5),data55v8(1:17,12),'Linewidth',1.25)
hold on
plot(data55v8(1:17,5),data55v8(1:17,13),'Linewidth',1.25)
plot(data55v8(1:17,5),data55v8(1:17,14),'Linewidth',1.25)
plot(data55v8(1:17,5),data55v8(1:17,15),'Linewidth',1.25)
xlabel('Cargo Energy(MWh)')
ylabel('Propulsion Energy(MWh)')
title('Propulsion Energy vs Cargo Energy - Ship speed 8m/s')
grid on
leg = [{'Stillwater'},{'1mps Current'},{'2mps Current'},{'3mps Current'}];
legend(leg,'Location','southeast');
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 13);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\plots\';
filename = ['PropEnergyvsCargoEnergyv8'];
saveas(gcf,fullfile(fpath,filename),'jpeg');

close all

cd('C:\users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\AUV\');

HullLengths = L(1:20);
HullL2DRatio = 7;
HullDias = HullLengths./HullL2DRatio ;
HullProps = [HullLengths,HullDias];

% TurbDias = 20;
AUVDes = [];
for i = 1:length(HullLengths)
    Lh = HullProps(i,1);
    Dh = HullProps(i,2);
    TurbDias = Dh;
    Chars = AUVCase(Lh,Dh,TurbDias);
    AUVDes = vertcat(AUVDes,Chars);
end

cd('C:\users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\');

figure(7)
plot(L(1:20),data55v5(1:20,3),'Linewidth',1.75)
hold on
plot(HullLengths,0.001*AUVDes(:,4),'LineWidth',1.75)
xlabel('Length (m)')
ylabel('Cargo Weight(tons)')
title('Cargo Weight vs Length')
leg = [{'Ship'},{'MUTS'}];
legend(leg,'location','southeast')
grid on
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1]);
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\plots\';
filename = ['ShipCase_CargoWeightvsLength'];
saveas(gcf,fullfile(fpath,filename),'jpeg');

figure(8)
plot(L(1:20),data55v5(1:20,5),'Linewidth',1.25);
xlabel('Length (m)')
ylabel('Cargo Energy(MWh)')
hold on
plot(HullLengths,AUVDes(:,6),'LineWidth',1.75)
title('Cargo Energy vs Length')
leg = [{'Ship'},{'MUTS'}];
legend(leg,'location','southeast')
grid on
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1]);
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\plots\';
filename = ['ShipCase_CargoEnergyvsLength'];
saveas(gcf,fullfile(fpath,filename),'jpeg');

figure(9)
plot(AUVDes(:,6),AUVDes(:,7),'LineWidth',1.75)
grid on
xlabel('Cargo Energy (MWh)')
ylabel('Propulsion Energy (MWh)')
hold on
plot(data55v5(1:26,5),data55v5(1:26,12),'Linewidth',1.75)
xlabel('Cargo Energy(MWh)')
ylabel('Propulsion Energy(MWh)')
title('Propulsion Energy vs Cargo Energy')
% title('Propulsion Energy vs Cargo Energy - Ship speed 5m/s')
grid on
leg = [{'MUTS'},{'Ship'}];
legend(leg,'Location','northeast');
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\plots\';
filename = ['ShipCase_CargoEnergyvsPropulsionEnergy'];
saveas(gcf,fullfile(fpath,filename),'jpeg');

close all

ECS = data55v5(1:26,5); % ship cargo
EPS = data55v5(1:26,14); % ship propulsion
k_ship = EPS./(5)^2;
EPSnew = k_ship.*1.5^2;
P = polyfit(ECS,EPSnew,3); % polynomial that gives EPS as a function of ECS

lengths = [6:2:10];

for i = 1:length(lengths)
    [temp1,temp2,temp3] = Propcharts_ShipCase(lengths(i));
    Des{i} = temp1;
    Des_Props_Hull{i} = temp2;
    Des_Props_Wing{i} = temp3;
end

TurbDia = [1:0.5:5];
HullDia = [0.5:0.5:5];

for i = 1:length(lengths)
    ShipDes{i}(:,1) = Des{i}(:,1);
    ShipDes{i}(:,2) = Des{i}(:,2);
    ShipDes{i}(:,3) = Des{i}(:,7);
    ShipDes{i}(:,4) = polyval(P,ShipDes{i}(:,3));
    ShipDes{i}(:,5) = ShipDes{i}(:,3) - ShipDes{i}(:,4);  
end

LD = 6;

lines = {'-','--','-.'};
clrs = [0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 0.5 0.5;1 0.5 0;0.5 0.5 0.5;0.5 0.5 1];

figure(1)

for j = 1:length(lengths)
    LD = lengths(j);
    for i = 1:length(HullDia)
        pos = find(ShipDes{j}(:,2) == HullDia(i));
        plot(ShipDes{j}(pos,1),ShipDes{j}(pos,5),'Color',clrs(i,:),'LineWidth',1.75);
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
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\plots\';
    filename = ['ShipCase_DelvEnergyvsTurbDia_LD',num2str(LD),'_v2'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');

    figure(2)
    for i = 1:length(HullDia)
        pos = find(ShipDes{j}(:,2) == HullDia(i));
        plot(ShipDes{j}(pos,1),1000*(ShipDes{j}(pos,5)./30),'Color',clrs(i,:),'LineWidth',1.75);
        hold on
    end
    for z = 1:length(HullDia)
        leg(z) = cellstr(['D_{h} = ',num2str(HullDia(z))]);
    end
    grid on
    xlabel('Turbine Diameter (m)');
    ylabel('House Days');
    xlim([min(TurbDia) max(TurbDia)]);
    yLimits = ylim;
    ylim([0 yLimits(2)*1.2])
    caption = ['House Days vs Turbine Diameter ',' (L/D = ',num2str(LD),')'];
    title(caption)
    legend(leg,'Location','Bestoutside')
    set(gca, 'FontName', 'Calibri');
    set(gca, 'FontSize', 17);   
    set(gcf, 'Color', [1, 1, 1])
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\plots\';
    filename = ['ShipCase_HouseDaysvsTurbDia_LD',num2str(LD),'_v2'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');

    close all

    clear leg;
    hulldia = [];
    figure(3)
    for i = 1:length(HullDia)
        if HullDia(i) == 0.5
            continue;
        elseif HullDia(i) == 1.0
            continue;
        else
            pos = find(ShipDes{j}(:,2) == HullDia(i));
            HDays = 1000*(ShipDes{j}(pos,5)./30);
            GST = Des{j}(pos,8);
            II = HDays./GST;
            plot(ShipDes{j}(pos,1),II,'Color',clrs(i,:),'LineWidth',1.75);
            hold on
            hulldia = horzcat(hulldia,HullDia(i));
        end
    end
    for z = 1:length(hulldia)
        leg(z) = cellstr(['D_{h} = ',num2str(hulldia(z))]);
    end
    grid on
    xlabel('Turbine Diameter (m)');
    ylabel('No. of Houses Powered');
    xlim([min(TurbDia) max(TurbDia)]);
    yLimits = max(II);
    ylim([0 yLimits*1.2]);
    caption = [{'No. of Houses powered for GS Time'}, {['vs Turbine Diameter ',' (L/D = ',num2str(LD),')']}];
    title(caption)
    legend(leg,'Location','Bestoutside')
    set(gca, 'FontName', 'Calibri');
    set(gca, 'FontSize', 17);   
    set(gcf, 'Color', [1, 1, 1])
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\plots\';
    filename = ['ShipCase_NumberofHousesvsTurbDia_LD',num2str(LD),'_v2'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
    close all

end
