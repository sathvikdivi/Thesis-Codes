
clear
close all
clc

cases = [1,2,3];

NumAUVs = 10;

cd('C:\users\sdivi\Documents\AUV\Matlab\All Cases - New\Case I\');
[Des1,Des1_Hull,Des1_Wings,HullCoords1] = FACaseI(3,2,14);

cd('C:\users\sdivi\Documents\AUV\Matlab\All Cases - New\Case II\');
[Des2,Des2_Hull,Des2_Wings,HullCoords2] = FACaseII(3,2,14);

cd('C:\users\sdivi\Documents\AUV\Matlab\All Cases - New\Case III\');
[Des3,HullCoords3,Des_Wt,Des_Vol] = FACaseIII(3,2,14);

cd('C:\users\sdivi\Documents\AUV\Matlab\All Cases - New\Case III');
[Des3b,HullCoords3b,Des_Wtb,Des_Volb] = FACaseIII(3,1.25,9);


data55v5 = csvread('C:\users\sdivi\Documents\AUV\Propulsion\ShipPropulsionEnergyEstimates55v5_HarvaldCSV.csv',1,0);
ECS = data55v5(1:26,5); % ship cargo
EPS = data55v5(1:26,14); % ship propulsion
P = polyfit(ECS,EPS,3); % polynomial that gives EPS as a function of ECS
EPSnew = (EPS./5^2)*1.5^2;
Pnew = polyfit(ECS,EPSnew,3);
    
% All the AUVs put together
E_SurpTot1 = NumAUVs*Des1(7);
E_CargoTot1 = NumAUVs*Des1(5);
E_PropTot1 = NumAUVs*Des1(6);
HPD_1 = (1000*E_SurpTot1/Des1(end))./30;

% With a Ship
E_ShipCargo = NumAUVs*Des2(7);
if NumAUVs == 1
    E_ShipProps = polyval(Pnew,E_ShipCargo);
else
    E_ShipProps = polyval(P,E_ShipCargo);
end
E_ShipSurp = E_ShipCargo - E_ShipProps;
HPD_2 = (1000*E_ShipSurp/Des2(end))./30;

% With a Cable
E_CableCargo = NumAUVs*Des3(6);
E_CableProps = NumAUVs*Des3(7);
E_CableSurp = NumAUVs*0.001*Des3(4)*24*(Des1(end) - Des3(end));
HPD_3 = (1000*E_CableSurp./30)./(Des1(end) - Des3(end));

clrs = [0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 0.5 0.5;1 0.5 0;0.5 0.5 0.5;0.5 0.5 1];

% figure(1)
% plot(HullCoords1(:,1),HullCoords1(:,2),'Color',clrs(1,:),'LineWidth',1.75);
% hold on
% plot(HullCoords2(:,1),HullCoords2(:,2),'Color',clrs(2,:),'LineWidth',1.75);
% plot(HullCoords3(:,1),HullCoords3(:,2),'Color',clrs(1,:),'LineWidth',1.75);
% grid on
% xlabel('Hull Length (m)');
% ylabel('Hull Diameter (m)');
% % leg = [{'Case I & II'},{'Case III'}];
% % legend(leg,'Location','northeast')
% title('MUTS Hull Sizes')
% axis equal
% set(gca, 'FontName', 'Calibri');
% set(gca, 'FontSize', 17);   
% set(gcf, 'Color', [1, 1, 1])
% fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case III\plots\';
% filename = ['AllCases_HullSizeComparison_New'];
% saveas(gcf,fullfile(fpath,filename),'jpeg');
% 
% 
% figure(2)
% Cases = [1,2,3];
% Surplus = [E_SurpTot1(1),E_ShipSurp(1),E_CableSurp(1)];
% bar(Cases,Surplus,0.25);
% set(gca,'XTickLabel',{'Case I','Case II','Case III'});
% text(Cases(1),Surplus(1),num2str(Surplus(1),'%0.1f'),...
%             'HorizontalAlignment','center',...
%             'VerticalAlignment','bottom','FontSize',17);
% text(Cases(2),Surplus(2),num2str(Surplus(2),'%0.1f'),...
%             'HorizontalAlignment','center',...
%             'VerticalAlignment','bottom','FontSize',17);
% text(Cases(3),Surplus(3),num2str(Surplus(3),'%0.1f'),...
%             'HorizontalAlignment','center',...
%             'VerticalAlignment','bottom','FontSize',17);
% grid on
% xlabel('Case Scenario');
% ylabel('Surplus Energy (MWh)');
% title('Surplus Energy Comparison - All Cases');
% set(gca, 'FontName', 'Calibri');
% set(gca, 'FontSize', 17);   
% set(gcf, 'Color', [1, 1, 1])
% fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case III\plots\';
% filename = ['AllCases_SurplusEnergyComparison_AUVS10'];
% saveas(gcf,fullfile(fpath,filename),'jpeg');
% 
% figure(3)
% Cases = [1,2,3];
% Surplus = [HPD_1,HPD_2,HPD_3];
% bar(Cases,Surplus,0.25);
% ylim([0 max(Surplus)+50])
% set(gca,'XTickLabel',{'Case I','Case II','Case III'});
% text(Cases(1),Surplus(1),num2str(Surplus(1),'%0.1f'),...
%             'HorizontalAlignment','center',...
%             'VerticalAlignment','bottom','FontSize',17);
% text(Cases(2),Surplus(2),num2str(Surplus(2),'%0.1f'),...
%             'HorizontalAlignment','center',...
%             'VerticalAlignment','bottom','FontSize',17);
% text(Cases(3),Surplus(3),num2str(Surplus(3),'%0.1f'),...
%             'HorizontalAlignment','center',...
%             'VerticalAlignment','bottom','FontSize',17);
% grid on
% xlabel('Case Scenario');
% ylabel('Houses Powered per Day');
% title(['Comparison of Impact - All Cases']);
% set(gca, 'FontName', 'Calibri');
% set(gca, 'FontSize', 17);   
% set(gcf, 'Color', [1, 1, 1])
% fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case III\plots\';
% filename = ['AllCases_HPDComparison_AUVS10'];
% saveas(gcf,fullfile(fpath,filename),'jpeg');


% figure(2)
% HPD(1) = (Des1(7)*1000./30)./Des1(8);
% HPD(2) = (Des2(9)*1000./30)./Des2(10);
% HPD(3) = (Des1(8) - Des3(8))*Des3(4)*24./30/Des1(8) ;
% Cases = [1,2,3];
% bar(Cases,HPD,0.3);
% ylim([0 max(HPD)+2]);
% text(Cases(1),HPD(1),num2str(HPD(1),'%0.1f'),...
%             'HorizontalAlignment','center',...
%             'VerticalAlignment','bottom','FontSize',17);
% text(Cases(2),HPD(2),num2str(HPD(2),'%0.1f'),...
%             'HorizontalAlignment','center',...
%             'VerticalAlignment','bottom','FontSize',17);
% text(Cases(3),HPD(3),num2str(HPD(3),'%0.1f'),...
%             'HorizontalAlignment','center',...
%             'VerticalAlignment','bottom','FontSize',17);
% grid on
% xlabel('Cases');
% ylabel('Houses Powerd Per Day');
% title('Comparison of Houses Powered Per Day');
% set(gca, 'FontName', 'Calibri');
% set(gca, 'FontSize', 17);   
% set(gcf, 'Color', [1, 1, 1])
% % fpath = 'C:\Users\sdivi\Documents\AUV\Thesis\Pictures';
% % filename = ['AllCases_HPDComparison_New'];
% % saveas(gcf,fullfile(fpath,filename),'jpeg');

figure(1)
plot(HullCoords3(:,1),HullCoords3(:,2),'Color',clrs(1,:),'LineWidth',1.75);
hold on
plot(HullCoords3b(:,1),HullCoords3b(:,2),'Color',clrs(5,:),'LineWidth',1.75);
grid on
xlabel('Hull Length (m)');
ylabel('Hull Diameter (m)');
leg = [{'Design A'},{'Design B'}];
legend(leg,'Location','northeast')
title('MUTS Hull Sizes (Case III)')
axis equal
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case III\plots\';
filename = ['CaseIII_HullSizeComparison_NewB'];
saveas(gcf,fullfile(fpath,filename),'jpeg');

figure(2)
HPDb(1) = (Des1(8) - Des3(8))*Des3(4)*24./30/(Des1(8) - Des3(8)) ;
HPDb(2) = (Des1(8) - Des3b(8))*Des3b(4)*24./30/(Des1(8) - Des3b(8)) ;
Designs = [1,2];

bar(Designs,HPDb,0.25);
ylim([0 max(HPDb)+5]);
text(Designs(1),HPDb(1),num2str(HPDb(1),'%0.1f'),...
            'HorizontalAlignment','center',...
            'VerticalAlignment','bottom','FontSize',17);
text(Designs(2),HPDb(2),num2str(HPDb(2),'%0.1f'),...
            'HorizontalAlignment','center',...
            'VerticalAlignment','bottom','FontSize',17);
set(gca,'XTickLabel',{'A','B'});
grid on
xlabel('Designs (Case III)');
ylabel('Houses Powered Per Day');
title('Comparison of Houses Powered Per Day');
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case III\plots\';
filename = ['CaseIII_HPDComparison_NewB'];
saveas(gcf,fullfile(fpath,filename),'jpeg');


figure(3)
SurpEng(1) = (Des1(8) - Des3(8))*Des3(4)*24*0.001 ;
SurpEng(2) = (Des1(8) - Des3b(8))*Des3b(4)*24*0.001 ;
Designs = [1,2];

bar(Designs,SurpEng,0.25);
ylim([0 max(SurpEng)+2]);
text(Designs(1),SurpEng(1),num2str(SurpEng(1),'%0.1f'),...
            'HorizontalAlignment','center',...
            'VerticalAlignment','bottom','FontSize',17);
text(Designs(2),SurpEng(2),num2str(SurpEng(2),'%0.1f'),...
            'HorizontalAlignment','center',...
            'VerticalAlignment','bottom','FontSize',17);
set(gca,'XTickLabel',{'A','B'});
grid on
xlabel('Designs (Case III)');
ylabel('Surplus Energy (MWh)');
title('Surplus Energy Comparison');
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\All Cases - New\Case III\plots\';
filename = ['CaseIII_SurpEngComparison_NewB'];
saveas(gcf,fullfile(fpath,filename),'jpeg');


