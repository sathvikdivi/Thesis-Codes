clear
clc

tic

v_rated = 2; % m/s
v_AUV = 3; % m/s
v_current = 0.2; % m/s

Designs = DesignGenerator(v_rated,v_AUV,v_current);

Designs_sortII2 = sortrows(Designs,'Designs10','descend');
Designs_sorttime = sortrows(Designs,'Designs6','ascend');
Designs_sorted = sortrows(Designs,'Designs11','descend');

X = ['GoodDesignCases_vrated',num2str(v_rated),'_vAUV',num2str(v_AUV)];
fpath = 'C:\Users\sdivi\Documents\AUV\Design Parameters';
filename = [X,'.csv'];
writetable(Designs_sorted,fullfile(fpath,filename));

toc









