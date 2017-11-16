function W = wtDTS(Dt)

% a = [1.3;3.6;20.4];
% b = [10500;36000;150000];
% P = polyfit(a,b,2);
% W = polyval(P,Dt); % Kg

Dttemp = [1:1:5];
Wttemp = [0.3,0.7,1.69,4.20,6.84]; % tons, weight estimates from CAD models
WtEqn = polyfit(Dttemp,Wttemp,3); % coming up with equation model for estimating weight based on the above available data

W = 1000 * polyval(WtEqn,Dt); % kg


end