
function [xtot,ytot,Lengths] = HullForm(L,D,PercentArray,na,nf)
    
    alpha = PercentArray(1);
    Lpmb = alpha*L; 

    beta = PercentArray(2)*(1 - alpha);
    Lf = beta*L; 

    gamma = PercentArray(3)*(1 - alpha);
    La = gamma*L; 

    xf = [0:0.005:Lf]';
    xa = [0:0.05:La]';

    xpmb = [0:0.1:Lpmb]';
    ypmb = 0.5*D.*ones(length(xpmb),1);
    
    yf = 0.5*D*(1 - (xf./Lf).^nf).^(1./nf);
    ya = 0.5*D*(1 - (xa./La).^na);

    xf = xf + Lpmb;

    xtot = vertcat((fliplr(-xa'))',xpmb);
    xtot = vertcat(xtot,xf);
    xtot = xtot + La;
    ytot = vertcat((fliplr(ya'))',ypmb);
    ytot = vertcat(ytot,yf);

    xd = xtot;
    yd = -ytot;
    xd = (fliplr(xd'))';
    yd = (fliplr(yd'))';
    xtot = vertcat(xtot,xd);
    ytot = vertcat(ytot,yd);
    
    Lengths = [Lpmb,Lf,La];
    
end