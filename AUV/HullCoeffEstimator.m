
function [Cpf,Cwsf,Cpa,Cwsa] = HullCoeffEstimator(nf,na)

    CpfData = [0.6667,0.75,0.8061,0.8455,0.8740,0.8944]';
    CwsfData = [0.7854,0.8452,0.8833,0.9089,0.9270,0.9476]';
    nfData = [2,2.5,3.0,3.5,4,4.5]';

    CpaData = [0.5333,0.5952,0.6429,0.6806,0.7111,0.7366]';
    CwsaData = [0.6667,0.7143,0.75,0.7778,0.80,0.8359]';
    naData = [2,2.5,3,3.5,4,4.5]';

    CpfPoly = polyfit(nfData,CpfData,4);
    Cpf = polyval(CpfPoly,nf);

    CwsfPoly = polyfit(nfData,CwsfData,4);
    Cwsf = polyval(CwsfPoly,nf);

    CpaPoly = polyfit(naData,CpaData,4);
    Cpa = polyval(CpaPoly,na);

    CwsaPoly = polyfit(naData,CwsaData,4);
    Cwsa = polyval(CwsaPoly,na);
    
end
