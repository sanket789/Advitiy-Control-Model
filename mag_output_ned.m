LLA = csvread('LLA.csv');
N = length(LLA);
B_NED = zeros(N,4);
for i=1:N
    T = LLA(i,1);
    LAT = 90-LLA(i,2);
    LONG = LLA(i,3);
    ALT = LLA(i,4)/1000;  % altitude in Km

    Dyear = 2018 + T/86400/365;
    B_NED(i,1) = T;
    B_NED(i,2:4) = igrf(ALT, LAT, LONG, Dyear, 13);
    
    if mod(i,round(N/100)) == 0 
        fprintf('done %i\n',round(100*i/N))
    end
   
end

dlmwrite('mag_output_ned.csv',B_NED)
fprintf('done ')