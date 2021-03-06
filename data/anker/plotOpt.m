function  plotOpt(ankerData,opt_begin,opt_end)
%PLOTODOM 此处显示有关此函数的摘要
%   此处显示详细说明
time_s = ankerData.time_s(opt_begin:opt_end);
opt_sumx = ankerData.opt_sumx(opt_begin:opt_end);
opt_sumy = ankerData.opt_sumy(opt_begin:opt_end);
theta = getTheta(ankerData,opt_begin,opt_end);
len = length(opt_sumx);

opt_sumxx = zeros(len,1);
opt_sumyy = zeros(len,1);
opt_sumxx(1) = opt_sumx(1);
opt_sumyy(1) = opt_sumy(1);
for i = 2:len
    dx = opt_sumx(i) - opt_sumx(i-1);
    dy = opt_sumy(i) - opt_sumy(i-1);
    dxx = cos(theta)*dx + sin(theta)*dy;
    dyy = -sin(theta)*dx + cos(theta)*dy;
    opt_sumxx(i) = opt_sumxx(i-1) + dxx;
    opt_sumyy(i) = opt_sumyy(i-1) + dyy;
end

plot(time_s,opt_sumx,'g',time_s,opt_sumxx,'r');
hold on;
plot(time_s,opt_sumy,'b',time_s,opt_sumyy,'m');
grid on;

end

