function theta = getTheta(ankerData,opt_begin,opt_end)
%GETLINEDATA 此处显示有关此函数的摘要
%   此处显示详细说明
dx = ankerData.opt_sumx(opt_end) - ankerData.opt_sumx(opt_begin);
dy = ankerData.opt_sumy(opt_end) - ankerData.opt_sumy(opt_begin);
theta = atan2(dy,dx);
end

