dis_x = 0.087;
dis_y = 0.101;
R = sqrt(dis_x*dis_x + dis_y*dis_y);
phi = atan2(dis_y,dis_x);
opt_sum_x = ankerData.opt_sumx;
opt_sum_y = ankerData.opt_sumy;
gz = ankerData.gz;
time_s = ankerData.time_s;
len = length(opt_sum_x);

opt_sum_xx = zeros(len,1);
opt_sum_yy = zeros(len,1);
opt_sum_xx(1) = opt_sum_x(1);
opt_sum_yy(1) = opt_sum_y(1);
for i = 2: len
   dt = time_s(i) - time_s(i-1);
   dtheta = gz(i) * dt;
   dL = R*dtheta;
   dphi = pi/2 - phi - dtheta/2;
   dx = -dL*cos(dphi);
   dy = dL*sin(dphi);
   opt_sum_xx(i) = opt_sum_xx(i-1) + dx;
   opt_sum_yy(i) = opt_sum_yy(i-1) + dy;
    
end

plot(time_s,opt_sum_x,'g',time_s,opt_sum_xx,'r');
hold on;
plot(time_s,opt_sum_y,'b',time_s,opt_sum_yy,'m');
