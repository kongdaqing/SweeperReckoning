function plotOpt2Odom(ankerData,begin_index,end_index)
%PLOTOPT2ODOM 此处显示有关此函数的摘要
%   此处显示详细说明

disx = 0.087;
disy = 0.101;
T = [0.983945902559416 -0.024543804521931 disy;
     0.024543804521931 0.983945902559416 -disx] 
R = sqrt(disx*disx + disy*disy);
phi = atan2(disy,disx);
theta = 0.024939090492672;
m = 1/1.016;
opt_sumx = ankerData.opt_sumx(begin_index:end_index);
opt_sumy = ankerData.opt_sumy(begin_index:end_index);
odom_lpos = ankerData.odo_lpos(begin_index:end_index);
odom_rpos = ankerData.odo_rpos(begin_index:end_index);
gz = ankerData.gz(begin_index:end_index);

time_s = ankerData.time_s(begin_index:end_index);
len = length(opt_sumx);
odom_posx = zeros(len,1);
odom_posy = zeros(len,1);
opt2odom_posx = zeros(len,1);
opt2odom_posy = zeros(len,1);
k = zeros(len,1);

for i = 2:len
   dt = time_s(i) - time_s(i-1); 
   dangle = 0.5*(gz(i) + gz(i-1))*dt; 
   dopt_x = (opt_sumx(i) - opt_sumx(i-1));
   dopt_y = (opt_sumy(i) - opt_sumy(i-1));
   ds = [dopt_x;dopt_y;dangle];
   dodom = T*ds;
   
   
   dopt_x = m*dopt_x;
   dopt_y = m*dopt_y;
   
   dopt2odo1_x =  dopt_x * cos(theta) - dopt_y * sin(theta);
   dopt2odo1_y =  dopt_x * sin(theta) + dopt_y * cos(theta);
   
   
   dL = R*dangle;
   %dphi = phi + dangle/2;
   dphi =  phi;
   dBiasx = -dL*sin(dphi);
   dBiasy = dL*cos(dphi);
   dopt2odo_x = dopt2odo1_x - dBiasx;
   dopt2odo_y = dopt2odo1_y - dBiasy;
   
   opt2odom_posx(i) = opt2odom_posx(i-1) + dodom(1);%dopt2odo_x;
   opt2odom_posy(i) = opt2odom_posy(i-1) + dodom(2);%dopt2odo_y;
   
   dodom_l = odom_lpos(i) - odom_lpos(i-1);
   dodom_r = odom_rpos(i) - odom_rpos(i-1);
   dOdom = 0.5*(dodom_l + dodom_r);
   odom_posx(i) = odom_posx(i-1) + dOdom;
   
   if(odom_posx(i) > 0.01)
       k(i) = sqrt(opt2odom_posx(i)*opt2odom_posx(i) + opt2odom_posy(i)*opt2odom_posy(i))/abs(odom_posx(i));
   end
       
       
   
end

figure(1)
plot(time_s,odom_posx,'g',time_s,opt2odom_posx,'r');
hold on;
plot(time_s,odom_posy,'g',time_s,opt2odom_posy,'m');
grid on;

figure(2)
plot(time_s,k);
grid on;



end

