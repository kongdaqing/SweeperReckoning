subplot(3,1,1);
plot(att.t,att.roll,'r',att.t,att.aroll,'b');
grid on;
subplot(3,1,2);
plot(att.t,att.pitch,'r',att.t,att.apitch,'b');
grid on;
subplot(3,1,3);
plot(att.t,att.yaw,'r');
grid on;