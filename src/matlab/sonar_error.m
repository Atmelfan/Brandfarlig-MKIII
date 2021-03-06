D = 0.07;%=7cm
vs = 343.2;%=300m/s
a = linspace(-40, 40, 8+1);
et = linspace(-100, 100, 100+1);
ec = 1+linspace(-10, 10, 100+1)*0.01;

close all;
grid on;
hold on;
title('Timing error: T + T_err')
xlabel('Timing error[µs]');
ylabel('Result[degrees]');
i = 1;
for dt = D*sin(pi*a/180)/vs
   ae = asin((dt + et*(10^-6))*vs/D); 
   plot(et, 180*ae/pi);
   legends{i}=num2str(a(i));
   i=i+1;
end
legend(legends);

figure;
grid on;
hold on;
title('Clock error: T*C_err');
xlabel('Clock error[%]');
ylabel('Result[degrees]');
i = 1;
for dt = D*sin(pi*a/180)/vs
   ae = asin((dt*ec)*vs/D); 
   plot(ec, 180*ae/pi);
   legends{i}=num2str(a(i));
   i=i+1;
end
legend(legends);