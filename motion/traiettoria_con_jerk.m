%Traiettoria di prova
% clear all
% close all
% clc
% 
% v_max = 0.1; %m/s
% w_max= 1;  %rad/s 
% 
% x0 = [0.0; 0.0; 0.0];   %in forma x y z
% att0 = [0.0; 0.0; 0.0]; %in forma r p y
% 
% x1 = [0.0; 1.0; -3.0];
% att1 = [0.0; 0.0; 1.0];
% 
% x2 = [0.0; 2.0; -2.0];
% att2 = [0.0; 0.0; 3.0];
% 
% wp_pos = [x0, x1, x2];
% wp_att = [att0, att1, att2];
% 
% t1 = 0;
% t2 = t1 + norm(x1-x0) / v_max;
% t3 = t1 + t2 + norm(x2-x1) / v_max;
% 
% Ttot = (norm(x1-x0) / v_max) + (norm(x2-x1) / v_max)
% t_wps = [t1, t2, t3]
% 
% t_vec = 0:0.01:Ttot;
% t_end = t_vec(end);
% 
% [q, dq, ddq]  = quinticpolytraj(wp_pos, t_wps, t_vec);
% [eta, ~, ~] = quinticpolytraj(wp_att, t_wps, t_vec);
% 
% pd = timeseries(q, t_vec);
% vd = timeseries(dq, t_vec);
% ad = timeseries(ddq, t_vec);
% 
% etad = timeseries(eta, t_vec);
% figure(1)
% plot(t_vec, q)
% grid on



%Traiettoria di prova
clear all
close all
clc

v_max = 1.5; %m/s
w_max= 1;  %rad/s 

x0 = [0.0; 0.0; 0.0];   %in forma x y z
att0 = 0.0; %in forma y

x1 = [0.0; 1.0; -3.0];
att1 = 1.0;

x2 = [0.0; 2.0; -2.0];
att2 = 3.0;

%wp_pos = [x0, x1, x2];
%wp_att = [att0, att1, att2];

wp_pos = [ [0.0; 0.0; 0.0], [0.0; 1.0; -3.0], [0.0; 2.0; -2.0], [0.0; 1.0; -2.0] ];
wp_o = [ 0.0; 1.0; 3.0; 1.0 ];

tp(1) = 0;
Ttotp = 0;
for i=2:size(wp_pos,2)
    tp(i) = tp(i-1) + (norm(wp_pos(:, i)-wp_pos(:, i-1)) / v_max)
    Ttotp = sum(tp);

end

Ttotp = sum(tp);
t_vec = 0:0.01:Ttotp;
t_end = t_vec(end);

[q, dq, ddq]  = quinticpolytraj(wp_pos, tp, t_vec);
figure(1)
plot(t_vec, q)
grid on
figure(2)
%plot(t_vec, eta)
plot(t_vec, dq)
grid on




[eta, ~, ~] = quinticpolytraj(wp_att, t_wps, t_vec);

pd = timeseries(q, t_vec);
vd = timeseries(dq, t_vec);
ad = timeseries(ddq, t_vec);

etad = timeseries(eta, t_vec);
figure(1)
plot(t_vec, q)
grid on
figure(2)
%plot(t_vec, eta)
plot(t_vec, dq)
grid on
