v_max = 90;
a_max = 180;
j_max = 360;
pi = 0;
pf = 17.691806;

[p,v,a,j,tf] = motion_profile(0, pi, pf, v_max, a_max, j_max);
i = 0;
time = 0:0.001:tf+1;
for t = time
    i = i+1;
    [p(i),v(i),a(i),j(i),tf] = motion_profile(t, pi, pf, v_max, a_max, j_max);
end
fprintf('Duration: %f\n', tf)

close all
figure
plot(time,p)
title('Position')
figure
plot(time,v)
title('Velocity')
figure
plot(time,a)
title('Acceleration')
figure
plot(time,j)
title('Jerk')

    
    