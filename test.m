wpts = [0 45 15 90 45; 90 45 -45 15 90];
[q, qd, qdd, tvec, pp] = trapveltraj(wpts, 501);
subplot(2,1,1)
plot(tvec, q)
xlabel('t')
ylabel('Positions')
legend('X','Y')
subplot(2,1,2)
plot(tvec, qd)
xlabel('t')
ylabel('Velocities')
legend('X','Y')