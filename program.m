function fvec = GEARBOX_MATLAB(t, X, B1, J1, J2, J3, k, R2, R3, B)
 % fvec = transpose([f1, f2, f3]) = transpose(dot[Omega1,
 % theta1 - theta2, Omega3])
 % X = [Omega1, theta1 - theta2, Omega3]
 if t <= 0
 Tm = 1;
 else
 Tm = 2;
 end
 fvec(1) = Tm/J1 - B1*X(1)/J1 - k*X(2)/J1;
 fvec(2) = X(1) - R3*X(3)/R2;
 fvec(3) = (R3*R2*k*X(2) - B*(X(3)^3)*(R2^2) - 1.5*X(3)*(R2^2))/(J2*(R3^2) + J3*(R2^2)); 
fvec = transpose(fvec);
end



clear;
close all;
J1 = 2;
J2 = 1;
J3 = 0.5;
k = 1000;
B1 = 10;
B = 5;
R2 = 0.5;
R3 = 0.25;
tspan = [-10,10];
Xini = [0.06135,0.000387,0.1227];
[tv, Xvec] = ode45(@(t, X) GEARBOX_MATLAB(t, X, B1, J1, J2, J3, k, R2, R3, B), tspan, Xini);
A = Xvec(:,2);
N = Xvec(:,1) - R2*Xvec(:,3)/R3;
plot(tv, A,'r')
xlabel('time(s)')
ylabel('\theta_1 - \theta_2')
grid on

figure()
plot(tv,N,'r')
xlabel('time(s)')
ylabel('\omega_1 - \omega_2')
grid on

steady_state = mean(A(end-2:end))

maximum_overshoot = max(A)

t = tv > 5;
[pks, locs] = findpeaks(A);
Period = mean(diff(t(locs)))





function fvec = GEARBOX_MATLAB(t, X, B1, J1, J2, J3, k, R2, R3, B)
 % fvec = transpose([f1, f2, f3]) = transpose(dot[Omega1,
 % theta1 - theta2, Omega3])
 % X = [Omega1, theta1 - theta2, Omega3]
 if t <= 0
 Tm = 1;
 else
 Tm = 30;
 end
 fvec(1) = Tm/J1 - B1*X(1)/J1 - k*X(2)/J1;
 fvec(2) = X(1) - R3*X(3)/R2;
 fvec(3) = (R3*R2*k*X(2) - B*(X(3)^3)*(R2^2) - 1.5*X(3)*(R2^2))/(J2*(R3^2) + J3*(R2^2)); 
fvec = transpose(fvec);
end



clear;
close all;
J1 = 2;
J2 = 1;
J3 = 0.5;
k = 1000;
B1 = 10;
B = 5;
R2 = 0.5;
R3 = 0.25;
tspan = [-10,10];
Xini = [0.06135,0.000387,0.1227];
[tv, Xvec] = ode45(@(t, X) gearbox2_MATLAB(t, X, B1, J1, J2, J3, k, R2, R3, B), tspan, Xini);
A = Xvec(:,2);
N = Xvec(:,1) - R2*Xvec(:,3)/R3;
plot(tv, A,'r')
xlabel('time(s)')
ylabel('\theta_1 - \theta_2')
grid on

figure()
plot(tv, N,'r')
xlabel('time(s)')
ylabel('Vibratory Angular Speed (rad/2)')
grid on

steady_state2 = mean(A(end-2:end))

maximum_overshoot2 = max(A)

t = tv > 5;
[pks, locs] = findpeaks(A);
Period2 = mean(diff(t(locs)))
