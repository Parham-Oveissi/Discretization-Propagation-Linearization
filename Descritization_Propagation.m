clc; clear; close all

syms Phi Theta Psi omega1 omega2 omega3
addpath './functions/'

dt = 0.1;
end_time = 20;
time = 0:dt:end_time;

IC = [0; 0; 0];

omega = [0; 0; 10];


[Time, Theta_Out] = ode45(@(Time, Theta) Euler_Angular_Velocity_ODE(Time, Theta, omega, 0), time, IC);


%% Kalman Filter Implementation
Estimate1 = [0; 0; 0]; % Initial state Estimate
Estimates_Vec1(1,:) = Estimate1;
[A_sym, B_sym] = local_linearizer(dt);

for kk = 2:length(Theta_Out)
    
    A = double(subs(A_sym,[Phi Theta Psi omega1 omega2 omega3], ...
    [Estimate1; omega]'));

    B = double(subs(B_sym,[Phi Theta Psi omega1 omega2 omega3], ...
    [Estimate1; omega]'));
    
    Estimate1 = A*Estimate1 + B*omega;

    Estimates_Vec1(kk,:) = Estimate1;
    
end

Estimate2 = [0; 0; 0]; % Initial state Estimate
Estimates_Vec2(1,:) = Estimate2;

for kk = 2:length(Theta_Out)
    Estimate2 = Estimate2 + dt*f(Estimate2,omega);

    Estimates_Vec2(kk,:) = Estimate2;
    
end

figure(1)
Animate_attitude(Time, Theta_Out, 'deg', 1e-10)
figure(2)
Animate_attitude(Time, Estimates_Vec1, 'deg', 1e-10)
figure(3)
Animate_attitude(Time, Estimates_Vec2, 'deg', 1e-10)

