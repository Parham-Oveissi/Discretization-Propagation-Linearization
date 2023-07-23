clc; clear; close all
m = 10; % mass > 0
k = 5; % spring constant > 0
b = 3; % damping coefficient ≥ 1
A = [0 1; -k/m -b/m]; 
B = [0; 1/m];
Force = 1;

C = [1 0];
D = 0;

dt = 0.2;
t = 0:dt:50;

u = Force; 



xinit = [1;0];


Dynamics_without_disturbance = @(t,x) A*x+B*u; % rhs of function
[tt,True_States] = ode45(Dynamics_without_disturbance, t, xinit);


mysys_continous = ss(A,B,C,D);
mysys_discrete = c2d(mysys_continous,dt,'zoh');

F = mysys_discrete.A; % State transition matrix
G = mysys_discrete.B; % Input/Control transition Matrix
H = mysys_discrete.C; % Measurement Matrix

Estimate = [1;0];

Estimates_Vec(:,1) = Estimate;

for i = 2:length(True_States)
    % Prediction step
    Estimate = F*Estimate + G*u;
    Estimates_Vec(:,i) = Estimate;
end

plot(t,True_States(:,1),'r',t,Estimates_Vec(1,:),'bo')
figure
plot(t,True_States(:,2),'r',t,Estimates_Vec(2,:),'bo')
