clc; clear all;

%Hecho por Juan Carlos Hernández Ríos (A01740821)
%% Parámetros físicos del sistema

%Parametros lineales
m = 0.045; % Masa de dron [kg]
l = 0.058; % Longitud [m]
KT = 3.334e-8; %Constante de empuje [N/(rad^2/s^2)]
KQ = 1.058e-10; %Coeficiente de torque [Nm/(rad^2/s^2)]
km = 803.9; %Ganancia de proceso
Ixx = 3.0738e-5; %Momento xx [kgm^2]
Iyy = 3.0849e-5; %Momento yy [kgm^2]
Izz = 5.9680e-5; %Momento zz [kgm^2]
Jr = -5.897e-8; %Momento de inercia rotatorio del motor [kgm^2]

%Parametros matriciales
%Momento de inercia
I = [Ixx, 0, 0;
      0, Iyy, 0;
      0, 0,  Izz];

%Vector de peso
mg = [0;0;m*9.81];

%Condiciones iniciales

w0 = [0;0;0]; %Velocidad angular
theta0 = [0;0;0]; %Postura [phi, theta, psi]
v0 = [0;0;0]; %Velocidad lineal
d0 = [0;0;-2]; %Posición lineal [x,y,z]

%Movimiento en y
% u1 = 5000;
% u2 = 10000;
% u3 = 10000;
% u4 = 5000;

%Movimiento en X
% u1 = 5000;
% u2 = 5000;
% u3 = 10000;
% u4 = 10000;

%Empuje
% u1 = 5000;
% u2 = 5000;
% u3 = 5000;
% u4 = 5000;

%Rotacion en z
u1 = 10000;
u2 = 5000;
u3 = 10000;
u4 = 5000;


%% Simulink

t = 5;

sim("simDron.slx")
%% Gráficas
close all;

figure()
plot(d.Time, d.Data(:,1) , 'b','LineWidth',1)
hold on
plot(d.Time, d.Data(:,2) , 'r','LineWidth',1)
hold on
plot(d.Time, d.Data(:,3) , 'black','LineWidth',1)
title("Posicion lineal $d \left( t \right)$", 'Interpreter', 'latex')
xlabel("Tiempo $\left[ seg \right]$",'Interpreter', 'latex')
ylabel("Amplitud $\left[ m \right]$", 'Interpreter', 'latex')
legend('$x$','$y$','$z$','Interpreter', 'latex')
grid on;

figure()
plot(Th.Time, Th.Data(:,1) , 'b','LineWidth',1)
hold on
plot(Th.Time, Th.Data(:,2) , 'r','LineWidth',1)
hold on
plot(Th.Time, Th.Data(:,3) , 'black','LineWidth',1)
title("Postura $\Theta \left( t \right)$", 'Interpreter', 'latex')
xlabel("Tiempo $\left[ seg \right]$",'Interpreter', 'latex')
ylabel("Amplitud $\left[ rad \right]$", 'Interpreter', 'latex')
legend('$\phi$','$\theta$','$\psi$','Interpreter', 'latex')
grid on;


figure()
plot(d_p.Time, d_p.Data(:,1) , 'b','LineWidth',1)
hold on
plot(d_p.Time, d_p.Data(:,2) , 'r','LineWidth',1)
hold on
plot(d_p.Time, d_p.Data(:,3) , 'black','LineWidth',1)
title("Velocidad lineal $\dot{d} \left( t \right)$", 'Interpreter', 'latex')
xlabel("Tiempo $\left[ seg \right]$",'Interpreter', 'latex')
ylabel("Amplitud $\left[ \frac{m}{s} \right]$", 'Interpreter', 'latex')
legend('$\dot{x}$','$\dot{y}$','$\dot{z}$','Interpreter', 'latex')
grid on;

figure()
plot(Th_p.Time, Th_p.Data(:,1) , 'b','LineWidth',1)
hold on
plot(Th_p.Time, Th_p.Data(:,2) , 'r','LineWidth',1)
hold on
plot(Th_p.Time, Th_p.Data(:,3) , 'black','LineWidth',1)
title("Velocidad angular $\dot{\Theta} \left( t \right)$", 'Interpreter', 'latex')
xlabel("Tiempo $\left[ seg \right]$",'Interpreter', 'latex')
ylabel("Amplitud $\left[ \frac{rad}{s} \right]$", 'Interpreter', 'latex')
legend('$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$','Interpreter', 'latex')
grid on;

figure()
plot(M.Time, M.Data(:,1) , 'b','LineWidth',1)
hold on
plot(M.Time, M.Data(:,2) , 'r','LineWidth',1)
hold on
plot(M.Time, M.Data(:,3) , 'black','LineWidth',1)
title("Torque $\tau \left( t \right)$", 'Interpreter', 'latex')
xlabel("Tiempo $\left[ seg \right]$",'Interpreter', 'latex')
ylabel("Amplitud $\left[ Nm \right]$", 'Interpreter', 'latex')
legend('$\tau_x$','$\tau_y$','$\tau_z$','Interpreter', 'latex')
grid on;

figure()
plot(F.Time, F.Data(:,1) , 'b','LineWidth',1)
hold on
plot(F.Time, F.Data(:,2) , 'r','LineWidth',1)
hold on
plot(F.Time, F.Data(:,3) , 'black','LineWidth',1)
title("Fuerza $F \left( t \right)$", 'Interpreter', 'latex')
xlabel("Tiempo $\left[ seg \right]$",'Interpreter', 'latex')
ylabel("Amplitud $\left[ N \right]$", 'Interpreter', 'latex')
legend('$F_x$','$F_y$','$F_z$','Interpreter', 'latex')
grid on;

figure()
plot(w_m.Time, w_m.Data(:,1) , 'b','LineWidth',1)
hold on
plot(w_m.Time, w_m.Data(:,2) , 'r','LineWidth',1)
hold on
plot(w_m.Time, w_m.Data(:,3) , 'black','LineWidth',1)
hold on
plot(w_m.Time, w_m.Data(:,4) , 'green','LineWidth',1)
title("Velocidad angular de motores $\omega_m \left( t \right)$", 'Interpreter', 'latex')
xlabel("Tiempo $\left[ seg \right]$",'Interpreter', 'latex')
ylabel("Amplitud $\left[ \frac{rad}{s} \right]$", 'Interpreter', 'latex')
legend('$\omega_{m1}$','$\omega_{m2}$','$\omega_{m3}$','$\omega_{m4}$', 'Interpreter', 'latex')
grid on;

figure()
plot(wp_m.Time, wp_m.Data(:,1) , 'b','LineWidth',1)
hold on
plot(wp_m.Time, wp_m.Data(:,2) , 'r','LineWidth',1)
hold on
plot(wp_m.Time, wp_m.Data(:,3) , 'black','LineWidth',1)
hold on
plot(wp_m.Time, wp_m.Data(:,4) , 'green','LineWidth',1)
title("Aceleracion angular de motores $\dot{\omega}_{m} \left( t \right)$", 'Interpreter', 'latex')
xlabel("Tiempo $\left[ seg \right]$",'Interpreter', 'latex')
ylabel("Amplitud $\left[ \frac{rad}{s^2} \right]$", 'Interpreter', 'latex')
legend('$\dot{\omega}_{m1}$','$\dot{\omega}_{m2}$','$\dot{\omega}_{m3}$','$\dot{\omega}_{m4}$', 'Interpreter', 'latex')
grid on;