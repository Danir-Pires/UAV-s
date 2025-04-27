%%Ex 4.4

g = 9.81;
m = 1;
J = diag([0.02,0.02,0.04]);
zI = [0;0;1];

%Parâmetros de Simulação

Dt = 0.01;
t = 0:Dt:20;
nx = 12;
ny = 4;
T = 0.5*m*g;
np = [0;0;0];
x0 = [0;0;0;
      10;0;0;
      0.01;0;0;
      0;0;0];

%Simulação
NSim = length(t);
x = zeros(nx,Nsim);
y = zeros(ny,Nsim);
x(:,1) = x0;
for k = 1:Nsim
    p = x(1:3,k);
    v = x(4:6,k);
    lbd = x(7:9,k);
    omega = x(10:12,k);
    R = Euler2R(lbd);
    Q = Euler2Q(lbd);
   
    p_dot = R*v;
    v_dot = -skew(omega)*v+g*R'*zI-(1/m)*T*zI;
    lbd_dot = Q*omega;
    omega_dot = (-J^-1*skew(omega)*J*omega)+(J^-1*np);

    x_dot = [p_dot;v_dot;lbd_dot;omega_dot];

    x(:,k+1) = x(:,k) + Dt*x_dot;
    y(1:9,k) = x(1:9,k);
end

figure(1);%de todos juntos
subplot(3,1,1);
plot(t,y(1,:),t,y(2,:),t,y(3,:));
ylabel('Posição [m]');
xlabel('Tempo [s]');
legend('p_x','p_y','p_z');
grid on;

subplot(3,1,2);
plot(t,y(4,:),t,y(5,:),t,y(6,:));
ylabel('Velocidade [m/s]');
xlabel('Tempo [s]');
legend('v_x','v_y','v_z');
grid on;

subplot(3,1,3);
plot(t,y(7,:),t,y(8,:),t,y(9,:));
ylabel('Ângulos de Euler [rad]');
xlabel('Tempo [s]');
legend('\lambda_x','\lambda_y','\lambda_z');
grid on;