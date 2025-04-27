%% Exercise 4.4 drone nonlinear model simulation

g = 9.81;
m = 1.37;
J = diag([0.022, 0.011, 0.031]);
beta = 0.1;
D = beta*eye(3);
zI = [0;0;1];


% simulation parameters
nx = 12;
ny = 4;
x0 = [0;    0; 0;
      0;    0; 0;      
      0.01; 0; 0;      
      0;    0; 0]; %[p;v;lambda;omega]
Dt = 0.01;
t = 0:Dt:20;

Te = m*g; % equilibirum thrust
u_L = [0.2*m*g;0.0;0.0;0.01]*(t>=5);
u_NL = [Te;0;0;0]*ones(size(t)) + u_L;

% simulate nonlinear system
Nsim = length(t);
x = zeros(nx,Nsim);
y = zeros(ny,Nsim);
x(:,1) = x0;
for k = 1:Nsim
    % prepare variables:
    p   = x(1:3,k);
    v   = x(4:6,k);
    lbd = x(7:9,k);
    om  = x(10:12,k);
    R = Euler2R(lbd);
    Q = Euler2Q(lbd);
    
    % input function
    T = u_NL(1,k);
    np = u_NL(2:4,k);

    % compute state derivative:
    p_dot = R*v;
    v_dot = -skew(om)*v + g*R'*zI - D*v - 1/m*T*zI;
    lbd_dot = Q*om;
    om_dot = -inv(J)*skew(om)*J*om + inv(J)*np;

    x_dot = [p_dot;v_dot;lbd_dot;om_dot];

    % integrate state
    x(:,k+1) = x(:,k) + Dt*x_dot;
end
x(:,k+1) = [];

figure(1);
plot(t,x(1:3,:),'-.');
ylabel('Posição [m]');
xlabel('Tempo [s]');
legend('$$p_x$$ (nlin)','$$p_y$$ (nlin)','$$p_z$$ (nlin)');
grid on;

figure(2);
plot(t,x(7:9,:)*180/pi,'-.');
ylabel('Ângulos de Euler');
xlabel('Tempo [s]');
legend('$$\phi$$ (nlin)','$$\theta$$ (nlin)','$$\psi$$ (nlin)');
grid on;

figure(3);
plot(t,u_NL);
ylabel('Thrust [N]');
xlabel('Tempo [s]');
legend('$$u_N_L$$ (nlin)')
grid on;
