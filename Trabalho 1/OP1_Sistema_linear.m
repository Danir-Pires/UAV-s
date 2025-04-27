clear 
close all
clc

%Constantes 
m = 0.036;
g = 9.81;
J = [2.4*10^-5 0 0
     0 2.4*10^-5 0
     0 0 3.23*10^-5];
Jx = 2.4*10^-5;
Jy = 2.4*10^-5;
Jz = 3.23*10^-5;
ro = 1.225;
D = [7.24*10^-10 0 0
     0 7.24*10^-10 0
     0 0 7.24*10^-10];
l = 0.046;
Cq = 7.2884*10^-10;
Ct = 2.3646*10^-8;
alpha = cos(deg2rad(45));
f_g = m*g;
z_I = [0;0;1];

%Posições

P1 = [l*alpha;-l*alpha;0];
P2 = [-l*alpha; l*alpha;0];
P3 = [l*alpha;l*alpha;0];
P4 = [-l*alpha;-l*alpha;0];

%Força Propulsiva 
T1=(f_g)/4;
T2=(f_g)/4;
T3=(f_g)/4;
T4=(f_g)/4;
fp1=[0;0;T1];
fp2=[0;0;T2];
fp3=[0;0;T3];
fp4=[0;0;T4];
fp = [0;0;T1+T2+T3+T4];
T = f_g;

%Momento propulsivo
np1 = cross(P1,fp1);
np2 = cross(P2,fp2);
np3 = cross(P3,fp3);
np4 = cross(P4,fp4);
np= np1+np2+np3+np4;
npx = np(1,1);
npy = np(2,1);
npz = np(3,1);

%simulação
Dt = 0.01;
t = 0:Dt:60;
nx = 12;
ny = 4;
Nsim = length(t);

%Linearização-equilibrio
px = 0;
py = 0;
pz = 10;
p = [px;py;pz];
wx = 0;  
wy = 0;
wz = 0;
omega = [wx;wy;wz]; 
vx = 0;
vy = 0;
vz = 0;
v = [vx;vy;vz];
phi = deg2rad(0);
theta = deg2rad(0);
psi = deg2rad(0);
lbd = [phi;theta;psi];
x0 = [p; v; lbd; omega];
u = ones(Nsim,1)*[T,npz,npy,npz];


b = [(wy*cos(phi)-wz*sin(phi))*tan(theta), (wy*sin(phi)+wz*cos(phi))/((cos(theta))^2), 0
     -wy*sin(phi)-wz*cos(phi), 0, 0
      (wy*cos(phi)-wz*sin(phi))/cos(theta), ((wy*sin(phi)+wz*cos(phi))*tan(theta))/cos(theta), 0];

A = [ zeros(3), Euler2R(lbd), zeros(3), zeros(3)
     zeros(3), -D, -g*skew(z_I), skew(v)
     zeros(3), zeros(3), b, Euler2Q(lbd)
     zeros(3), zeros(3), zeros(3), zeros(3)];
B = [zeros(3,4)
     [0;0;1/m], zeros(3)
     zeros(3,4)
     zeros(3,1), [1/Jx;0;0], [0;1/Jy;0], [0;0;1/Jz]];
C = eye(12);
D = zeros(12,4);

sys = ss(A,B,C,D);
y_L = lsim(sys,u,t,x0);

[V,D,W] = eig(A) % W'*A = D*W'
lbd0 = diag(D);
figure(904211);
plot(real(lbd0),imag(lbd0),'x');
grid on;

% Teste de estabilidade, controlabilidade e observabilidade
[M,J] = jordan(A),
if rank(ctrb(A,B)) < size(A,1), disp('O Sistema não é controlável'); end
if rank(obsv(A,C)) < size(A,1), disp('O Sistema não é observável'); end
ctrb_modes = W'*B,
obsv_modes = C*V,


s = tf('s');
G_theta = 9.81/s^2;
figure(10);
set(gcf,'defaultLineLineWidth',0.5);
rlocus(G_theta);
grid on;

s = tf('s');
G_phi = -9.81/s^2;
figure(11);
set(gcf,'defaultLineLineWidth',0.5);
rlocus(G_phi);
grid on;

s = tf('s');
G_T = 0.0028/s^2;
figure(12);
set(gcf,'defaultLineLineWidth',0.5);
rlocus(G_T);
grid on;