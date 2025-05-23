% crazyflie load usd card log csv file for sensor analysis
clear all

% available files:
csvfilename_motors_off = 'L2Data1.csv';
csvfilename_hover = 'L2Data2.csv';
csvfilename_motion_x = 'L2Data3.csv';
csvfilename_motion_y = 'L2Data4.csv';
csvfilename_motion_z = 'L2Data5.csv';
csvfilename_motion_inf = 'L2Data6.csv';

% read file
csvfilename = csvfilename_motion_inf;
array = dlmread(csvfilename,',',1,0);
T = table2array(readtable(csvfilename)); % Matlab only

% get data from table
time = array(:,1)'*1e-3;
pos = array(:,2:4)'; % [m] groundtruth
vel = array(:,5:7)'; % [m/s] groundtruth
lbd = array(:,8:10)'; % [deg] groundtruth
gyro = array(:,11:13)'; % [deg/s] sensor
acc = array(:,14:16)'; % [Gs] sensor
baro_asl = array(:,17)'; % [m] sensor
% lighthouse = array(:,18:20))'; % [m]

% Converter data para formato de impressão
t = time - time(1);

 A  = [0 0 1 0 0 0 
       0 0 0 1 0 0
       0 0 0 0 0 0
       0 0 0 0 0 0
       0 0 0 0 0 0
       0 0 0 0 0 0];

 B = zeros(6,0);

 C = [1 0 0 0 0 0
      0 1 0 0 0 0
      0 0 1 0 1 0
      0 0 0 1 0 1];

 D = zeros(2,0);

Ts = time(2) - time(1);        % Amostragem de tempo
F = expm(A*Ts);                % Matriz de estado discreta
G = B*Ts;            
H = C;                         
ny = size(H,1);
nx = size(H,2);

%Ganhos de Kalman

Q = Ts^2*eye(nx);
Q(1,1)=Q(1,1)*15.5;
Q(2,2)=Q(2,2)*14;
Q(3,3)=Q(3,3)*15;
Q(4,4)=Q(4,4)*14; 
Q(5,5)=Q(5,5)*1;
Q(6,6)=Q(6,6)*1;
R = eye(ny);
R(1,1) = 1.376*10^-2; %1.376*10^-2
R(2,2) = 1.251*10^-2; %1.251*10^-2
R(3,3) = 0.02; %0.0199
R(4,4) = 0.05; %0.0323

% Valores iniciais
P0 = 2*eye(nx);
x0 = zeros(nx,1);

% Simular sistema e estimativa
Nsim = length(t);
x = zeros(nx,Nsim);
y = [lbd(1:2,:);gyro(1:2,:)];
xe = zeros(nx,Nsim);    % x estimado
Pe = zeros(nx,nx,Nsim);
x(:,1) = x0;
xe(:,1) = x0;
Pe(:,:,1) = P0;
u = zeros(nx,1);
Gu = u; %Visto G ser uma matriz vazia e G*u dar uma matriz coluna com nx linhas 

for k = 1:Nsim

    % predict next estimate:
    [xem,Pem] = kalman_predict(xe(:,k),Pe(:,:,k),F,Gu,Q);

    % update estimate with measurement info
    [xe(:,k),Pe(:,:,k),K] = kalman_update(xem,Pem,y(:,k),H,R);

    % simulate system dynamics and process noise
    % w_noise = sqrtm(Q)*randn(nx,1); % process noise not added
    xp = F*x(:,k) + Gu; % + w_noise; % process noise not added
    if k < Nsim % update next state until last instant
        x(:,k+1) = xp;
    end
end

% Mostrar resultados roll (ângulo de rotação)
figure(1);
plot(t,xe(1,:)*180/pi,t,lbd(1,:)); 
grid on;
xlabel('$$t$$[s]');
ylabel('$$\phi[deg]$$');
legend('Filtro de Kalman','DataSet6');

% Mostrar resultados pitch (ângulo de inclinação)
figure(2);
plot(t,xe(2,:)*180/pi,t,lbd(2,:)); 
grid on;
xlabel('$$t$$[s]');
ylabel('$$\theta[deg]$$');
legend('Filtro de Kalman','DataSet6');  

dados_kalmanfilter_inf = xe;
save ('dados_kalmanfilter_inf.mat', 'dados_kalmanfilter_inf')

