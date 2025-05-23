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
csvfilename = csvfilename_hover;
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

% convert date to print format
t = time - time(1);

% plot data
initPlots;
crazyflie_show_sensor_data(t,acc,gyro,baro_asl,pos,vel,lbd);

figure(10)
subplot(3,1,1)
plot (t(:,816:7063),acc(1,816:7063))
xlabel('$$t$$ [s]');
ylabel('$$a_x[m/s^2]$$');
set(groot,'defaultLineLineWidth',0.5)
grid on

subplot(3,1,2)
plot (t(:,816:7063),acc(2,816:7063))
xlabel('$$t$$ [s]');
ylabel('$$a_y[m/s^2]$$');
set(groot,'defaultLineLineWidth',0.5)
grid on

subplot(3,1,3)
plot (t(:,816:7063),acc(3,816:7063))
xlabel('$$t$$ [s]');
ylabel('$$a_z[m/s^2]$$');
set(groot,'defaultLineLineWidth',0.5)
grid on

figure(11)
subplot(3,1,1)
plot (t(:,816:7063),gyro(1,816:7063))
xlabel('$$t$$ [s]');
ylabel('$$\omega_x[deg/s]$$');
set(groot,'defaultLineLineWidth',0.5)
grid on

subplot(3,1,2)
plot (t(:,816:7063),gyro(2,816:7063))
xlabel('$$t$$ [s]');
ylabel('$$\omega_y[deg/s]$$');
set(groot,'defaultLineLineWidth',0.5)
grid on

subplot(3,1,3)
plot (t(:,816:7063),gyro(3,816:7063))
xlabel('$$t$$ [s]');
ylabel('$$\omega_z[deg/s]$$');
set(groot,'defaultLineLineWidth',0.5)
grid on

X_a = mean (acc(:,816:7063)') ;
cov_a = cov (acc(:,816:7063)') ;
disp(['Média Acelerómetro: ', num2str(X_a)]) ;
disp('Covariância Acelerómetro: ') ;
disp (diag(cov_a)) ;

X_g = mean (gyro(:,816:7063)') ;
cov_g = cov (gyro')  ;
disp(['Média Gyro: ', num2str(X_g)]);
disp('Covariância Gyro: ');
disp(diag(cov_g));
