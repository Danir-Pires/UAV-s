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
csvfilename = csvfilename_motion_x;
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

%Movimento em x, theta diferente 0
theta_m = atan(-(acc (1,:)./(acc (3,:))))*180/pi;

figure(13)
plot(t,lbd(2,:),t,theta_m);
xlabel('$$t$$ [s]');
ylabel('$$\theta[deg]$$');
grid on
legend({'DataSet3','Accelerometer'},'FontSize',8)