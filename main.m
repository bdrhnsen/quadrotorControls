close all;
clc;clear;

%% drone parameters
global x r dr euler w u T M R phi theta psi g Ixx Iyy Izz;
global dx t dt m;

m=1.25;
armLength=0.265;
Ixx=0.0232;
Iyy=0.0232;
Izz=0.0468;

% X=[x y z dx dy dz phi theta psi p q r] 
initStates=[0.0, 0.0, 0.0,... %x y z
                  0.0,0.0,0.0,... %dx dy dz
                  0.0,0.0,0.0,... % phi theta psi
                  0.0,0.0,0.0]'; %pqr
              
% u1 u2 u3 u4
initInputs=[0.0 0.0 0.0 -2];

%for plotting in 3d graph
drone_body=[armLength 0.0 0.0 1
            0.0 -armLength 0.0 1
            -armLength 0.0 0.0 1
            0.0 armLength 0.0 1 
            0.0 0.0 0.0 1
            0.0 0.0 -0.15 1]';
       
 %% PID parameters
 
 P_phi=0;I_phi=0;D_phi=0;
 P_theta=0;I_theta=0;D_theta=0;
 P_psi=0;I_psi=0;D_psi=0;
 P_zdot=0;I_zdot=0;D_zdot=0;
 
 %% environment parameters
g=9.81;
t=0.0;
dt=0.01;
simTime=2;
% x-> [x y z dx dy dz phi theta psi p q r]
% r-> x y z 
% dr-> dx dy dz 
% euler -> phi theta psi
% w -> pqr
% u u1 u2 u3 u4  ->another notation [T, M1, M2, M3]
% u1 is total trust T-> total trust M->[M1 M2 M3]'

%% other variables
phi_des=0;
phi_err=0;
phi_err_prev=0;
phi_err_sum=0;

theta_des=0;
theta_err=0;
theta_err_prev=0;
theta_err_sum=0;

psi_des=0;
psi_err=0;
psi_err_prev=0;
psi_err_sum=0;

zdot_des=0;
zdot_err=0;
zdot_err_prev=0;
zdot_err_sum=0;
%% initializing the drone
x=initStates;
r=x(1:3);
dr=x(4:6);
euler=x(7:9);
w=x(10:12);
u=initInputs;
T=u(1);
M=u(2:4);
dx=[0 0 0 0 0 0 0 0 0 0 0 0];
R=generate_R(phi,theta,psi);
%% 3D figures for simulation
fig1=figure('pos',[0 200 800 500]);
h=gca;
view(3);
fig1.CurrentAxes.ZDir ='Reverse';
fig1.CurrentAxes.YDir ='Reverse';
xlabel('X[m]');
ylabel('Y[m]');
zlabel('Z[m]');
grid minor;
xlim([-5 5]);ylim([-5 5]);zlim([-8 0]);
axis equal;
hold(gca,'on');

wHb=[R x(1:3); % a transformation matrix for both attitude and position
     0.0 0.0 0.0 1];
 
 info = wHb * drone_body ; % body to inertial transformation
 d_attitude=info(1:3, :);
 
 %% drawing of the drone itself
 
 fig1_ARM13=plot3(gca,d_attitude(1,[1 3]),d_attitude(2,[1 3]) , d_attitude(3,[1 3]),'-ro','MarkerSize',5);
 fig1_ARM24=plot3(gca,d_attitude(1,[2 4]),d_attitude(2,[2 4]) , d_attitude(3,[2 4]),'-bo','MarkerSize',5);
 fig1_payload=plot3(gca,d_attitude(1,[5 6]),d_attitude(2,[5 6]) , d_attitude(3,[5 6]),'-k','LineWidth',3);
 %fig1_shadow=plot3()

%% data figures
fig2=figure('pos',[800 400 800 400]);
grid minor;
subplot(2,3,1);
title('phi(deg)');
hold on;
subplot(2,3,2);
title('theta(deg)');
hold on;
subplot(2,3,3);
title('psi(deg)');
hold on;
subplot(2,3,4);
title('X(m)');
hold on;
subplot(2,3,5);
title('Y(m)');
hold on;
subplot(2,3,6);
title('Z_dot(m/s)');
hold on;

%% main loop
commandSig=[0 0 0 0];

for i=1:simTime/0.01
     AttitudeCtrl(commandSig);
     UpdateState();
   %% drawing
wHb=[R x(1:3); % a transformation matrix for both attitude and position
     0.0 0.0 0.0 1];
 info = wHb * drone_body ; % body to inertial transformation
 d_attitude=info(1:3, :);
 
 set(fig1_ARM13,...
     'xData',d_attitude(1,[1 3]),...
     'yData',d_attitude(2,[1 3]),...
     'zData',d_attitude(3,[1 3]));
  set(fig1_ARM24,...
     'xData',d_attitude(1,[2 4]),...
     'yData',d_attitude(2,[2 4]),...
     'zData',d_attitude(3,[2 4]));
 
  set(fig1_payload,...
     'xData',d_attitude(1,[5 6]),...
     'yData',d_attitude(2,[5 6]),...
     'zData',d_attitude(3,[5 6]));
 
end

%% MAIN FUNCTIONs
function EvalEOM()
global p q x r dr euler w u T M R phi theta psi m g Ixx Iyy Izz;
global dx t dt;
%dx -> dx dy dz ddx ddy ddz dphi dtheta dpsi dp dq dr
dx(1:3)= dr;
phi = euler(1);
theta = euler(2);
psi=euler(3);
%m*dx(4:6)' = [0 0 mg]' + R*[0 0 -u1]'

dx(4:6)= (1/m) * ([0 0 m*g]'+R*[0 0 -u(1)]');

% dot euler part's generation
dx(7:9)=in_transform(theta,phi,w)';
p=w(1);
q=w(2);
r=w(3);
dx(10)=M(1)/Ixx + (Iyy-Izz)*q*r*(1/Ixx);
dx(11)=M(2)/Iyy + (Izz-Ixx)*r*p*(1/Iyy);
dx(12)=M(3)/Izz + (Ixx-Iyy)*p*q*(1/Izz);
end

function UpdateState()
global r dr euler w u T M m g;
global t dt x dx ;
t=t+dt;
EvalEOM();
x=x+dx*dt;%simple integration
r=x(1:3);
dr=x(4:6);
euler=x(7:9);
w=x(10:12);
end
function AttitudeCtrl(refSig)
global r dr euler w u T M m g;
global t dt x dx ;

%test case(hover)
u(1)=m*g;
u(2)=0;
u(3)=0;
u(4)=0;

end
%% R matrix generation functions
function Lpsi_matris = Lpsi(psi)
Lpsi_matris =[
    cos(psi) sin(psi) 0
    -sin(psi) cos(psi) 0
    0   0   1];
end%around z axis yaw
function Ltheta_matris = Ltheta(theta)
Ltheta_matris=[
    cos(theta) 0 -sin(theta)
    0   1   0
    sin(theta) 0 cos(theta)];
end%around y axis pitch
function Lfi_matris = Lfi(fi)
Lfi_matris=[
    1 0 0
    0 cos(fi) sin(fi)
    0 -sin(fi) cos(fi)];
end%around x axis roll
function R_matrix=generate_R(fi,theta,psi)
R_matrix = Lfi(fi)*Ltheta(theta)*Lpsi(psi);
R_matrix=R_matrix';
end%R matrix

%% pqr to dot_euler matrix generation functions(euler parametrization)
%use in_tranform for pqr -> dot matrix
%use transform for dot -> pqr
function transformed_matrix=transform(theta,fi,angular_velocity_vector)
    %angular_velocity vector is in terms of fi_dot theta_dot and psi_dot 
    %dot vector to pqr
    transformation_matrix= [
                   1 0 -sin(theta)
                   0 cos(fi) sin(fi)*cos(theta)
                   0 -sin(fi) cos(fi)*cos(theta)];
               
              transformed_matrix=transformation_matrix*angular_velocity_vector; 
               
end
function transformed_matrix=in_transform(theta,fi,angular_velocity_vector)
    %pqr to dot_vector
    transformation_matrix=[
                   1 sin(theta)*tan(theta) cos(fi)*tan(theta);0 cos(fi) -sin(theta);0 sin(fi)*sec(theta) cos(fi)*sec(theta);];
               
              transformed_matrix=transformation_matrix*angular_velocity_vector; 
               
end
