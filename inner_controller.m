close all;
clc;clear;
global x r dr euler w u T M R phi theta psi g Ixx Iyy Izz;
global dx t dt m Ra;
global phi_des phi_err phi_err_prev phi_err_sum theta_des theta_err theta_err_prev theta_err_sum psi_des psi_err psi_err_prev psi_err_sum z_des z_err z_err_prev z_err_sum;
global P_phi I_phi D_phi P_theta I_theta D_theta P_psi I_psi D_psi P_z I_z D_z;
global x_des x_err x_err_prev x_err_sum y_des y_err y_err_prev y_err_sum;
global P_x I_x D_x P_y I_y D_y;
%% drone parameters
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
initInputs=[0.0 0.0 0.0 0];

%for plotting in 3d graph
drone_body=[armLength 0.0 0.0 1
            0.0 -armLength 0.0 1
            -armLength 0.0 0.0 1
            0.0 armLength 0.0 1 
            0.0 0.0 0.0 1
            0.0 0.0 -0.15 1]';
       
 %% PID parameters
 
%  P_phi=0.2;I_phi=0;D_phi=0.15;
%  P_theta=0.2;I_theta=0;D_theta=0.15;
%  P_psi=0.4;I_psi=0;D_psi=0.3;
%  P_z=10;I_z=0.2;D_z=0;
 P_phi=1.2;I_phi=0;D_phi=0.7;
 P_theta=1.2;I_theta=0;D_theta=0.7;
 P_psi=4.2;I_psi=0;D_psi=1;
 P_z=90;I_z=0 ;D_z=30;
 
 
 %% environment parameters
g=9.81;
t=0.0;
dt=0.01;
simTime=1.5;
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

z_des=0;
z_err=0;
z_err_prev=0;
z_err_sum=0;

x_des=0;
x_err=0;
x_err_prev=0;
x_err_sum=0;

y_des=0;
y_err=0;
y_err_prev=0;
y_err_sum=0;
%% initializing the drone
%x x y z dx dy dz phi theta psi p q r
x=initStates;
r=x(1:3);
dr=x(4:6);
euler=x(7:9);
phi=euler(1);
theta=euler(2);
psi=euler(3);
w=x(10:12);

u=initInputs;
T=u(1);
M=u(2:4);
dx=[0 0 0 0 0 0 0 0 0 0 0 0]';
R=generate_R(phi,theta,psi);
%% 3D figures for simulation
fig1=figure('pos',[0 200 800 500]);
h=gca;
view(3);
%fig1.CurrentAxes.ZDir ='Reverse'; %commented for testing purposes
fig1.CurrentAxes.YDir ='Reverse';
axis equal;
hold(gca,'on');
xlabel('X[m]');
ylabel('Y[m]');
zlabel('Z[m]');
grid minor;
xlim([-5 5]);ylim([-5 5]);zlim([-8 8]);


wHb=[R x(1:3); % a transformation matrix for both attitude and position
     0.0 0.0 0.0 1];
 
 info = wHb * drone_body ; % body to inertial transformation
 d_attitude=info(1:3, :);
 
 %% drawing of the drone itself
 
 fig1_ARM13=plot3(gca,d_attitude(1,[1 3]),d_attitude(2,[1 3]) , d_attitude(3,[1 3]),'-ro','MarkerSize',5);
 fig1_ARM24=plot3(gca,d_attitude(1,[2 4]),d_attitude(2,[2 4]) , d_attitude(3,[2 4]),'-bo','MarkerSize',5);
 fig1_payload=plot3(gca,d_attitude(1,[5 6]),d_attitude(2,[5 6]) , d_attitude(3,[5 6]),'-k','LineWidth',3);
 fig1_shadow=plot3(gca,0,0,0,'xk','lineWidth',3);
 
 hold(gca,'off');

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
title('Z(m)');
hold on;

%% main loop
commandSig=[0 0 0 10 0 0];
commandSig(1:3)=commandSig(1:3)*(pi/180);

for i=1:simTime/0.01
     AttitudeCtrl(commandSig);
     UpdateState();
    if(u(1)>=m*g+2)
     u(1)=m*g+2; 
    end
     if(u(1)<-m*g)
     u(1)=-m*g; 
    end
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
 set(fig1_shadow, ...
     'xData',x(1),...
     'yData',x(2),...
     'zData',0);
 figure(2)
  grid minor;
subplot(2,3,1);
plot(i/100,x(7)*(180/pi),'.');
subplot(2,3,2);
plot(i/100,x(8)*(180/pi),'.');
subplot(2,3,3);
plot(i/100,x(9)*(180/pi),'.');
subplot(2,3,4);
plot(i/100,x(1),'.');
subplot(2,3,5);
plot(i/100,x(2),'.');
subplot(2,3,6);
plot(i/100,x(3),'.');%x(6)
 drawnow;

%     if(x(3)>=0)
%      msgbox('z=0','Error','error');
%      break;
%     end

end
%% MAIN FUNCTIONs
function evaluate()
global phi_des phi_err phi_err_prev phi_err_sum theta_des theta_err theta_err_prev theta_err_sum psi_des psi_err psi_err_prev psi_err_sum zdot_des zdot_err zdot_err_prev zdot_err_sum;
global p q x r dr euler w u T M R phi theta psi m g Ixx Iyy Izz;
global dx t dt Ra;
global x_des x_err x_err_prev x_err_sum y_des y_err y_err_prev y_err_sum;
global P_x I_x D_x P_y I_y D_y;
%x x y z dx dy dz phi theta psi p q r
%dx -> dx dy dz ddx ddy ddz dphi dtheta dpsi dp dq dr8
R=R_gen(phi,theta,psi);
phi = euler(1);
theta = euler(2);
psi=euler(3);
%m*dx(4:6)' = [0 0 mg]' + R*[0 0 -u1]'

dx(1:3)= x(4:6);
dx(4:6)= (1/m) * ([0 0 m*g]'+R*[0 0 -u(1)]');
% dot euler part's generation
dx(7:9)=in_transform(theta,phi,w);
p=w(1);
q=w(2);
Ra=w(3);



dx(10)=M(1)/Ixx + (Iyy-Izz)*q*Ra*(1/Ixx);
dx(11)=M(2)/Iyy + (Izz-Ixx)*Ra*p*(1/Iyy);
dx(12)=M(3)/Izz + (Ixx-Iyy)*p*q*(1/Izz);
end

function UpdateState()

global r dr euler w u T M m g;
global t dt x dx ;
t=t+dt;
evaluate();
x=x+dx*dt;
%simple integration
r=x(1:3);
dr=x(4:6);
euler=x(7:9);
w=x(10:12);
end
function AttitudeCtrl(refSig)
global phi_des phi_err phi_err_prev phi_err_sum theta_des theta_err theta_err_prev theta_err_sum psi_des psi_err psi_err_prev psi_err_sum z_des z_err z_err_prev z_err_sum;
global r dr euler w u T M m g phi theta psi;
global t dt x dx ;
global P_phi I_phi D_phi P_theta I_theta D_theta P_psi I_psi D_psi P_z I_z D_z;
global x_des x_err x_err_prev x_err_sum y_des y_err y_err_prev y_err_sum;
global P_x I_x D_x P_y I_y D_y;

phi_des=refSig(1);
theta_des=refSig(2);
psi_des=refSig(3);
z_des=refSig(4);

phi_err = phi_des - phi;
theta_err=theta_des-theta;
psi_err=psi_des-psi;
z_err=-z_des+x(3);


%% inner controller
u(1)= P_z*z_err+D_z*(z_err-z_err_prev)/dt+I_z*z_err_sum;
u(2)= P_phi*phi_err+D_phi*(phi_err-phi_err_prev)/dt+I_phi*phi_err_sum;
u(3)= P_theta*theta_err+D_theta*(theta_err-theta_err_prev)/dt+I_theta*theta_err_sum;
u(4)= P_psi*psi_err+D_psi*(psi_err-psi_err_prev)/dt+I_psi*psi_err_sum;
z_err_sum=+z_err;
z_err_prev=z_err;
phi_err_sum=+phi_err;
phi_err_prev=phi_err;
psi_err_sum=+psi_err;
psi_err_prev=psi_err;
theta_err_sum=+theta_err;
theta_err_prev=theta_err;


%% outer controller
%when u(1)>mg z(m) decreases
%test case(hover)
%u(1)=m*g+1;
% u(2)=1;
% u(3)=0.0;
% u(4)=0.0;
T=u(1);
M=u(2:4);

end
%% R matrix generation functions
function Lpsi_matris = Lpsi(psi)
Lpsi_matris =[
    cos(psi) sin(psi) 0
    -sin(psi) cos(psi) 0
    0 0 1];
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
    %pqr to dot_vector phi* theta* psi* 
    transformation_matrix=[
                   1, sin(theta)*tan(theta), cos(fi)*tan(theta)
                   0, cos(fi), -sin(theta)
                   0, sin(fi)*sec(theta), cos(fi)*sec(theta)];
               
              transformed_matrix=transformation_matrix*angular_velocity_vector; 
               
end
function R_matr = R_gen(phi,theta,psi)
R_matr =[
cos(psi)*cos(theta) cos(psi)*sin(phi)*sin(theta) sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta)
cos(theta)*sin(psi) cos(theta)*cos(psi)+sin(psi)*sin(phi)*sin(theta) cos(theta)*sin(psi)*sin(theta)-cos(psi)*sin(theta)
-sin(theta) cos(theta)*sin(phi) cos(phi)*cos(theta)];
end
