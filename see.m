clear all;  clc; close all;
%% 
% Dynamic model of HPURV
% 06/08/2018
%% model parameters
m = 13;       % mass in kg
g = 9.81;
rho = 1021;     % density of water

a = 0.17; b = 0.24; L = 0.740;      % dimensions of robot(assumed to be a rectangular cross section)
xG = 0; yG = 0; zG = 0;             % position of centre of gravity in body fixed frame
xB = 0; yB = 0; zB = 0;             % postion of centre of buoyancy in body fixed frame

IxG = 0.05; IyG = 0.45; IzG = 0.44; % principal MoI about CoG from Soldworks Ixx Iyy and Izz
Ixy = 0; Iyz = 0; Ixz = 0;          % product of inertias
Ix = IxG + m*(yG^2 + zG^2);         % principal MoI about BFF
Iy = IyG + m*(xG^2 + zG^2);
Iz = IzG + m*(xG^2 + yG^2);
i=1;

x1 = -0.257; y1 = 0; z1 = 0;        % Distances of each fin from CoG
x2 = 0.177; y2 = 0.097; z2 = 0.0495;
x3 = 0.177; y3 = -0.097; z3 = 0.0495;

Cl = 0.92; Cd = 1.12;               % Coeff of lift and drags of the fin
S1 = 0.024;    L_f1 = 0.2;           % Surface area of caudal fin
S2 = 0.044;    L_f2 = 0.1;
S3 = 0.044;    L_f3 = 0.1;
% Fin shape factors (modify these values to represent different fin shapes)
b_f1 = 0.1;    % Caudal fin span [m]
b_f2 = 0.1;    % Pectoral fin (port) span [m]
b_f3 = 0.1;    % Pectoral fin (starboard) span [m]

% Calculate aspect ratios for each fin (AR = span^2 / area)
AR1 = b_f1^2 / S1;
AR2 = b_f2^2 / S2;
AR3 = b_f3^2 / S3;

% Use the Sowald formula to compute the shape coefficient for each fin
shape1 = AR1 / (AR1 + 2);
shape2 = AR2 / (AR2 + 2);
shape3 = AR3 / (AR3 + 2);



PF1max = 5;

%A1 = pi/4;                          % amplitude of oscillation in radians
%gamax = pi/6;
freqmax = 2;

% %% control parameters
% k1 = 4;
% k2 = 4;
% 
% % k1=4;k2=2;
%% Mass matrix
% mass of rigid body
Mrb = [ m         0        0        0          m*zG      -m*yG;
        0         m        0       -m*zG       0          m*xG;
        0         0        m        m*yG      -m*xG          0;
        0        -m*zG     m*yG     Ix        -Ixy        -Ixz;
        m*zG      0       -m*xG    -Ixy        Iy         -Iyz;
       -m*yG      m*xG     0       -Ixz       -Iyz          Iz];


% Xudot = -1.3;
% Yvdot = -23.3;
% Zwdot = -50.5;
% Kpdot = -0.06;
% Nrdot = -2.04;
% Mqdot = -0.86;
Xudot = 1.3;
Yvdot = -2.3;
Zwdot = -5.5;
Kpdot = -0.06;
Nrdot = -2.04;
Mqdot = -0.86;

D = -1*[Xudot Yvdot Zwdot Kpdot Nrdot Mqdot];
% added mass on the model
Mad = diag(D);
% total mass
M = Mrb+0*Mad;

% control gains
Kp = 4 *Mrb;
Kd = 4*Mrb;

%simulation settings
T=320;
dt=0.05;
t=0:dt:T;

nu=zeros(6,length(t));
eta=zeros(6,length(t));
%% intial conditions
eta(:,1)=[ 0,0,0,0,0,0]';
%% 
Ux=0.4;Uy=0.4;Uz=0;
free_flow_vel=[Ux;Uy;Uz];

f=zeros(6,length(t));
PF1=zeros(1,length(t));
PF2=zeros(1,length(t));
PF3=zeros(1,length(t));
BFA1=zeros(1,length(t));
BFA2=zeros(1,length(t));
BFA3=zeros(1,length(t));
alpha1=zeros(1,length(t));
alpha2=zeros(1,length(t));
alpha3=zeros(1,length(t));
Fx1=zeros(1,length(t));
Fy1=zeros(1,length(t));
Fx2=zeros(1,length(t));
Fz2=zeros(1,length(t));
Fx3=zeros(1,length(t));
Fz3=zeros(1,length(t));
V1=zeros(1,length(t));
V2=zeros(1,length(t));
V3=zeros(1,length(t));
V1x=zeros(1,length(t));
V1y=zeros(1,length(t));
V2x=zeros(1,length(t));
V2z=zeros(1,length(t));
V3x=zeros(1,length(t));
V3z=zeros(1,length(t));
NV1=zeros(1,length(t));
NV2=zeros(1,length(t));
NV3=zeros(1,length(t));
NV1x=zeros(1,length(t));
NV1y=zeros(1,length(t));
NV2x=zeros(1,length(t));
NV2z=zeros(1,length(t));
NV3x=zeros(1,length(t));
NV3z=zeros(1,length(t));
freq1=zeros(1,length(t));
freq2=zeros(1,length(t));
freq3=zeros(1,length(t));
freq11=zeros(1,length(t));
freq22=zeros(1,length(t));
freq33=zeros(1,length(t));
ga1=zeros(1,length(t));
ga2=zeros(1,length(t));
ga3=zeros(1,length(t));
e1=zeros(6,length(t));
etad=zeros(6,length(t));
eta_dot=zeros(6,length(t));
V=zeros(length(t),1);
alpha=zeros(length(t),1);
b1=zeros(length(t),1);
b2=zeros(length(t),1);
b3=zeros(length(t),1);

tau=zeros(6,length(t));

b1(i,1) = 30*pi/180;                     % Angle beta in the fin configuration
b2(i,1) = 30*pi/180;
b3(i,1) = 30*pi/180;

for i=1:length(t)
    u=nu(1,i);v=nu(2,i);w=nu(3,i);p=nu(4,i);q=nu(5,i);r=nu(6,i);
    x = eta(1,i); y = eta(2,i); z = eta(3,i); phi = eta(4,i); theta = eta(5,i); psi = eta(6,i);
    J1 = [ cos(psi)*cos(theta)  -cos(phi)*sin(psi)+cos(psi)*sin(phi)*sin(theta)     sin(psi)*sin(phi)+cos(phi)*cos(psi)*sin(theta);
           sin(psi)*cos(theta)   cos(psi)*cos(theta)+sin(psi)*sin(phi)*sin(theta)  -cos(psi)*sin(phi)+cos(phi)*sin(psi)*sin(theta);
          -sin(theta)            sin(phi)*cos(theta)                                cos(phi)*cos(theta)];
    % Jacobian for orientation
    J2 = [ 1   sin(phi)*tan(theta)  cos(phi)*tan(theta);
           0   cos(phi)            -sin(phi);
           0   sin(phi)/cos(theta)  cos(phi)/cos(theta)];

    null = zeros(3,3);
    % Jacobian matrix of the model
    J =  [J1   null;
          null J2];
      nv=J*nu(:,i);
      phi_dot = nv(4); theta_dot = nv(5); psi_dot = nv(6);
      
       U=J1*(free_flow_vel);
      Ux=U(1);Uy=U(2);Uz=U(3);
  
      %% Simple eight shape profile with the achievable velocity
        fi = 0.01;
        etad(:,i) = [4*sin(2*fi*t(i));4-4*cos(fi*t(i));(2-2*cos(fi*t(i)));0;0;0];
        etad_dot = [4*2*fi*cos(2*fi*t(i));4*fi*sin(fi*t(i));2*fi*sin(fi*t(i));0;0;0];
        etad_ddot = [-4*4*fi*fi*sin(2*fi*t(i));4*fi*fi*cos(fi*t(i));2*fi*fi*cos(fi*t(i));0;0;0];
        etad(6,i) = atan2(etad_dot(2),etad_dot(1)); % based on LoS
        
        %% Simple circular profile (Creating problem at the mid trajectory location)
 %        etad(:,i) = [2*sin(0.1*t(i));2-2*cos(0.1*t(i));2-2*cos(0.1*t(i));0*pi/6*sin(0.1*t(i));-0*pi/4*sin(0.1*t(i));0*pi/3*sin(0.1*t(i))] + eta(:,1);
 %        etad_dot = [0.2*cos(0.1*t(i));0.2*sin(0.1*t(i));0.2*sin(0.1*t(i));0*0.1*pi/6*cos(0.1*t(i));-0*0.1*pi/4*cos(0.1*t(i));0*0.1*pi/3*cos(0.1*t(i))];
 %        etad_ddot = [-0.2*0.1*sin(0.1*t(i));0.2*0.1*cos(0.1*t(i));0.2*0.1*cos(0.1*t(i));-0*0.1*0.1*pi/6*sin(0.1*t(i));+0*0.1*0.1*pi/4*sin(0.1*t(i));-0*0.1*0.1*pi/3*sin(0.1*t(i))];
 %        etad(6,i) = atan2(etad_dot(2),etad_dot(1)); % based on LoS
 
       crb = [ 0               0               0                 m*(yG*q+zG*r)        -m*(xG*q-w)          -m*(xG*r+v);
            0               0               0                -m*(yG*p+w)            m*(zG*r+xG*p)       -m*(yG*r-u);
            0               0               0                -m*(zG*p-v)           -m*(xG*q+u)           m*(zG*p+yG*q);
           -m*(yG*q+zG*r)   m*(yG*p+w)      m*(zG*p-v)        0                    -q*Iyz-p*Ixz+r*Iz     r*Iyz+p*Ixy-q*Iy;
            m*(xG*q-w)     -m*(zG*r+xG*p)   m*(xG*q+u)        q*Iyz+p*Ixz-r*Iz      0                   -r*Ixz-q*Ixy+p*Ix;
            m*(xG*r+v)      m*(yG*r-u)     -m*(zG*p+yG*q)    -r*Iyz-p*Ixy+q*Iy      r*Ixz+q*Ixy-p*Ix     0 ];
    
    % Centripetal and Coriolis matrix for added mass
    cad = [ 0       0         0       0        -Zwdot*w   Yvdot*v;
            0       0         0      Zwdot*w    0        -Xudot*u;
            0       0         0      -Yvdot*v    Xudot*u    0;
            0    -Zwdot*w    Yvdot*v   0        -Nrdot*r   Mqdot*q;
           Zwdot*w   0      -Xudot*u  Nrdot*r    0        -Kpdot*p;
          -Yvdot*v   Xudot*u     0    -Mqdot*q   Kpdot*p    0  ];
   
    % total Centripetal and Coriolis matrix
    cn = crb + 0*cad;
    cm = cn*nu(:,i);

    % damping matrix 
    dn =   [ 0.97*abs(u)         0                0           0           0                   0;
                0          5.08*abs(v)            0           0           0                   0;
                0                 0        3.38*abs(w)        0           0                   0;
                0                 0                0        0.004*abs(p)    0                   0;
                0                 0                0           0          0.004*abs(q)          0;
                0                 0                0           0           0           0.05*abs(r)];

    
    dm = dn*nu(:,i);

    e1(:,i)= etad(:,i)-eta(:,i);
    e1dot=etad_dot-J*nu(:,i);
    %% control scheme
% tau(:,i)=M*inv(J)*(0*etad_ddot+k2*e1dot+k1*e1(:,i))+0*cm+0*dm;
    tau(:,i)=J'*(Kd*e1dot+Kp*e1(:,i)); % simple PD control
    
   %%
    B = [sin(b1(i,1)),     cos(b1(i,1)),    sin(b2(i,1)),    cos(b2(i,1)),    sin(b3(i,1)),    cos(b3(i,1));
         cos(b1(i,1)),   -sin(b1(i,1)),       0,          0,          0,            0;
               0,          0,    cos(b2(i,1)),    -sin(b2(i,1)),   cos(b3(i,1)),   -sin(b3(i,1));
         -z1*cos(b1(i,1)), z1*sin(b1(i,1)), y2*cos(b2(i,1)), -y2*sin(b2(i,1)), y3*cos(b3(i,1)), -y3*sin(b3(i,1));
         z1*sin(b1(i,1)), z1*cos(b1(i,1)), (z2*sin(b2(i,1))-x2*cos(b2(i,1))), (z2*cos(b2(i,1))+x2*sin(b2(i,1))), (z3*sin(b3(i,1))-x3*cos(b3(i,1))), (z3*cos(b3(i,1))+x3*sin(b3(i,1)));
         (-x1*cos(b1(i,1))-y1*sin(b1(i,1))), -(-x1*sin(b1(i,1))+y1*cos(b1(i,1))), -y2*sin(b2(i,1)), -y2*cos(b2(i,1)), -y3*sin(b3(i,1)), -y3*cos(b3(i,1));];
   
   
  f(:,i) = (B)\tau(:,i);
 
  %paddle forces
  PF1(i)=sqrt(f(1,i)^2+f(2,i)^2);
  PF2(i)=sqrt(f(3,i)^2+f(4,i)^2);
  PF3(i)=sqrt(f(5,i)^2+f(6,i)^2);
  
  %Body force angle
  BFA1(i)= atan2(f(2,i),0.0001+f(1,i));
  BFA2(i)= atan2(f(4,i),0.0001+f(3,i));
  BFA3(i)= atan2(f(6,i),0.0001+f(5,i));
  
  % alpha
  
  alpha1(i)=atan((f(1,i)*Cl)/(0.0001+f (2,i)*Cd));
  alpha2(i)=atan((f(3,i)*Cl)/(0.0001+f(4,i)*Cd));
  alpha3(i)=atan((f(5,i)*Cl)/(0.0001+f(6,i)*Cd));
  
  %body forces along X, and Y axis of the body
  Fx1(i)= PF1(i)*sin(BFA1(i)+b1(i,1));
  Fy1(i)= PF1(i)*cos(BFA1(i)+b1(i,1));
  Fx2(i)= PF2(i)*sin(BFA2(i)+b2(i,1));
  Fz2(i)= PF2(i)*cos(BFA2(i)+b2(i,1));
  Fx3(i)= PF3(i)*sin(BFA3(i)+b3(i,1));
  Fz3(i)= PF3(i)*cos(BFA3(i)+b3(i,1));
  
   b1(i,1) = atan2(Fx1(i), 0.0001+Fy1(i)) - BFA1(i);  
   b2(i,1) = atan2(Fx2(i), 0.0001+Fz2(i)) - BFA2(i);  
   b3(i,1) = atan2(Fx3(i), 0.0001+Fz3(i)) - BFA3(i);  
   
   V1(i) = sqrt(abs(f(2,i)/(0.0001+0.5*rho*Cl*S1*sin(2*alpha1(i)))));    
   V2(i) = sqrt(abs(f(4,i)/(0.0001+0.5*rho*Cl*S2*sin(2*alpha2(i)))));    
   V3(i) = sqrt(abs(f(6,i)/(0.0001+0.5*rho*Cl*S3*sin(2*alpha3(i)))));    
   
   V1x(i) = -V1(i)*sin(b1(i,1));
   V1y(i) = +V1(i)*cos(b1(i,1));
   V2x(i) = -V2(i)*sin(b2(i,1));
   V2z(i) = +V2(i)*cos(b2(i,1));
   V3x(i) = -V3(i)*sin(b3(i,1));
   V3z(i) = +V3(i)*cos(b3(i,1));
   
    NV1x(i) = V1x(i)-(nu(1,i)-Ux);          % Normal Velocity x component
    NV1y(i) = V1y(i)+(nu(2,i)-Uy);          % Normal Velocity y component      
    NV2x(i) = V2x(i)-(nu(1,i)-Ux);          % Normal Velocity x component
    NV2z(i) = V2z(i)+(nu(2,i)-Uz);          % Normal Velocity y component      
    NV3x(i) = V3x(i)-(nu(1,i)-Ux);          % Normal Velocity x component
    NV3z(i) = V3z(i)+(nu(2,i)-Uz);          % Normal Velocity y component      
   
    NV1(i) = sqrt(NV1x(i)^2+NV1y(i)^2);
    NV2(i) = sqrt(NV2x(i)^2+NV2z(i)^2);
    NV3(i) = sqrt(NV3x(i)^2+NV3z(i)^2);
   
    %Frequency of the fin  
    freq1(i) = NV1(i)/(2*pi*L_f1);         % omega*r = 2pi*f*r
    freq2(i) = NV2(i)/(2*pi*L_f2);
    freq3(i) = NV3(i)/(2*pi*L_f3);   
     
   if freq1(i) >= freqmax
        freq1(i) = freqmax; 
       elseif freq1(i) <= -freqmax
        freq1(i) = -freqmax;      
       else 
         freq1(i) = freq1(i);
    end
   if freq2(i) >= freqmax
        freq2(i) = freqmax; 
       elseif freq2(i) <= -freqmax
        freq2(i) = -freqmax;      
       else 
         freq2(i) = freq2(i);
   end
   if freq3(i) >= freqmax
        freq3(i) = freqmax; 
       elseif freq3(i) <= -freqmax
        freq3(i) = -freqmax;      
       else 
         freq3(i) = freq3(i);
   end
   
   freq11(:,i) = (1-exp(-1*t(i)))*freq1(i);
   freq22(:,i) = (1-exp(-1*t(i)))*freq2(i);
   freq33(:,i) = (1-exp(-1*t(i)))*freq3(i);
         
   ga1(i+1) = pi/6*sin(2*pi*freq11(1,i)*t(i));
   ga2(i+1) = pi/6*sin(2*pi*freq22(1,i)*t(i));
   ga3(i+1) = pi/6*sin(2*pi*freq33(1,i)*t(i));
   
   b1(i,1) = ga1(i+1)+alpha1(i);
   b2(i,1) = ga2(i+1)+alpha2(i);
   b3(i,1) = ga3(i+1)+alpha3(i);
   
   BFA1(i) = atan2(Fx1(i),0.0001+Fy1(i)) - b1(i,1); 
   BFA2(i) = atan2(Fx2(i),0.0001+Fz2(i)) - b2(i,1); 
   BFA3(i) = atan2(Fx3(i),0.0001+Fz3(i)) - b3(i,1);
   
   %% Just a sample modification please correct with the actual relations
NV1(i)= freq11(i)*(2*pi*L_f1);
PF1(i) = sqrt((0.5*rho*Cl*(S1*shape1)*sin(2*alpha1(i))*abs(NV1(i))*NV1(i))^2 + (0.5*rho*Cd*(S1*shape1)*cos(2*alpha1(i))*abs(NV1(i))*NV1(i))^2);
NV2(i)= freq22(i)*(2*pi*L_f2);
PF2(i) = sqrt((0.5*rho*Cl*(S2*shape2)*sin(2*alpha2(i))*abs(NV2(i))*NV2(i))^2 + (0.5*rho*Cd*(S2*shape2)*cos(2*alpha2(i))*abs(NV2(i))*NV2(i))^2);
NV3(i)= freq33(i)*(2*pi*L_f3);
PF3(i) = sqrt((0.5*rho*Cl*(S3*shape3)*sin(2*alpha3(i))*abs(NV3(i))*NV3(i))^2 + (0.5*rho*Cd*(S3*shape3)*cos(2*alpha3(i))*abs(NV3(i))*NV3(i))^2);

  
   % modified forces,lift and drag after limiting the forces.
   
   f(1,i)=PF1(i)*cos(BFA1(i));
   f(2,i)=PF1(i)*sin(BFA1(i));
   f(3,i)=PF2(i)*cos(BFA2(i));
   f(4,i)=PF2(i)*sin(BFA2(i));
   f(5,i)=PF3(i)*cos(BFA3(i));
   f(6,i)=PF3(i)*sin(BFA3(i));
   
   B = [sin(b1(i,1)),     cos(b1(i,1)),    sin(b2(i,1)),    cos(b2(i,1)),    sin(b3(i,1)),    cos(b3(i,1));
         cos(b1(i,1)),   -sin(b1(i,1)),       0,          0,          0,            0;
               0,          0,    cos(b2(i,1)),    -sin(b2(i,1)),   cos(b3(i,1)),   -sin(b3(i,1));
         -z1*cos(b1(i,1)), z1*sin(b1(i,1)), y2*cos(b2(i,1)), -y2*sin(b2(i,1)), y3*cos(b3(i,1)), -y3*sin(b3(i,1));
         z1*sin(b1(i,1)), z1*cos(b1(i,1)), (z2*sin(b2(i,1))-x2*cos(b2(i,1))), (z2*cos(b2(i,1))+x2*sin(b2(i,1))), (z3*sin(b3(i,1))-x3*cos(b3(i,1))), (z3*cos(b3(i,1))+x3*sin(b3(i,1)));
         (-x1*cos(b1(i,1))-y1*sin(b1(i,1))), -(-x1*sin(b1(i,1))+y1*cos(b1(i,1))), -y2*sin(b2(i,1)), -y2*cos(b2(i,1)), -y3*sin(b3(i,1)), -y3*cos(b3(i,1));];
   
   
   
  tau(:,i)=B*f(:,i);
  
  nu_dot=M\(tau(:,i)-cm-dm);
  eta_dot = J * nu(:,i);
  
  % State update
     nu(:,i+1) = nu(:,i) + dt * nu_dot;
     eta(:,i+1) = eta(:,i) + dt*J*nu(:,i+1); 
     
    V(i) = sqrt(u*u+v*v);
    Vavg = mean(V);

end
%%

figure
video = VideoWriter('infinity1.avi');
video.FrameRate = 60;
open(video);
for i =1:20:length(t)
%body
xt1=[0,0.2,0.2]-[0.3,0.3,0.3];yt1=[0,0.1,-0.1];zt1=[0,0.2,0.2];X1=[xt1',yt1',zt1'];
xt2=xt1;yt2=[0,0.1,0.1];zt2=[0,0.2,-0.2];X2=[xt2',yt2',zt2'];
xt3=xt1;yt3=[0,-0.1,-0.1];zt3=[0,0.2,-0.2]; X3=[xt3',yt3',zt3'];
xt4=xt1;yt4=yt1;zt4=[0,-0.2,-0.2]; X4=[xt4',yt4',zt4'];
xt5=[0.2,0.2,0.5,0.5]-[0.3,0.3,0.3,0.3];yt5=[-0.1,0.1,0.1,-0.1];zt5=[0.2,0.2,0.2,0.2];  X5=[xt5',yt5',zt5'];
xt6=[0.2,0.2,0.5,0.5]-[0.3,0.3,0.3,0.3];yt6=[-0.1,0.1,0.1,-0.1];zt6=[-0.2,-0.2,-0.2,-0.2]; X6=[xt6',yt6',zt6'];
xt7=[0.2,0.5,0.5,0.2]-[0.3,0.3,0.3,0.3];yt7=[-0.1,-0.1,-0.1,-0.1];zt7=[0.2,0.2,-0.2,-0.2]; X7=[xt7',yt7',zt7'];
xt8=[0.2,0.5,0.5,0.2]-[0.3,0.3,0.3,0.3];yt8=[0.1,0.1,0.1,0.1];zt8=[0.2,0.2,-0.2,-0.2]; X8=[xt8',yt8',zt8'];
xt9=[0.5,0.5,0.6,0.6]-[0.3,0.3,0.3,0.3];yt9=[-0.1,0.1,0.075,-0.075];zt9=[0.2,0.2,0.1,0.1]; X9=[xt9',yt9',zt9'];
xt10=[0.5,0.5,0.6,0.6]-[0.3,0.3,0.3,0.3];yt10=[-0.1,0.1,0.075,-0.075];zt10=[-0.2,-0.2,-0.1,-0.1]; X10=[xt10',yt10',zt10'];
xt11=[0.5,0.6,0.6,0.5]-[0.3,0.3,0.3,0.3];yt11=[0.1,0.075,0.075,0.1];zt11=[0.2,0.1,-0.1,-0.2]; X11=[xt11',yt11',zt11'];
xt12=[0.5,0.6,0.6,0.5]-[0.3,0.3,0.3,0.3];yt12=[-0.1,-0.075,-0.075,-0.1];zt12=[0.2,0.1,-0.1,-0.2]; X12=[xt12',yt12',zt12'];
xb=[0.6,0.6,0.6,0.6]-[0.3,0.3,0.3,0.3];yb=[0.075,-0.075,-0.075,0.075];zb=[0.1,0.1,-0.1,-0.1]; Xb=[xb',yb',zb'];
%caudal fin
xc=[0,-0.2,-0.2];
yc=[0,0,0];
zc=[0,0.1,-0.1];
 Xc=[xc',yc',zc'];
  Rcaudal=[cos(ga1(i)), -sin(ga1(i)), 0;
          sin(ga1(i)), cos(ga1(i)), 0;
          0,0,1];
 Xc=(Rcaudal*Xc')';   
 Xc(:,1)=Xc(:,1)-[0.3,0.3,0.3]';
%pectoral fin (left)
xpl=[0.05,0.05,-0.05,-0.05];
ypl=[-0.1,-0.3,-0.3,-0.1];
zpl=[0,0,0,0];
 Xpl=[xpl',ypl',zpl'];
 
   Rpect=[cos(ga2(i)),0, sin(ga2(i));
          0,1,0;
          -sin(ga2(i)),0, cos(ga2(i))];
 Xpl=(Rpect*Xpl')'; 
 Xpl(:,1)=Xpl(:,1)+[0.1,0.1,0.1,0.1]';
%pectoral fin (right)     
xpr=[0.05,0.05,-0.05,-0.05];
ypr=[0.1,0.3,0.3,0.1];
zpr=[0,0,0,0];
Xpr=[xpr',ypr',zpr'];

   Rpect=[cos(ga3(i)),0, sin(ga3(i));
          0,1,0;
          -sin(ga3(i)),0, cos(ga3(i))];
Xpr=(Rpect*Xpr')'; 
 Xpr(:,1)=Xpr(:,1)+[0.1,0.1,0.1,0.1]';

 phi=eta(4,i);theta=eta(5,i);psi=eta(6,i);
 a=psi;b=theta;c=phi;
 R=[ cos(a)*cos(b) , cos(a)*sin(b)*sin(c)-sin(a)*cos(c), cos(a)*sin(b)*cos(c)+sin(a)*sin(c);
     sin(a)*cos(b), sin(a)*sin(b)*sin(c)+cos(a)*cos(c), sin(a)*sin(b)*cos(c)-cos(a)*sin(c);
     -sin(b),cos(b)*sin(c),cos(b)*cos(c)];

x = eta(1,i); y = eta(2,i);  z = eta(3,i); 
pos=[x,y,z];
X1=(R*X1')';X2=(R*X2')';X3=(R*X3')';X4=(R*X4')';X5=(R*X5')';X6=(R*X6')';X7=(R*X7')';X8=(R*X8')';X9=(R*X9')';X10=(R*X10')';X11=(R*X11')';X12=(R*X12')';Xb=(R*Xb')';Xc=(R*Xc')';Xpl=(R*Xpl')';Xpr=(R*Xpr')';

fill3(X1(:,1)+x*ones(size(X1(:,1))),X1(:,2)+y*ones(size(X1(:,1))),X1(:,3)+z*ones(size(X1(:,1))),'y',X2(:,1)+x*ones(size(X2(:,1))),X2(:,2)+y*ones(size(X2(:,1))),X2(:,3)+z*ones(size(X2(:,1))),'y',X3(:,1)+x*ones(size(X1(:,1))),X3(:,2)+y*ones(size(X1(:,1))),X3(:,3)+z*ones(size(X1(:,1))),'y',X4(:,1)+x*ones(size(X1(:,1))),X4(:,2)+y*ones(size(X1(:,1))),X4(:,3)+z*ones(size(X1(:,1))),'y',X5(:,1)+x*ones(size(X5(:,1))),X5(:,2)+y*ones(size(X5(:,1))),X5(:,3)+z*ones(size(X5(:,1))),'y',X6(:,1)+x*ones(size(X5(:,1))),X6(:,2)+y*ones(size(X5(:,1))),X6(:,3)+z*ones(size(X5(:,1))),'y',X7(:,1)+x*ones(size(X5(:,1))),X7(:,2)+y*ones(size(X5(:,1))),X7(:,3)+z*ones(size(X5(:,1))),'y',X8(:,1)+x*ones(size(X5(:,1))),X8(:,2)+y*ones(size(X5(:,1))),X8(:,3)+z*ones(size(X5(:,1))),'y',X9(:,1)+x*ones(size(X5(:,1))),X9(:,2)+y*ones(size(X5(:,1))),X9(:,3)+z*ones(size(X5(:,1))),'y',X10(:,1)+x*ones(size(X5(:,1))),X10(:,2)+y*ones(size(X5(:,1))),X10(:,3)+z*ones(size(X5(:,1))),'y',X11(:,1)+x*ones(size(X5(:,1))),X11(:,2)+y*ones(size(X5(:,1))),X11(:,3)+z*ones(size(X5(:,1))),'y',X12(:,1)+x*ones(size(X5(:,1))),X12(:,2)+y*ones(size(X5(:,1))),X12(:,3)+z*ones(size(X5(:,1))),'y',Xb(:,1)+x*ones(size(Xb(:,1))),Xb(:,2)+y*ones(size(Xb(:,1))),Xb(:,3)+z*ones(size(Xb(:,1))),'y',Xc(:,1)+x*ones(size(Xc(:,1))),Xc(:,2)+y*ones(size(Xc(:,1))),Xc(:,3)+z*ones(size(Xc(:,1))),'r',Xpl(:,1)+x*ones(size(Xpl(:,1))),Xpl(:,2)+y*ones(size(Xpl(:,1))),Xpl(:,3)+z*ones(size(Xpl(:,1))),'r',Xpr(:,1)+x*ones(size(Xpr(:,1))),Xpr(:,2)+y*ones(size(Xpr(:,1))),Xpr(:,3)+z*ones(size(Xpr(:,1))),'r')
hold on
x=eta(1,1:i);y=eta(2,1:i);z=eta(3,1:i);
plot3(etad(1,:),etad(2,:),etad(3,:),'r-.','linewidth',0.5);
f=plot3(x,y,z,'b-','linewidth',1);
plot3([-4.5,4.5],[-0.5,8.5],[-0.5,4.5],'w.');
set(gca, 'XDir','reverse');
set(gca, 'ZDir','reverse');
view(45,45) 
grid on
axis equal
xlabel('x,[m]');
ylabel('y,[m]');
zlabel('z,[m]');
set(gca,'fontsize',12,'fontname','Times New Roman');
hold off;
pause(0.01)
Fg(i) = getframe(gcf);
writeVideo(video,Fg(i));
end

close(video);
close(gcf)



figure
plot(t,V,'linewidth',2);
xlabel('Time,[s]');
ylabel('Absolute resultant velocity,[m/s]');

%% errors

figure
e1=etad-eta(:,1:end-1);
plot(t,e1(1,:),'r-',t,e1(2,:),'b-.',t,e1(3,:),'g--','linewidth',1); % position errors
legend('x_e,[m]','y_e,[m]','z_e,[m]')
xlabel('Time,[s]');
ylabel('Position error (\eta_d-\eta)');
grid on
set(gca,'fontsize',12,'fontname','Times New Roman');

%% Position and velocities
figure
plot(t, eta(1:3,1:i));
legend('x(m)', 'y(m)', 'z(m)');
set(gca,'fontsize',12,'fontname','Times New Roman');
grid on
xlabel('Time,[s]');
ylabel('\eta');

figure
plot(t, nu(1:3,1:length(t)));
legend('u(m/s)', 'v(m/s)', 'w(m/s');
set(gca,'fontsize',12,'fontname','Times New Roman');
grid on
xlabel('Time,[s]');
ylabel('\nu');

%% Frequency of oscillation
figure
plot(t,freq11,'r-',t,freq22,'b-',t,freq33,'g-'); % Amplitude
legend('caudal fin','pectoral fin (port side)','pectoral fin (starboard side)')
xlabel('Time,[s]');
ylabel('Frequency of oscillations,[Hz]');
grid on
set(gca,'fontsize',12,'fontname','Times New Roman');
