%% author : Siddharth Deore
clc;clear all;
global integ_e
integ_e=0;
tau=0;
X0=[0;0;0;0;0];
tspan=linspace(0,5,500);
opts = odeset('RelTol',1e-6,'AbsTol',1e-6);
[t,X] = ode45(@bldcDynamics,tspan,X0,opts);
subplot(3,1,1);
plot(t,X(:,[1:3]));
subplot(3,1,2);
plot(t,X(:,4));
subplot(3,1,3);
plot(t,X(:,5));

%plot((min([X(:,1:3)]')+max([X(:,1:3)]'))/2);
function X_dot = bldcDynamics(t,X)
global integ_e
% States
%%{
i_a=X(1);       % current a [Amp]
i_b=X(2);       % current b [Amp]
i_c=X(3);       % current c [Amp]
%%}
w = X(4);       % angular velocity [rad/sec]
theta = X(5);   % rotor angle [rad]

% Motor Constants
L = 1.0e-3;     % self-inductance phase  (L_aa = L_bb = L_cc) [H]
M = 0.25e-3;    % mutual-inductance ( L_ab = L_bc = L_ca = M) [H]
L1=L-M;         % [H]
R = 0.64;       % stator resistance per phase [ohm]
J = 5.0e-4;     % inertia [Kg-m/sec^2]
B_coef = 0.002; % friction coefficient  [N-m/rad/sec]
lamda = 0.105;  % flux linkage wb
P = 100;          % number of poles
fa=sin(theta);
fb=sin(theta-2*pi/3);
fc=sin(theta+2*pi/3);
lfa=lamda*fa/J;
lfb=lamda*fb/J;
lfc=lamda*fc/J;
A = [
        -R/L1,   0,  0,  lfa,  0;
        0,   -R/L1,  0,  lfb,  0;
        0,   0,  -R/L1,  lfc,  0;
        lfa, lfb, lfc,  -B_coef/J, 0;
        0,  0,  0,  P/2,    0 ];

B = [
        1/L1,    0,  0,  0;
        0,    1/L1,  0,  0;
        0,    0,  1/L1,  0;
        0,    0,  0, 1/L1;];
C = [
        -1/L1,0,0;
        0,-1/L1,0;
        0,0,-1/L1;
    ];

emf = w*lamda*[fa;fb;fc];
%ctrl = -switching(theta+pi/2);
%th=theta;ctrl=-[sin(th);sin(th-2*pi/3);sin(th+2*pi/3)];
omega_e = 1-w;
ctrl=svpwm(theta+pi/2) * (omega_e*3+integ_e*0.0002);
U = [ctrl;0];
X_dot = A*X+[B*U;0]+[C*emf;0;0];
integ_e=integ_e+omega_e;
end

%% unit trapizoidal Back EMF
function y=bemf_a(theta)
ct=cos(theta);
st=sin(theta);
theta=atan2(st,ct)+pi;
    if(theta>=0)&&(theta<=pi/6)
        y=6/pi*theta;
    elseif(theta>pi/6)&&(theta<=5*pi/6)
        y=1;
    elseif(theta>5*pi/6)&&(theta<=7*pi/6)
        y=-pi/6*theta+6;
    elseif(theta>7*pi/6)&&(theta<=11*pi/6)
        y=-1;
    elseif(theta>11*pi/6)&&(theta<=2*pi)
        y=6/pi*theta-12;
    end
end


function y=emf_ref_d(theta)
if (theta>-180)&&(theta<=-120)
    y=[-1;0;1];
elseif(theta>-120)&&(theta<=-60)
    y=[0;-1;1];
elseif(theta>-60)&&(theta<=0)
    y=[1;-1;0];
elseif(theta>0)&&(theta<=60)
    y=[1;0;-1];
elseif(theta>60)&&(theta<=120)
    y=[0;1;-1];
elseif(theta>120)&&(theta<=180)
    y=[-1;1;0];
end
end

function y=switching(theta)
ct=cos(theta);
st=sin(theta);
theta=(atan2(st,ct)+pi)*180/pi;

    if(theta>=0)&&(theta<=60)
        y=[1;-1;0];
    elseif(theta>60)&&(theta<=120)
        y=[1;0;0];
    elseif(theta>120)&&(theta<=180)
        y=[0;1;-1];
    elseif(theta>180)&&(theta<=240)
        y=[-1;1;0];
    elseif(theta>240)&&(theta<=360)
        y=[-1;0;1];
    elseif(theta>300)&&(theta<=360)
        y=[0;-1;1];
    end
end

function y=svpwm(theta)
a=sin(theta);
b=sin(theta-2*pi/3);
c=sin(theta+2*pi/3);
v_offset = (min([a,b,c])+max([a,b,c]))/2;
y=[a;b;c]-v_offset;
end
function y=sinpwm(theta)
a=sin(theta);
b=sin(theta-2*pi/3);
c=sin(theta+2*pi/3);
y=[a;b;c];
end
% SVPWM
%pwmSin=[128, 147, 166, 185, 203, 221, 238, 243, 248, 251, 253, 255, 255, 255, 253, 251, 248, 243, 238, 243, 248, 251, 253, 255, 255, 255, 253, 251, 248, 243, 238, 221, 203, 185, 166, 147, 128, 109, 90, 71, 53, 35, 18, 13, 8, 5, 3, 1, 1, 1, 3, 5, 8, 13, 18, 13, 8, 5, 3, 1, 1, 1, 3, 5, 8, 13, 18, 35, 53, 71, 90, 109];


function y = clip(x,bl,bu)
  % return bounded value clipped between bl and bu
  y=min(max(x,bl),bu);
 end
