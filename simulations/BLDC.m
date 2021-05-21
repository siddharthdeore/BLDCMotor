%% author : Siddharth Deore
clc;clear all;
global integ_e
integ_e=0;
tau=0;
X0=[0;0;0;0;0];
tspan=linspace(0,5,100);
opts = odeset('RelTol',1e-6,'AbsTol',1e-6);
[t,X] = ode45(@bldcDynamics,tspan,X0,opts);
subplot(3,1,1);
plot(t,X(:,1:3));
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
L = 2.72e-3;         % self-inductance phase  (L_aa = L_bb = L_cc) [H]
M = 1.5e-3;        % mutual-inductance ( L_ab = L_bc = L_ca = M) [H]
L1=L-M;             % [H]
R = 0.75;           % stator resistance per phase [ohm]
J = 0.00284;         % inertia [Kg-m/sec^2]
B_coef = 0.02;      % friction coefficient  [N-m/rad/sec]
P = 4;              % number of poles
lamda =  0.105;     % flux linkage wb
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

% Velocity PI Controller
omega_e = 10-w;
PI_vel=(omega_e*1.5+integ_e*0.0009);
integ_e=integ_e+omega_e;

[VA,VB,VC]=foc(i_a,i_b,i_c,theta,PI_vel);
U = [VA;VB;VC;0];

% Space Vector
%ctrl = -switching(theta+pi/2);
%th=theta;ctrl=-[sin(th);sin(th-2*pi/3);sin(th+2*pi/3)];
%ctrl=svpwm(theta+pi/2);
%U = [ctrl;0];

X_dot = A*X+[B*U;0]+[C*emf;0;0];
end

%% field oriented control
function [A,B,C] = foc(a,b,c,theta,tau)
    theta = theta - pi/2; % Phase correction for a alinged with q axis
    ct = cos(theta);
    st = sin(theta);
 
    % /*  Clarke Transform - abc to alpha beta */
    alpha = ((2*a) - b - c)/3;
    beta  = (b-c)*0.5773;
    % /* Clarke to Park Transform - alpha beta to dq */
    % // phase a alinged with q axis
    d = alpha*st - beta*ct;
    q = alpha*ct + beta*st;
    % // phase a aligned with d axis
    d = alpha*ct + beta*st;
    % q =-alpha*st + beta*ct;
    
    % // flux PI Regulator
    d=0-d;
    q=tau-q;
    % /* Park to Clarke Transform - alpha beta to dq */
    % // phase a alinged with q axis
    alpha = d*st+q*ct; 
    beta = -d*ct+q*st;
    % // phase a aligned with d axis
    %alpha = d*ct-q*st; 
    %beta =  d*st+q*ct;
    A =   alpha;
    B = - alpha*0.5 + 0.8661*beta; 
    C = - alpha*0.5 - 0.8661*beta;
%{
% Clarke a-b-c to alpha-beta
[alpha,beta]=clarke(a,b,c);

% Park alpha-beta to d-q
[d,q]=park(alpha,beta,theta);

% Torque PI Controller
q_err=(tau-q);
q_command = q_err*1; % PI

% inverse Park d-q to alpha-beta
[a,b] = inv_park(-d,-q_command,theta);

% inverse Clarke alpha-beta to a-b-c
[A,B,C] = inv_clarke(a,b);
end
function [alpha,beta]= clarke(a,b,c)
% abc to alpha beta
alpha = a;
beta =  0.5774*(a+2*b);
%}
end
function [a,b,c]= inv_clarke(alpha,beta)
% alpha-beta to  abc
%
a = alpha;
b = -0.5*alpha+0.8660*beta;
c = -0.5*alpha-0.8660*beta;
end
function [d,q]= park(alpha,beta,theta)
% alfa beta to d

% Original Park Transform
T = 2/3*[
    cos(theta), cos(theta-2*pi/3), cos(theta+2*pi/3);
    sin(theta), sin(theta-2*pi/3), sin(theta+2*pi/3);
    0.5,0.5,0.5];
y=T*[alpha;beta;0];
d = y(1);
q = y(2);
end
function [alpha,beta]= inv_park(d,q,theta)
% dq to alfa beta
T=[
    cos(theta),         sin(theta),          1;
    cos(theta-2*pi/3), sin(theta-2*pi/3),    1;
    cos(theta+2*pi/3), sin(theta+2*pi/3),    1];
y=T*[d;q;0];
alpha = y(1);
beta = y(2);

% Simple but inverse not working
%alpha = d * cos(theta) - q * sin(theta)
%beta = q * cos(theta) + d * sin(theta)
end
function y = clip(x,bl,bu)
  % return bounded value clipped between bl and bu
  y=min(max(x,bl),bu);
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
function y = clarke1(a,b,c)
% abc to alpha beta
T = [
    0.6667   -0.3333   -0.3333;
         0    0.5774   -0.5774;
    0.3333    0.3333    0.3333];
y=T*[a;b;c];
end
function y = inv_clarke1(a,b)
% alpha-beta to  abc
T=[
    1.0000         0    1.0000;
   -0.5000    1.2247    1.0000;
   -0.5000   -1.2247    1.0000];
y=T*[a;b;0];
end
function y = clarke_to_park(a,b,theta)
% alfa beta to d q
T=[sin(theta),sin(theta-2*pi/3), sin(theta+2*pi/3);
    cos(theta), cos(theta-2*pi/3),cos(theta+2*pi/3);
    0.5,0.5,0.5];
y=T*[a;b;0];
end
function y = inv_park1(d,q,theta)
% dq to alfa beta
T=[
    sin(theta),         cos(theta),         1;
    sin(theta-2*pi/3),  cos(theta-2*pi/3),  1;
    sin(theta+2*pi/3),  cos(theta+2*pi/3),  1];
y=T*[d;q;0];
end
