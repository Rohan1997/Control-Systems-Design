%%%%%%%%%%%%%%%%%%  1   %%%%%%%%%%%%%%%%%%%%%%

%% PID controller design
%% Open loop function
Z = [ ] ; %zeros
P = [ 0 -2 -4 -5] ; % poles
K = 1 ; %gain
G = zpk(Z,P,K) ;

%% Closed loop tranfer function
Gcl = feedback(G,1)

% Step Response
figure
step(Gcl)

figure
rlocus(G)

OS = 10 ;
zeta = -log(OS/100)/(pi^2+(log(OS/100))^2)^0.5

figure
rlocus(G)
sgrid( zeta , 2 )

%% P controller...
% Find the Gain from the plot
gain = 23.4 % For an overshoot of 7.42

%% Closed loop step response . . .
Gp = feedback (G*gain,1) ;
figure
step(Gp)

%%%%%%%%%%%%%%%%%%  2   %%%%%%%%%%%%%%%%%%%%%%


%% PD controller

%Specifications . . .
Ts = 5.5 ;
omega_n = (4/Ts)/zeta ;

figure
rlocus(G)
sgrid( zeta , omega n )

%dominant pole locations
sigma = zeta*omega_n
omega_d = omega_n*(sqrt(1-zeta^2) )

dom_pole = complex(-sigma , omega_d )

%Poles
p1 = 0 ;
p2 = -2;
p3 = -4;
p4 = -5;

theta1 = angle ( dom_pole-p1 ) ;
theta2 = angle ( dom_pole-p2 ) ;
theta3 = angle ( dom_pole-p3 ) ;
theta4 = angle ( dom_pole-p4 ) ;

angle_sum = (-theta1-theta2-theta3-theta4 ) %in radian

% Compensated zero
theta0 = pi-angle_sum ;

z1 = (omega_d/tan(theta0)) + sigma
%zero with sign
z1 = -1*z1

%% Gain Calculation
s = dom_pole
Gain = abs( ( s-p1 )*( s-p2 )*( s-p3 )*( s-p4 )/( 1*( s-z1 ) ) )

% New Open Loop transfer function
Z1 = [z1]; %zeros
P1 = [ p1 p2 p3 p4 ]; % poles
K1 = Gain  %gain
Gpd = zpk(Z1,P1,K1);

%% Step Response and Pole Zero Map a f t e r PD c o n t r o l
Gcl2 = feedback(Gpd , 1 ) ;
figure
step(Gcl2 )
figure
pzmap(Gcl2 )


%%%%%%%%%%%%%%%%%%  3   %%%%%%%%%%%%%%%%%%%%

%% Ramp Response
t1 =0:1:10000;
ramp = t1 ; % Your input signal
model=Gcl2 ; % Your transfer function
[ y1 , t1 ]=lsim (model , ramp , t1 ) ;
figure
plot ( t1 , y1'-ramp)


%% PID Compenesation :
z2 = -0.001
p5 = 0

%% Open loop transfer function . . .
Z2 = [ z1 z2 ] ; % zeros
P2 = [ p1 p2 p3 p4 p5 ] ; % po l e s
K2 = Gain ; %gain
Gpid = zpk (Z2 ,P2 ,K2 ) ;

%% Closed Loop
Gcl3 = feedback(Gpid , 1 ) ;
model=Gcl3 ; % Your transfer function
[y2,t1]=lsim(model , ramp , t1 ) ;
figure
plot( t1 , y2'-ramp)

%% Step Response and Pole Zero Map a f t e r PID c o n t r o l
figure
step (Gcl3 )
figure
pzmap(Gcl3 )