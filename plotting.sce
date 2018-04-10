//////////////////////////////   1    ///////////////////////////////

// Open loop transfer function

s=poly( 0 ,'s')

num = 1 ;
den = s*(s+2)*(s+4)*(s+5);

g1 = syslin( 'c' ,num/den ) // syslin is for linear system definition

// Closed loop transfer function
g2 = g1/(1+g1)

t =0 : 0.1 : 500
y=csim( 'step',t, g2 ) // simulation ( step response ) of linear system

f0=scf(0) ; // create figure with id==0 and make it the current one
plot2d(t,y) // plotting step response

//////////////////////////////   2   ///////////////////////////////

f1=scf(1) ;
evans(g1) ; // plot the root locus


//////////////////////////////   3   ///////////////////////////////

// Calculation of zeta
pi = 3.1 4 ;
OS = 10 ;
zeta = -log(OS/100)/(pi^2 +(log(OS/100) )^2 )^0.5 ;

// Iso-over shootline
f2=scf(2) ;
evans( g1 ) ;
sgrid ( zeta , 2 ) // the value 2 corresponds to omega_n .
// Just to meet the overshoot specification omega_n is not crucial .


//////////////////////////////   4   ///////////////////////////////

// P controller

K = 25.38

// Closed loop step response

CL1 = K*g1/(1+K*g1 )

gs1=csim ( 'step' , t ,CL1 ) ;
f3=scf( 3 ) ;
plot2d ( t , gs1 ) ;

//////////////////////////////   5   ///////////////////////////////

// Specifications
OS = 10 ;
Ts = 5.5 ;

// for OS = 10 , zeta was already calculated
// Calculation of omega_n

omega_n = (4/Ts )/zeta

// Plot the root locus with iso-zeta and iso-omega_n lines
f4=scf(4) ;
evans( g1 )
sgrid( zeta , omega n )




// Dominant pole locations
sigma = zeta*omega_n
omega_d = omega_n*( sqrt(1-zeta^2) )

dompole = complex(-sigma , omega_d )

// Original poles of the system
p1 = complex ( 0 , 0 ) ;
p2 = complex ( -2 ,0) ;
p3 = complex ( -4 ,0) ;
p4 = complex ( -5 ,0) ;

// Angle subtended by e x i s t i n g po l e s :
theta1 = phasemag ( dom_pole-p1 ) ;
theta2 = phasemag ( dom_pole-p2 ) ;
theta3 = phasemag ( dom_pole-p3 ) ;
theta4 = phasemag ( dom_pole-p4 ) ;

angle_sum = (-theta1-theta2-theta3-theta4 )

// To satisfy the angle criterion a zero needs to be added
theta0 = 180 - angle_sum ;

// Calculating the location of the zero
z1 = (omega_d/ tan(theta0*pi/180) ) + sigma

// zero with sign . . .
z1 = -z1

// Calculating the gain to satisfy the gain criterion
Kd = abs( ( ( dom_pole-p1 )*( dom_pole-p2 )*( dom_pole-p3 )*( dom_pole-p4 ) )/( dom_pole-z1 ) )


//////////////////////////////   6   ///////////////////////////////

// Open loop transfer function with PD controller

n = Kd*( s-z1 ) ; // PD controller
g3 =syslin( 'c' , n/den ) // define t f

// Plot the step response of the closed loop system with PD controller
CL2 = g3/(1+g3 )
gs2 = csim ( 'step' , t ,CL2 ) ;
f5 = scf( 5 ) ;
plot2d ( t , gs2 )


//////////////////////////////   7   ///////////////////////////////

// Plot the ramp response of the closed loop system with PD controller
t1 = 0 : 1 : 10000
gs3 = csim( t1 , t1 ,CL2)
f5 = scf( 5 ) ;
plot2d ( t1 , gs3 ) ;

// Plot the error
f7=scf ( 7 ) ;
plot2d ( t1 , gs3-t1 ) ;




// PID Controller :
z2 = -0.001;

// Open loop transfer function with PID controller
n1 = Kd*( s-z1 )*( s-z2 ) ;
d1 = s*den ;
g4 =syslin ( 'c' , n1/d1 )
// Closed loop transfer function with PID controller
CL3 = g4/(1+g4 )
gs4=csim( t1 , t1 ,CL3 ) ;
gs5=csim ( 'step' , t ,CL3 ) ;
f8=scf ( 8 ) ;
plot2d ( t1 , gs4 ) //ramp response
f9=scf( 9 )
plot2d ( t , gs5 ) ; // step response
f10=scf (10)
plot2d ( t1 , gs4-t1 ) ; // error plot