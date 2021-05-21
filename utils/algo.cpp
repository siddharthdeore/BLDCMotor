/* Field Oriented Control Algorithm */
void foc(double a, double b, double c, double theta, double q_ref, double &u double &v double &w) {
    double ct = cos(theta);
    double st = sin(theta);
 
    /*  Clarke Transform - abc to alpha beta */
    alpha = ((2*a) - b - c)/3;
    beta  = (b-c)*0.5773;
    /* Clarke to Park Transform - alpha beta to dq */
    // phase a alinged with q axis
    //d = alpha*st - beta*ct;
    //q = alpha*ct + beta*st;
    // phase a aligned with d axis
     d = alpha*ct + beta*st;
     q =-alpha*st + beta*ct;
    
    // flux PI Regulator
    d=0-d;
    q=q_ref-q;
    /* Park to Clarke Transform - alpha beta to dq */
    // phase a alinged with q axis
    //alpha = d*st+q*ct; 
    //beta = -d*ct+q*st;
    // phase a aligned with d axis
    alpha = d*ct-q*st; 
    beta =  d*st+q*ct;

    /* Inverse Clarke Transform -  alpha beta to abc */
    u =   alpha;
    v = - alpha*0.5 + 0.8661*beta; 
    w = - alpha*0.5 - 0.8661*beta;
    //

}
/* Make sure correct phase alignment of dq frame, find out some idea to autocorrect
// This is using direct Park
// Park - abc to dq, phase a align to d axis
d =   (2*a*cos(theta))/3 + (2*b*cos(theta - (2*pi)/3))/3 + (2*c*cos(theta + (2*pi)/3))/3;
q = - (2*a*sin(theta))/3 - (2*b*sin(theta - (2*pi)/3))/3 - (2*c*sin(theta + (2*pi)/3))/3;

// Park Inverse dq to abc, phase a align to d axis
a = d*cos(theta) - q*sin(theta);
b = d*cos(theta - (2*pi)/3) - q*sin(theta - (2*pi)/3);
c = d*cos(theta + (2*pi)/3) - q*sin(theta + (2*pi)/3);

// Park - abc to dq, phase a align to q axis
d = (1633*a*sin(theta))/2000 + (1633*b*sin(theta - (2*pi)/3))/2000 + (1633*c*sin(theta + (2*pi)/3))/2000;
q = (1633*a*cos(theta))/2000 + (1633*b*cos(theta - (2*pi)/3))/2000 + (1633*c*cos(theta + (2*pi)/3))/2000;

// Park Inverse dq to abc, phase a align to q axis
a =                       (1633*q*cos(theta))/2000 + (1633*d*sin(theta))/2000;
b = (1633*q*cos(theta - (2*pi)/3))/2000 + (1633*d*sin(theta - (2*pi)/3))/2000;
c = (1633*q*cos(theta + (2*pi)/3))/2000 + (1633*d*sin(theta + (2*pi)/3))/2000;

*/
