void foc(double a, double b, double c, double theta, double q_ref, double &u double &v double &w) {
    // Clarke Transform
    double alpha = a;
    double beta  = 0.5774*(a+2*b);

    // Original Park Transform

    double ct = cos(theta);
    double st = sin(theta);
    double ctm2pi3 = cos(theta-2*pi/3);
    double stm2pi3 = sin(theta-2*pi/3);

    d = (alpha*ct + beta*ctm2pi3)*2/3;
    q = (alpha*st + beta*stm2pi3)*2/3;

    e_d = 0 - d;    // d_ref is GND
    e_q = q_ref - q;

    // inverse Park Transform
    alpha = e_d*ct + e_q*st;
    beta = e_d*ctm2pi3 + e_q*stm2pi3;

    // inverse Clarke Transform
    u = alpha;
    v = -0.5*alpha+0.8660*beta;
    w = -0.5*alpha-0.8660*beta;
}
