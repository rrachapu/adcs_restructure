import numpy as np

def rk45(numelm, x, dxdt, t, dt, foo, props) :
    a2=0.2
    a3=0.3
    a4=0.6
    a5=1.0
    a6=0.875
    b21=0.2
    b31=3.0/40.0
    b32=9.0/40.0
    b41=0.3
    b42=-0.9
    b43=1.2
    b51=-11.0/54.0
    b52=2.5
    b53=-70.0/27.0
    b54=35.0/27.0
    b61=1631.0/55296.0
    b62=175.0/512.0
    b63=575.0/13824.0
    b64=44275.0/110592.0
    b65=253.0/4096.0
    c1=37.0/378.0
    c3=250.0/621.0
    c4=125.0/594.0
    c6=512.0/1771.0
    dc5=-277.0/14336.0
    dc1=c1-2825.0/27648.0
    dc3=c3-18575.0/48384.0
    dc4=c4-13525.0/55296.0
    dc6=c6-0.25
	# if numelm is more than 10, then we need to change these array values below
    ak2 = np.zeros(numelm)
    ak3 = np.zeros(numelm)
    ak4 = np.zeros(numelm)
    ak5 = np.zeros(numelm)
    ak6 = np.zeros(numelm)
    x_temp = np.zeros(numelm)
    
    # for i in range(0, numelm):
    #     ak2[i] = 0
    #     ak3[i] = 0
    #     ak4[i] = 0
    #     ak5[i] = 0
    #     ak6[i] = 0
    #     x_temp[i] = 0
    
    # for i in range(0, numelm):
    #     x_temp[i] = (x[i]) + b21*dt*(dxdt[i])
    
    x_temp = x + b21*dt*dxdt

    ak2 = foo(t + a2*dt, x_temp, props)

    # for i in range(0, numelm):
    #     x_temp[i] = (x[i]) + dt*(b31*(dxdt[i]) + b32*(ak2[i]))
    
    x_temp = x + dt*b31*dxdt + b32*ak2
    
    ak3 = foo(t+a3*dt, x_temp, props)

    # for i in range(0, numelm):
    #     x_temp[i] = (x[i]) + dt*(b41*(dxdt[i]) + b42*(ak2[i]) + b43*(ak3[i]))
        
    x_temp = x + dt*b41*dxdt + b42*ak2 + b43*ak3
    
    ak4 = foo(t+a4*dt,x_temp, props)

    # for (i=0; i < numelm; i++) {
    #     x_temp[i] = (x[i]) + dt*(b51*(dxdt[i]) + b52*(ak2[i]) + b53*(ak3[i]) + b54*(ak4[i]));
    # }
    
    x_temp = x + dt*b51*dxdt + b52*ak2 + b53*ak3 + b54*ak4
    
    ak5 = foo(t+a5*dt,x_temp, props)

# 	for (i=0; i < numelm; i++) {
# 		x_temp[i] = (x[i]) + dt*(b61*(dxdt[i]) + b62*(ak2[i]) + b63*(ak3[i]) + b64*(ak4[i]) + b65*(ak5[i]));
# 	}
    
    x_temp = x + dt*b61*dxdt + b62*ak2 + b63*ak3 + b64*ak4 + b65*ak5
    
    ak6 = foo(t + a6 * dt, x_temp, props);

# 	for (i=0; i < numelm; i++) {
# 		y[i] = (x[i]) + dt*(c1*(dxdt[i]) + c3*(ak3[i]) + c4*(ak4[i]) + c6*(ak6[i]));
# 	}
    
    y = (x) + dt*(c1*(dxdt) + c3*(ak3) + c4*(ak4) + c6*(ak6));

# 	for (i=0; i < numelm; i++) {
# 		y_err[i] = dt*(dc1*(dxdt[i]) + dc3*(ak3[i]) + dc4*(ak4[i]) + dc5*(ak5[i]) + dc6*(ak6[i]));
# 	}
    
    y_err = dt*(dc1*(dxdt) + dc3*(ak3) + dc4*(ak4) + dc5*(ak5) + dc6*(ak6));


    # returns predicted value at (t + dt)
    # returns predicted error for this value
    return [y, y_err]

def stepper45(numel, x, dxdt, t, dt_try, eps, x_scale, foo, props):
    #print("stepper45")
    
    # numel: the number elements in vector
    # x: current x values
    # dxdt: current dxdt values
    # t: time for which approximation has happened
    # eps: desired fractional accuracy aka "absolute precision"
    # x_scale:
    # foo: function for dxdt given x and t
    dxdt_save = np.zeros(numel)
    x_save = np.zeros(numel)
    x_temp = np.zeros(numel)
    x_err = np.zeros(numel)      
        
# 	for (i=0; i < numel; i++){
# 			dxdt_save[i] = 0;
# 			x_save[i] = 0;
# 			x_temp[i] = 0;
# 			x_err[i] = 0;
# 	}
    
    t_save = t; # save initial t

# 	for (i=0; i < numel; i++) {
# 		x_save[i] = x[i];
# 		dxdt_save[i] = dxdt[i];
# 	}
    
    x_save = x*1
    dxdt_save = dxdt*1
    
	# dt = dt_try;
    dt = dt_try
    m = 0
    num_runs = 50

    while (True) :

		# Try integrating over interval using 1 step sized dt
        [x_temp, x_err] = rk45(numel, x_save, dxdt_save, t, dt, foo, props) 
        # single step

        # Compare errors between 2 step and 1 step methods
        errmax = 0.0;
        for i in range(0, numel):
            if ((x_err[i]) > errmax):
                # if (m > 40):
                     # print("-----error")
                     # print(x_err[i])
                     # print(dt)
                errmax = (x_err[i])
        errmax /= eps

        # If the error is too large, try again & make step size smaller
        # If the error is small, move on and make next step size larger
        if (errmax <= 1.0):
            t = t + dt
            x = x_temp * 1
            # print("entered 2")
            dt_act = dt
            if (errmax > 6e-4) and (m <= num_runs):
                dt_next = 0.9*dt*np.exp(-0.20*np.log(errmax))
                # print("entered 3")
            else:
                dt_next = 4.0*dt
                if (m > num_runs):
                    print(".....error: " + str(np.max(x_err)))
                # print("entered 4")
                break
		
        dt = 0.9*dt*np.exp(-0.25*np.log(errmax))
        m += 1
    return [x, dt_act, dt_next, t]

def ode45(numel, x_start, t1, t2, eps, dt1, dt_min, fun, props):
    #print("ode45")
    t, dt_next, dt_act, dt = 0,0,0,0

    x_scale = np.zeros(numel)
    x = np.zeros(numel)
    dxdt = np.zeros(numel)

    t = t1
    if (t2 > 1):    
        dt = abs(dt1)
    else:
        dt = -abs(dt1)

	# dt = (t2 > t1) ? fabs(dt1) : -fabs(dt1); //integrate backwards if desired

    x = x_start * 1

	# Take at most MAXstep steps
	# MAXstep = 1000;
    MAXstep = 2
    for nstep in range(0,MAXstep):
        
        # USB_sprintf("A");
        
        dxdt = fun(t,x, props) # calculate initial derivatives
# 		for (i=0; i < numel; i++) { //Set error tolerancing
# 			x_scale[i] = fabs(x[i]) + fabs(dxdt[i]*dt) + 1.0e-30; //scale of acceptable error
# 		}
        x_scale = abs(x) + abs(dxdt*dt) + 1e-30
        
        # USB_sprintf("B");

		# can save intermediate results here

		# check if next step will overshoot final time
        if ((t+dt-t2)*(t+dt-t1) > 0.0): # if it does, set the step size to hit the end time
            dt = t2 - t;
		
        
        # USB_sprintf("C");

		# Take a step with stepper routine
        [x, dt_act, dt_next, t] = stepper45(numel, x, dxdt, t, dt, eps, x_scale, fun, props)
        
        # USB_sprintf("D");

		# If step is successful, return current value of x to x_start and RETURN!
        if ((t-t2)*(t2-t1) >= 0.0):
            x_start = x
            return x_start
		
        
        # USB_sprintf("E");

        dt = dt_next;

    print("ODE TOO MANY STEPS")
    return x_start

	# USB_sprintf("ODE45 too many steps\n");


