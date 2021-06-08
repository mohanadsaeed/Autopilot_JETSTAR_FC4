function [tout, yout] = rk1_4(ode_function, tspan, y0, h, rk)
    switch rk
        case 1
            n_stages=1;
            a=0;
            b=0;
            c=1;
        case 2
            n_stages=2;
            a=[0 1];
            b=[0 1]';
            c=[1/2 1/2];
        case 3
            n_stages=3;
            a=[0 1/2 1];
            b=[0 0 ; 1/2 0 ; -1 2];
            c=[1/6 2/3 1/6];
        case 4
            n_stages=4;
            a=[0 1/2 1/2 1];
            b=[0 0 0 ; 1/2 0 0 ; 0 1/2 0 ; 0 0 1];
            c=[1/6 1/3 1/3 1/6];
        otherwise
            error('The parameter rk must have the value 1, 2, 3 or 4.')
    end
    t0=min(tspan);
    tf=max(tspan);
    t=t0;
    y=y0;
    tout=t;
    yout=y';
    while t<tf
        ti=t;
        yi=y;
        for i=1:n_stages
            t_inner=ti+a(i)*h;
            y_inner=yi;
            for j=1:i-1
                y_inner=y_inner+h*b(i,j)*f(:,j);
            end
            f(:,i)=feval(ode_function,t_inner,y_inner);
        end
        h=min(h,tf-t);
        t=t+h;
        y=yi+h*f*c';
        tout=[tout;t];
        yout=[yout;y'];
    end
end