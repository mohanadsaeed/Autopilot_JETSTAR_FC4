function [Xe,Ye,Ze,u,v,w,phai,theta,psi,p,q,r] = RBDsolver(Initial_States,Forces,Moments,m,I,t0,tf,h)
    tspan=t0:h:tf;
    states_final=zeros(length(tspan),12);
    states_final(1,:)=Initial_States;
    K=zeros(4,12);
    for i=1:(length(tspan)-1)
        K(1,:)=(h*feval(@odefunc,states_final(i,:)'))';
        K(2,:)=(h*feval(@odefunc,(states_final(i,:)'+0.5*K(1,:)')))'; 
        K(3,:)=(h*feval(@odefunc,(states_final(i,:)'+0.5*K(2,:)')))';
        K(4,:)=(h*feval(@odefunc,(states_final(i,:)'+K(3,:)')))';    
        states_final(i+1,:)=states_final(i,:)+(1/6)*(K(1,:)+2*K(2,:)+2*K(3,:)+K(4,:));
        % Euler Angles must be bounded by pi , -pi
        if (states_final(i+1,7)>=pi)
            states_final(i+1,7)=states_final(i+1,7)-2*pi;
        elseif (states_final(i+1,7)<=-pi)
            states_final(i+1,7)=states_final(i+1,7)+2*pi;
        elseif (states_final(i+1,8)>=pi)
            states_final(i+1,8)=states_final(i+1,8)-2*pi;
        elseif (states_final(i+1,8)<=-pi)
            states_final(i+1,8)=states_final(i+1,8)+2*pi;
        elseif (states_final(i+1,9)>=pi)
            states_final(i+1,9)=states_final(i+1,9)-2*pi;
        elseif (states_final(i+1,9)<=-pi)
            states_final(i+1,9)=states_final(i+1,9)+2*pi;
        end
    end
    Xe=states_final(:,1);
    Ye=states_final(:,2);
    Ze=states_final(:,3);
    u=states_final(:,4);
    v=states_final(:,5);
    w=states_final(:,6);
    phai=states_final(:,7);
    theta=states_final(:,8);
    psi=states_final(:,9);
    p=states_final(:,10);
    q=states_final(:,11);
    r=states_final(:,12);

    function [dstates_dt] = odefunc(states)
        DCM_be=[cos(states(8))*cos(states(9)) , (sin(states(7))*sin(states(8))*cos(states(9))-cos(states(7))*sin(states(9))) , (cos(states(7))*sin(states(8))*cos(states(9))+sin(states(7))*sin(states(9)));
             cos(states(8))*sin(states(9)) , (sin(states(7))*sin(states(8))*sin(states(9))+cos(states(7))*cos(states(9))) , (cos(states(7))*sin(states(8))*sin(states(9))-sin(states(7))*cos(states(9)));
             -sin(states(8))               ,  sin(states(7))*cos(states(8))                                               , cos(states(7))*cos(states(8))];

        Euler_Matrix=[1 ,  sin(states(7))*tan(states(8))   ,    cos(states(7))*tan(states(8));
                      0 ,  cos(states(7))                  ,   -sin(states(7)); 
                      0 ,  sin(states(7))/cos(states(8))   ,   cos(states(7))/cos(states(8))];
        
        Gravity_e=[0;0;m*9.807];
        
        Forces_tot=Forces+inv(DCM_be)*Gravity_e;

        dposition_dt=DCM_be*[states(4) ; states(5) ; states(6)];

        dvelocity_dt=Forces_tot./m-((cross([states(10),states(11),states(12)],[states(4),states(5),states(6)]))');

        dorientation_dt=Euler_Matrix*[states(10) ; states(11) ; states(12)];

        dpqr_dt=inv(I)*(Moments-cross([states(10) ; states(11) ; states(12)],(I*[states(10) ; states(11) ; states(12)])));

        dstates_dt=[dposition_dt ; dvelocity_dt ; dorientation_dt ; dpqr_dt];
    end               
end