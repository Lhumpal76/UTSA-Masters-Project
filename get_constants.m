function [Xit,Yit,Zit,Xp,Yp,Zp,time,velocity,gamma,beta,alpha,...
    Xp_array,Yp_array,Zp_array,Xbt,Ybt,Zbt] = get_constants()
    
    Xit = 5;
    Yit = 5;
    Zit = 0;

    %plane location with respect to earth coordinate system
    %plane location at T1
    Xp = 0;
    Yp = -10;
    Zp = 10;
    ploc_initial = [Xp,Yp,Zp];
  
    %straight path constants
    velocity = 2;
    
    gamma = deg2rad(0);  %roll
    beta = deg2rad(0);  %pitch
    alpha = deg2rad(0); %yaw
    
    %circle path constants
    Xr = Xit - Xp;
    Yr = Yit - Yp;
    radius = sqrt(Xr^2+Yr^2);
    
    %----------straight flight path, no YPR, parallel to y axis
    time = 0:1:35;
    [Xp_array,Yp_array,Zp_array] = straight_flight_path(ploc_initial,velocity,time);
    
    %----------circle flight path, no YPR, one complete revolution  0 to 2pi
    %time = 0:pi/20:6*pi;
    %[Xp_array, Yp_array, Zp_array] = circle_flight_path(ploc_initial,time,radius);

    [Xbt,Ybt,Zbt] = transformation_matrices(Xit,Yit,Zit,gamma,beta,alpha);
end

function [x,y,z] = transformation_matrices(Xt,Yt,Zt,gamma,beta,alpha)
        
        
        R1Gamma = [1 0 0;
            0 cos(gamma) sin(gamma);                %Gamma = Roll
            0 -sin(gamma) cos(gamma)];
        
        R2Beta = [cos(beta) 0 -sin(beta);
            0 1 0;                          %Beta = Pitch
            sin(beta) 0 cos(beta)];
        
        R3Alpha = [cos(alpha) sin(alpha) 0;
            -sin(alpha) cos(alpha) 0;           %Alpha = Yaw
            0 0 1];
        
        vector = [Xt;Yt;Zt];
        
        transformed_vector = R1Gamma*R2Beta*R3Alpha*vector;
        x = transformed_vector(1);
        y = transformed_vector(2);
        z = transformed_vector(3);
    end