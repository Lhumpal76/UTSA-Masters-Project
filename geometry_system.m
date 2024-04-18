function [theta1_goal, theta2_goal,Xbt,Ybt,Zbt] = geometry_system(t)

    [Xit,Yit,Zit,~,~,~,time,~,gamma,alpha,beta,Xp_array,...
        Yp_array,Zp_array,~,~,~] = get_constants();  
       
    Xp_requested = interp1(time,Xp_array,t);
    Yp_requested = interp1(time,Yp_array,t);
    Zp_requested = interp1(time,Zp_array,t);
    
    %change target location 
%     if t > (time(end)/2)
%         Xit = -20;
%         Yit = 20;
%         Zit = 0;
%     end
   
    
    %%%%%%%%%%%%%%%%%%%%%%%%%% RUN FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [~,m] = size(Xp_requested);

    %transform target from inertial F.O.R to body F.O.R
    [Xbt,Ybt,Zbt] = transformation_matrices(Xit,Yit,Zit,gamma,beta,alpha);
    %now target is in body F.O.R
    for i=1:m
        Xp = Xp_requested(1,i);
        Yp = Yp_requested(1,i);
        Zp = Zp_requested(1,i);
        %get angles between target in body F.O.R and plane
        [theta_z, theta_x] = get_angles(Xbt,Ybt,Zbt,Xp,Yp,Zp);
        %[theta_z,theta_x] = goal_angles(Xbt,Ybt,Zbt,Xp,Yp,Zp)
        theta1_goal = theta_z;
        theta2_goal = theta_x;
        
        %create_plot(Xbs,Ybs,Zbs,Xp,Yp,Zp,Xbt,Ybt,Zbt,i);
        
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%% TRANSFORM %%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

    function [theta_z, theta_x] = get_angles(Xbt,Ybt,Zbt,Xp,Yp,Zp)
        
        Xv = Xbt - Xp;
        Yv = Ybt - Yp;
        Zv = Zp - Zbt;    %positive z value down, negative up
        
        magLine = sqrt(Xv^2 + Yv^2 + Zv^2);
        
        theta_z = atan2(sqrt(Xv^2+Yv^2), -Zv)
        theta_x = atan2(Yv, Xv)
        
        s = 0:.1:magLine;
        Zs = Zp+cos(theta_z)*s;
        Xs = Xp+sin(theta_z)*cos(theta_x)*s;
        Ys = Yp+sin(theta_z)*sin(theta_x)*s;
      
    end

function create_plot(Xs,Ys,Zs,Xp,Yp,Zp,Xt,Yt,Zt,i)

    %plot UAV location in orange
    f = figure('visible','off');
    plot3(Xp,Yp,Zp,'-s','MarkerSize',10, 'MarkerFaceColor',[0.9,0.5,0]);
    hold on; grid on
    %plot the target location in yellow
    plot3(Xt,Yt,Zt,'-o','MarkerSize',10, 'MarkerFaceColor',[1,0.9,0.2]);
    hold on;

    %vector endpoint
    Xf = Xs(end);
    Yf = Ys(end);
    Zf = Zs(end);
    
    plot3(Xs, Ys, Zs, '*');
    plot3(Xf, Yf, Zf, 'k*')
    
    xlim([-40, 40]);
    ylim([-40,40]);
    zlim([0, 40]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    %save plot to file 
    exportgraphics(f,sprintf('output/images/Location_%d.png', i));
    
end 



end
