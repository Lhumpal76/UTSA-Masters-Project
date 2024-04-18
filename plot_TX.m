function plot_TX(X,times,tspan)

    [Xit,Yit,Zit,Xp,Yp,Zp,time,velocity,gamma,beta,alpha,...
        Xp_array,Yp_array,Zp_array,Xbt,Ybt,Zbt] = get_constants();
        
    Xp_requested = interp1(time,Xp_array,tspan);
    Yp_requested = interp1(time,Yp_array,tspan);
    Zp_requested = interp1(time,Zp_array,tspan);
    
    
    
    m = length(times);
    for i=1:m
        
        THETA1_con = X(i,1);
        THETA2_con = X(i,3);
        
        Xp = Xp_requested(1,i);
        Yp = Yp_requested(1,i);
        Zp = Zp_requested(1,i);
        
        %change target plot
        
%         if i > m/2
%             Xit = -20;
%             Yit = 20;
%             Zit = 0;
%             [Xbt,Ybt,Zbt] = transformation_matrices(Xit,Yit,Zit,gamma,beta,alpha);
%         end
        
        Xv = Xbt - Xp;
        Yv = Ybt - Yp;
        Zv = Zp - Zbt;    
        
        magLine = sqrt(Xv^2+Yv^2+Zv^2);
        s = 0:.1:magLine;
        Zs = Zp+cos(THETA1_con)*s;
        Xs = Xp+sin(THETA1_con)*cos(THETA2_con)*s;
        Ys = Yp+sin(THETA1_con)*sin(THETA2_con)*s;
        
        %plot UAV location in orange
        f = figure('visible','off');
        plot3(Xp,Yp,Zp,'-s','MarkerSize',10, 'MarkerFaceColor',[0.9,0.5,0]);
        hold on; grid on
        %plot the target location in yellow
        plot3(Xbt,Ybt,Zbt,'-o','MarkerSize',10, 'MarkerFaceColor',[1,0.9,0.2]);
        hold on;
    
        %vector endpoint
        Xf = Xs(end);
        Yf = Ys(end);
        Zf = Zs(end);
    
        plot3(Xs, Ys, Zs, '*');
        plot3(Xf, Yf, Zf, 'k*');
    
        xlim([-50, 50]);
        ylim([-50,50]);
        zlim([0, 40]);
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
    
        %save plot to file
        exportgraphics(f,sprintf('output/images/Location_%d.png', i));
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
end