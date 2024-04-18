function [Xp_array, Yp_array, Zp_array] = circle_flight_path(ploc_initial,circle_path,radius)

    Xp_array = radius * cos(circle_path) + ploc_initial(1);    %cirlce in xy plane
    Yp_array = radius * sin(circle_path) + ploc_initial(2);    %circle in xy plane
    Zp_array = ploc_initial(3)*ones(size(circle_path));                    %constant z height of circle
    
end 

