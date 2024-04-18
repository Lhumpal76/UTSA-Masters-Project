function [Xp_array,Yp_array,Zp_array] = straight_flight_path(ploc_initial,velocity,time)

Xp_array = ploc_initial(1)+0*time;
Yp_array = ploc_initial(2)+velocity*time;
Zp_array = ploc_initial(3)+0*time;

end


