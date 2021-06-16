function [p,v,a,j,O,w,dw,ddw,tf] = trajectory_jerk(pi, pf,...
          v_max, a_max, j_max, Oi, Of, w_max, dw_max, ddw_max)
    %traiettoria in orientamento e posizione con jerk continuo

    tf_i=zeros(6,1);
    for y=1:1:3
        [p,v,a,j,tf_i(y)] = motion_profile(0, pi(y), pf(y), v_max, a_max, j_max);
    end
    
    [O,w,dw,ddw,tf_i(4)] = motion_profile(0, Oi, Of, w_max, dw_max, ddw_max);
   
    i = 0;
    tf= max(tf_i);
    time = 0:0.001:max(tf_i);
    for t = time
         i = i+1;
         for y = 1:1:3
             [p(y,i),v(y,i),a(y,i),j(y,i),~] = motion_profile(t, pi(y), pf(y), v_max, a_max, j_max);
         end
         [O(i),w(i),dw(i),ddw(i),~] = motion_profile(t, Oi, Of, w_max, dw_max, ddw_max);
     
    end
 
     plot(time, p)
     figure(2)
     plot(time, v)
    
     figure(3);
     plot(time, O);
     figure(4);
     plot(time,w);
end