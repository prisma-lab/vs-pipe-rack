function [p,v,a,j,tf] = motion_profile(t, pi, pf, v_max, a_max, j_max)
    if(pi > pf)
        v_max = -v_max;
        a_max = -a_max;
        j_max = -j_max;
    end
    s = pf-pi;
    a_max2 = a_max*a_max;
    va = abs(a_max2/j_max);
    sa = abs(2.0*va*a_max/j_max);

    a_max_div_j_max = abs(a_max/j_max);
    sqrt_v_max_div_j_max = sqrt(v_max/j_max);

    if (v_max*j_max < a_max2)
        sv = abs(v_max*2.0*sqrt_v_max_div_j_max);
    else
        sv = abs(v_max*(v_max/a_max+a_max_div_j_max));
    end

    if (v_max <= va && s >= sa)
        tj = sqrt_v_max_div_j_max;
        ta = tj;
        tv = s/v_max;
    elseif (v_max >= va && s <= sa)
        tj = (s/(2.0*j_max))^(1/3.0); 
        ta = tj;
        tv = 2.0*tj;
    elseif (v_max <= va && s <= sa)
        if (s >= sv)
            tj = sqrt_v_max_div_j_max;
            ta = tj;
            tv = s/v_max;
        else
            tj = (s/(2.0*j_max))^(1/3.0);
            ta = tj;
            tv = 2.0*tj;
        end
    elseif (v_max >= va && s >= sa)
        tj = a_max_div_j_max;
        if (s >= sv)
           ta = v_max/a_max;
           tv = s/v_max;
        else
           ta = 0.5*(sqrt((4.0*s+a_max*a_max_div_j_max*a_max_div_j_max)/a_max) - a_max_div_j_max);
           tv = ta+tj;
        end
    end
    
    t1 = tj;
    t2 = ta;
    t3 = ta+tj;
    t4 = tv;
    t5 = tv+tj;
    t6 = tv+ta;
    t7 = tv+tj+ta;

    if (t < 0)
        j = 0;
        a = 0;
        v = 0;
        p = pi;
    elseif (t < t1)
        j = j_max;
        a = j_max*t;
        v = a/2.0*t;
        p = pi + v*t/3.0;
    else
        a1 = j_max*t1;
        v1 = a1*t1/2.0;
        p1 = pi + v1*t1/3.0;
        if (t < t2)
            dt = t-t1;
            j = 0;
            a = a1;
            v = v1 + a1*dt;
            p = p1 + (v1+a1/2.0*dt)*dt;
        else
            dt = t2-t1;
            a2 = a1;
            v2 = v1 + a1*dt;
            p2 = p1 + (v1+a1/2.0*dt)*dt;
            if (t < t3)
                dt = t-t2;
                j = -j_max;
                a = a2-j_max*dt;
                v = v2+(a2-j_max/2.0*dt)*dt;
                p = p2+(v2+(a2-j_max/3.0*dt)/2.0*dt)*dt;
            else
                dt = t3-t2;
                v3 = v2+(a2-j_max/2.0*dt)*dt;
                p3 = p2+(v2+(a2-j_max/3.0*dt)/2.0*dt)*dt;
                if (t < t4)
                    j = 0;
                    a = 0;
                    v = v3;
                    p = p3 + v3*(t-t3);
                else
                    v4 = v3;
                    p4 = p3 + v3*(t4-t3);
                    if (t < t5)
                        dt = t-t4;
                        j = -j_max;
                        a = -j_max*dt;
                        v = v4-j_max/2.0*dt*dt;
                        p = p4+(v4-j_max/6.0*dt*dt)*dt;
                    else
                        dt = t5-t4;
                        a5 = -j_max*dt;
                        v5 = v4-j_max/2.0*dt*dt;
                        p5 = p4+(v4-j_max/6.0*dt*dt)*dt;
                        if (t < t6)
                            dt = t-t5;
                            j = 0;
                            a = a5;
                            v = v5-a_max*dt;
                            p = p5+(v5+a5/2.0*dt)*dt;
                        else
                            dt = t6-t5;
                            a6 = a5;
                            v6 = v5-a_max*dt;
                            p6 = p5+(v5+a5/2.0*dt)*dt;
                            if (t < t7)
                                dt = t-t6;
                                j = j_max;
                                a = a6+j_max*dt;
                                v = v6+(a6+j_max/2.0*dt)*dt;
                                p = p6+(v6+(a6+j_max/3.0*dt)/2.0*dt)*dt;
                            else
                                j = 0;
                                a = 0;
                                v = 0;
                                p = pf;
                            end
                        end
                    end
                end
            end
        end
    end
    tf = t7;

end    
