function [pd,vd,ad,jd,etad,wd,d_wd,dd_wd,tf]= generate_trajectory(P, Eta, Vaj, Waj, ti, t_wait, dt, time_series)
n_ele = size(P,2);
tf_temp = zeros(6,n_ele-1);
t_f = zeros(n_ele-1, 1);
i=1;
for y=1:1:(n_ele-1)
    for r=1:1:3
        [~,~,~,~,tf_temp(r,y)] = motion_profile(0, P(r,y), P(r,y+1),...
            Vaj(r,1), Vaj(r,2), Vaj(r,3));
    end
    for r=1:1:3
        [~,~,~,~,tf_temp(r+3,y)] = motion_profile(0, Eta(r,y), Eta(r,y+1),...
            Waj(r,1), Waj(r,2), Waj(r,3));
    end
    
    if(y>1)
        t_f(y) = max(tf_temp(:,y)) + t_wait + t_f(y-1);
    else
        t_f(y) = max(tf_temp(:,y)) + t_wait + ti;
    end
    
    time = 0:dt:max(tf_temp(:,y))+t_wait;
    for t = time
        for r = 1:1:3
            [p(r,i),v(r,i),a(r,i),j(r,i),~] = motion_profile(t,...
                P(r,y), P(r,y+1), Vaj(r,1), Vaj(r,2), Vaj(r,3));
            [eta(r,i),w(r,i),d_w(r,i),dd_w(r,i),~] = motion_profile(t,...
                Eta(r,y), Eta(r,y+1), Waj(r,1), Waj(r,2), Waj(r,3));
        end
        i = i+1;
    end
end
end_time= (size(p,2)-1)*dt;
tf= ti:dt:(end_time+ti);
if(time_series)
    pd= timeseries;
    pd.data= p';
    pd.time= tf;

    vd= timeseries;
    vd.data= v';
    vd.time= tf;

    ad= timeseries;
    ad.data= a';
    ad.time= tf;

    jd= timeseries;
    jd.data= j';
    jd.time= tf;
    
    etad= timeseries;
    etad.data= eta';
    etad.time= tf;

    wd= timeseries;
    wd.data= w';
    wd.time= tf;

    d_wd= timeseries;
    d_wd.data= d_w';
    d_wd.time= tf;

    dd_wd= timeseries;
    dd_wd.data= dd_w';
    dd_wd.time= tf;
else
    pd= p;
    vd= v;
    ad= a;
    jd= j;
    etad= eta;
    wd= w;
    d_wd= d_w;
    dd_wd= dd_w;
end

end