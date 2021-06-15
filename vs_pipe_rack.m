 % vs_pipe_rack Robust visual localization of a UAV over a pipe-rack based on the Lie group SE(3)
 %
 %   vs_pipe_rack(radius, Nrp, tf, fps, online_plot):
 %          - radius: a vector containing the radius of each pipe of the pipe-rack. Must contain at least two elements
 %          - Nrp: number of random feature points generated over the pipes
 %          - fps: the framerate of the camera 
 %          - online_plot: generate visual plot online: be careful, if this
 %          is true, the framerate of the camera will not be guaranteed
 %          - TODO: add other params 
 
 
 % Copyright (C) 2021, Prisma Lab, university of Naples Federico II,
 % Authors: Vincenzo Lippiello, Jonathan Cacace.
 % Emails: vincenzo.lippiello@unina.it, jonathan.cacace@unina.it
 %
 % Redistribution and use in source and binary forms, with or without
 % modification, are permitted provided that the following conditions are met:
 %   * Redistributions of source code must retain the above copyright notice,
 %     this list of conditions and the following disclaimer.
 %   * Redistributions in binary form must reproduce the above copyright
 %     notice, this list of conditions and the following disclaimer in the
 %     documentation and/or other materials provided with the distribution.
 %   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 %     contributors may be used to endorse or promote products derived from
 %     this software without specific prior written permission.
 %
 % THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 % AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 % IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 % ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 % LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 % CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 % SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 % INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 % CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 % ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 % POSSIBILITY OF SUCH DAMAGE.
 %
 %

function vs_pipe_rack(radius, Nrp, Nsp, tf, fps, online_plot)

    if size(radius, 2) < 2
        error('Error. The pipe rack should contain at least two pipes')
    end
    
    close all
    set(0, 'DefaultTextInterpreter', 'none')

    %Simulation config flags
    discrete_pixel = false;
    pipe_model_calib = false;

    %Gaussian standard distribution of pixel noise
    noise_pixel_std = 2;

    %% CONSTANTS
    ix = [1;0;0];
    iy = [0;1;0];
    iz = [0;0;1];
    c = ['r','b'];
    ce = ['r','b'];
    %Lie group generators
    Gx = zeros(4);
    Gx(1,4) = 1;
    Gy = zeros(4);
    Gy(2,4) = 1;
    Gz = zeros(4);
    Gz(3,4) = 1;
    Grx = zeros(4);
    Grx(2,3) = -1;
    Grx(3,2) = 1;
    Gry = zeros(4);
    Gry(1,3) = 1;
    Gry(3,1) = -1;
    Grz = zeros(4);
    Grz(1,2) = -1;
    Grz(2,1) = 1;

    Gfu = zeros(3);
    Gfu(1,1) = 1;
    Gfv = zeros(3);
    Gfv(2,2) = 1;
    Gu0 = zeros(3);
    Gu0(1,3) = 1;
    Gv0 = zeros(3);
    Gv0(2,3) = 1;

    G = {Gx, Gy, Gz, Grx, Gry, Grz, ...
        Gx, Gx, Gx, ...
        Gfu, Gfv, Gu0, Gv0 };

    ind_act = [1,2,3,4,5,6];
    if pipe_model_calib
        ind_act = [ind_act, 7,8,9];
    end
    
    l_ind_act = length(ind_act);

    %Camera
    T = 1/fps; %Camera frame rate
    K = [630, 0.0, 360; 
         0.0, 630, 240; 
         0.0, 0.0, 1.0];
  
    Ke = K;
    FoV = [0 720 0 480]; % Camera field of view (normalized image plane)
    np = size(radius, 2);

    %% PIPE 3D MODEL (in wrf)
    k_dist = 0.1;

    for i=1:np
        if i==1
            d_pipe(i) = 0.0;
        else
            d_pipe(i) = k_dist*(i-1);
        end
    end
    %d_pipe = [0 0.1 0.2 0.3];
    d_pipe
    if pipe_model_calib
        d_pipe_e = [0 0.15 0.35 0.50];
    else
        d_pipe_e = d_pipe;
    end

    pipe_length = 10; % Offset between adjacent pipes
    i2m = 0.0254;
    for i=1:np
        P(i).radius = radius(i)*i2m;
        P(i).length = pipe_length;
        P(i).v = iy;
        if i==1
            P(i).p = [P(i).radius;0;P(i).radius]; % A point on the central axis
        else
            P(i).p = [P(i-1).p(1)+P(i-1).radius+P(i).radius;0;P(i).radius]; % A point on the central axis
        end
        P(i).R = [ix cross(P(i).v,ix) P(i).v]; 
    end
        
    rack_width = P(i).p(1) + P(i).radius;
    rack_length = pipe_length;
    
    np = length(P); % Number of pipes

    %% Generate random texture points on the pipes
    Nrp = 25; %Number of random points on each pipe
    for i=1:np
        for j=1:Nrp
            p = P(i).p + [d_pipe(i);0;0] + (rand(1) * P(i).length) * P(i).v + ...
                Ry(pi/2*(-1 + 2*rand(1)))*[0;0;P(i).radius];
            P(i).RP(j,:) = p';
        end
    end


    %% SIMULATION
    Nsp = 50; %Number of sample points per each pipe contour
    t = 0;
    index = 1;
   
    %% Show the pipes in 3D scene
    figure(1)
   
    hold off
    for i=1:length(P)
        [X,Y,Z] = cylinder(P(i).radius);
        Z = -Z*P(i).length;
        X = X + P(i).p(1) + d_pipe(i);
        Y = Y + P(i).radius;
        
        
        hm = surf(X,Y,Z);
        rotate(hm, [1 0 0], 90, [0,0,0])
        %if i>=5
        %    rotate(hm, [0 0 1], 90, P(i).p + [d_pipe(i);0;0])
        %end
        hm.FaceColor = 'flat';
        hm.EdgeColor = 'none';
        %hm.FaceAlpha = 0.9;
        hold on
        %Show random points
        for j=1:size(P(i).RP,1)
            plot3(P(i).RP(j,1),P(i).RP(j,2),P(i).RP(j,3),'b*');
        end
    end
    axis equal
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');

    CAM_POSES = [];
    CAM_POSES_E = [];
    PL1 = [];
    PL2 = [];

    CONT_LEFT = [];
    CONT_RIGHT = [];
    Mx = [];
    My = [];
    P2Ex = [];
    P2Ey = [];
    itr = 1;
    
    
    
    while t < tf
    

        %% CAMERA Motion
        tic;
        cam_traj_angle = 4*pi*min(t,tf)/tf; %!!!
        %Initial position: center of the pipe
        %todo: z must be a param
        cam_p =  [rack_width/2; rack_length/2; 3] + [0.25*cos(cam_traj_angle);0.5*sin(0.5*cam_traj_angle);0.25*sin(cam_traj_angle)];
          
        cam_look_p = [1.0;pipe_length/2;0];
        cam_opt_axis = cam_look_p - cam_p;
        cam_opt_axis  = cam_opt_axis / norm(cam_opt_axis);
        cam_ix = cross(-iy, cam_opt_axis);
        cam_ix = cam_ix/norm(cam_ix);
        cam_iy = cross(cam_opt_axis, cam_ix); 
        cam_iy = cam_iy/norm(cam_iy);
        ang_z = pi/8*sin(2*pi*min(t,tf)/tf);
        cam_R = [cam_ix cam_iy cam_opt_axis]*Rz(ang_z);

        if t==0
            cam_pe = cam_p;
            cam_Re = cam_R;
        end

        CAM_POSES = [CAM_POSES; rigid3d(cam_R, cam_p')];
        CAM_POSES_E = [CAM_POSES_E; rigid3d(cam_Re, cam_pe')];
 
        if online_plot
           cam_pose = rigid3d(cam_R, cam_p');
           cam = plotCamera('AbsolutePose', cam_pose, 'Opacity', 0.1, 'Size', 0.1, 'AxesVisible', true);
           cam_pose_e = rigid3d(cam_Re, cam_pe');
           cam_e = plotCamera('AbsolutePose', cam_pose_e, 'Opacity', 0.5, 'Size', 0.1, 'AxesVisible', false, 'Color', [0 1 0]);
        end
        
        %% Compute and highlight the visible contours from the camera
        for i=1:np
            %REAL
            %Calcolo punti di passaggio delle due rette visibili ai lati della pipe
            p0c = cam_p - (P(i).p + [d_pipe(i);0;0]);
            p0p = P(i).p + [d_pipe(i);0;0] + (p0c'*P(i).v) * P(i).v;
            pcp0 = p0p-cam_p;
            dcp = norm(pcp0);
            u = pcp0/dcp;
            beta = asin(P(i).radius / dcp);
            n_pct = sqrt(dcp^2-P(i).radius^2);
            pt1 = cam_p + axang2rotm([P(i).v' beta])*u*n_pct;
            pt2 = cam_p + axang2rotm([P(i).v' -beta])*u*n_pct;

            ci = c(mod(i,2)+1);

            P(i).pil = pt1 - P(i).v * P(i).length/2;
            P(i).pfl = pt1 + P(i).v * P(i).length/2;
            pl = [P(i).pil';P(i).pfl'];
            if online_plot
                plot3(pl(:,1),pl(:,2),pl(:,3),ci,'Linewidth', 3);
            end
            
            P(i).pir = pt2 - P(i).v * P(i).length/2;
            P(i).pfr = pt2 + P(i).v * P(i).length/2;
            pl = [P(i).pir';P(i).pfr'];
            if online_plot
                plot3(pl(:,1),pl(:,2),pl(:,3),ci,'Linewidth', 3);
            end
            
            %ESTIMATED
            %Calcolo punti di passaggio delle due rette visibili ai lati della pipe
            p0c = cam_p - (P(i).p + [d_pipe_e(i);0;0]);
            p0p = P(i).p + [d_pipe_e(i);0;0] + (p0c'*P(i).v) * P(i).v;
            pcp0 = p0p-cam_p;
            dcp = norm(pcp0);
            u = pcp0/dcp;
            beta = asin(P(i).radius / dcp);
            n_pct = sqrt(dcp^2-P(i).radius^2);
            pt1 = cam_p + axang2rotm([P(i).v' beta])*u*n_pct;
            pt2 = cam_p + axang2rotm([P(i).v' -beta])*u*n_pct;

            P(i).pil_e = pt1 - P(i).v * P(i).length/2;
            P(i).pfl_e = pt1 + P(i).v * P(i).length/2;

            P(i).pir_e = pt2 - P(i).v * P(i).length/2;
            P(i).pfr_e = pt2 + P(i).v * P(i).length/2;
        end
 
        %% CAMERA VIEW
        if online_plot
            figure(2)
            hold off
        end
        Pc = K*[cam_R' -cam_R'*cam_p];
        Pce = Ke*[cam_Re' -cam_Re'*cam_pe];
        %Create the current image scene
        for i=1:np
            %Camera projection matrix
            %Plot contour left
            ri = Pc*[P(i).pil;1];
            rf = Pc*[P(i).pfl;1];
            mixt = ri(1)/ri(3);
            mfxt = rf(1)/rf(3);
            miyt = ri(2)/ri(3);
            mfyt = rf(2)/rf(3);
            %Saturate wrt the FoV of the camera: y=m*x+b | x=m*y+b
            [pix,piy,pfx,pfy] = ApplyFoV(mixt,miyt,mfxt,mfyt,FoV);
            %Plot segment
            ci = c(mod(i,2)+1);
            if online_plot
                plot([pix;pfx],[piy;pfy],ci)
                hold on
            end
            CONT_LEFT(itr, i, 1) = pix;
            CONT_LEFT(itr, i, 2) = pfx;
            CONT_LEFT(itr, i, 3) = piy;
            CONT_LEFT(itr, i, 4) = pfy;

            %Plot contour right
            ri = Pc*[P(i).pir;1];
            rf = Pc*[P(i).pfr;1];
            mixt = ri(1)/ri(3);
            mfxt = rf(1)/rf(3);
            miyt = ri(2)/ri(3);
            mfyt = rf(2)/rf(3);
            %Saturate wrt the FoV of the camera: y=m*x+b | x=m*y+b
            [pix,piy,pfx,pfy] = ApplyFoV(mixt,miyt,mfxt,mfyt,FoV);
            %Plot segment
            ci = c(mod(i,2)+1);
            if online_plot
                plot([pix;pfx],[piy;pfy],ci)
            end
            CONT_RIGHT(itr, i, 1) = pix;
            CONT_RIGHT(itr, i, 2) = pfx;
            CONT_RIGHT(itr, i, 3) = piy;
            CONT_RIGHT(itr, i, 4) = pfy;
    
            %Plot the random points
            
            for j=1:size(P(i).RP,1)
                    r = Pc*[P(i).RP(j,:)';1];
                    m = [r(1)/r(3); r(2)/r(3)];
                    if IsInFoV(m,FoV)
                        %Mx(itr, i, j) = m(1);
                        %My(itr, i, j) = m(2);
                        if online_plot
                            plot(m(1),m(2),'b*');
                        end
                    end
                end
            end
            if online_plot
                axis equal
                grid on
                axis(FoV);
                %title(sprintf('Camera image (time %.3f s)', t));
            end
        
        %Plot the estimated scene
        clear Data
        for i=1:np
%             %LEFT SEGMENT
            %Real line ([u_tilde,v_tilde] reali)
            pril = Pc*[P(i).pil;1];%cam_R'*(P(i).pil-cam_p);
            prfl = Pc*[P(i).pfl;1];%cam_R'*(P(i).pfl-cam_p);
            prixt = pril(1)/pril(3);
            prfxt = prfl(1)/prfl(3);
            priyt = pril(2)/pril(3);
            prfyt = prfl(2)/prfl(3);
            [prix,priy,prfx,prfy] = ApplyFoV(prixt,priyt,prfxt,prfyt,FoV);
            if isempty(prix) || isempty(priy) || isempty(prfx) ||isempty(prfy)
                disp('Error: isempty(prix) || isempty(priy) || isempty(prfx) ||isempty(prfy)');
            end
            %Estimated line ([u_tilde,v_tilde] stimati)
            pil = Pce*[P(i).pil_e;1];%cam_Re'*(P(i).pil-cam_pe);
            pfl = Pce*[P(i).pfl_e;1];%cam_Re'*(P(i).pfl-cam_pe);
            pixt = pil(1)/pil(3);
            pfxt = pfl(1)/pfl(3);
            piyt = pil(2)/pil(3);
            pfyt = pfl(2)/pfl(3);
            [pix,piy,pfx,pfy] = ApplyFoV(pixt,piyt,pfxt,pfyt,FoV);
            if isempty(pix) || isempty(piy) || isempty(pfx) ||isempty(pfy)
                disp('Error: isempty(pix) || isempty(piy) || isempty(pfx) ||isempty(pfy)');
            end
            ci = ce(mod(i,2)+1);
            if online_plot
                plot([pix;pfx],[piy;pfy],sprintf('--%s',ci))
                hold on
            end

            %Find corresponding sample points
            pri = [pix;piy];
            prf = [pfx;pfy];
            vs = prf-pri;
            n_vs = norm(vs);
            if n_vs==0
                disp('ERR: n_vs==0');
            end
            vs = vs / n_vs;
            index_b = 2*Nsp*(i-1);
            for j=1:Nsp
                ps = pri + (j * n_vs/(Nsp+1)) * vs; %sample point su linea stimata ([u_tilde,v_tilde])
                [xx, yy, d] = pDistance(ps(1), ps(2), prix, priy, prfx, prfy); %sample point su linea "reale" ([u_tilde,v_tilde])
                if online_plot
                    plot([ps(1);xx], [ps(2);yy], ci);
                end
                if discrete_pixel
                    ve = round([xx;yy]+[normrnd(0,noise_pixel_std);normrnd(0,noise_pixel_std)])-round(ps);
                    d = norm(ve);
                else
                    ve = [xx;yy]-ps;
                end
                Data(index_b+j).d = d; %Distanza (errore) tra campione su linea stimata e quella "reale"
                if d==0
                    Data(index_b+j).n = [0;0];
                else
                    Data(index_b+j).n = ve/d;
                end
                %Compute the partial derivative of projective image coordinates with respect the ith generating motion
                S1.P0 = cam_Re'*(P(i).pil_e-cam_pe);
                S1.P1 = cam_Re'*(P(i).pfl_e-cam_pe);
                S2.P0 = zeros(3,1); %Camera center
                pt = inv(Ke)*[xx;yy;1];
                S2.P1 = pt*(1e2/norm(pt));
                [~, p1, p2] = dist3D_Segment_to_Segment( S1, S2);
                p_xyz = (p1+p2)/2.0;
                uvw = Ke * p_xyz;
                if uvw(3) == 0
                    disp('ERR: uvw(3) == 0')
                end
                for k=1:l_ind_act
                    if ind_act(k) < 7
                        uvw_p = Pce*cell2mat(G(ind_act(k)))*[cam_Re cam_pe;0 0 0 1]*[p_xyz;1];
                    elseif ind_act(k) < 10
                        if ind_act(k)-5 == i
                            uvw_p = Pce*cell2mat(G(ind_act(k)))*[cam_Re cam_pe;0 0 0 1]*[p_xyz;1];
                        else
                            uvw_p = zeros(3,1);
                        end
                    else
                        uvw_p = cell2mat(G(ind_act(k)))*Ke*p_xyz;
                    end
                    Data(index_b+j).L(:,k) = [uvw_p(1)/uvw(3)-uvw(1)*uvw_p(3)/uvw(3)^2; uvw_p(2)/uvw(3)-uvw(2)*uvw_p(3)/uvw(3)^2 ];
                    Data(index_b+j).f(k) = Data(index_b+j).L(:,k)' * Data(index_b+j).n;% *Data(index_b+j).n;
                end
            end
% 
            %RIGTH SEGMENT
            %Real line
            prir = Pc*[P(i).pir;1];%cam_R'*(P(i).pir-cam_p);
            prfr = Pc*[P(i).pfr;1];%cam_R'*(P(i).pfr-cam_p);
            if prir(3)<0 || prfr(3) < 0
                disp('Error: prir(3)<0 || prfr(3) < 0');
            end
            prix = prir(1)/prir(3);
            prfx = prfr(1)/prfr(3);
            priy = prir(2)/prir(3);
            prfy = prfr(2)/prfr(3);
            [prix,priy,prfx,prfy] = ApplyFoV(prix,priy,prfx,prfy,FoV);
            %Estimated line
            pir = Pce*[P(i).pir_e;1];%cam_Re'*(P(i).pir-cam_pe);
            pfr = Pce*[P(i).pfr_e;1];%cam_Re'*(P(i).pfr-cam_pe);
            pix = pir(1)/pir(3);
            piy = pir(2)/pir(3);
            pfx = pfr(1)/pfr(3);
            pfy = pfr(2)/pfr(3);
            [pix,piy,pfx,pfy] = ApplyFoV(pix,piy,pfx,pfy,FoV);
            ci = c(mod(i,2)+1);
            if online_plot
                plot([pix;pfx],[piy;pfy],sprintf('--%s',ci))
            end
            %Find corresponding sample points
            pri = [pix;piy];
            prf = [pfx;pfy];
            vs = prf-pri;
            n_vs = norm(vs);
            if n_vs==0
                disp('ERR: n_vs==0');
            end
            vs = vs / n_vs;
            index_b = 2*Nsp*i-Nsp;
            for j=1:Nsp
                ps = pri + (j * n_vs/(Nsp+1)) * vs;
                [xx, yy, d] = pDistance(ps(1), ps(2), prix, priy, prfx, prfy);
                if online_plot
                    plot([ps(1);xx], [ps(2);yy], ci);
                end
                if discrete_pixel
                    ve = round([xx;yy]+[normrnd(0,noise_pixel_std);normrnd(0,noise_pixel_std)])-round(ps);
                    d = norm(ve);
                else
                    ve = [xx;yy]-ps;
                end
                Data(index_b+j).d = d;
                if d==0
                    Data(index_b+j).n = [0;0];
                else
                    Data(index_b+j).n = ve/d;
                end
                %Compute the partial derivative of projective image coordinates with respect the ith generating motion
                S1.P0 = cam_Re'*(P(i).pir_e-cam_pe);
                S1.P1 = cam_Re'*(P(i).pfr_e-cam_pe);
                S2.P0 = zeros(3,1);
                pt = inv(Ke)*[xx;yy;1];
                S2.P1 = pt*(1e2/norm(pt));
                [~, p1, p2] = dist3D_Segment_to_Segment( S1, S2);
                p_xyz = (p1+p2)/2.0;
                uvw = Ke * p_xyz;
                if uvw(3) == 0
                    disp('ERR: uvw(3) == 0')
                end
                for k=1:l_ind_act
                     if ind_act(k) < 7
                         uvw_p = Pce*cell2mat(G(ind_act(k)))*[cam_Re cam_pe;0 0 0 1]*[p_xyz;1];
                     elseif ind_act(k) < 10
                         if ind_act(k)-5 == i
                             uvw_p = Pce*cell2mat(G(ind_act(k)))*[cam_Re cam_pe;0 0 0 1]*[p_xyz;1];
                         else
                             uvw_p = zeros(3,1);
                         end
                     else
                         uvw_p = cell2mat(G(ind_act(k)))*Ke*p_xyz;
                     end
                     Data(index_b+j).L(:,k) = [uvw_p(1)/uvw(3)-uvw(1)*uvw_p(3)/uvw(3)^2; uvw_p(2)/uvw(3)-uvw(2)*uvw_p(3)/uvw(3)^2];
                     Data(index_b+j).f(k) = Data(index_b+j).L(:,k)' * Data(index_b+j).n;% *Data(index_b+j).n;
                 end
            end
        end
                 
        

        %Random points
        index_b = length(Data);
        nrp = 0;
        for i=1:np
            for j=1:size(P(i).RP,1)
                prp = P(i).RP(j,:)';
                p3 = K*cam_R'*(prp-cam_p);
                p2 = [p3(1)/p3(3); p3(2)/p3(3)];
                prp_e = prp + [d_pipe_e(i)-d_pipe(i);0;0];
                p3e = Ke*cam_Re'*(prp_e-cam_pe);
                p2e = [p3e(1)/p3e(3); p3e(2)/p3e(3)];
                if IsInFoV(p2e,FoV)
                    nrp = nrp + 1;
                    %P2Ex(itr, i, j) = p2e(1);
                    %P2Ey(itr, i, j) = p2e(2);
                    if online_plot
                        plot(p2e(1),p2e(2),'ro');
                    end
                    %Error
                    index_b = index_b+1;
                    if discrete_pixel
                        ep = round(p2+[normrnd(0,noise_pixel_std);normrnd(0,noise_pixel_std)])-round(p2e);
                    else
                        ep = p2-p2e;
                    end
                    nep = norm(ep);
                    Data(index_b).d = nep; %Distanza (errore) tra campione su pipe stimata e quella "reale"
                    if nep==0
                        Data(index_b).n = [0;0];
                    else
                        Data(index_b).n = ep/nep;
                    end
                    %Compute the partial derivative of projective image coordinates with respect the ith generating motion
                    uvw = p3e;
                    if uvw(3) == 0
                        disp('ERR: uvw(3) == 0')
                    end
                    for k=1:l_ind_act
                        if ind_act(k) < 7
                            uvw_p = Pce*cell2mat(G(ind_act(k)))*[cam_Re cam_pe;0 0 0 1]*[cam_R'*(prp_e-cam_p);1];
                        elseif ind_act(k) < 10
                            if ind_act(k)-5 == i
                                uvw_p = Pce*cell2mat(G(ind_act(k)))*[cam_Re cam_pe;0 0 0 1]*[cam_R'*(prp_e-cam_p);1];
                            else
                                uvw_p = zeros(3,1);
                            end
                        else
                            uvw_p = cell2mat(G(ind_act(k)))*Ke*(cam_R'*(prp-cam_p));
                        end
                        Data(index_b).L(:,k) = [uvw_p(1)/uvw(3)-uvw(1)*uvw_p(3)/uvw(3)^2; uvw_p(2)/uvw(3)-uvw(2)*uvw_p(3)/uvw(3)^2];
                        Data(index_b).f(k) = Data(index_b).L(:,k)' * Data(index_b).n;% *Data(index_b+j).n;
                    end
                end
            end
        end

        v = zeros(l_ind_act,1);
        for k=1:l_ind_act
            for i=1:length(Data)
                v(k) = v(k) + Data(i).d*Data(i).f(k);
            end
        end

        C = zeros(l_ind_act);
        for k1=1:l_ind_act
            for k2=k1:l_ind_act
                Cij = 0;
                for i=1:length(Data)
                    Cij = Cij + Data(i).f(k1)*Data(i).f(k2);
                end
                C(k1,k2) = Cij;
            end
        end
        for k1=2:l_ind_act
            for k2=1:k1-1
                C(k1,k2) = C(k2,k1);
            end
        end
        %fprintf('cond(C) = %f\n',cond(C));

        cn = cond(C);
        if (cn < 1e4)
            invC=inv(C);
        else
            invC=inv(C+1e-4*eye(size(C)));
        end

        alpha = zeros(l_ind_act,1);
        for k1=1:l_ind_act
            for k2=1:l_ind_act
                if abs(v(k2)) > 0
                    if abs(Cij) > 1e-6
                        alpha(k1) = alpha(k1) + invC(k1,k2)*v(k2);
                    end
                end
            end
        end

        nep = norm(cam_p - cam_pe);
        fprintf('[t = %.3f s] |ep|: %.3f (x: %.3f, y: %.3f, z: %.3f) m\n', t, nep, cam_p-cam_pe);

        %Store data for plot
        err.time(index) = t;
        err.cam_p(index,:) = cam_p';
        err.cam_pe(index,:) = cam_pe';
        err.eul_cam_R(index,:) = rotm2eul(cam_R)';
        err.eul_cam_Re(index,:) = rotm2eul(cam_Re)';
        err.eul(index,:) = rotm2eul(cam_Re'*cam_R)';
        err.nrp(index) = nrp;
        err.d_pipe_e(index,:) = d_pipe_e;
        err.d_pipe_err(index,:) = d_pipe-d_pipe_e;
        err.Ke_params(index,:) = [Ke(1,1) Ke(2,2) Ke(1,3) Ke(2,3)];
        err.Ke_params_err(index,:) = [K(1,1)-Ke(1,1) K(2,2)-Ke(2,2) K(1,3)-Ke(1,3) K(2,3)-Ke(2,3)];

        %Compensate errors
        dEe = zeros(4);
        dKe = zeros(3);
        for k=1:l_ind_act
            if ind_act(k) < 7
                dEe = dEe + alpha(k)*cell2mat(G(ind_act(k)));
            elseif ind_act(k) < 10
                d_pipe_e(k-5) = d_pipe_e(k-5) + alpha(k);
            else
                dKe = dKe + alpha(k)*cell2mat(G(ind_act(k)));
            end
        end

    

        dE = expm(dEe); % E(t+1) = E(t)*dE
        %dE = eye(4) + dEe; % E(t+1) = E(t)*dE
        Et=[cam_Re cam_pe;0 0 0 1];
        Etp = invH(dE)*Et;
        if isempty(find(ind_act==2))
            Etp(2,4) = Et(2,4);
        end
        cam_pe = Etp(1:3,4);
        cam_Re = Etp(1:3,1:3);
        %cam_pe = cam_pe + dp;
        %cam_Re = dE(1:3,1:3)' * cam_Re;

        t=t+T;
        index = index + 1;

        et = toc;
        if T-et > 0
            pause(T-et);
        else
            pause(0.001);
        end

        itr = itr+1;
    end


    for p=1:size(CAM_POSES,1)
        %% Show the pipes in 3D scene
        figure(3)
        hold off
        for i=1:length(P)
            [X,Y,Z] = cylinder(P(i).radius);
            Z = -Z*P(i).length;
            X = X + P(i).p(1) + d_pipe(i);
            Y = Y + P(i).radius;
            hm = surf(X,Y,Z);
            rotate(hm, [1 0 0], 90, [0,0,0])
            hm.FaceColor = 'flat';
            hm.EdgeColor = 'none';
            %hm.FaceAlpha = 0.9;
            hold on
            %Show random points
            for j=1:size(P(i).RP,1)
                plot3(P(i).RP(j,1),P(i).RP(j,2),P(i).RP(j,3),'b*');
            end
        end
        axis equal
        xlabel('x [m]');
        ylabel('y [m]');
        zlabel('z [m]');
 
        cam = plotCamera('AbsolutePose', CAM_POSES(p), 'Opacity', 0.1, 'Size', 0.1, 'AxesVisible', true);
        cam_e = plotCamera('AbsolutePose', CAM_POSES_E(p), 'Opacity', 0.5, 'Size', 0.1, 'AxesVisible', false, 'Color', [0 1 0]);
            
        figure(2)
        hold off
        grid on
        for i=1:length(P)
            plot( [CONT_LEFT(p,i, 1); CONT_LEFT(p,i, 2)], [CONT_LEFT(p,i, 3),CONT_LEFT(p,i, 4)], 'r' );
            hold on
            plot( [CONT_RIGHT(p,i, 1); CONT_RIGHT(p,i, 2)], [CONT_RIGHT(p,i, 3),CONT_RIGHT(p,i, 4)], 'b' );
            %plot( Mx(p,i), My(p,i), 'b*');
            %for f=1:size( Mx, 3 )
            %    plot( Mx(p,i,f), My(p,i,f), 'b*');
            %end

            %for f=1:size( P2Ex, 3 )
            %    plot( P2Ex(p,i,f), P2Ex(p,i,f), 'ro');
            %end


        end
        pause(T);
    end
    size(CAM_POSES)
    fprintf('[t = %.3f s] \n', t);

    %Store data for plot
    err.time(index) = t;
    err.cam_p(index,:) = cam_p';
    err.cam_pe(index,:) = cam_pe';
    err.eul_cam_R(index,:) = rotm2eul(cam_R)';
    err.eul_cam_Re(index,:) = rotm2eul(cam_Re)';
    err.eul(index,:) = rotm2eul(cam_Re'*cam_R)';
    err.nrp(index) = nrp;
    err.d_pipe_e(index,:) = d_pipe_e;
    err.d_pipe_err(index,:) = d_pipe-d_pipe_e;
    err.Ke_params(index,:) = [Ke(1,1) Ke(2,2) Ke(1,3) Ke(2,3)];
    err.Ke_params_err(index,:) = [K(1,1)-Ke(1,1) K(2,2)-Ke(2,2) K(1,3)-Ke(1,3) K(2,3)-Ke(2,3)];

    %save err
 
    %plotErr(err);
end


%% FUNCTIONS
function plotErr(err)
    set(0,'defaulttextinterpreter','latex')

    figure('Name', 'Position error')
    plot(err.time, err.cam_p-err.cam_pe)
    xlabel('s');
    ylabel('m');
    legend('$x$','$y$','$z$','Interpreter','latex');
    ax=axis; axis([0,err.time(end),ax(3),ax(4)])
    grid on
    set(gcf,'Position', [10 10 300 250])

    figure('Name', 'Camera Position')
    plot(err.time, err.cam_p)
    xlabel('s');
    ylabel('m');
    legend('$x$','$y$','$z$','Interpreter','latex')
    ax=axis; axis([0,err.time(end),ax(3),ax(4)])
    grid on
    set(gcf,'Position', [10 10 300 250])

    % figure('Name', 'Position and estimate')
    % plot(err.time, err.cam_p, err.time, err.cam_pe, '--')
    % xlabel('s');
    % ylabel('m');
    % legend('$x$','$y$','$z$','$\bar{x}$','$\bar{y}$','$\bar{z}$','Interpreter','latex')
    % ax=axis; axis([0,err.time(end),ax(3),ax(4)])
    % grid on
    % set(gcf,'Position', [10 10 300 250])

    figure('Name', 'Orientation error (Euler ZYX)')
    plot(err.time, err.eul*(180/pi))
    xlabel('s');
    ylabel('deg');
    legend('Z','Y','X','Interpreter','latex');
    ax=axis; axis([0,err.time(end),ax(3),ax(4)])
    grid on
    set(gcf,'Position', [10 10 300 250])

    figure('Name', 'Orientation (Euler ZYX)')
    plot(err.time, err.eul_cam_R*(180/pi))
    xlabel('s');
    ylabel('deg');
    legend('$Z$','$Y$','$X$','Interpreter','latex')
    ax=axis; axis([0,err.time(end),ax(3),ax(4)])
    grid on
    set(gcf,'Position', [10 10 300 250])

    % figure('Name', 'Orientation and estimate (Euler ZYX)')
    % plot(err.time, err.eul_cam_R*(180/pi), err.time, err.eul_cam_Re*(180/pi), '--')
    % xlabel('s');
    % ylabel('deg');
    % legend('$Z$','$Y$','$X$','$\bar{Z}$','$\bar{Y}$','$\bar{X}$','Interpreter','latex')
    % ax=axis; axis([0,err.time(end),ax(3),ax(4)])
    % grid on
    % set(gcf,'Position', [10 10 300 250])

    figure('Name', 'Random (texture) points')
    plot(err.time, err.nrp)
    xlabel('s');
    %ylabel('Random points');
    ax=axis; axis([0,err.time(end),ax(3),ax(4)])
    grid on
    set(gcf,'Position', [10 10 300 250])

    figure('Name', 'Pipe distances')
    plot(err.time, err.d_pipe_e(:,2:4))
    xlabel('s');
    ylabel('m');
    legend('$\bar{d}_1$','$\bar{d}_2$','$\bar{d}_3$','Interpreter','latex')
    ax=axis; axis([0,err.time(end),ax(3),ax(4)])
    grid on
    set(gcf,'Position', [10 10 300 250])
    
    figure('Name', 'Pipe distance error')
    plot(err.time, err.d_pipe_err(:,2:4))
    xlabel('s');
    ylabel('m');
    legend('$\bar{d}_1$','$\bar{d}_2$','$\bar{d}_3$','Interpreter','latex')
    ax=axis; axis([0,err.time(end),ax(3),ax(4)])
    grid on
    set(gcf,'Position', [10 10 300 250])
    
    figure('Name', 'Camera parameters')
    plot(err.time, err.Ke_params)
    xlabel('s');
    ylabel('Pixel');
    legend('$f_u$','$f_v$','$u_0$','$v_0$','Interpreter','latex')
    ax=axis; axis([0,err.time(end),ax(3),ax(4)])
    grid on
    set(gcf,'Position', [10 10 300 250])
    
    figure('Name', 'Camera focal lenght ext err')
    plot(err.time, err.Ke_params_err(:,1), err.time, err.Ke_params_err(:,2))
    xlabel('s');
    ylabel('Pixel');
    legend('$f_u$','$f_v$','Interpreter','latex')
    ax=axis; axis([0,err.time(end),ax(3),ax(4)])
    grid on
    set(gcf,'Position', [10 10 300 250])

    figure('Name', 'Camera optical center ext err')
    plot(err.time, err.Ke_params_err(:,3), err.time, err.Ke_params_err(:,4))
    xlabel('s');
    ylabel('Pixel');
    legend('$u_0$','$v_0$','Interpreter','latex')
    ax=axis; axis([0,err.time(end),ax(3),ax(4)])
    grid on
    set(gcf,'Position', [10 10 300 250])

    
    set(0, 'DefaultTextInterpreter', 'none')
end

%Invert an homogeneous transformation matrix
function inv_H = invH(H)
    inv_H = [H(1:3,1:3)' -H(1:3,1:3)'*H(1:3,4); 0 0 0 1];
end

% Rotation matrix around x-axis
function R = Rx(angle)
    c = cos(angle);
    s = sin(angle);
    R = [1 0 0; 0 c -s; 0 s c];
end

% Rotation matrix around y-axis
function R = Ry(angle)
    c = cos(angle);
    s = sin(angle);
    R = [c 0 s; 0 1 0; -s 0 c];
end

% Rotation matrix around z-axis
function R = Rz(angle)
    c = cos(angle);
    s = sin(angle);
    R = [c -s 0; s c 0; 0 0 1];
end

%https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
function [xx, yy, d] = pDistance(x, y, x1, y1, x2, y2)

  A = x - x1;
  B = y - y1;
  C = x2 - x1;
  D = y2 - y1;

  dot = A * C + B * D;
  len_sq = C * C + D * D;
  param = -1;
  if (len_sq ~= 0) %in case of 0 length line
      param = dot / len_sq;
  end
  if (param < 0)
    xx = x1;
    yy = y1;
  elseif (param > 1)
    xx = x2;
    yy = y2;
  else
    xx = x1 + param * C;
    yy = y1 + param * D;
  end
  dx = x - xx;
  dy = y - yy;
  
  d = sqrt(dx * dx + dy * dy);
end

function isVisible = IsInFoV(p,FoV)
    %Check points in or out
    isVisible = p(1) >= FoV(1) && p(1) <= FoV(2) && p(2) >= FoV(3) && p(2) <= FoV(4);
end

function [pix,piy,pfx,pfy] = ApplyFoV(pix,piy,pfx,pfy,FoV)
    if length(FoV) == 4
        %Check points in or out
        is_pi_in = IsInFoV([pix,piy],FoV); %pix > -fov && pix < fov && piy > -fov && piy < fov;
        is_pf_in = IsInFoV([pfx,pfy],FoV); %pfx > -fov && pfx < fov && pfy > -fov && pfy < fov;
        if is_pi_in && is_pf_in %Both inside
            return;
        end
        %Find intersections
        [px1, py1] = LineLineIntersection( pix, piy, pfx, pfy, FoV(1), FoV(4), FoV(2), FoV(4) );
        [px2, py2] = LineLineIntersection( pix, piy, pfx, pfy, FoV(2), FoV(4), FoV(2), FoV(3) );
        [px3, py3] = LineLineIntersection( pix, piy, pfx, pfy, FoV(2), FoV(3), FoV(1), FoV(3) );
        [px4, py4] = LineLineIntersection( pix, piy, pfx, pfy, FoV(1), FoV(3), FoV(1), FoV(4) );
        vx = [px1,px2,px3,px4];
        vx = vx(vx<inf);
        vy = [py1,py2,py3,py4];
        vy = vy(vy<inf);
        %Chose the solution
        if ~is_pi_in && ~is_pf_in %Both points are outside
            if length(vx)==2
                pix = vx(1);
                piy = vy(1);
                pfx = vx(2);
                pfy = vy(2);
            else % Pipe not in the FoV
                pix=0;
                piy=0;
                pfx=0;
                pfy=0;
            end
        elseif is_pi_in && ~is_pf_in % Initial point is inside, final point is outside
            pfx = vx(1);
            pfy = vy(1);
        else % Initial point is outside, final point is inside
            pix = vx(1);
            piy = vy(1);
        end
    end
end

% LINE/LINE (http://www.jeffreythompson.org/collision-detection/line-rect.php)
function [px, py] = LineLineIntersection( x1, y1, x2, y2, x3, y3, x4, y4 )
  % calculate the direction of the lines
  uA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));
  uB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));
  % if uA and uB are between 0-1, lines are colliding
  if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) 
    % optionally, draw a circle where the lines meet
    px = x1 + (uA * (x2-x1));
    py = y1 + (uA * (y2-y1));
  else
    px=inf;
    py=inf;
  end
end


% dist3D_Segment_to_Segment(): get the 3D minimum distance between 2 segments
%    Input:  two 3D line segments S1 and S2
%    Return: the shortest distance between S1 and S2
% http://geomalgorithms.com/a07-_distance.html#dist3D_Segment_to_Segment()
function [d, P1, P2] = dist3D_Segment_to_Segment( S1, S2)
    u = S1.P1 - S1.P0;
    v = S2.P1 - S2.P0;
    w = S1.P0 - S2.P0;
    a = u'*u; %always >= 0
    b = u'*v;
    c = v'*v; % always >= 0
    d = u'*w;
    e = v'*w;
    D = a*c - b*b; % always >= 0
    sc = D; % sc = sN / sD, default sD = D >= 0
    sN = D; 
    sD = D; 
    tc = D; % tc = tN / tD, default tD = D >= 0
    tN = D; 
    tD = D; 
    SMALL_NUM = 1e-9;

    % compute the line parameters of the two closest points
    if D < SMALL_NUM  % the lines are almost parallel
        sN = 0.0;         % force using point P0 on segment S1
        sD = 1.0;         % to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    else                 % get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0)         % sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        elseif (sN > sD) % sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        end
    end

    if (tN < 0.0) % tc < 0 => the t=0 edge is visible
        tN = 0.0;
        % recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        elseif (-d > a)
            sN = sD;
        else
            sN = -d;
            sD = a;
        end
    elseif (tN > tD) % tc > 1  => the t=1 edge is visible
        tN = tD;
        % recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        elseif ((-d + b) > a)
            sN = sD;
        else
            sN = (-d +  b);
            sD = a;
        end
    end
    % finally do the division to get sc and tc
    if abs(sN) < SMALL_NUM
        sc = 0.0;
    else
        sc = sN / sD;
    end
    if abs(tN) < SMALL_NUM
        tc = 0.0;
    else
        tc = tN / tD;
    end
    % get the difference of the two closest points
    P1 = w + sc * u;
    P2 = tc * v;
    dP = P1 - P2;  % =  S1(sc) - S2(tc)
    %dP = w + (sc * u) - (tc * v);  % =  S1(sc) - S2(tc)

    d = norm(dP);   % return the closest distance
end