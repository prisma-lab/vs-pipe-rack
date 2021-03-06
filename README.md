
## Robust visual localization of a UAV over a pipe-rack based on the Lie group SE(3)

This repositoy contains the Matlab source code to implement the visual localization of a UAV over a pipe-rack based on the Lie group SE(3). 

#### License
The source code is released under a MIT license.

#### Summary
This MATLAB function contains the source code to implement a robust visual tracking algorithm for UAVs inspecting pipe racks. This algorithm allows a UAV to automatically stabilize its position and orientation over a pipe rack and simultaneously localize its motion with respect to a local reference frame fixed with a pipe rack. 


#### Usage

    >> load_parm
    >> vs_pipe_rack(radius, Nrp, Nsp, k_dist, k_dist_e, rack_length, camera_altitude, v_max, a_max, j_max, w_max, dw_max, ddw_max, fps, noise_pixel_std, live_plot)
