function display_michael(folder_name, varargin)

fileID = fopen(['/home/egallo/utm/trunk/phd/outputs/nav/' folder_name(1:2) '/' folder_name '/michael_test.txt']);
fgetl(fileID);
input = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);

t_sec           = input{1};
psi_deg         = input{2};
theta_deg       = input{3};
xi_deg          = input{4};
Alambda_deg     = input{5};
Aphi_deg        = input{6};
Ah_m            = input{7};
Av1_n_mps       = input{8};
Av2_n_mps       = input{9};
Av3_n_mps       = input{10};
Bf1_ibb_mps2    = input{11};
Bf2_ibb_mps2    = input{12};
Bf3_ibb_mps2    = input{13};
Cf1_ibb_mps2    = input{14};
Cf2_ibb_mps2    = input{15};
Cf3_ibb_mps2    = input{16};
Df1_ibb_mps2    = input{17};
Df2_ibb_mps2    = input{18};
Df3_ibb_mps2    = input{19};
Blambda_deg     = input{20};
Bphi_deg        = input{21};
Bh_m            = input{22};
Bv1_n_mps       = input{23};
Bv2_n_mps       = input{24};
Bv3_n_mps       = input{25};
Clambda_deg     = input{26};
Cphi_deg        = input{27};
Ch_m            = input{28};
Cv1_n_mps       = input{29};
Cv2_n_mps       = input{30};
Cv3_n_mps       = input{31};
Dlambda_deg     = input{32};
Dphi_deg        = input{33};
Dh_m            = input{34};
Dv1_n_mps       = input{35};
Dv2_n_mps       = input{36};
Dv3_n_mps       = input{37};
clear input;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Integrate my own solutions 
% B --> true attitude and true specific force
% C --> true attitude and sensed specific force
% D --> true attitude and estimated specific force

a_m   = 6378137.0;
f     = 1 / 298.257223563;
e2    = f * (2. - f);
b_m   = a_m * (1. - f);
a2    = a_m * a_m;
b2    = b_m * b_m;
g2    = (a2 - b2) / b2;

w_ieeiii_rps = 7.292115e-5;
gcMSLE_mps2  = 9.7803253359;
gcMSLP_mps2  = 9.8321849378;
GM_m3ps2     = 3986004.418e8;
k            = (b_m * gcMSLP_mps2) / (a_m * gcMSLE_mps2) - 1.0;
m            = (power(w_ieeiii_rps,2) * power(a_m,2) * b_m) / GM_m3ps2;
Deltat_sec   = 0.01;

nel          = numel(t_sec);
   
XBv_n_mps    = zeros(nel,3);
XCv_n_mps    = zeros(nel,3);
XDv_n_mps    = zeros(nel,3);
XBxgdt_rad_m = zeros(nel,3);
XCxgdt_rad_m = zeros(nel,3);
XDxgdt_rad_m = zeros(nel,3);
XBf_ibn_mps2 = zeros(nel,3);
XCf_ibn_mps2 = zeros(nel,3);
XDf_ibn_mps2 = zeros(nel,3);


XBv_n_mps(1,:) = [Av1_n_mps(1) Av2_n_mps(1) Av3_n_mps(1)];
XCv_n_mps(1,:) = [Av1_n_mps(1) Av2_n_mps(1) Av3_n_mps(1)];
XDv_n_mps(1,:) = [Av1_n_mps(1) Av2_n_mps(1) Av3_n_mps(1)];
XBxgdt_rad_m(1,:) = [Alambda_deg(1) * pi/180 Aphi_deg(1) * pi/180 Ah_m(1)]; 
XCxgdt_rad_m(1,:) = [Alambda_deg(1) * pi/180 Aphi_deg(1) * pi/180 Ah_m(1)]; 
XDxgdt_rad_m(1,:) = [Alambda_deg(1) * pi/180 Aphi_deg(1) * pi/180 Ah_m(1)]; 

for (i = 1:(nel-1)) 
    Aeuler_nb_rad = [psi_deg(i) theta_deg(i) xi_deg(i)] * pi/180;
            
    XBN_m                = local_radius_vert(XBxgdt_rad_m(i,2), a_m, e2);
    XBM_m                = local_radius_mer(XBxgdt_rad_m(i,2), XBN_m, g2);
    XBw_enn_rps          = local_compute_wenn_rps(XBv_n_mps(i,:), XBN_m, XBM_m, XBxgdt_rad_m(i,2), XBxgdt_rad_m(i,3));
    XBw_ien_rps          = local_compute_wien_rps(XBxgdt_rad_m(i,2), w_ieeiii_rps);
    XBgc_n_mps2          = local_compute_gravity_ned(XBxgdt_rad_m(i,:), gcMSLE_mps2, k, e2, a_m, f, m);
    XBacor_n_mps2        = local_compute_coriolis_ned(XBv_n_mps(i,:), XBw_ien_rps);
    XBf_ibb_mps2         = [Bf1_ibb_mps2(i) Bf2_ibb_mps2(i) Bf3_ibb_mps2(i)];
    XBf_ibn_mps2(i,:)    = local_rotate(Aeuler_nb_rad, XBf_ibb_mps2);
    XBdv_n_mps_dt        = XBf_ibn_mps2(i,:) - cross(XBw_enn_rps, XBv_n_mps(i,:)) + XBgc_n_mps2 - XBacor_n_mps2;
    XBv_n_mps(i+1,:)     = XBv_n_mps(i,:) + XBdv_n_mps_dt * Deltat_sec;
    XBdx_gdt_rad_m_dt    = local_vned_to_xgdtdot(XBv_n_mps(i,:), XBxgdt_rad_m(i,:), XBN_m, XBM_m);
    XBxgdt_rad_m(i+1,:)  = XBxgdt_rad_m(i,:) + XBdx_gdt_rad_m_dt * Deltat_sec;
    
    XCN_m                = local_radius_vert(XCxgdt_rad_m(i,2), a_m, e2);
    XCM_m                = local_radius_mer(XCxgdt_rad_m(i,2), XCN_m, g2);
    XCw_enn_rps          = local_compute_wenn_rps(XCv_n_mps(i,:), XCN_m, XCM_m, XCxgdt_rad_m(i,2), XCxgdt_rad_m(i,3));
    XCw_ien_rps          = local_compute_wien_rps(XCxgdt_rad_m(i,2), w_ieeiii_rps);
    XCgc_n_mps2          = local_compute_gravity_ned(XCxgdt_rad_m(i,:), gcMSLE_mps2, k, e2, a_m, f, m);
    XCacor_n_mps2        = local_compute_coriolis_ned(XCv_n_mps(i,:), XCw_ien_rps);
    XCf_ibb_mps2         = [Cf1_ibb_mps2(i) Cf2_ibb_mps2(i) Cf3_ibb_mps2(i)];
    XCf_ibn_mps2(i,:)    = local_rotate(Aeuler_nb_rad, XCf_ibb_mps2);
    XCdv_n_mps_dt        = XCf_ibn_mps2(i,:) - cross(XCw_enn_rps, XCv_n_mps(i,:)) + XCgc_n_mps2 - XCacor_n_mps2;
    XCv_n_mps(i+1,:)     = XCv_n_mps(i,:) + XCdv_n_mps_dt * Deltat_sec;
    XCdx_gdt_rad_m_dt    = local_vned_to_xgdtdot(XCv_n_mps(i,:), XCxgdt_rad_m(i,:), XCN_m, XCM_m);        
    XCxgdt_rad_m(i+1,:)  = XCxgdt_rad_m(i,:) + XCdx_gdt_rad_m_dt * Deltat_sec;
                
    XDN_m                = local_radius_vert(XDxgdt_rad_m(i,2), a_m, e2);
    XDM_m                = local_radius_mer(XDxgdt_rad_m(i,2), XDN_m, g2);
    XDw_enn_rps          = local_compute_wenn_rps(XDv_n_mps(i,:), XDN_m, XDM_m, XDxgdt_rad_m(i,2), XDxgdt_rad_m(i,3));
    XDw_ien_rps          = local_compute_wien_rps(XDxgdt_rad_m(i,2), w_ieeiii_rps);
    XDgc_n_mps2          = local_compute_gravity_ned(XDxgdt_rad_m(i,:), gcMSLE_mps2, k, e2, a_m, f, m);
    XDacor_n_mps2        = local_compute_coriolis_ned(XDv_n_mps(i,:), XDw_ien_rps);
    XDf_ibb_mps2         = [Df1_ibb_mps2(i) Df2_ibb_mps2(i) Df3_ibb_mps2(i)];
    XDf_ibn_mps2(i,:)    = local_rotate(Aeuler_nb_rad, XDf_ibb_mps2);
    XDdv_n_mps_dt        = XDf_ibn_mps2(i,:) - cross(XDw_enn_rps, XDv_n_mps(i,:)) + XDgc_n_mps2 - XDacor_n_mps2;
    XDv_n_mps(i+1,:)     = XDv_n_mps(i,:) + XDdv_n_mps_dt * Deltat_sec;
    XDdx_gdt_rad_m_dt    = local_vned_to_xgdtdot(XDv_n_mps(i,:), XDxgdt_rad_m(i,:), XDN_m, XDM_m);
    XDxgdt_rad_m(i+1,:)  = XDxgdt_rad_m(i,:) + XDdx_gdt_rad_m_dt * Deltat_sec;
end

XBlambda_deg = XBxgdt_rad_m(:,1) * 180/pi;
XBphi_deg    = XBxgdt_rad_m(:,2) * 180/pi;
XBh_m        = XBxgdt_rad_m(:,3);

XClambda_deg = XCxgdt_rad_m(:,1) * 180/pi;
XCphi_deg    = XCxgdt_rad_m(:,2) * 180/pi;
XCh_m        = XCxgdt_rad_m(:,3);

XDlambda_deg = XDxgdt_rad_m(:,1) * 180/pi;
XDphi_deg    = XDxgdt_rad_m(:,2) * 180/pi;
XDh_m        = XDxgdt_rad_m(:,3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot them together with previously C++ integrated solutions

figure('units','normalized','position',[0 0.15 1.0 0.85]);

ax1 = subplot(2,2,1);
l = plot(Alambda_deg, Aphi_deg, Blambda_deg, Bphi_deg, XBlambda_deg, XBphi_deg, Clambda_deg, Cphi_deg, XClambda_deg, XCphi_deg, Dlambda_deg, Dphi_deg, XDlambda_deg, XDphi_deg); hold on;
xlabel('\lambda [deg]'); 
ylabel('\phi [deg]');
l(1).Color = 'k';
l(2).Color = 'b';
l(3).Color = 'b';
l(3).Marker = 'x';
l(3).LineStyle = 'None';
l(4).Color = 'r';
l(5).Color = 'r';
l(5).Marker = 'x';
l(5).LineStyle = 'None';
l(6).Color = 'g';
l(7).Color = 'g';
l(7).Marker = 'x';
l(7).LineStyle = 'None';
axis equal;
ax2.YColor = 'k';
ax2.XMinorGrid = 'on';
ax2.YMinorGrid = 'on';
legend(l, 'truth', ...
    'C++ integrated based on true quaternion and true specific force', ...
    'MTL integrated based on true quaternion and true specific force', ...
    'C++ integrated based on true quaternion and sensed specific force', ...
    'MTL integrated based on true quaternion and sensed specific force', ...
    'C++ integrated based on true quaternion and estimated specific force', ...
    'MTL integrated based on true quaternion and estimated specific force');
grid on;

ax3 = subplot(2,2,3);
l = plot(t_sec, [Ah_m, Bh_m, XBh_m, Ch_m, XCh_m, Dh_m, XDh_m]); hold on;
xlabel('t [sec]');
ylabel('h [m]');
l(1).Color = 'k';
l(2).Color = 'b';
l(3).Color = 'b';
l(3).Marker = 'x';
l(3).LineStyle = 'None';
l(4).Color = 'r';
l(5).Color = 'r';
l(5).Marker = 'x';
l(5).LineStyle = 'None';
l(6).Color = 'g';
l(7).Color = 'g';
l(7).Marker = 'x';
l(7).LineStyle = 'None';
axis equal;
ax2.YColor = 'k';
ax2.XMinorGrid = 'on';
ax2.YMinorGrid = 'on';
legend(l, 'truth', ...
    'C++ integrated based on true quaternion and true specific force', ...
    'MTL integrated based on true quaternion and true specific force', ...
    'C++ integrated based on true quaternion and sensed specific force', ...
    'MTL integrated based on true quaternion and sensed specific force', ...
    'C++ integrated based on true quaternion and estimated specific force', ...
    'MTL integrated based on true quaternion and estimated specific force');
grid on;

ax2 = subplot(3,2,2);
l = plot(t_sec, [Av1_n_mps, Bv1_n_mps, XBv_n_mps(:,1), Cv1_n_mps, XCv_n_mps(:,1), Dv1_n_mps, XDv_n_mps(:,1)]); hold on;
xlabel('t [sec]'); 
ylabel('v_{NORTH} [mps]');
l(1).Color = 'k';
l(2).Color = 'b';
l(3).Color = 'b';
l(3).Marker = 'x';
l(3).LineStyle = 'None';
l(4).Color = 'r';
l(5).Color = 'r';
l(5).Marker = 'x';
l(5).LineStyle = 'None';
l(6).Color = 'g';
l(7).Color = 'g';
l(7).Marker = 'x';
l(7).LineStyle = 'None';
axis equal;
ax2.YColor = 'k';
ax2.XMinorGrid = 'on';
ax2.YMinorGrid = 'on';
legend(l, 'truth', ...
    'C++ integrated based on true quaternion and true specific force', ...
    'MTL integrated based on true quaternion and true specific force', ...
    'C++ integrated based on true quaternion and sensed specific force', ...
    'MTL integrated based on true quaternion and sensed specific force', ...
    'C++ integrated based on true quaternion and estimated specific force', ...
    'MTL integrated based on true quaternion and estimated specific force');
grid on;

ax4 = subplot(3,2,4);
l = plot(t_sec, [Av2_n_mps, Bv2_n_mps, XBv_n_mps(:,2), Cv2_n_mps, XCv_n_mps(:,2), Dv2_n_mps, XDv_n_mps(:,2)]); hold on;
xlabel('t [sec]'); 
ylabel('v_{EAST} [mps]');
l(1).Color = 'k';
l(2).Color = 'b';
l(3).Color = 'b';
l(3).Marker = 'x';
l(3).LineStyle = 'None';
l(4).Color = 'r';
l(5).Color = 'r';
l(5).Marker = 'x';
l(5).LineStyle = 'None';
l(6).Color = 'g';
l(7).Color = 'g';
l(7).Marker = 'x';
l(7).LineStyle = 'None';
ax4.YColor = 'k';
ax4.XMinorGrid = 'on';
ax4.YMinorGrid = 'on';
legend(l, 'truth', ...
    'C++ integrated based on true quaternion and true specific force', ...
    'MTL integrated based on true quaternion and true specific force', ...
    'C++ integrated based on true quaternion and sensed specific force', ...
    'MTL integrated based on true quaternion and sensed specific force', ...
    'C++ integrated based on true quaternion and estimated specific force', ...
    'MTL integrated based on true quaternion and estimated specific force');
grid on;

ax6 = subplot(3,2,6);
l = plot(t_sec, [Av3_n_mps, Bv3_n_mps, XBv_n_mps(:,3), Cv3_n_mps, XCv_n_mps(:,3), Dv3_n_mps, XDv_n_mps(:,3)]); hold on;
xlabel('t [sec]'); 
ylabel('v_{DOWN} [mps]');
l(1).Color = 'k';
l(2).Color = 'b';
l(3).Color = 'b';
l(3).Marker = 'x';
l(3).LineStyle = 'None';
l(4).Color = 'r';
l(5).Color = 'r';
l(5).Marker = 'x';
l(5).LineStyle = 'None';
l(6).Color = 'g';
l(7).Color = 'g';
l(7).Marker = 'x';
l(7).LineStyle = 'None';
axis equal;
ax6.YColor = 'k';
ax6.XMinorGrid = 'on';
ax6.YMinorGrid = 'on';
legend(l, 'truth', ...
    'C++ integrated based on true quaternion and true specific force', ...
    'MTL integrated based on true quaternion and true specific force', ...
    'C++ integrated based on true quaternion and sensed specific force', ...
    'MTL integrated based on true quaternion and sensed specific force', ...
    'C++ integrated based on true quaternion and estimated specific force', ...
    'MTL integrated based on true quaternion and estimated specific force');
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function N_m = local_radius_vert(phi_rad, a_m, e2)
    N_m =  a_m / sqrt(1 - e2 * power(sin(phi_rad), 2));

function M_m = local_radius_mer(phi_rad, N_m, g2)
    M_m = N_m / (1 + g2 * power(cos(phi_rad), 2));

function w_enn_rps = local_compute_wenn_rps(v_ned_mps, N_m, M_m, phi_rad, h_m) 
    w_enn_rps = [v_ned_mps(2) / (N_m + h_m), -v_ned_mps(1) / (M_m + h_m), -v_ned_mps(2) * tan(phi_rad) / (N_m + h_m)];


function w_ien_rps = local_compute_wien_rps(phi_rad, w_ieeiii_rps) 
    w_ien_rps = [w_ieeiii_rps * cos(phi_rad), 0.0, -w_ieeiii_rps * sin(phi_rad)];

function gc_n_mps2 = local_compute_gravity_ned(x_gdt_rad_m, gcMSLE_mps2, k, e2, a_m, f, m) 
    sinphi = sin(x_gdt_rad_m(2));
    pow2sinphi = sinphi * sinphi;
    
    gcMSL_mps2 = gcMSLE_mps2 * (1 + k * pow2sinphi) / sqrt(1 - e2 * pow2sinphi);
    
    
    
    gc_mps2    = gcMSL_mps2 * (1 - 2.0 / a_m * (1.0 + f + m - 2.0 * f * pow2sinphi) * x_gdt_rad_m(3) + 3 / power(a_m,2) * power(x_gdt_rad_m(3), 2));
    gc_n_mps2 = [0 0 gc_mps2];

function acor_n_mps2 = local_compute_coriolis_ned(v_n_mps, w_ien_rps)
    acor_n_mps2 = cross(w_ien_rps, v_n_mps) * 2.0;

function Xn = local_rotate(euler_nb, Xb)
    sy = sin(euler_nb(1));
    cy = cos(euler_nb(1));
    sp = sin(euler_nb(2));
    cp = cos(euler_nb(2));
    sr = sin(euler_nb(3));
    cr = cos(euler_nb(3));

    vec11 = Xb(1);
    vec12 = cr * Xb(2) - sr * Xb(3);
    vec13 = cr * Xb(3) + sr * Xb(2);

    vec21 = cp * vec11 + sp * vec13;
    vec22 = vec12;
    vec23 = cp * vec13 - sp * vec11;

    Xn = [cy * vec21 - sy * vec22, cy * vec22 + sy * vec21, vec23];    
    
function dx_gdt_rad_m_dt = local_vned_to_xgdtdot(v_n_mps, x_gdt_rad_m, N_m, M_m)
    dx_gdt_rad_m_dt = [v_n_mps(2) / ((N_m + x_gdt_rad_m(3)) * cos(x_gdt_rad_m(2))), v_n_mps(1) / (M_m + x_gdt_rad_m(3)), - v_n_mps(3)];















