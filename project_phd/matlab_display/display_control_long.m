function display_control_long(folder_name, varargin)

fileID = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/control_long.txt']);
uavTruth = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_control_long'];

t_sec            = uavTruth{1};
vtas_mps_truth   = uavTruth{2};
vtas_mps_est     = uavTruth{3};
deltaT           = uavTruth{4};
target_vtas_mps  = uavTruth{5};
accum_vtas_mps   = uavTruth{6};
theta_deg_truth  = uavTruth{7};
theta_deg_est    = uavTruth{8};
deltaE_deg       = uavTruth{9};
target_theta_deg = uavTruth{10};
accum_theta_deg  = uavTruth{11};
gamma_deg_truth  = uavTruth{12};
gamma_deg_est    = uavTruth{13};
alpha_deg_truth  = uavTruth{14};
alpha_deg_est    = uavTruth{15};
gammaTAS_deg     = uavTruth{16};
h_m_truth        = uavTruth{17};
h_m_est          = uavTruth{18};
Hp_m_truth       = uavTruth{19};
Hp_m_est         = uavTruth{20};
target_elv_aux   = uavTruth{21};
id_elv           = uavTruth{22};
clear uavTruth;

if (nargin > 2)
    tmin_sec = varargin{1};
    tmax_sec = varargin{2};
elseif (nargin == 2)
    tmin_sec = varargin{1};
    tmax_sec = t_sec(end);
else
    tmin_sec = 0.;
    tmax_sec = t_sec(end);
end

I = find((t_sec >= tmin_sec) & (t_sec <= tmax_sec));
t_sec            = t_sec(I);
vtas_mps_truth   = vtas_mps_truth(I);
vtas_mps_est     = vtas_mps_est(I);
deltaT           = deltaT(I);
target_vtas_mps  = target_vtas_mps(I);
accum_vtas_mps   = accum_vtas_mps(I);
theta_deg_truth  = theta_deg_truth(I);
theta_deg_est    = theta_deg_est(I);
deltaE_deg       = deltaE_deg(I);
target_theta_deg = target_theta_deg(I);
accum_theta_deg  = accum_theta_deg(I);
gamma_deg_truth  = gamma_deg_truth(I);
gamma_deg_est    = gamma_deg_est(I);
alpha_deg_truth  = alpha_deg_truth(I);
alpha_deg_est    = alpha_deg_est(I);
gammaTAS_deg     = gammaTAS_deg(I);
h_m_truth        = h_m_truth(I);
h_m_est          = h_m_est(I);
Hp_m_truth       = Hp_m_truth(I);
Hp_m_est         = Hp_m_est(I);
target_elv_aux   = target_elv_aux(I);
id_elv           = id_elv(I);

target_Hp_m         = nan(size(t_sec));
target_h_m          = nan(size(t_sec));
target_gammaTAS_deg = nan(size(t_sec));
target_gamma_deg    = nan(size(t_sec));

for i = 1:numel(t_sec)
    switch id_elv(i)
        case 1
            target_Hp_m(i) = target_elv_aux(i);
        case 2
            target_h_m(i) = target_elv_aux(i);
        case 3 
            target_gammaTAS_deg(i) = target_elv_aux(i);
    end
end

h = figure('units','normalized','position',[0 0.15 1.0 0.85]);

ax1 = subplot('Position',[0.05 0.54 0.43 0.43]);
[hAx, hLine1, hLine2] = plotyy(t_sec, [theta_deg_truth theta_deg_est target_theta_deg], t_sec, deltaE_deg); hold on;
xlabel('t [sec]'); 
ylabel(hAx(1),'\theta [deg]');
ylabel(hAx(2),'\delta_{E} [deg]');
hLine1(1).Color = 'k';
hLine1(2).Color = 'k';
hLine1(2).LineStyle = '--';
hLine1(3).Color = 'k';
hLine1(3).LineStyle = '-.';
hLine2.Color = 'b';
hLine2.LineWidth = 1.0;
hAx(1).YColor = 'k';
hAx(2).YColor = 'b';
hAx(1).XMinorGrid = 'on';
hAx(1).YMinorGrid = 'on';
%hAx(2).YMinorGrid = 'on';
%hAx(2).set('Ydir', 'reverse');
legend(hAx(1), '\theta [deg] truth', '\theta [deg] est', '\theta [deg] target', '\delta_{E}');
grid on;

ax2 = subplot('Position',[0.55 0.54 0.43 0.43]);
[jAx, jLine1, jLine2] = plotyy(t_sec, [vtas_mps_truth vtas_mps_est target_vtas_mps], t_sec, deltaT); hold on;
xlabel('t [sec]'); 
ylabel(jAx(1),'v_{TAS} [mps]');
ylabel(jAx(2),'\delta_{T}');
jLine1(1).Color = 'k';
jLine1(2).Color = 'k';
jLine1(2).LineStyle = '--';
jLine1(3).Color = 'k';
jLine1(3).LineStyle = '-.';
jLine2.Color = 'b';
jLine2.LineWidth = 1.0;
jAx(1).YColor = 'k';
jAx(2).YColor = 'b';
jAx(1).XMinorGrid = 'on';
jAx(1).YMinorGrid = 'on';
%jAx(2).YMinorGrid = 'on';
%jAx(2).set('Ydir', 'reverse');
legend(jAx(1), 'v_{TAS} [mps] truth', 'v_{TAS} [mps] est', 'v_{TAS} [mps] target', '\delta_{T}');

grid on;

ax3 = subplot('Position',[0.05 0.05 0.43 0.43]);
l = plot(t_sec, [theta_deg_truth theta_deg_est target_theta_deg gammaTAS_deg target_gammaTAS_deg gamma_deg_truth gamma_deg_est target_gamma_deg alpha_deg_truth alpha_deg_est]); hold on;
xlabel('t [sec]'); 
ylabel('Pitch angles [deg]');
l(1).Color = 'k';
l(2).Color = 'k';
l(2).LineStyle = '--';
l(3).Color = 'k';
l(3).LineStyle = '-.';
l(4).Color = 'b';
l(5).Color = 'b';
l(5).LineStyle = '-.';
l(6).Color = 'r';
l(7).Color = 'r';
l(7).LineStyle = '--';
l(8).Color = 'r';
l(8).LineStyle = '-.';
l(9).Color = 'm';
l(10).Color = 'm';
l(10).LineStyle = '--';
ax3.YColor = 'k';
ax3.XMinorGrid = 'on';
ax3.YMinorGrid = 'on';
legend(l, '\theta [deg] truth', '\theta [deg] est', '\theta [deg] target', '\gamma_{TAS} [deg]', '\gamma_{TAS} [deg] target', '\gamma [deg] truth', '\gamma [deg] est', '\gamma [deg] target', '\alpha [deg] truth', '\alpha [deg] est');
grid on;

ax4 = subplot('Position',[0.55 0.05 0.43 0.43]);
[kAx, kLine1, kLine2] = plotyy(t_sec, [h_m_truth h_m_est target_h_m Hp_m_truth Hp_m_est target_Hp_m], t_sec, [theta_deg_truth, theta_deg_est]); hold on;
xlabel('t [sec]'); 
ylabel(kAx(1),'Altitudes [m]');
ylabel(kAx(2),'\theta [deg]');
kLine1(1).Color = 'k';
kLine1(2).Color = 'k';
kLine1(2).LineStyle = '--';
kLine1(3).Color = 'k';
kLine1(3).LineStyle = '-.';
kLine1(4).Color = 'b';
kLine1(5).Color = 'b';
kLine1(5).LineStyle = '--';
kLine1(6).Color = 'b';
kLine1(6).LineStyle = '-.';
kLine2(1).Color = 'r';
kLine2(2).Color = 'r';
kLine2(2).LineStyle = '--';
kAx(1).YColor = 'k';
kAx(2).YColor = 'r';
kAx(1).XMinorGrid = 'on';
kAx(1).YMinorGrid = 'on';
%kAx(2).YMinorGrid = 'on';
%kAx(2).set('Ydir', 'reverse');
legend(kAx(1), 'h [m] truth', 'h [m] est', 'h [m] target', 'Hp [m] truth', 'Hp [m] est', 'Hp [m] target', '\theta [deg] truth', '\theta [deg] est');
grid on;

%ax3 = subplot(4,1,3);
%[kAx, kLine1, kLine2] = plotyy(t_sec, accum_theta_deg, t_sec, accum_vtas_mps); hold on;
%xlabel('t_{sec}'); 
%ylabel(kAx(1),'Acumulated \theta');
%ylabel(kAx(2),'Acumulated v_{TAS}');
%kAx(1).XMinorGrid = 'on';
%grid on;

%savefig([plot_name '.fig']);

%Image = getframe(gcf);
%imwrite(Image.cdata, [plot_name '.jpg']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    