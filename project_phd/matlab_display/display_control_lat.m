function display_control_lat(folder_name, varargin)

fileID = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/control_lat.txt']);
uavTruth = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_control_lat'];

t_sec           = uavTruth{1};
xi_deg_truth    = uavTruth{2};
xi_deg_est      = uavTruth{3};
deltaA_deg      = uavTruth{4};
target_xi_deg   = uavTruth{5};
accum_xi_deg    = uavTruth{6};
beta_deg_truth  = uavTruth{7};
beta_deg_est    = uavTruth{8};
deltaR_deg      = uavTruth{9};
target_beta_deg = uavTruth{10};
accum_beta_deg  = uavTruth{11};
chi_deg_truth   = uavTruth{12};
chi_deg_est     = uavTruth{13};
psi_deg_truth   = uavTruth{14};
psi_deg_est     = uavTruth{15};
muTAS_deg       = uavTruth{16};
chiTAS_deg      = uavTruth{17};
target_ail_aux  = uavTruth{18};
id_ail          = uavTruth{19};
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
t_sec           = t_sec(I);
xi_deg_truth    = xi_deg_truth(I);
xi_deg_est      = xi_deg_est(I);
deltaA_deg      = deltaA_deg(I);
target_xi_deg   = target_xi_deg(I);
accum_xi_deg    = accum_xi_deg(I);
beta_deg_truth  = beta_deg_truth(I);
beta_deg_est    = beta_deg_est(I);
deltaR_deg      = deltaR_deg(I);
target_beta_deg = target_beta_deg(I);
accum_beta_deg  = accum_beta_deg(I);
chi_deg_truth   = chi_deg_truth(I);
chi_deg_est     = chi_deg_est(I);
psi_deg_truth   = psi_deg_truth(I);
psi_deg_est     = psi_deg_est(I);
muTAS_deg       = muTAS_deg(I);
chiTAS_deg      = chiTAS_deg(I);
target_ail_aux  = target_ail_aux(I);
id_ail          = id_ail(I);

target_chi_deg    = nan(size(t_sec));
target_psi_deg    = nan(size(t_sec));
target_muTAS_deg  = nan(size(t_sec));
target_chiTAS_deg = nan(size(t_sec));

for i = 1:numel(t_sec)
    switch id_ail(i)
        case 1
            target_chi_deg(i) = target_ail_aux(i);
        case 2
            target_psi_deg(i) = target_ail_aux(i);
        case 3 
            target_muTAS_deg(i) = target_ail_aux(i);
        case 4 
            target_chiTAS_deg(i) = target_ail_aux(i);
    end
end

h = figure('units','normalized','position',[0.5 0.15 1.0 0.85]);

ax1 = subplot('Position',[0.05 0.54 0.43 0.43]);
[hAx, hLine1, hLine2] = plotyy(t_sec, [xi_deg_truth xi_deg_est target_xi_deg], t_sec, deltaA_deg); hold on;
xlabel('t [sec]'); 
ylabel(hAx(1),'\xi [deg]');
ylabel(hAx(2),'\delta_{A} [deg]');
hLine1(1).Color = 'k';
hLine1(2).Color = 'k';
hLine1(2).LineStyle = '--';
hLine1(3).Color = 'k';
hLine1(3).LineStyle = '-.';
hLine2.Color = 'b';
hAx(1).YColor = 'k';
hAx(2).YColor = 'b';
hAx(1).XMinorGrid = 'on';
hAx(1).YMinorGrid = 'on';
%hAx(2).YMinorGrid = 'on';
hAx(2).set('Ydir', 'reverse');
legend(hAx(1), '\xi [deg] truth', '\xi [deg] est', '\xi [deg] target', '\delta_{A}');
grid on;

ax2 = subplot('Position',[0.55 0.54 0.43 0.43]);
[jAx, jLine1, jLine2] = plotyy(t_sec, [beta_deg_truth beta_deg_est target_beta_deg], t_sec, deltaR_deg); hold on;
xlabel('t [sec]'); 
ylabel(jAx(1),'\beta [deg]');
ylabel(jAx(2),'\delta_{R} [deg]');
jLine1(1).Color = 'k';
jLine1(2).Color = 'k';
jLine1(2).LineStyle = '--';
jLine1(3).Color = 'k';
jLine1(3).LineStyle = '-.';
jLine2.Color = 'b';
jAx(1).YColor = 'k';
jAx(2).YColor = 'b';
jAx(1).XMinorGrid = 'on';
jAx(1).YMinorGrid = 'on';
%jAx(2).YMinorGrid = 'on';
legend(jAx(1), '\beta [deg] truth', '\beta [deg] est', '\beta [deg] target', '\delta_{R}');
grid on;

ax3 = subplot('Position',[0.05 0.05 0.43 0.43]);
[kAx, kLine1, kLine2] = plotyy(t_sec, [chi_deg_truth chi_deg_est target_chi_deg psi_deg_truth psi_deg_est target_psi_deg chiTAS_deg target_chiTAS_deg], t_sec, [xi_deg_truth xi_deg_est target_xi_deg]); hold on;
xlabel('t [sec]'); 
ylabel(kAx(1),'Yaw angles [deg]');
ylabel(kAx(2),'\xi [deg]');
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
kLine1(7).Color = 'm';
kLine1(8).Color = 'm';
kLine1(8).LineStyle = '-.';
kLine2(1).Color = 'r';
kLine2(2).Color = 'r';
kLine2(2).LineStyle = '--';
kLine2(3).Color = 'r';
kLine2(3).LineStyle = '-.';
kAx(1).YColor = 'k';
kAx(2).YColor = 'r';
kAx(1).XMinorGrid = 'on';
kAx(1).YMinorGrid = 'on';
%kAx(2).YMinorGrid = 'on';
legend(kAx(1), '\chi [deg] truth', '\chi [deg] est', '\chi [deg] target', '\psi [deg] truth', '\psi [deg] est', '\psi [deg] target', '\chi_{TAS} [deg]', '\chi_{TAS} [deg] target', '\xi [deg] truth', '\xi [deg] est', '\xi [deg] target');
grid on;

ax4 = subplot('Position',[0.55 0.05 0.43 0.43]);
[lAx, lLine1, lLine2] = plotyy(t_sec, [muTAS_deg target_muTAS_deg], t_sec, [xi_deg_truth xi_deg_est]); hold on;
xlabel('t [sec]'); 
ylabel(lAx(1),'Bank angles [deg]');
ylabel(lAx(2),'\xi_{deg}');
lLine1(1).Color = 'k';
lLine1(2).Color = 'k';
lLine1(2).LineStyle = '-.';
lLine2(1).Color = 'r';
lLine2(2).Color = 'r';
lLine2(2).LineStyle = '--';
lAx(1).YColor = 'k';
lAx(2).YColor = 'r';
lAx(1).XMinorGrid = 'on';
lAx(1).YMinorGrid = 'on';
%lAx(2).YMinorGrid = 'on';
legend(lAx(1), '\mu_{TAS} [deg]','\mu_{TAS} [deg] target', '\xi [deg] truth', '\xi [deg] est');
grid on;
        
%savefig([plot_name '.fig']);

%Image = getframe(gcf);
%imwrite(Image.cdata, [plot_name '.jpg']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
