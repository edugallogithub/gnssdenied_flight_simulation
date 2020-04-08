function display_filter_att_Egyr(folder_name, varargin)

fileID = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_att_Egyr.txt']);
fgetl(fileID);
input = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_filter_att_Egyr'];

t_sec                             = input{1}; % time
E_gyr_dps_est(:,1)                = input{2}; % E_gyr_dps estimated (a posteriori)
E_gyr_dps_est(:,2)                = input{3};
E_gyr_dps_est(:,3)                = input{4};
E_gyr_dps_est_apriori(:,1)        = input{5}; % E_gyr_dps estimated (a priori)
E_gyr_dps_est_apriori(:,2)        = input{6};
E_gyr_dps_est_apriori(:,3)        = input{7};
E_gyr_dps_truth(:,1)              = input{8}; % E_gyr_dps truth
E_gyr_dps_truth(:,2)              = input{9};
E_gyr_dps_truth(:,3)              = input{10};
sigma_E_gyr_dps_est(:,1)          = input{11}; % E_gyr_dps estimated error std (a posteriori)
sigma_E_gyr_dps_est(:,2)          = input{12};
sigma_E_gyr_dps_est(:,3)          = input{13};
sigma_E_gyr_dps_est_apriori(:,1)  = input{14}; % E_gyr_dps estimated error std (a priori)
sigma_E_gyr_dps_est_apriori(:,2)  = input{15};
sigma_E_gyr_dps_est_apriori(:,3)  = input{16};
mean_E_gyr_dps_est(:,1)           = input{17}; % E_gyr_dps estimated error mean (a posteriori)
mean_E_gyr_dps_est(:,2)           = input{18};
mean_E_gyr_dps_est(:,3)           = input{19};
mean_E_gyr_dps_est_apriori(:,1)   = input{20}; % E_gyr_dps estimated error mean (a priori)
mean_E_gyr_dps_est_apriori(:,2)   = input{21};
mean_E_gyr_dps_est_apriori(:,3)   = input{22};
clear input;

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
t_sec                         = t_sec(I);
E_gyr_dps_est                 = E_gyr_dps_est(I,:);
E_gyr_dps_est_apriori         = E_gyr_dps_est_apriori(I,:);
E_gyr_dps_truth               = E_gyr_dps_truth(I,:);
sigma_E_gyr_dps_est           = sigma_E_gyr_dps_est(I,:);
sigma_E_gyr_dps_est_apriori   = sigma_E_gyr_dps_est_apriori(I,:);
mean_E_gyr_dps_est            = mean_E_gyr_dps_est(I,:);
mean_E_gyr_dps_est_apriori    = mean_E_gyr_dps_est_apriori(I,:);

lims_high(1,:) = 1.0 * max(E_gyr_dps_truth);
lims_high(2,:) = 1.0 * max(E_gyr_dps_est_apriori);
lims_high(3,:) = 1.0 * max(E_gyr_dps_est);
lim_high       = max(lims_high);
lims_low(1,:) = 1.0 * min(E_gyr_dps_truth);
lims_low(2,:) = 1.0 * min(E_gyr_dps_est_apriori);
lims_low(3,:) = 1.0 * min(E_gyr_dps_est);
lim_low       = min(lims_low);

figure('units','normalized','position',[0 0.15 1.0 0.85]);

ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
l = plot(t_sec, [E_gyr_dps_est_apriori(:,1) E_gyr_dps_truth(:,1) E_gyr_dps_est(:,1)]); hold on;
xlabel('t [sec]'); 
ylabel('E_{GYR,1} [dps]');
l(1).Color = 'b';
l(1).LineWidth = 1.0;
l(1).LineStyle = '--';
l(2).Color = 'k';
l(2).LineWidth = 1.0;
l(3).Color = 'b';
ax1.YColor = 'k';
ax1.XMinorGrid = 'on';
ax1.YMinorGrid = 'on';
ax1.YLim = [lim_low(1) lim_high(1)];
legend(l, 'est a priori', 'truth', 'est a posteriori');
grid on;

ax3 = subplot('Position',[0.05 0.38 0.43 0.27]);
l = plot(t_sec, [E_gyr_dps_est_apriori(:,2) E_gyr_dps_truth(:,2) E_gyr_dps_est(:,2)]); hold on;
xlabel('t [sec]'); 
ylabel('E_{GYR,2} [dps]');
l(1).Color = 'b';
l(1).LineWidth = 1.0;
l(1).LineStyle = '--';
l(2).Color = 'k';
l(2).LineWidth = 1.0;
l(3).Color = 'b';
ax3.YColor = 'k';
ax3.XMinorGrid = 'on';
ax3.YMinorGrid = 'on';
ax3.YLim = [lim_low(2) lim_high(2)];
grid on;

ax5 = subplot('Position',[0.05 0.05 0.43 0.27]);
l = plot(t_sec, [E_gyr_dps_est_apriori(:,3) E_gyr_dps_truth(:,3) E_gyr_dps_est(:,3)]); hold on;
xlabel('t [sec]'); 
ylabel('E_{GYR,3} [dps]');
l(1).Color = 'b';
l(1).LineWidth = 1.0;
l(1).LineStyle = '--';
l(2).Color = 'k';
l(2).LineWidth = 1.0;
l(3).Color = 'b';
ax5.YColor = 'k';
ax5.XMinorGrid = 'on';
ax5.YMinorGrid = 'on';
ax5.YLim = [lim_low(3) lim_high(3)];
grid on;

ax2 = subplot('Position',[0.55 0.71 0.43 0.27]);
l = plot(t_sec, [(E_gyr_dps_est_apriori(:,1) - E_gyr_dps_truth(:,1)) (-sigma_E_gyr_dps_est_apriori(:,1)) sigma_E_gyr_dps_est_apriori(:,1) mean_E_gyr_dps_est_apriori(:,1) ...
                 (E_gyr_dps_est(:,1)         - E_gyr_dps_truth(:,1)) (-sigma_E_gyr_dps_est(:,1))         sigma_E_gyr_dps_est(:,1)         mean_E_gyr_dps_est(:,1)]); hold on;
xlabel('t [sec]'); 
legend('priori error', 'priori std -', 'priori std +', 'priori error mean', ...
       'poster error', 'poster std -', 'poster std +', 'poster error mean');
ylabel('Error E_{GYR,1} [dps]');
l(1).Color = 'k';
l(2).Color = 'k';
l(2).LineStyle = '--';
l(3).Color = 'k';
l(3).LineStyle = '--';
l(4).Color = 'k';
l(4).LineStyle = ':';
l(5).Color = 'b';
l(6).Color = 'b';
l(6).LineStyle = '--';
l(7).Color = 'b';
l(7).LineStyle = '--';
l(8).Color = 'b';
l(8).LineStyle = ':';
ax2.YColor = 'k';
ax2.XMinorGrid = 'on';
ax2.YMinorGrid = 'on';
grid on;

ax4 = subplot('Position',[0.55 0.38 0.43 0.27]);
l = plot(t_sec, [(E_gyr_dps_est_apriori(:,2) - E_gyr_dps_truth(:,2)) (-sigma_E_gyr_dps_est_apriori(:,2)) sigma_E_gyr_dps_est_apriori(:,2) mean_E_gyr_dps_est_apriori(:,2) ...
                 (E_gyr_dps_est(:,2)         - E_gyr_dps_truth(:,2)) (-sigma_E_gyr_dps_est(:,2))         sigma_E_gyr_dps_est(:,2)         mean_E_gyr_dps_est(:,2)]); hold on;
xlabel('t [sec]'); 
ylabel('Error E_{GYR,2} [dps]');
l(1).Color = 'k';
l(2).Color = 'k';
l(2).LineStyle = '--';
l(3).Color = 'k';
l(3).LineStyle = '--';
l(4).Color = 'k';
l(4).LineStyle = ':';
l(5).Color = 'b';
l(6).Color = 'b';
l(6).LineStyle = '--';
l(7).Color = 'b';
l(7).LineStyle = '--';
l(8).Color = 'b';
l(8).LineStyle = ':';
ax4.YColor = 'k';
ax4.XMinorGrid = 'on';
ax4.YMinorGrid = 'on';
grid on;

ax6 = subplot('Position',[0.55 0.05 0.43 0.27]);
l = plot(t_sec, [(E_gyr_dps_est_apriori(:,3) - E_gyr_dps_truth(:,3)) (-sigma_E_gyr_dps_est_apriori(:,3)) sigma_E_gyr_dps_est_apriori(:,3) mean_E_gyr_dps_est_apriori(:,3) ...
                 (E_gyr_dps_est(:,3)         - E_gyr_dps_truth(:,3)) (-sigma_E_gyr_dps_est(:,3))         sigma_E_gyr_dps_est(:,3)         mean_E_gyr_dps_est(:,3)]); hold on;
xlabel('t [sec]'); 
ylabel('Error E_{GYR,3} [dps]');
l(1).Color = 'k';
l(2).Color = 'k';
l(2).LineStyle = '--';
l(3).Color = 'k';
l(3).LineStyle = '--';
l(4).Color = 'k';
l(4).LineStyle = ':';
l(5).Color = 'b';
l(6).Color = 'b';
l(6).LineStyle = '--';
l(7).Color = 'b';
l(7).LineStyle = '--';
l(8).Color = 'b';
l(8).LineStyle = ':';
ax6.YColor = 'k';
ax6.XMinorGrid = 'on';
ax6.YMinorGrid = 'on';
grid on;

%savefig([plot_name '.fig']);

%Image = getframe(gcf);
%imwrite(Image.cdata, [plot_name '.jpg']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    