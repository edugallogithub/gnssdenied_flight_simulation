function display_filter_att_Emag01(folder_name, varargin)

fileID = fopen(['/home/egallo/utm/trunk/phd/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_att_Emag.txt']);
fgetl(fileID);
input = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);
plot_name = ['/home/egallo/utm/trunk/phd/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_att_Emag'];

t_sec                           = input{1}; % time
Emag_nT_est(:,1)                = input{2}; % Emag_nT estimated (a posteriori)
Emag_nT_est(:,2)                = input{3};
Emag_nT_est(:,3)                = input{4};
Emag_nT_est_apriori(:,1)        = input{5}; % Emag_nT estimated (a priori)
Emag_nT_est_apriori(:,2)        = input{6};
Emag_nT_est_apriori(:,3)        = input{7};
Emag_nT_truth(:,1)              = input{8}; % Emag_nT truth
Emag_nT_truth(:,2)              = input{9};
Emag_nT_truth(:,3)              = input{10};
sigma_Emag_nT_est(:,1)          = input{11}; % Emag_nT estimated error std (a posteriori)
sigma_Emag_nT_est(:,2)          = input{12};
sigma_Emag_nT_est(:,3)          = input{13};
sigma_Emag_nT_est_apriori(:,1)  = input{14}; % Emag_nT estimated error std (a priori)
sigma_Emag_nT_est_apriori(:,2)  = input{15};
sigma_Emag_nT_est_apriori(:,3)  = input{16};
mean_Emag_nT_est(:,1)           = input{17}; % Emag_nT estimated error mean (a posteriori)
mean_Emag_nT_est(:,2)           = input{18};
mean_Emag_nT_est(:,3)           = input{19};
mean_Emag_nT_est_apriori(:,1)   = input{20}; % Emag_nT estimated error mean (a priori)
mean_Emag_nT_est_apriori(:,2)   = input{21};
mean_Emag_nT_est_apriori(:,3)   = input{22};
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
t_sec                       = t_sec(I);
Emag_nT_est                 = Emag_nT_est(I,:);
Emag_nT_est_apriori         = Emag_nT_est_apriori(I,:);
Emag_nT_truth               = Emag_nT_truth(I,:);
sigma_Emag_nT_est           = sigma_Emag_nT_est(I,:);
sigma_Emag_nT_est_apriori   = sigma_Emag_nT_est_apriori(I,:);
mean_Emag_nT_est            = mean_Emag_nT_est(I,:);
mean_Emag_nT_est_apriori    = mean_Emag_nT_est_apriori(I,:);

lims(1,:) = 1.2 * max(abs(Emag_nT_truth));
lims(2,:) = 1.2 * max(abs(Emag_nT_est_apriori));
lims(3,:) = 1.2 * max(abs(Emag_nT_est));
lim = max(lims);

figure('units','normalized','position',[0 0.15 1.0 0.85]);

ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
l = plot(t_sec, [Emag_nT_est_apriori(:,1) Emag_nT_truth(:,1) Emag_nT_est(:,1)]); hold on;
xlabel('t [sec]'); 
ylabel('E_{MAG,1} [nT]');
l(1).Color = 'b';
l(1).LineWidth = 1.0;
l(1).LineStyle = '--';
l(2).Color = 'k';
l(2).LineWidth = 1.0;
l(3).Color = 'b';
ax1.YColor = 'k';
ax1.XMinorGrid = 'on';
ax1.YMinorGrid = 'on';
ax1.YLim = [-lim(1) lim(1)];
legend(l, 'est a priori', 'truth', 'est a posteriori');
grid on;

ax3 = subplot('Position',[0.05 0.38 0.43 0.27]);
l = plot(t_sec, [Emag_nT_est_apriori(:,2) Emag_nT_truth(:,2) Emag_nT_est(:,2)]); hold on;
xlabel('t [sec]'); 
ylabel('E_{MAG,2} [nT]');
l(1).Color = 'b';
l(1).LineWidth = 1.0;
l(1).LineStyle = '--';
l(2).Color = 'k';
l(2).LineWidth = 1.0;
l(3).Color = 'b';
ax3.YColor = 'k';
ax3.XMinorGrid = 'on';
ax3.YMinorGrid = 'on';
ax3.YLim = [-lim(2) lim(2)];
grid on;

ax5 = subplot('Position',[0.05 0.05 0.43 0.27]);
l = plot(t_sec, [Emag_nT_est_apriori(:,3) Emag_nT_truth(:,3) Emag_nT_est(:,3)]); hold on;
xlabel('t [sec]'); 
ylabel('E_{MAG,3} [nT]');
l(1).Color = 'b';
l(1).LineWidth = 1.0;
l(1).LineStyle = '--';
l(2).Color = 'k';
l(2).LineWidth = 1.0;
l(3).Color = 'b';
ax5.YColor = 'k';
ax5.XMinorGrid = 'on';
ax5.YMinorGrid = 'on';
ax5.YLim = [-lim(3) lim(3)];
grid on;

ax2 = subplot('Position',[0.55 0.71 0.43 0.27]);
l = plot(t_sec, [(Emag_nT_est_apriori(:,1) - Emag_nT_truth(:,1)) (-sigma_Emag_nT_est_apriori(:,1)) sigma_Emag_nT_est_apriori(:,1) mean_Emag_nT_est_apriori(:,1) ...
                 (Emag_nT_est(:,1)         - Emag_nT_truth(:,1)) (-sigma_Emag_nT_est(:,1))         sigma_Emag_nT_est(:,1)         mean_Emag_nT_est(:,1)]); hold on;
xlabel('t [sec]'); 
legend('priori error', 'priori std -', 'priori std +', 'priori error mean', ...
       'poster error', 'poster std -', 'poster std +', 'poster error mean');
ylabel('Error E_{MAG,1} [nT]');
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
l = plot(t_sec, [(Emag_nT_est_apriori(:,2) - Emag_nT_truth(:,2)) (-sigma_Emag_nT_est_apriori(:,2)) sigma_Emag_nT_est_apriori(:,2) mean_Emag_nT_est_apriori(:,2) ...
                 (Emag_nT_est(:,2)         - Emag_nT_truth(:,2)) (-sigma_Emag_nT_est(:,2))         sigma_Emag_nT_est(:,2)         mean_Emag_nT_est(:,2)]); hold on;
xlabel('t [sec]'); 
ylabel('Error E_{MAG,2} [nT]');
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
l = plot(t_sec, [(Emag_nT_est_apriori(:,3) - Emag_nT_truth(:,3)) (-sigma_Emag_nT_est_apriori(:,3)) sigma_Emag_nT_est_apriori(:,3) mean_Emag_nT_est_apriori(:,3) ...
                 (Emag_nT_est(:,3)         - Emag_nT_truth(:,3)) (-sigma_Emag_nT_est(:,3))         sigma_Emag_nT_est(:,3)         mean_Emag_nT_est(:,3)]); hold on;
xlabel('t [sec]'); 
ylabel('Error E_{MAG,3} [nT]');
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
    