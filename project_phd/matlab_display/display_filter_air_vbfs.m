function display_filter_air_vbfs(folder_name, varargin)

fileID = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_air_vbfs.txt']);
fgetl(fileID);
input = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_air_vbfs'];

t_sec                           = input{1};  % time
vtas_mps_est                    = input{2};  % vtas_mps estimated (a posteriori)
vtas_mps_est_apriori            = input{3};  % vtas_mps estimated (a priori)
vtas_mps_truth                  = input{4};  % vtas_mps truth
vtas_mps_sensed                 = input{5};  % vtas_mps sensed
sigma_vtas_mps_est              = input{6};  % vtas_mps estimated error std (a posteriori)
sigma_vtas_mps_est_apriori      = input{7};  % vtas_mps estimated error std (a priori)
mean_vtas_mps_est               = input{8};  % vtas_mps estimated error mean (a posteriori)
mean_vtas_mps_est_apriori       = input{9};  % vtas_mps estimated error mean (a priori)
bias_vtas_mps                   = input{10}; % vtas_mps bias
alpha_deg_est                   = input{11}; % alpha_deg estimated (a posteriori)
alpha_deg_est_apriori           = input{12}; % alpha_deg estimated (a priori)
alpha_deg_truth                 = input{13}; % alpha_deg truth
alpha_deg_sensed                = input{14}; % alpha_deg sensed
sigma_alpha_deg_est             = input{15}; % alpha_deg estimated error std (a posteriori)
sigma_alpha_deg_est_apriori     = input{16}; % alpha_deg estimated error std (a priori)
mean_alpha_deg_est              = input{17}; % alpha_deg estimated error mean (a posteriori)
mean_alpha_deg_est_apriori      = input{18}; % alpha_deg estimated error mean (a priori)
bias_alpha_deg                  = input{19}; % alpha_deg bias
beta_deg_est                    = input{20}; % beta_deg estimated (a posteriori)
beta_deg_est_apriori            = input{21}; % beta_deg estimated (a priori)
beta_deg_truth                  = input{22}; % beta_deg truth
beta_deg_sensed                 = input{23}; % beta_deg sensed
sigma_beta_deg_est              = input{24}; % beta_deg estimated error std (a posteriori)
sigma_beta_deg_est_apriori      = input{25}; % beta_deg estimated error std (a priori)
mean_beta_deg_est               = input{26}; % beta_deg estimated error mean (a posteriori)
mean_beta_deg_est_apriori       = input{27}; % beta_deg estimated error mean (a priori)
bias_beta_deg                   = input{28}; % beta_deg bias
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
t_sec                           = t_sec(I);
vtas_mps_est                    = vtas_mps_est(I);
vtas_mps_est_apriori            = vtas_mps_est_apriori(I);
vtas_mps_truth                  = vtas_mps_truth(I);
sigma_vtas_mps_est              = sigma_vtas_mps_est(I);
sigma_vtas_mps_est_apriori      = sigma_vtas_mps_est_apriori(I);
mean_vtas_mps_est               = mean_vtas_mps_est(I);
mean_vtas_mps_est_apriori       = mean_vtas_mps_est_apriori(I);
vtas_mps_sensed                 = vtas_mps_sensed(I);
bias_vtas_mps                   = bias_vtas_mps(I);
alpha_deg_est                   = alpha_deg_est(I);
alpha_deg_est_apriori           = alpha_deg_est_apriori(I);
alpha_deg_truth                 = alpha_deg_truth(I);
sigma_alpha_deg_est             = sigma_alpha_deg_est(I);
sigma_alpha_deg_est_apriori     = sigma_alpha_deg_est_apriori(I);
mean_alpha_deg_est              = mean_alpha_deg_est(I);
mean_alpha_deg_est_apriori      = mean_alpha_deg_est_apriori(I);
alpha_deg_sensed                = alpha_deg_sensed(I);
bias_alpha_deg                  = bias_alpha_deg(I);
beta_deg_est                    = beta_deg_est(I);
beta_deg_est_apriori            = beta_deg_est_apriori(I);
beta_deg_truth                  = beta_deg_truth(I);
sigma_beta_deg_est              = sigma_beta_deg_est(I);
sigma_beta_deg_est_apriori      = sigma_beta_deg_est_apriori(I);
mean_beta_deg_est               = mean_beta_deg_est(I);
mean_beta_deg_est_apriori       = mean_beta_deg_est_apriori(I);
beta_deg_sensed                 = beta_deg_sensed(I);
bias_beta_deg                   = bias_beta_deg(I);

max_vtas  = max(vtas_mps_truth) + 0.5;
min_vtas  = min(vtas_mps_truth) - 0.5;
max_alpha = max(alpha_deg_truth) + 0.5;
min_alpha = min(alpha_deg_truth) - 0.5;
max_beta  = max(beta_deg_truth) + 0.5;
min_beta  = min(beta_deg_truth) - 0.5;
lim_vtas_err  = 1.2 * max(abs(vtas_mps_est  - vtas_mps_truth));
lim_alpha_err = 1.2 * max(abs(alpha_deg_est - alpha_deg_truth));
lim_beta_err  = 1.2 * max(abs(beta_deg_est  - beta_deg_truth));

figure('units','normalized','position',[0 0.15 1.0 0.85]);

ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
l = plot(t_sec, [vtas_mps_sensed(:,1) vtas_mps_est_apriori(:,1) vtas_mps_est(:,1) vtas_mps_truth(:,1)]); hold on;
xlabel('t [sec]'); 
ylabel('v_{TAS} [mps]');
l(1).Color = 'g';
l(1).Marker = 'x'; 
l(1).LineStyle = 'none'; 
l(1).MarkerSize = 6;  
l(2).Color = 'b';
l(2).LineStyle = '--';
l(3).Color = 'b';
l(4).Color = 'k';
ax1.YColor = 'k';
ax1.XMinorGrid = 'on';
ax1.YMinorGrid = 'on';
ax1.YLim = [min_vtas max_vtas];
legend(l, 'sensed', 'est a priori', 'est a posteriori', 'truth');
grid on;

ax3 = subplot('Position',[0.05 0.38 0.43 0.27]);
l = plot(t_sec, [alpha_deg_sensed(:,1) alpha_deg_est_apriori(:,1) alpha_deg_est(:,1) alpha_deg_truth(:,1)]); hold on;
xlabel('t [sec]'); 
ylabel('\alpha [deg]');
l(1).Color = 'g';
l(1).Marker = 'x'; 
l(1).LineStyle = 'none'; 
l(1).MarkerSize = 6;  
l(2).Color = 'b';
l(2).LineStyle = '--';
l(3).Color = 'b';
l(4).Color = 'k';
ax3.YColor = 'k';
ax3.XMinorGrid = 'on';
ax3.YMinorGrid = 'on';
ax3.YLim = [min_alpha max_alpha];
grid on;

ax5 = subplot('Position',[0.05 0.05 0.43 0.27]);
l = plot(t_sec, [beta_deg_sensed(:,1) beta_deg_est_apriori(:,1) beta_deg_est(:,1) beta_deg_truth(:,1)]); hold on;
xlabel('t [sec]'); 
ylabel('\beta [deg]');
l(1).Color = 'g';
l(1).Marker = 'x'; 
l(1).LineStyle = 'none'; 
l(1).MarkerSize = 6;  
l(2).Color = 'b';
l(2).LineStyle = '--';
l(3).Color = 'b';
l(4).Color = 'k';
ax5.YColor = 'k';
ax5.XMinorGrid = 'on';
ax5.YMinorGrid = 'on';
ax5.YLim = [min_beta max_beta];
grid on;

ax2 = subplot('Position',[0.55 0.71 0.43 0.27]);
l = plot(t_sec, [(vtas_mps_sensed(:,1) - vtas_mps_truth(:,1)) ...
                 (vtas_mps_est_apriori(:,1) - vtas_mps_truth(:,1)) ...
                 mean_vtas_mps_est_apriori(:,1) ...
                 (vtas_mps_est(:,1)         - vtas_mps_truth(:,1)) ...
                 (-sigma_vtas_mps_est(:,1)) ...
                 sigma_vtas_mps_est(:,1) ...
                 mean_vtas_mps_est(:,1) ...
                 bias_vtas_mps]); hold on;
xlabel('t [sec]'); 
ylabel('Error v_{TAS} [mps]');
l(1).Color = 'g';
l(1).Marker = 'x'; 
l(1).LineStyle = 'none'; 
l(1).MarkerSize = 6;  
l(2).Color = 'k';
l(3).Color = 'k';
l(3).LineStyle = ':';
l(4).Color = 'b';
l(5).Color = 'b';
l(5).LineStyle = '--';
l(6).Color = 'b';
l(6).LineStyle = '--';
l(7).Color = 'b';
l(7).LineStyle = ':';
l(8).Color = 'r';

ax2.YColor = 'k';
ax2.XMinorGrid = 'on';
ax2.YMinorGrid = 'on';
ax2.YLim = [-lim_vtas_err lim_vtas_err];
grid on;

ax4 = subplot('Position',[0.55 0.38 0.43 0.27]);
l = plot(t_sec, [(alpha_deg_sensed(:,1) - alpha_deg_truth(:,1)) ...
                 (alpha_deg_est_apriori(:,1) - alpha_deg_truth(:,1)) ...
                 mean_alpha_deg_est_apriori(:,1) ...
                 (alpha_deg_est(:,1)         - alpha_deg_truth(:,1)) ...
                 (-sigma_alpha_deg_est(:,1)) ...
                 sigma_alpha_deg_est(:,1) ...
                 mean_alpha_deg_est(:,1) ...
                 bias_alpha_deg]); hold on;
xlabel('t [sec]'); 
ylabel('Error \alpha [deg]');
l(1).Color = 'g';
l(1).Marker = 'x'; 
l(1).LineStyle = 'none'; 
l(1).MarkerSize = 6;  
l(2).Color = 'k';
l(3).Color = 'k';
l(3).LineStyle = ':';
l(4).Color = 'b';
l(5).Color = 'b';
l(5).LineStyle = '--';
l(6).Color = 'b';
l(6).LineStyle = '--';
l(7).Color = 'b';
l(7).LineStyle = ':';
l(8).Color = 'r';

ax4.YColor = 'k';
ax4.XMinorGrid = 'on';
ax4.YMinorGrid = 'on';
ax4.YLim = [-lim_alpha_err lim_alpha_err];
grid on;

ax6 = subplot('Position',[0.55 0.05 0.43 0.27]);
l = plot(t_sec, [(beta_deg_sensed(:,1) - beta_deg_truth(:,1)) ...
                 (beta_deg_est_apriori(:,1) - beta_deg_truth(:,1)) ...
                 mean_beta_deg_est_apriori(:,1) ...
                 (beta_deg_est(:,1)         - beta_deg_truth(:,1)) ...
                 (-sigma_beta_deg_est(:,1)) ...
                 sigma_beta_deg_est(:,1) ...
                 mean_beta_deg_est(:,1) ...
                 bias_beta_deg]); hold on;
legend('sensed error', ...
       'priori error', 'priori error mean', ...
       'poster error', 'poster std -', 'poster std +', 'poster error mean', 'bias');
xlabel('t [sec]'); 
ylabel('Error \beta [deg]');
l(1).Color = 'g';
l(1).Marker = 'x'; 
l(1).LineStyle = 'none'; 
l(1).MarkerSize = 6;  
l(2).Color = 'k';
l(3).Color = 'k';
l(3).LineStyle = ':';
l(4).Color = 'b';
l(5).Color = 'b';
l(5).LineStyle = '--';
l(6).Color = 'b';
l(6).LineStyle = '--';
l(7).Color = 'b';
l(7).LineStyle = ':';
l(8).Color = 'r';
ax6.YColor = 'k';
ax6.XMinorGrid = 'on';
ax6.YMinorGrid = 'on';
ax6.YLim = [-lim_beta_err lim_beta_err];
grid on;

%savefig([plot_name '.fig']);

%Image = getframe(gcf);
%imwrite(Image.cdata, [plot_name '.jpg']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    