function display_filter_air_atm(folder_name, varargin)

fileID = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_air_atm.txt']);
fgetl(fileID);
input = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_air_atm'];

t_sec                      = input{1};  % time
T_degK_est                 = input{2};  % T_degK estimated (a posteriori)
T_degK_est_apriori         = input{3};  % T_degK estimated (a priori)
T_degK_truth               = input{4};  % T_degK truth
T_degK_sensed              = input{5};  % T_degK sensed
sigma_T_degK_est           = input{6};  % T_degK estimated error std (a posteriori)
sigma_T_degK_est_apriori   = input{7};  % T_degK estimated error std (a priori)
mean_T_degK_est            = input{8};  % T_degK estimated error mean (a posteriori)
mean_T_degK_est_apriori    = input{9};  % T_degK estimated error mean (a priori)
bias_oat_degK              = input{10}; % bias T_degK
Hp_m_est                   = input{11}; % Hp_m estimated (a posteriori)
Hp_m_est_apriori           = input{12}; % Hp_m estimated (a priori)
Hp_m_truth                 = input{13}; % Hp_m truth
Hp_m_sensed                = input{14}; % Hp_m sensed
sigma_Hp_m_est             = input{15}; % Hp_m estimated error std (a posteriori)
sigma_Hp_m_est_apriori     = input{16}; % Hp_m estimated error std (a priori)
mean_Hp_m_est              = input{17}; % Hp_m estimated error mean (a posteriori)
mean_Hp_m_est_apriori      = input{18}; % Hp_m estimated error mean (a priori)
bias_Hp_m                  = input{19}; % bias Hp_m
ROC_mps_est                = input{20}; % ROC_mps estimated (a posteriori)
ROC_mps_est_apriori        = input{21}; % ROC_mps estimated (a priori)
ROC_mps_truth              = input{22}; % ROC_mps truth
sigma_ROC_mps_est          = input{23}; % ROC_mps estimated error std (a posteriori)
sigma_ROC_mps_est_apriori  = input{24}; % ROC_mps estimated error std (a priori)
mean_ROC_mps_est           = input{25}; % ROC_mps estimated error mean (a posteriori)
mean_ROC_mps_est_apriori   = input{26}; % ROC_mps estimated error mean (a priori)
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
t_sec                      = t_sec(I);
T_degK_est                 = T_degK_est(I);
T_degK_est_apriori         = T_degK_est_apriori(I);
T_degK_truth               = T_degK_truth(I);
T_degK_sensed              = T_degK_sensed(I);
sigma_T_degK_est           = sigma_T_degK_est(I);
sigma_T_degK_est_apriori   = sigma_T_degK_est_apriori(I);
mean_T_degK_est            = mean_T_degK_est(I);
mean_T_degK_est_apriori    = mean_T_degK_est_apriori(I);
bias_oat_degK              = bias_oat_degK(I);
Hp_m_est                   = Hp_m_est(I);
Hp_m_est_apriori           = Hp_m_est_apriori(I);
Hp_m_truth                 = Hp_m_truth(I);
Hp_m_sensed                = Hp_m_sensed(I);
sigma_Hp_m_est             = sigma_Hp_m_est(I);
sigma_Hp_m_est_apriori     = sigma_Hp_m_est_apriori(I);
mean_Hp_m_est              = mean_Hp_m_est(I);
mean_Hp_m_est_apriori      = mean_Hp_m_est_apriori(I);
bias_Hp_m                  = bias_Hp_m(I);
ROC_mps_est                = ROC_mps_est(I);
ROC_mps_est_apriori        = ROC_mps_est_apriori(I);
ROC_mps_truth              = ROC_mps_truth(I);
sigma_ROC_mps_est          = sigma_ROC_mps_est(I);
sigma_ROC_mps_est_apriori  = sigma_ROC_mps_est_apriori(I);
mean_ROC_mps_est           = mean_ROC_mps_est(I);
mean_ROC_mps_est_apriori   = mean_ROC_mps_est_apriori(I);

max_T  = max(T_degK_truth) + 0.1;
min_T  = min(T_degK_truth) - 0.1;
max_Hp = max(Hp_m_truth) + 5;
min_Hp = min(Hp_m_truth) - 5;
lim_T_err  = 1.2 * max(abs(T_degK_est  - T_degK_truth));
lim_Hp_err = 1.2 * max(abs(Hp_m_est - Hp_m_truth));

figure('units','normalized','position',[0 0.15 1.0 0.85]);

ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
%ax1 = subplot(3,2,1);
l = plot(t_sec, [T_degK_sensed(:,1) T_degK_est_apriori(:,1) T_degK_est(:,1) T_degK_truth(:,1)]); hold on;
xlabel('t [sec]'); 
ylabel('T [degK]');
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
ax1.YLim = [min_T max_T];
legend(l, 'sensed', 'est a priori', 'est a posteriori', 'truth');
grid on;

ax3 = subplot('Position',[0.05 0.38 0.43 0.27]);
%ax3 = subplot(3,2,3);
l = plot(t_sec, [Hp_m_sensed(:,1) Hp_m_est_apriori(:,1) Hp_m_est(:,1) Hp_m_truth(:,1)]); hold on;
xlabel('t [sec]'); 
ylabel('H_{P} [m]');
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
ax3.YLim = [min_Hp max_Hp];
grid on;

ax5 = subplot('Position',[0.05 0.05 0.43 0.27]);
%ax5 = subplot(3,2,5);
l = plot(t_sec, [ROC_mps_est_apriori(:,1) ROC_mps_truth(:,1) ROC_mps_est(:,1)]); hold on;
xlabel('t [sec]'); 
ylabel('ROC [mps]');
l(1).Color = 'b';
l(1).LineWidth = 1.0;
l(1).LineStyle = '--';
l(2).Color = 'k';
l(2).LineWidth = 1.0;
l(3).Color = 'b';
ax5.YColor = 'k';
ax5.XMinorGrid = 'on';
ax5.YMinorGrid = 'on';
grid on;

ax2 = subplot('Position',[0.55 0.71 0.43 0.27]);
%ax2 = subplot(3,2,2);
l = plot(t_sec, [(T_degK_sensed(:,1) - T_degK_truth(:,1)) ...
                 (T_degK_est_apriori(:,1) - T_degK_truth(:,1)) ...
                 mean_T_degK_est_apriori(:,1) ...
                 (T_degK_est(:,1)         - T_degK_truth(:,1)) ...
                 (-sigma_T_degK_est(:,1)) ...
                 sigma_T_degK_est(:,1) ...
                 mean_T_degK_est(:,1) ...
                 bias_oat_degK(:,1)]); hold on;
xlabel('t [sec]'); 
legend('sensed error', ...
       'priori error', 'priori error mean', ...
       'poster error', 'poster std -', 'poster std +', 'poster error mean', 'bias');
ylabel('Error T [degK]');
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
ax2.YLim = [-lim_T_err lim_T_err];
grid on;

ax4 = subplot('Position',[0.55 0.38 0.43 0.27]);
%ax4 = subplot(3,2,4);
l = plot(t_sec, [(Hp_m_sensed(:,1) - Hp_m_truth(:,1)) ...
                 (Hp_m_est_apriori(:,1) - Hp_m_truth(:,1)) ...
                 (-sigma_Hp_m_est_apriori(:,1)) sigma_Hp_m_est_apriori(:,1) mean_Hp_m_est_apriori(:,1) ...
                 (Hp_m_est(:,1)         - Hp_m_truth(:,1)) (-sigma_Hp_m_est(:,1))         sigma_Hp_m_est(:,1)         mean_Hp_m_est(:,1) bias_Hp_m(:,1)]); hold on;
xlabel('t [sec]'); 
ylabel('Error H_{P} [m]');
l(1).Color = 'g';
l(1).Marker = 'x'; 
l(1).LineStyle = 'none'; 
l(1).MarkerSize = 6;  
l(2).Color = 'k';
l(3).Color = 'k';
l(3).LineStyle = '--';
l(4).Color = 'k';
l(4).LineStyle = '--';
l(5).Color = 'k';
l(5).LineStyle = ':';
l(6).Color = 'b';
l(7).Color = 'b';
l(7).LineStyle = '--';
l(8).Color = 'b';
l(8).LineStyle = '--';
l(9).Color = 'b';
l(9).LineStyle = ':';
l(10).Color = 'r';
ax4.YColor = 'k';
ax4.XMinorGrid = 'on';
ax4.YMinorGrid = 'on';
ax4.YLim = [-lim_Hp_err lim_Hp_err];
grid on;

ax6 = subplot('Position',[0.55 0.05 0.43 0.27]);
%ax6 = subplot(3,2,6);
l = plot(t_sec, [(ROC_mps_est_apriori(:,1) - ROC_mps_truth(:,1)) (-sigma_ROC_mps_est_apriori(:,1)) sigma_ROC_mps_est_apriori(:,1) mean_ROC_mps_est_apriori(:,1) ...
                 (ROC_mps_est(:,1)         - ROC_mps_truth(:,1)) (-sigma_ROC_mps_est(:,1))         sigma_ROC_mps_est(:,1)         mean_ROC_mps_est(:,1)]); hold on;
xlabel('t [sec]'); 
ylabel('Error ROC [mps]');
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
    