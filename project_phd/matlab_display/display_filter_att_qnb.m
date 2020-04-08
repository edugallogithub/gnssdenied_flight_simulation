function display_att_qnb(folder_name, varargin)

fileID = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_att_qnb.txt']);
fgetl(fileID);
input = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_filter_att_qnb'];

t_sec                         = input{1}; % time
q_nb_est(:,1)                 = input{2}; % q_nb estimated (a posteriori)
q_nb_est(:,2)                 = input{3};
q_nb_est(:,3)                 = input{4};
q_nb_est(:,4)                 = input{5};
q_nb_est_norm(:,1)            = input{6}; % q_nb estimated norm (a posteriori)
q_nb_est_apriori(:,1)         = input{7}; % q_nb estimated (a priori)
q_nb_est_apriori(:,2)         = input{8};
q_nb_est_apriori(:,3)         = input{9};
q_nb_est_apriori(:,4)         = input{10};
q_nb_est_apriori_norm(:,1)    = input{11}; % q_nb estimated norm (a priori)
q_nb_truth(:,1)               = input{12}; % q_nb truth
q_nb_truth(:,2)               = input{13};
q_nb_truth(:,3)               = input{14};
q_nb_truth(:,4)               = input{15};
sigma_q_nb_est(:,1)           = input{16}; % q_nb estimated error std (a posteriori)
sigma_q_nb_est(:,2)           = input{17};
sigma_q_nb_est(:,3)           = input{18};
sigma_q_nb_est(:,4)           = input{19};
sigma_q_nb_est_apriori(:,1)   = input{20}; % q_nb estimated error std (a priori)
sigma_q_nb_est_apriori(:,2)   = input{21};
sigma_q_nb_est_apriori(:,3)   = input{22};
sigma_q_nb_est_apriori(:,4)   = input{23};
mean_q_nb_est(:,1)            = input{24}; % q_nb estimated error mean (a posteriori)
mean_q_nb_est(:,2)            = input{25};
mean_q_nb_est(:,3)            = input{26};
mean_q_nb_est(:,4)            = input{27};
mean_q_nb_est_apriori(:,1)    = input{28}; % q_nb estimated error mean (a priori)
mean_q_nb_est_apriori(:,2)    = input{29};
mean_q_nb_est_apriori(:,3)    = input{30};
mean_q_nb_est_apriori(:,4)    = input{31};
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
t_sec                    = t_sec(I);
q_nb_est                 = q_nb_est(I,:);
q_nb_est_norm            = q_nb_est_norm(I,:);
q_nb_est_apriori         = q_nb_est_apriori(I,:);
q_nb_est_apriori_norm    = q_nb_est_apriori_norm(I,:);
q_nb_truth               = q_nb_truth(I,:);
sigma_q_nb_est           = sigma_q_nb_est(I,:);
sigma_q_nb_est_apriori   = sigma_q_nb_est_apriori(I,:);
mean_q_nb_est            = mean_q_nb_est(I,:);
mean_q_nb_est_apriori    = mean_q_nb_est_apriori(I,:);

h = figure('units','normalized','position',[0 0.15 1.0 0.85]);

ax1 = subplot('Position',[0.05 0.80 0.43 0.19]);
%ax1 = subplot(4,2,1);
l = plot(t_sec, [q_nb_est_apriori(:,1) q_nb_truth(:,1) q_nb_est(:,1)]); hold on;
xlabel('t [sec]'); 
ylabel('q_0');
l(1).Color = 'b';
l(1).LineWidth = 1.0;
l(1).LineStyle = '--';
l(2).Color = 'k';
l(2).LineWidth = 1.0;
l(3).Color = 'b';
ax1.YColor = 'k';
ax1.XMinorGrid = 'on';
ax1.YMinorGrid = 'on';
legend(l, 'est a priori', 'truth', 'est a posteriori');
grid on;

x3 = subplot('Position',[0.05 0.55 0.43 0.19]);
%ax3 = subplot(4,2,3);
l = plot(t_sec, [q_nb_est_apriori(:,2) q_nb_truth(:,2) q_nb_est(:,2)]); hold on;
xlabel('t [sec]'); 
ylabel('q_1');
l(1).Color = 'b';
l(1).LineWidth = 1.0;
l(1).LineStyle = '--';
l(2).Color = 'k';
l(2).LineWidth = 1.0;
l(3).Color = 'b';
l(3).LineWidth = 1.0;
ax3.YColor = 'k';
ax3.XMinorGrid = 'on';
ax3.YMinorGrid = 'on';
grid on;

ax5 = subplot('Position',[0.05 0.30 0.43 0.19]);
%ax5 = subplot(4,2,5);
l = plot(t_sec, [q_nb_est_apriori(:,3) q_nb_truth(:,3) q_nb_est(:,3)]); hold on;
xlabel('t [sec]'); 
ylabel('q_2');
l(1).Color = 'b';
l(1).LineWidth = 1.0;
l(1).LineStyle = '--';
l(2).Color = 'k';
l(2).LineWidth = 1.0;
l(3).Color = 'b';
l(3).LineWidth = 1.0;
ax5.YColor = 'k';
ax5.XMinorGrid = 'on';
ax5.YMinorGrid = 'on';
grid on;

ax7 = subplot('Position',[0.05 0.05 0.43 0.19]);
%ax7 = subplot(4,2,7);
l = plot(t_sec, [q_nb_est_apriori(:,4) q_nb_truth(:,4) q_nb_est(:,4)]); hold on;
xlabel('t [sec]'); 
ylabel('q_3');
l(1).Color = 'b';
l(1).LineWidth = 1.0;
l(1).LineStyle = '--';
l(2).Color = 'k';
l(2).LineWidth = 1.0;
l(3).Color = 'b';
l(3).LineWidth = 1.0;
ax7.YColor = 'k';
ax7.XMinorGrid = 'on';
ax7.YMinorGrid = 'on';
grid on;

ax2 = subplot('Position',[0.55 0.80 0.43 0.19]);
%ax2 = subplot(4,2,2);
l = plot(t_sec, [(q_nb_est_apriori(:,1) - q_nb_truth(:,1)) (-sigma_q_nb_est_apriori(:,1)) sigma_q_nb_est_apriori(:,1) mean_q_nb_est_apriori(:,1) ...
                 (q_nb_est(:,1)         - q_nb_truth(:,1)) (-sigma_q_nb_est(:,1))         sigma_q_nb_est(:,1)         mean_q_nb_est(:,1)]); hold on;
xlabel('t [sec]'); 
legend('priori error', 'priori std -', 'priori std +', 'priori error mean', ...
       'poster error', 'poster std -', 'poster std +', 'poster error mean');
ylabel('Error q_0');
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

ax4 = subplot('Position',[0.55 0.55 0.43 0.19]);
%ax4 = subplot(4,2,4);
l = plot(t_sec, [(q_nb_est_apriori(:,2) - q_nb_truth(:,2)) (-sigma_q_nb_est_apriori(:,2)) sigma_q_nb_est_apriori(:,2) mean_q_nb_est_apriori(:,2) ...
                 (q_nb_est(:,2)         - q_nb_truth(:,2)) (-sigma_q_nb_est(:,2))         sigma_q_nb_est(:,2)         mean_q_nb_est(:,2)]); hold on;
xlabel('t [sec]'); 
ylabel('Error q_1');
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

ax6 = subplot('Position',[0.55 0.30 0.43 0.19]);
%ax6 = subplot(4,2,6);
l = plot(t_sec, [(q_nb_est_apriori(:,3) - q_nb_truth(:,3)) (-sigma_q_nb_est_apriori(:,3)) sigma_q_nb_est_apriori(:,3) mean_q_nb_est_apriori(:,3) ...
                 (q_nb_est(:,3)         - q_nb_truth(:,3)) (-sigma_q_nb_est(:,3))         sigma_q_nb_est(:,3)         mean_q_nb_est(:,3)]); hold on;
xlabel('t [sec]'); 
ylabel('Error q_2');
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

ax8 = subplot('Position',[0.55 0.05 0.43 0.19]);
%ax8 = subplot(4,2,8);
l = plot(t_sec, [(q_nb_est_apriori(:,4) - q_nb_truth(:,4)) (-sigma_q_nb_est_apriori(:,4)) sigma_q_nb_est_apriori(:,4) mean_q_nb_est_apriori(:,4) ...
                 (q_nb_est(:,4)         - q_nb_truth(:,4)) (-sigma_q_nb_est(:,4))         sigma_q_nb_est(:,4)         mean_q_nb_est(:,4)]); hold on;
xlabel('t [sec]'); 
ylabel('Error q_3');
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
ax8.YColor = 'k';
ax8.XMinorGrid = 'on';
ax8.YMinorGrid = 'on';
grid on;

%savefig([plot_name '.fig']);

%Image = getframe(gcf);
%imwrite(Image.cdata, [plot_name '.jpg']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    