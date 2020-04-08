function display_filter_att_euler(folder_name, varargin)

fileID = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_att_euler.txt']);
fgetl(fileID);
input = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_att_euler'];

t_sec                         = input{1};  % time
psi_deg_est(:,1)              = input{2};  % psi estimated (a posteriori)
theta_deg_est(:,1)            = input{3};  % theta estimated
xi_deg_est(:,1)               = input{4};  % xi estimated
psi_deg_est_apriori(:,1)      = input{5};  % psi estimated (a priori)
theta_deg_est_apriori(:,1)    = input{6};  % theta estimated
xi_deg_est_apriori(:,1)       = input{7};  % xi estimated
psi_deg_truth(:,1)            = input{8};  % psi truth
theta_deg_truth(:,1)          = input{9};  % theta truth
xi_deg_truth(:,1)             = input{10}; % xi truth
error_angle_deg_est           = input{11}; % angle error (a posteriori)
error_angle_deg_apriori       = input{12}; % angle error (a priori)
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
psi_deg_est              = psi_deg_est(I,:);
theta_deg_est            = theta_deg_est(I,:);
xi_deg_est               = xi_deg_est(I,:);
psi_deg_est_apriori      = psi_deg_est_apriori(I,:);
theta_deg_est_apriori    = theta_deg_est_apriori(I,:);
xi_deg_est_apriori       = xi_deg_est_apriori(I,:);
psi_deg_truth            = psi_deg_truth(I,:);
theta_deg_truth          = theta_deg_truth(I,:);
xi_deg_truth             = xi_deg_truth(I,:);
error_angle_deg_est      = error_angle_deg_est(I,:);
error_angle_deg_apriori  = error_angle_deg_apriori(I,:);

error_psi_deg_est_apriori   = psi_deg_est_apriori - psi_deg_truth;
error_psi_deg_est           = psi_deg_est - psi_deg_truth;
error_theta_deg_est_apriori = theta_deg_est_apriori - theta_deg_truth;
error_theta_deg_est         = theta_deg_est - theta_deg_truth;
error_xi_deg_est_apriori    = xi_deg_est_apriori - xi_deg_truth;
error_xi_deg_est            = xi_deg_est - xi_deg_truth;

error_psi_deg_est_apriori(error_psi_deg_est_apriori > 180)   = error_psi_deg_est_apriori(error_psi_deg_est_apriori > 180) - 360;
error_psi_deg_est        (error_psi_deg_est         > 180)   = error_psi_deg_est        (error_psi_deg_est         > 180) - 360;
error_psi_deg_est_apriori(error_psi_deg_est_apriori <= -180) = error_psi_deg_est_apriori(error_psi_deg_est_apriori <= -180) + 360;
error_psi_deg_est        (error_psi_deg_est         <= -180) = error_psi_deg_est        (error_psi_deg_est         <= -180) + 360;

maxs_psi(1) = max(psi_deg_truth);
maxs_psi(2) = max(psi_deg_est_apriori);
maxs_psi(3) = max(psi_deg_est);
max_psi = max(maxs_psi) + 10;
mins_psi(1) = min(psi_deg_truth);
mins_psi(2) = min(psi_deg_est_apriori);
mins_psi(3) = min(psi_deg_est);
min_psi = min(mins_psi) - 10;

maxs_theta(1) = max(theta_deg_truth);
maxs_theta(2) = max(theta_deg_est_apriori);
maxs_theta(3) = max(theta_deg_est);
max_theta = max(maxs_theta);
mins_theta(1) = min(theta_deg_truth);
mins_theta(2) = min(theta_deg_est_apriori);
mins_theta(3) = min(theta_deg_est);
min_theta = min(mins_theta);
Delta_theta = max_theta - min_theta;
max_theta = max_theta + 0.05 * Delta_theta;
min_theta = min_theta - 0.05 * Delta_theta;

maxs_xi(1) = max(xi_deg_truth);
maxs_xi(2) = max(xi_deg_est_apriori);
maxs_xi(3) = max(xi_deg_est);
max_xi = max(maxs_xi);
mins_xi(1) = min(xi_deg_truth);
mins_xi(2) = min(xi_deg_est_apriori);
mins_xi(3) = min(xi_deg_est);
min_xi = min(mins_xi);
Delta_xi = max_xi - min_xi;
max_xi = max_xi + 0.05 * Delta_xi;
min_xi = min_xi - 0.05 * Delta_xi;

h = figure('units','normalized','position',[0 0.15 1.0 0.85]);

ax1 = subplot('Position',[0.05 0.77 0.91 0.19]);
[hAx,hLine1,hLine2] = plotyy(t_sec, [psi_deg_est_apriori psi_deg_truth psi_deg_est], ...
                             t_sec, [error_psi_deg_est_apriori error_psi_deg_est]); hold on;
xlabel('t [sec]');
ylabel(hAx(1),'\psi [deg]');
ylabel(hAx(2),'Error \psi [deg]');
hLine1(1).Color = 'b';       
hLine1(1).LineStyle = '--'; 
hLine1(2).Color = 'k';       
hLine1(3).Color = 'b';
hLine2(1).Color = 'r';
hLine2(1).LineStyle = '--'; 
hLine2(2).Color = 'r';
hAx(1).YColor = 'k';
hAx(2).YColor = 'r';
%ax1.YLim = [min_psi max_psi];
hAx(1).XMinorGrid = 'on';
hAx(1).YMinorGrid = 'on';
legend(hAx(1), 'est a priori', 'truth', 'est a posteriori', 'Error a priori', 'Error a posteriori');
grid on;   

ax2 = subplot('Position',[0.05 0.53 0.91 0.19]);
[jAx,jLine1,jLine2] = plotyy(t_sec, [theta_deg_est_apriori theta_deg_truth theta_deg_est], ...
                             t_sec, [error_theta_deg_est_apriori error_theta_deg_est]); hold on;
xlabel('t [sec]');
ylabel(jAx(1),'\theta [deg]');
ylabel(jAx(2),'Error \theta [deg]');
jLine1(1).Color = 'b';       
jLine1(1).LineStyle = '--'; 
jLine1(2).Color = 'k';       
jLine1(3).Color = 'b';
jLine2(1).Color = 'r';
jLine2(1).LineStyle = '--'; 
jLine2(2).Color = 'r';
jAx(1).YColor = 'k';
jAx(2).YColor = 'r';
%ax2.YLim = [min_theta max_theta];
jAx(1).XMinorGrid = 'on';
jAx(1).YMinorGrid = 'on';
grid on;   

ax3 = subplot('Position',[0.05 0.29 0.91 0.19]);
[kAx,kLine1,kLine2] = plotyy(t_sec, [xi_deg_est_apriori xi_deg_truth xi_deg_est], ...
                             t_sec, [error_xi_deg_est_apriori error_xi_deg_est]); hold on;
%kAx(1).YLim = [min_xi max_xi];
xlabel('t [sec]');
ylabel(kAx(1),'\xi [deg]');
ylabel(kAx(2),'Error \xi [deg]');
kLine1(1).Color = 'b';       
kLine1(1).LineStyle = '--'; 
kLine1(2).Color = 'k';       
kLine1(3).Color = 'b';
kLine2(1).Color = 'r';
kLine2(1).LineStyle = '--'; 
kLine2(2).Color = 'r';
kAx(1).YColor = 'k';
kAx(2).YColor = 'r';
kAx(1).XMinorGrid = 'on';
kAx(1).YMinorGrid = 'on';
grid on;   

ax4 = subplot('Position',[0.05 0.05 0.91 0.19]);
l = plot(t_sec, [error_angle_deg_apriori error_angle_deg_est]); hold on;
xlabel('t [sec]'); 
ylabel('Error Rotation vector [deg]');
l(1).Color = 'r';
l(1).LineStyle = '--'; 
l(2).Color = 'r';
ax4.YColor = 'r';
ax4.XMinorGrid = 'on';
ax4.YMinorGrid = 'on';
legend('Error a priori', 'Error a posteriori');
grid on;

%savefig([plot_name '.fig']);

%Image = getframe(gcf);
%imwrite(Image.cdata, [plot_name '.jpg']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    