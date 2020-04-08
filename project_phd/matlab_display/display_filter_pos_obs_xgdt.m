function display_filter_pos_obs_xgdt(folder_name, varargin)

fileID = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_gps_obs_xgdt.txt']);
fgetl(fileID);
input = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_filter_pos_obs_xgdt'];

t_sec                  = input{1};
lambda_deg_truth       = input{2};  % lambda_deg truth - sensor input
phi_deg_truth          = input{3};  % phi_deg    truth - sensor input
h_m_truth              = input{4};  % h_m        truth - sensor input
lambda_deg_sens        = input{5};  % lambda_deg sensor output - observations y
phi_deg_sens           = input{6};  % phi_deg    sensor output - observations y
h_m_sens               = input{7};  % h_m        sensor output - observations y
lambda_deg_innov       = input{8};  % lambda_deg innovations r
phi_deg_innov          = input{9};  % phi_deg    innovations r
h_m_innov              = input{10}; % h_m        innovations r
lambda_deg_innov_std   = input{11}; % lambda_deg innovations standard deviations
phi_deg_innov_std      = input{12}; % phi_deg    innovations standard deviations
h_m_innov_std          = input{13}; % h_m        innovations standard deviations
lambda_deg_innov_mean  = input{14}; % lambda_deg innovations running mean
phi_deg_innov_mean     = input{15}; % phi_deg    innovations running mean
h_m_innov_mean         = input{16}; % h_m        innovations running mean
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
t_sec                  = t_sec(I);
lambda_deg_truth       = lambda_deg_truth(I,:);
phi_deg_truth          = phi_deg_truth(I,:);
h_m_truth              = h_m_truth(I,:);
lambda_deg_sens        = lambda_deg_sens(I,:);
phi_deg_sens           = phi_deg_sens(I,:);
h_m_sens               = h_m_sens(I,:);
lambda_deg_innov       = lambda_deg_innov(I,:);
phi_deg_innov          = phi_deg_innov(I,:);
h_m_innov              = h_m_innov(I,:);
lambda_deg_eval        = lambda_deg_sens - lambda_deg_innov;
phi_deg_eval           = phi_deg_sens - phi_deg_innov;
h_m_eval               = h_m_sens - h_m_innov;
lambda_deg_innov_std   = lambda_deg_innov_std(I,:);
phi_deg_innov_std      = phi_deg_innov_std(I,:);
h_m_innov_std          = h_m_innov_std(I,:);
lambda_deg_innov_mean  = lambda_deg_innov_mean(I,:);
phi_deg_innov_mean     = phi_deg_innov_mean(I,:);
h_m_innov_mean         = h_m_innov_mean(I,:);

figure('units','normalized','position',[0 0.15 1.0 0.85]);

ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
l = plot(t_sec, [lambda_deg_sens lambda_deg_truth lambda_deg_eval]); hold on;
xlabel('t [sec]');
ylabel('\lambda [deg]');
l(1).Color = 'g';     
l(1).Marker = 'x'; 
l(1).LineStyle = 'None'; 
l(2).Color = 'k';       
l(3).Color = 'b';       
ax1.YColor = 'k';
ax1.XMinorGrid = 'on';
ax1.YMinorGrid = 'on';
legend('sensed', 'truth', 'evaluated');
grid on;    

ax3 = subplot('Position',[0.05 0.38 0.43 0.27]);
l = plot(t_sec, [phi_deg_sens phi_deg_truth phi_deg_eval]); hold on;
xlabel('t [sec]');
ylabel('\phi [deg]');
l(1).Color = 'g';     
l(1).Marker = 'x'; 
l(1).LineStyle = 'None'; 
l(2).Color = 'k';       
l(3).Color = 'b';       
ax3.YColor = 'k';
ax3.XMinorGrid = 'on';
ax3.YMinorGrid = 'on';
grid on;    

ax5 = subplot('Position',[0.05 0.05 0.43 0.27]);
l = plot(t_sec, [h_m_sens h_m_truth h_m_eval]); hold on;
xlabel('t [sec]');
ylabel('h [m]');
l(1).Color = 'g';     
l(1).Marker = 'x'; 
l(1).LineStyle = 'None'; 
l(2).Color = 'k';       
l(3).Color = 'b';       
ax5.YColor = 'k';
ax5.XMinorGrid = 'on';
ax5.YMinorGrid = 'on';
grid on;    

ax2 = subplot('Position',[0.55 0.71 0.43 0.27]);
l = plot(t_sec, [lambda_deg_innov (-lambda_deg_innov_std) lambda_deg_innov_std lambda_deg_innov_mean]); hold on;
xlabel('t [sec]'); 
legend('innovation', 'std -', 'std +', 'innov mean');
ylabel('Error \lambda [deg]');
l(1).Color = 'b';
l(2).Color = 'r';
l(2).LineStyle = '--';
l(2).LineWidth = 3.;
l(3).Color = 'r';
l(3).LineStyle = '--';
l(3).LineWidth = 3.;
l(4).Color = 'b';
l(4).LineStyle = ':';
ax2.YColor = 'k';
ax2.XMinorGrid = 'on';
ax2.YMinorGrid = 'on';
grid on;

ax4 = subplot('Position',[0.55 0.38 0.43 0.27]);
l = plot(t_sec, [phi_deg_innov (-phi_deg_innov_std) phi_deg_innov_std phi_deg_innov_mean]); hold on;
xlabel('t [sec]'); 
ylabel('Error \phi [deg]');
l(1).Color = 'b';
l(2).Color = 'r';
l(2).LineStyle = '--';
l(2).LineWidth = 3.;
l(3).Color = 'r';
l(3).LineStyle = '--';
l(3).LineWidth = 3.;
l(4).Color = 'b';
l(4).LineStyle = ':';
ax4.YColor = 'k';
ax4.XMinorGrid = 'on';
ax4.YMinorGrid = 'on';
grid on;

ax6 = subplot('Position',[0.55 0.05 0.43 0.27]);
l = plot(t_sec, [h_m_innov (-h_m_innov_std) h_m_innov_std h_m_innov_mean]); hold on;
xlabel('t [sec]'); 
ylabel('Error h [m]');
l(1).Color = 'b';
l(2).Color = 'r';
l(2).LineStyle = '--';
l(2).LineWidth = 3.;
l(3).Color = 'r';
l(3).LineStyle = '--';
l(3).LineWidth = 3.;
l(4).Color = 'b';
l(4).LineStyle = ':';
ax6.YColor = 'k';
ax6.XMinorGrid = 'on';
ax6.YMinorGrid = 'on';
grid on;

%savefig([plot_name '.fig']);

%Image = getframe(gcf);
%imwrite(Image.cdata, [plot_name '.jpg']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




