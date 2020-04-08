function display_filter_pos_obs_vned(folder_name, varargin)

fileID = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_gps_obs_vned.txt']);
fgetl(fileID);
input = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_filter_pos_obs_vned'];

t_sec                      = input{1};
v_ned_mps_truth(:,1)       = input{2}; % v_ned_mps truth - sensor input
v_ned_mps_truth(:,2)       = input{3};
v_ned_mps_truth(:,3)       = input{4};
v_ned_mps_sens(:,1)        = input{5}; % v_ned_mps sensor output - observations y
v_ned_mps_sens(:,2)        = input{6};
v_ned_mps_sens(:,3)        = input{7};
v_ned_mps_innov(:,1)       = input{8}; % v_ned_mps innovations r
v_ned_mps_innov(:,2)       = input{9};
v_ned_mps_innov(:,3)       = input{10};
v_ned_mps_innov_std(:,1)   = input{11}; % v_ned_mps innovations standard deviations
v_ned_mps_innov_std(:,2)   = input{12};
v_ned_mps_innov_std(:,3)   = input{13};
v_ned_mps_innov_mean(:,1)  = input{14}; % v_ned_mps innovations running mean
v_ned_mps_innov_mean(:,2)  = input{15};
v_ned_mps_innov_mean(:,3)  = input{16};
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
t_sec                 = t_sec(I);
v_ned_mps_truth       = v_ned_mps_truth(I,:);
v_ned_mps_sens        = v_ned_mps_sens(I,:);
v_ned_mps_innov       = v_ned_mps_innov(I,:);
v_ned_mps_eval        = v_ned_mps_sens - v_ned_mps_innov;
v_ned_mps_innov_std   = v_ned_mps_innov_std(I,:);
v_ned_mps_innov_mean  = v_ned_mps_innov_mean(I,:);

figure('units','normalized','position',[0 0.15 1.0 0.85]);

ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
l = plot(t_sec, [v_ned_mps_sens(:,1) v_ned_mps_truth(:,1) v_ned_mps_eval(:,1)]); hold on;
xlabel('t [sec]');
ylabel('v_{1}^{N} [mps]');
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
l = plot(t_sec, [v_ned_mps_sens(:,2) v_ned_mps_truth(:,2) v_ned_mps_eval(:,2)]); hold on;
xlabel('t [sec]');
ylabel('v_{2}^{N} [mps]');
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
l = plot(t_sec, [v_ned_mps_sens(:,3) v_ned_mps_truth(:,3) v_ned_mps_eval(:,3)]); hold on;
xlabel('t [sec]');
ylabel('v_{3}^{N} [mps]');
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
l = plot(t_sec, [v_ned_mps_innov(:,1) (-v_ned_mps_innov_std(:,1)) v_ned_mps_innov_std(:,1) v_ned_mps_innov_mean(:,1)]); hold on;
xlabel('t [sec]'); 
legend('innovation', 'std -', 'std +', 'innov mean');
ylabel('Error v_{1}^{N} [mps]');
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
l = plot(t_sec, [v_ned_mps_innov(:,2) (-v_ned_mps_innov_std(:,2)) v_ned_mps_innov_std(:,2) v_ned_mps_innov_mean(:,2)]); hold on;
xlabel('t [sec]'); 
ylabel('Error v_{2}^{N} [mps]');
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
l = plot(t_sec, [v_ned_mps_innov(:,3) (-v_ned_mps_innov_std(:,3)) v_ned_mps_innov_std(:,3) v_ned_mps_innov_mean(:,3)]); hold on;
xlabel('t [sec]'); 
ylabel('Error v_{3}^{N} [mps]');
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




