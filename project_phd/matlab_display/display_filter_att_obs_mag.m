function display_filter_att_obs_mag(folder_name, varargin)

fileID = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_att_obs_mag.txt']);
fgetl(fileID);
input = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_filter_att_obs_mag'];

t_sec                            = input{1};
B_b_nT_truth(:,1)                = input{2}; % B_b_nT truth - sensor input
B_b_nT_truth(:,2)                = input{3};
B_b_nT_truth(:,3)                = input{4};
B_b_nT_sens(:,1)                 = input{5}; % B_b_nT sensor output - observations y
B_b_nT_sens(:,2)                 = input{6};
B_b_nT_sens(:,3)                 = input{7};
B_b_nT_innov(:,1)                = input{8}; % B_b_nT innovations r
B_b_nT_innov(:,2)                = input{9};
B_b_nT_innov(:,3)                = input{10};
B_b_nT_innov_std(:,1)            = input{11}; % B_b_nT innovations standard deviations
B_b_nT_innov_std(:,2)            = input{12};
B_b_nT_innov_std(:,3)            = input{13};
B_b_nT_innov_mean(:,1)           = input{14}; % B_b_nT innovations running mean
B_b_nT_innov_mean(:,2)           = input{15};
B_b_nT_innov_mean(:,3)           = input{16};
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
B_b_nT_truth                = B_b_nT_truth(I,:);
B_b_nT_sens                 = B_b_nT_sens(I,:);
B_b_nT_innov                = B_b_nT_innov(I,:);
B_b_nT_eval                 = B_b_nT_sens - B_b_nT_innov;
B_b_nT_innov_std            = B_b_nT_innov_std(I,:);
B_b_nT_innov_mean           = B_b_nT_innov_mean(I,:);

figure('units','normalized','position',[0 0.15 1.0 0.85]);

ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
%ax1 = subplot(3,2,1);
l = plot(t_sec, [B_b_nT_sens(:,1) B_b_nT_truth(:,1) B_b_nT_eval(:,1)]); hold on;
xlabel('t [sec]');
ylabel('B_{1}^{B} [nT]');
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
%ax3 = subplot(3,2,3);
l = plot(t_sec, [B_b_nT_sens(:,2) B_b_nT_truth(:,2) B_b_nT_eval(:,2)]); hold on;
xlabel('t [sec]');
ylabel('B_{2}^{B} [nT]');
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
%ax5 = subplot(3,2,5);
l = plot(t_sec, [B_b_nT_sens(:,3) B_b_nT_truth(:,3) B_b_nT_eval(:,3)]); hold on;
xlabel('t [sec]');
ylabel('B_{3}^{B} [nT]');
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
%ax2 = subplot(3,2,2);
l = plot(t_sec, [B_b_nT_innov(:,1) (-B_b_nT_innov_std(:,1)) B_b_nT_innov_std(:,1) B_b_nT_innov_mean(:,1)]); hold on;
xlabel('t [sec]'); 
legend('innovation', 'std -', 'std +', 'innov mean');
ylabel('Error B_{1}^{B} [nT]');
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
%ax4 = subplot(3,2,4);
l = plot(t_sec, [B_b_nT_innov(:,2) (-B_b_nT_innov_std(:,2)) B_b_nT_innov_std(:,2) B_b_nT_innov_mean(:,2)]); hold on;
xlabel('t [sec]'); 
legend('innovation', 'std -', 'std +', 'innov mean');
ylabel('Error B_{2}^{B} [nT]');
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
%ax6 = subplot(3,2,6);
l = plot(t_sec, [B_b_nT_innov(:,3) (-B_b_nT_innov_std(:,3)) B_b_nT_innov_std(:,3) B_b_nT_innov_mean(:,3)]); hold on;
xlabel('t [sec]'); 
legend('innovation', 'std -', 'std +', 'innov mean');
ylabel('Error B_{3}^{B} [nT]');
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
    



