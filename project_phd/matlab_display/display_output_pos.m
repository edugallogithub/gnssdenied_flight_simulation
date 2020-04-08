function display_output_pos(folder_name, varargin)

fileID = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/output_pos.txt']);
fgetl(fileID);
input = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_output_pos'];

t_sec                        = input{1};  % time
lambda_deg_est               = input{2};  % lambda_deg estimated
phi_deg_est                  = input{3};  % phi_deg    estimated
h_m_est                      = input{4};  % h_m        estimated
lambda_deg_truth             = input{5};  % lambda_deg truth
phi_deg_truth                = input{6};  % phi_deg    truth
h_m_truth                    = input{7}; % h_m        truth
lambda_deg_sensed            = input{8}; % lambda_deg sensed
phi_deg_sensed               = input{9}; % phi_deg    sensed
h_m_sensed                   = input{10}; % h_m        sensed
dist_north_m_est             = input{11}; % North distance to initial point estimated
dist_east_m_est              = input{12}; % East distance to initial point estimated
dist_north_m_truth           = input{13}; % North distance to initial point truth
dist_east_m_truth            = input{14}; % East distance to initial point truth
dist_north_m_sensed          = input{15}; % North distance to initial point sensed
dist_east_m_sensed           = input{16}; % East distance to initial point sensed
dist_air_hor_m               = input{17}; % horizontal distance (air)
dist_grd_hor_m               = input{18}; % horizontal distance (ground)
error_hor_m                  = input{19}; % horizontal error
error_cross_m                = input{20}; % cross track error
error_long_m                 = input{21}; % long track error
error_ver_m                  = input{22}; % vertical error

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

for i = 1:numel(t_sec)
    if (t_sec(i) >= 0)
        i_init = i;
        break;
    end
end
dist_air_hor_m = dist_air_hor_m - dist_air_hor_m(i_init);
dist_grd_hor_m = dist_grd_hor_m - dist_grd_hor_m(i_init);

I = find((t_sec >= tmin_sec) & (t_sec <= tmax_sec));
t_sec                        = t_sec(I);
lambda_deg_est               = lambda_deg_est(I,:);
phi_deg_est                  = phi_deg_est(I,:);
h_m_est                      = h_m_est(I,:);
lambda_deg_truth             = lambda_deg_truth(I,:);
phi_deg_truth                = phi_deg_truth(I,:);
h_m_truth                    = h_m_truth(I,:);
lambda_deg_sensed            = lambda_deg_sensed(I,:);
phi_deg_sensed               = phi_deg_sensed(I,:);
h_m_sensed                   = h_m_sensed(I,:);
dist_north_m_est             = dist_north_m_est(I,:);
dist_east_m_est              = dist_east_m_est(I,:);
dist_north_m_truth           = dist_north_m_truth(I,:);
dist_east_m_truth            = dist_east_m_truth(I,:);
dist_north_m_sensed          = dist_north_m_sensed(I,:);
dist_east_m_sensed           = dist_east_m_sensed(I,:);
dist_air_hor_m               = dist_air_hor_m(I,:);
dist_grd_hor_m               = dist_grd_hor_m(I,:);
error_hor_m                  = error_hor_m(I,:);
error_cross_m                = error_cross_m(I,:);
error_long_m                 = error_long_m(I,:);
error_ver_m                  = error_ver_m(I,:);

figure('units','normalized','position',[0 0.15 1.0 0.85]);

ax1 = subplot('Position',[0.05 0.05 0.43 0.90]);
l = plot(dist_east_m_sensed,      dist_north_m_sensed, ...
         dist_east_m_est,         dist_north_m_est, ...
         dist_east_m_truth,       dist_north_m_truth, ...
         dist_east_m_truth(1),    dist_north_m_truth(1), ...
         dist_east_m_truth(end),  dist_north_m_truth(end), ...
         dist_east_m_est(end),    dist_north_m_est(end)); hold on;
xlabel('East distance [m]'); 
ylabel('North distance [m]');
axis equal;
l(1).Color = 'g';
l(1).Marker = 'x'; 
l(1).LineStyle = 'none'; 
l(1).MarkerSize = 6;  
l(2).Color = 'b';
l(3).Color = 'k';
l(4).Color = 'r';
l(4).Marker = 'o';
l(4).LineStyle = 'none'; 
l(5).Color = 'r';
l(5).Marker = 'pentagram';
l(5).LineStyle = 'none'; 
l(6).Color = 'r';
l(6).Marker = 'pentagram';
l(6).LineStyle = 'none'; 
ax1.YColor = 'k';
ax1.XMinorGrid = 'on';
ax1.YMinorGrid = 'on';
legend(l, 'sensed', 'est', 'truth', 'start', 'finish truth', 'finish est');
grid on;

ax3 = subplot('Position',[0.55 0.55 0.43 0.40]);
[hAx, hLine1, hLine2] = plotyy(dist_air_hor_m, [error_cross_m, error_long_m, error_hor_m], dist_air_hor_m, error_ver_m); hold on;
xlabel('Horizontal "air" distance [m]'); 
ylabel(hAx(1),'Horizontal Error (est - truth) [m]');
ylabel(hAx(2),'Vertical Error (est - truth) [m]');
hLine1(1).Color = 'b';
hLine1(2).Color = 'r';
hLine1(3).Color = 'k';
hLine2.Color = 'm';
hAx(1).YColor = 'k';
hAx(2).YColor = 'm';
hAx(1).XMinorGrid = 'on';
hAx(1).YMinorGrid = 'on';
legend(hAx(1), 'cross track', 'long track', 'horizontal', 'vertical');
grid on;

ax4 = subplot('Position',[0.55 0.05 0.43 0.40]);
[jAx, jLine1, jLine2] = plotyy(dist_grd_hor_m, [error_cross_m, error_long_m, error_hor_m], dist_grd_hor_m, error_ver_m); hold on;
xlabel('Horizontal "ground" distance [m]'); 
ylabel(jAx(1),'Horizontal Error (est - truth) [m]');
ylabel(jAx(2),'Vertical Error (est - truth) [m]');
jLine1(1).Color = 'b';
jLine1(2).Color = 'r';
jLine1(3).Color = 'k';
jLine2.Color = 'm';
jAx(1).YColor = 'k';
jAx(2).YColor = 'm';
jAx(1).XMinorGrid = 'on';
jAx(1).YMinorGrid = 'on';
legend(jAx(1), 'cross track', 'long track', 'horizontal', 'vertical');
grid on;

%savefig([plot_name '.fig']);

%Image = getframe(gcf);
%imwrite(Image.cdata, [plot_name '.jpg']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    






