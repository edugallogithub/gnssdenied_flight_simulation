function display_sens_other(folder_name, varargin)

fileID = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/sens_other.txt']);
fgetl(fileID);
uavSensors = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_sens_other'];

t_sec           = uavSensors{1};
B_b_nT_in(:,1)  = uavSensors{2};
B_b_nT_in(:,2)  = uavSensors{3};
B_b_nT_in(:,3)  = uavSensors{4};
B_b_nT_out(:,1) = uavSensors{5};
B_b_nT_out(:,2) = uavSensors{6};
B_b_nT_out(:,3) = uavSensors{7};
E_mag_nT(:,1)   = uavSensors{8};
E_mag_nT(:,2)   = uavSensors{9};
E_mag_nT(:,3)   = uavSensors{10};
beta_deg_in     = uavSensors{11};
beta_deg_out    = uavSensors{12};
bias_beta_deg   = uavSensors{13};
alpha_deg_in    = uavSensors{14};
alpha_deg_out   = uavSensors{15};
bias_alpha_deg  = uavSensors{16};
vtas_mps_in     = uavSensors{17};
vtas_mps_out    = uavSensors{18};
bias_vtas_mps   = uavSensors{19};
p_pa_in         = uavSensors{20};          
p_pa_out        = uavSensors{21};
bias_osp_pa     = uavSensors{22};
clear uavSensors;

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
t_sec          = t_sec(I);
B_b_nT_in      = B_b_nT_in(I,:);
B_b_nT_out     = B_b_nT_out(I,:);
E_mag_nT       = E_mag_nT(I,:);
beta_deg_in    = beta_deg_in(I); 
beta_deg_out   = beta_deg_out(I);
bias_beta_deg  = bias_beta_deg(I);
alpha_deg_in   = alpha_deg_in(I); 
alpha_deg_out  = alpha_deg_out(I);
bias_alpha_deg = bias_alpha_deg(I);
vtas_mps_in    = vtas_mps_in(I); 
vtas_mps_out   = vtas_mps_out(I);
bias_vtas_mps  = bias_vtas_mps(I);
p_pa_in        = p_pa_in(I); 
p_pa_out       = p_pa_out(I);
bias_osp_pa    = bias_osp_pa(I);

h = figure('units','normalized','position',[0 0.15 1.0 0.85]);

ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
[hAx,hLine1,hLine2] = plotyy(t_sec, [B_b_nT_out(:,1) B_b_nT_in(:,1)], ...
                             t_sec, [(B_b_nT_out(:,1) - B_b_nT_in(:,1)) E_mag_nT(:,1)]); hold on;
xlabel('t [sec]');
ylabel(hAx(1),'B_{B,1} [nT]');
ylabel(hAx(2),'Error B_{B,1} [nT]');

hLine1(1).Color = 'g';       
hLine1(1).Marker = 'x'; 
hLine1(1).LineStyle = 'none'; 
hLine1(1).MarkerSize = 6;  

hLine1(2).Color = 'k';       
hLine1(2).LineStyle = '-'; 
hLine1(2).LineWidth = 2.;  

hLine2(1).Color = 'r';     
hLine2(1).Marker = '.'; 
hLine2(1).LineStyle = 'none'; 
hLine2(1).MarkerSize = 1; 

hLine2(2).Color = 'b';
hLine2(2).LineStyle = '-'; 
hLine2(2).LineWidth = 2.;  

hAx(1).YColor = 'k';
hAx(2).YColor = 'r';
hAx(1).XMinorGrid = 'on';
hAx(1).YMinorGrid = 'on';
%hAx(2).YMinorGrid = 'on';
legend(hAx(1), 'Sensed', 'Truth', 'Error (sens - truth)', 'Error w/o noise');
grid on;    

ax2 = subplot('Position',[0.05 0.38 0.43 0.27]);
[jAx,jLine1,jLine2] = plotyy(t_sec, [B_b_nT_out(:,2) B_b_nT_in(:,2)], ...
                             t_sec, [(B_b_nT_out(:,2) - B_b_nT_in(:,2)) E_mag_nT(:,2)]); hold on;
xlabel('t [sec]');
ylabel(jAx(1),'B_{B,2} [nT]');
ylabel(jAx(2),'Error B_{B,1} [nT]');

jLine1(1).Color = 'g';       
jLine1(1).Marker = 'x'; 
jLine1(1).LineStyle = 'none'; 
jLine1(1).MarkerSize = 6;  

jLine1(2).Color = 'k';       
jLine1(2).LineStyle = '-'; 
jLine1(2).LineWidth = 2.;  

jLine2(1).Color = 'r';     
jLine2(1).Marker = '.'; 
jLine2(1).LineStyle = 'none'; 
jLine2(1).MarkerSize = 1; 

jLine2(2).Color = 'b';
jLine2(2).LineStyle = '-'; 
jLine2(2).LineWidth = 2.;  

jAx(1).YColor = 'k';
jAx(2).YColor = 'r';
jAx(1).XMinorGrid = 'on';
jAx(1).YMinorGrid = 'on';
%jAx(2).YMinorGrid = 'on';
grid on;    

ax3 = subplot('Position',[0.05 0.05 0.43 0.27]);
[kAx,kLine1,kLine2] = plotyy(t_sec, [B_b_nT_out(:,3) B_b_nT_in(:,3)], ...
                             t_sec, [(B_b_nT_out(:,3) - B_b_nT_in(:,3)) E_mag_nT(:,3)]); hold on;
xlabel('t [sec]');
ylabel(kAx(1),'B_{B,3} [nT]');
ylabel(kAx(2),'Error B_{B,1} [nT]');

kLine1(1).Color = 'g';       
kLine1(1).Marker = 'x'; 
kLine1(1).LineStyle = 'none'; 
kLine1(1).MarkerSize = 6;  

kLine1(2).Color = 'k';       
kLine1(2).LineStyle = '-'; 
kLine1(2).LineWidth = 2.;  

kLine2(1).Color = 'r';     
kLine2(1).Marker = '.'; 
kLine2(1).LineStyle = 'none'; 
kLine2(1).MarkerSize = 1; 

kLine2(2).Color = 'b';
kLine2(2).LineStyle = '-'; 
kLine2(2).LineWidth = 2.;  

kAx(1).YColor = 'k';
kAx(2).YColor = 'r';
kAx(1).XMinorGrid = 'on';
kAx(1).YMinorGrid = 'on';
%kAx(2).YMinorGrid = 'on';
grid on;    

%Aax1 = subplot(4,2,2);
Aax1 = subplot('Position',[0.55 0.77 0.42 0.19]);
[AhAx,AhLine1,AhLine2] = plotyy(t_sec, [alpha_deg_out alpha_deg_in], ...
                                t_sec, [(alpha_deg_out - alpha_deg_in) bias_alpha_deg]); hold on;
xlabel('t [sec]');
ylabel(AhAx(1),'\alpha [deg]');
ylabel(AhAx(2),'Error \alpha [deg]');

% Sensed
AhLine1(1).Color = 'g';       
AhLine1(1).Marker = 'x'; 
AhLine1(1).LineStyle = 'none'; 
AhLine1(1).MarkerSize = 6;  

% Truth
AhLine1(2).Color = 'k';       
AhLine1(2).LineStyle = '-'; 
AhLine1(2).LineWidth = 2.;  

% Error
AhLine2(1).Color = 'r';     
AhLine2(1).Marker = '.'; 
AhLine2(1).LineStyle = 'none'; 
AhLine2(1).MarkerSize = 1; 

AhLine2(2).Color = 'b';
AhLine2(2).LineStyle = '-'; 
AhLine2(2).LineWidth = 2.;  

AhAx(1).YColor = 'k';
AhAx(2).YColor = 'r';
AhAx(1).XMinorGrid = 'on';
AhAx(1).YMinorGrid = 'on';
%AhAx(2).YMinorGrid = 'on';
legend(AhAx(1), 'Sensed', 'Truth', 'Error (sens - truth)', 'Bias');
grid on;    

%Aax2 = subplot(4,2,4);
Aax2 = subplot('Position',[0.55 0.53 0.42 0.19]);
[AjAx,AjLine1,AjLine2] = plotyy(t_sec, [beta_deg_out beta_deg_in], ...
                                t_sec, [(beta_deg_out - beta_deg_in) bias_beta_deg]); hold on;
xlabel('t [sec]');
ylabel(AjAx(1),'\beta [deg]');
ylabel(AjAx(2),'Error \beta [deg]');

AjLine1(1).Color = 'g';       
AjLine1(1).Marker = 'x'; 
AjLine1(1).LineStyle = 'none'; 
AjLine1(1).MarkerSize = 6;  

AjLine1(2).Color = 'k';       
AjLine1(2).LineStyle = '-'; 
AjLine1(2).LineWidth = 2.;  

AjLine2(1).Color = 'r';     
AjLine2(1).Marker = '.'; 
AjLine2(1).LineStyle = 'none'; 
AjLine2(1).MarkerSize = 1; 

AjLine2(2).Color = 'b';
AjLine2(2).LineStyle = '-'; 
AjLine2(2).LineWidth = 2.;  

AjAx(1).YColor = 'k';
AjAx(2).YColor = 'r';
AjAx(1).XMinorGrid = 'on';
AjAx(1).YMinorGrid = 'on';
%AjAx(2).YMinorGrid = 'on';
grid on;   


%Aax3 = subplot(4,2,6);
Aax3 = subplot('Position',[0.55 0.29 0.42 0.19]);
[AkAx,AkLine1,AkLine2] = plotyy(t_sec, [vtas_mps_out vtas_mps_in], ...
                                t_sec, [(vtas_mps_out - vtas_mps_in) bias_vtas_mps]); hold on;
xlabel('t [sec]');
ylabel(AkAx(1),'v_{TAS} [mps]');
ylabel(AkAx(2),'Error v_{TAS} [mps]');

AkLine1(1).Color = 'g';       
AkLine1(1).Marker = 'x'; 
AkLine1(1).LineStyle = 'none'; 
AkLine1(1).MarkerSize = 6;  

AkLine1(2).Color = 'k';       
AkLine1(2).LineStyle = '-'; 
AkLine1(2).LineWidth = 2.;  

AkLine2(1).Color = 'r';     
AkLine2(1).Marker = '.'; 
AkLine2(1).LineStyle = 'none'; 
AkLine2(1).MarkerSize = 1; 

AkLine2(2).Color = 'b';
AkLine2(2).LineStyle = '-'; 
AkLine2(2).LineWidth = 2.;  

AkAx(1).YColor = 'k';
AkAx(2).YColor = 'r';
AkAx(1).XMinorGrid = 'on';
AkAx(1).YMinorGrid = 'on';
%AkAx(2).YMinorGrid = 'on';
grid on;   


Aax4 = subplot(4,2,8);
Aax4 = subplot('Position',[0.55 0.05 0.42 0.19]);
[AlAx,AlLine1,AlLine2] = plotyy(t_sec, [p_pa_out p_pa_in], ...
                                t_sec, [(p_pa_out - p_pa_in) bias_osp_pa]); hold on;
xlabel('t [sec]');
ylabel(AlAx(1),'p [pa]');
ylabel(AlAx(2),'Error p [pa]');

AlLine1(1).Color = 'g';       
AlLine1(1).Marker = 'x'; 
AlLine1(1).LineStyle = 'none'; 
AlLine1(1).MarkerSize = 6;  

AlLine1(2).Color = 'k';       
AlLine1(2).LineStyle = '-'; 
AlLine1(2).LineWidth = 2.;  

AlLine2(1).Color = 'r';     
AlLine2(1).Marker = '.'; 
AlLine2(1).LineStyle = 'none'; 
AlLine2(1).MarkerSize = 1; 

AlLine2(2).Color = 'b';
AlLine2(2).LineStyle = '-'; 
AlLine2(2).LineWidth = 2.;  

AlAx(1).YColor = 'k';
AlAx(2).YColor = 'r';
AlAx(1).XMinorGrid = 'on';
AlAx(1).YMinorGrid = 'on';
%AlAx(2).YMinorGrid = 'on';
grid on;   

%savefig([plot_name '.fig']);

%Image = getframe(gcf);
%imwrite(Image.cdata, [plot_name '.jpg']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    