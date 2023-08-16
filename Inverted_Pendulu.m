 
function Inverted_Pendulum_2021_06_09
close all
clear
clc
global t_end POLES
global u_max u_min
global CART_V0 WHEEL_1_V0 WHEEL_2_V0 WHEEL_3_V0 WHEEL_4_V0
global SHAFT_V0 PENDULUM_V0
global CART_V WHEEL_1_V WHEEL_2_V WHEEL_3_V WHEEL_4_V
global SHAFT_V PENDULUM_V
global CART WHEEL_1 WHEEL_2 WHEEL_3 WHEEL_4
global SHAFT PENDULUM PENDULUM0
global point_x1 point_x3 point_u
global Rx Rz
global factor
global ploting
global end_program
global m M g L u mL ML
global line_x1 line_x3 line_u
global replot N_lines
N_lines  = 100;
line_x1  = cell(N_lines,1);
line_x3  = cell(N_lines,1);
line_u   = cell(N_lines,1);
point_x1 = cell(N_lines,1);
point_x3 = cell(N_lines,1);
point_u  = cell(N_lines,1);
factor   = 1;
%--- Schematic diagram ----------------------------------------------------
%                         
%                         │
%                         │     /
%                         │    / 
%                         │   / L,m
%                         │θ /  
%                         │ /
%                         │/
%                     ┌────┐
%               u --->│   M   │---> x
%                     └⊙──⊙┘
%   ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔                       
%
%--- Parameters of inverted pendulum and cart system ----------------------
m = 0.1;         % (kg) Pendulum mass
M = 0.5;         % (kg) Cart mass
L = 0.3;         % (m) Pendulum length
g = 9.8;         % (m/s^2) Gravitational acceleration
u_max =  20;     % (N) ┐Saturation driving control force of the cart
u_min = -u_max;  % (N) ┘
%--- Linearization matrix of inverted pendulum system ---------------------
mL = m*L;
ML = M*L;
A = [0,         1, 0, 0;
    (M+m)*g/ML, 0, 0, 0;
    0,          0, 0, 1;
    -m*g/M,     0, 0, 0];
b = [0, -1/ML, 0, 1/M]';
%--------------------------------------------------------------------------
% Use place to compute a gain matrix K such that the state feedback
% u = -kx places the closed-loop poles at the locations POLES.
v_eps = -[ 0, 1e-2, 2e-2, 3e-2 ];
switch 6
    case 1
        POLES = -[ 25.0, 25.1, 3.0, 3.1 ];
    case 2
        POLES = -[ 20.0, 20.1, 3.0, 3.1 ];
    case 3
        POLES = -[ 10.0, 10.1, 4.0, 4.1 ];
    case 4
        POLES = -4 +v_eps;
    case 5
        POLES = -5 +v_eps;
    case 6
        POLES = -6 +v_eps;
    case 7
        POLES = -7 +v_eps;
    case 8
        POLES = -8 +v_eps;
    case 9
        POLES = -9 +v_eps;
    case 10
        POLES = -5 +[0,-1,-2,-3];
    otherwise
        POLES = -[ 25.0, 25.1, 3.0, 3.1 ];
end
k = place( A, b, POLES );
%--------------------------------------------------------------------------
dt = 0.005;            % (s) Sampling time
t_end = 2;             % (s) Single simulation end time
t  = (0:dt:t_end)';    % Vector of sampling time
N_loops  = length(t);  % Number of loops for a single simulation
x1 = zeros(N_loops,1); % (rad) The angle between the inverted pendulum and the plumb line
x2 = zeros(N_loops,1); % (rad/s) Pendulum angular velocity
x3 = zeros(N_loops,1); % (m) Cart displacement
x4 = zeros(N_loops,1); % (m/s) Cart velovity
uu = zeros(N_loops,1); % (N) 台車所受之水平等效控制力 (藉由車上馬達驅動輪子輸出)
%--------------------------------------------------------------------------
x0 = [ deg2rad( 35 )*(2*rand-1),  0,  0.6*(2*rand-1),  0 ]';
%--------------------------------------------------------------------------
Creat_Graphic_Objects

%--- Wait for the user to press the 'Run' button --------------------------
ploting = 0;
end_program = 0;
replot = 1;
while ~ploting && ~end_program
    pause(0)
end
%--------------------------------------------------------------------------
i = 2;
while (i<=N_loops) && (end_program==0)
    if ploting==1 && replot<=N_lines
        tspan = t(i-1:i);
        x1(i) = x0(1);
        x2(i) = x0(2);
        x3(i) = x0(3);
        x4(i) = x0(4);
        u = -k*x0;
        %--- Controller saturation ----------------------------------------
        if u>u_max     
            u = u_max;
        elseif u<u_min
            u = u_min;
        end
        %------------------------------------------------------------------
        uu(i) = u;
        
        [T,X]= ode45(@plant,tspan,x0);
        x0 = X(end,:)';
        
        addpoints( line_x1{replot}, t(i)-dt, rad2deg(x1(i)) )
        addpoints( line_x3{replot}, t(i)-dt, 1000*x3(i) )
        addpoints( line_u{replot}, t(i)-dt, uu(i) )
        % clearpoints( point_x1{replot} )
        % clearpoints( point_x3{replot} )
        % clearpoints( point_u{replot} )
        % addpoints( point_x1{replot}, t(i)-dt, rad2deg(x1(i)) )
        % addpoints( point_x3{replot}, t(i)-dt, 1000*x3(i) )
        % addpoints( point_u{replot}, t(i)-dt, uu(i) )
        
        CART_V(:,2) = CART_V0(:,2) +1000*x3(i);        % Update the y coordinates of all vertices of CART
        set(CART,'Vertices',CART_V);                   % Update the (x,y,z) coordinates of all vertices of CART
        WHEEL_1_V(:,2) = WHEEL_1_V0(:,2) +1000*x3(i);  % Update the y coordinates of all vertices of WHEEL_1
        set(WHEEL_1,'Vertices',WHEEL_1_V);             % Update the (x,y,z) coordinates of all vertices of WHEEL_1
        WHEEL_2_V(:,2) = WHEEL_2_V0(:,2) +1000*x3(i);  % Update the y coordinates of all vertices of WHEEL_2
        set(WHEEL_2,'Vertices',WHEEL_2_V);             % Update the (x,y,z) coordinates of all vertices of WHEEL_2
        WHEEL_3_V(:,2) = WHEEL_3_V0(:,2) +1000*x3(i);  % Update the y coordinates of all vertices of WHEEL_3
        set(WHEEL_3,'Vertices',WHEEL_3_V);             % Update the (x,y,z) coordinates of all vertices of WHEEL_3
        WHEEL_4_V(:,2) = WHEEL_4_V0(:,2) +1000*x3(i);  % Update the y coordinates of all vertices of WHEEL_4
        set(WHEEL_4,'Vertices',WHEEL_4_V);             % Update the (x,y,z) coordinates of all vertices of WHEEL_4
        SHAFT_V(:,2) = SHAFT_V0(:,2) +1000*x3(i);      % Update the y coordinates of all vertices of SHAFT 
        set(SHAFT,'Vertices',SHAFT_V);                 % Update the (x,y,z) coordinates of all vertices of SHAFT
        rotate( PENDULUM0, [1,0,0], rad2deg(-x1(i)), [Rx,x3(i),Rz]);
        PENDULUM_V = get(PENDULUM0,'Vertices');
        set( PENDULUM0,'Vertices',PENDULUM_V0);
        PENDULUM_V(:,2) = PENDULUM_V(:,2) +1000*x3(i); % Update the y coordinates of all vertices of PENDULUM0
        set(PENDULUM,'Vertices',PENDULUM_V);           % Update the (x,y,z) coordinates of all vertices of PENDULUM0
        i = i+1;
    end  % END: if ploting==1
    %----------------------------------------------------------------------
    if i>N_loops
        clearpoints( point_x1{replot} )
        clearpoints( point_x3{replot} )
        clearpoints( point_u{replot} )
        x0 = [ deg2rad( 30 )*(2*rand-1),  0,  0.6*(2*rand-1),  0 ]';
        i = 2;
        replot = replot +1;
        if replot<=N_lines
            set(findobj('Tag','pb_Run'),'string','Pause')
        else
            set( findobj('Tag','pb_Run'), 'Enable', 'off' )
        end
    end  % END: if i>N_loops
    drawnow limitrate
end % END: while (i<=N_loops) && (end_program==0)
%--------------------------------------------------------------------------
while ~end_program
    pause(0)
end
close( findobj('Tag','fig1') )
close( findobj('Tag','fig2') )
drawnow
%--------------------------------------------------------------------------
end  % END: MAIN

%% === Dynamic equations of the inverted pendulum systems =================
function xp=plant(t,x)
global m M g mL ML u
xp = zeros(4,1);
sin_x1 = sin(x(1));
cos_x1 = cos(x(1));
xp(1) = x(2);
xp(2) = ( u*cos_x1-(M+m)*g*sin_x1+mL*(cos_x1*sin_x1)*x(2)^2 )...
    /( mL*cos_x1^2-(ML+mL) );
xp(3) = x(4);
xp(4) = ( u+mL*sin_x1*x(2)^2-m*g*cos_x1*sin_x1 )...
    /( M+m-m*cos_x1^2 );
end  % END: function xp=plant(t,x)

%% === Creat graphic objects ==============================================
function Creat_Graphic_Objects
global t_end POLES
global u_max u_min
global CART_V0 WHEEL_1_V0 WHEEL_2_V0 WHEEL_3_V0 WHEEL_4_V0 SHAFT_V0 PENDULUM_V0
global CART_V WHEEL_1_V WHEEL_2_V WHEEL_3_V WHEEL_4_V SHAFT_V
global CART WHEEL_1 WHEEL_2 WHEEL_3 WHEEL_4 SHAFT PENDULUM PENDULUM0 BASE
global Rx Rz
global line_color
global line_x1 line_x3 line_u
global point_x1 point_x3 point_u
global N_lines
  
pb_End = uicontrol(fig1,...
    'Style','pushbutton',...
    'Units','Normalized',...
    'Position',[0.82 0.92 0.15 0.05],...
    'FontSize',12,...
    'String','End',...
    'CallBack',...
    ['global end_program,',...
    'end_program=1;']);
pop_SetViewPoint = uicontrol(fig1,...
    'Tag','pop_SetViewPoint',...
    'Style','popupmenu',...
    'String','　Top left view |　Top view |　Front view |　Front view (Close-up) ',...
    'Units','Normalized',...
    'Position',[0.65 0.85 0.32 0.05],...
    'FontSize',12,...
    'CallBack',@CB_pop_SetViewPoint);
%--- 繪圖軸 ---------------------------------------------------------------
fig1_ax1 = axes('Parent',fig1,...
    'Tag','fig1_ax1',...
    'XLim',XLim,...
    'YLim',YLim,...
    'ZLim',ZLim,...
    'Unit','Normalized');
set(fig1_ax1,...
    'XGrid','on',...
    'YGrid','on',...
    'ZGrid','on',...
    'Box','on',...
    'Clipping','off',...
    'Visible','off');
axis equal
cameratoolbar('Show')
rotate3d on
xlabel('x')
ylabel('y')
zlabel('z')
line('Parent',gca,...
    'XData',[-46,500],...
    'YData',[0,0],...
    'ZData',[1,1],...
    'Color',[1,1,0],...
    'LineWidth',1);
line('Parent',gca,...
    'XData',-46*[1,1],...
    'YData',[0,0],...
    'ZData',[0,1200],...
    'Color',[1,1,0],...
    'LineWidth',1);
%--- 建立基座 --------------------------------------------------------------
BASE = CreateCuboid([0,-1000,-base_thickness],...
    base_width,...
    base_length,...
    base_thickness,...
    base_color);
%--- 建立台車 --------------------------------------------------------------
CART = CreateCuboid([10+wheel_thickness, -0.5*cart_length, 20],...
    cart_width,...
    cart_length,...
    cart_thickness,...
    cart_color);
CART_V0 = get(CART,'Vertices');
CART_V = CART_V0;
%--- 建立輪子 --------------------------------------------------------------
wheel_radius_v = [ 0,...
    wheel_radius,...
    wheel_radius,...
    0 ];
wheel_thickness_v = [0, wheel_thickness, 0];
WHEEL_1 = CreateCylinder(...
    [ 10, 100, wheel_radius],...
    wheel_radius_v,...
    wheel_thickness_v,...
    20,...
    wheel_color );
WHEEL_2 = CreateCylinder(...
    [ 10+cart_width+wheel_thickness, 100, wheel_radius],...
    wheel_radius_v,...
    wheel_thickness_v,...
    20,...
    wheel_color );
WHEEL_3 = CreateCylinder(...
    [ 10+cart_width+wheel_thickness,-100, wheel_radius],...
    wheel_radius_v,...
    wheel_thickness_v,...
    20,...
    wheel_color );
WHEEL_4 = CreateCylinder(...
    [ 10,-100, wheel_radius],...
    wheel_radius_v,...
    wheel_thickness_v,...
    20,...
    wheel_color );
rotate(WHEEL_1,[0,1,0], 90,[ 10, 100, wheel_radius] );
rotate(WHEEL_2,[0,1,0], 90,[10+cart_width+wheel_thickness, 100, wheel_radius] );
rotate(WHEEL_3,[0,1,0], 90,[10+cart_width+wheel_thickness,-100, wheel_radius] );
rotate(WHEEL_4,[0,1,0], 90,[ 10,-100, wheel_radius] );
WHEEL_1_V0 = get(WHEEL_1,'Vertices');
WHEEL_2_V0 = get(WHEEL_2,'Vertices');
WHEEL_3_V0 = get(WHEEL_3,'Vertices');
WHEEL_4_V0 = get(WHEEL_4,'Vertices');
WHEEL_1_V = WHEEL_1_V0;
WHEEL_2_V = WHEEL_2_V0;
WHEEL_3_V = WHEEL_3_V0;
WHEEL_4_V = WHEEL_4_V0;
%--- 建立轉軸 --------------------------------------------------------------
shaft_radius_v = [0,...
    shaft_radius,...
    shaft_radius,...
    0];
shaft_high_v = [   0, shaft_high, 0];
SHAFT = CreateCylinder([-100,0,100],shaft_radius_v,shaft_high_v,32,shaft_color);
rotate( SHAFT,[0,1,0], 90, [-100,0,100] );
SHAFT_V0 = get(SHAFT,'Vertices');
SHAFT_V = SHAFT_V0;
%--- 建立單擺 --------------------------------------------------------------
PENDULUM = CreateCuboid(...
    [-5-pendulum_width, -0.5*pendulum_length, 50],...
    pendulum_width,...
    pendulum_length,...
    pendulum_thickness,...
    pendulum_color,'on',pendulum_color*0.6);
PENDULUM0 = CreateCuboid(...
    [-5-pendulum_width, -0.5*pendulum_length, 50],...
    pendulum_width,...
    pendulum_length,...
    pendulum_thickness,...
    pendulum_color);
set(PENDULUM0,'visible','off');
PENDULUM_V0 = get(PENDULUM,'Vertices');
Rx = -5-0.5*pendulum_width;
Rz = 100;
%--- MATLAB 3D 效果設定 ----------------------------------------------------
SetViewPoint(1);     % 呼叫設定視點函式
%--- 設定光源位置及3d渲染模式   ★★★ 所有物件接建立後方可設定 ★★★
material metal
camlight right
lighting gouraud
%--- Creat fig2 -----------------------------------------------------------
fig2 = figure(2);
set(fig2,...
    'Tag','fig2',...
    'ToolBar','figure',...
    'NumberTitle','off',...
    'Name','Time Responses',...
    'DockControls','off',...
    'ToolBar','none',...
    'MenuBar','none',...
    'Color',[0.9 0.9 1],...
    'Unit','normalized',...
    'OuterPosition',[0.492, 0.022, 0.52, 0.98]);
%--- ax_x1 ----------------------------------------------------------------
ax_x1 = subplot(311);
set( ax_x1,...
    'Tag','ax_x1',...
    'Box','on',...
    'XGrid','on',...
    'YGrid','on',...
    'FontSize',12,...
    'FontName','Times New Roman',...
    'YLim',[-inf,inf],...
    'XLim',[0,t_end]);
ylabel('$ \theta \rm \; (degree) $',...
    'FontSize',12,...
    'Interpreter','latex');
poles_str = [num2str(POLES(1),'%.3f'),', ',...
    num2str(POLES(2),'%.3f'),', ',...
    num2str(POLES(3),'%.3f'),', ',...
    num2str(POLES(4),'%.3f')];
title( {'$ \textrm{ Closed-loop poles of the linearized system: }$';
    ['$\qquad\qquad',poles_str, '$']},...
    'FontSize',12,...
    'HorizontalAlignment','center',...
    'Interpreter','latex');
%--- ax_x3 ----------------------------------------------------------------
ax_x3 = subplot(312);
ylabel('$ x \rm \; (mm) $',...
    'FontSize',12,...
    'Interpreter','latex');
set( ax_x3,...
    'Tag','ax_x3',...
    'Box','on',...
    'XGrid','on',...
    'YGrid','on',...
    'FontSize',12,...
    'FontName','Times New Roman',...
    'YLim',[-inf,inf],...
    'XLim',[0,t_end]);
%--- ax_u ----------------------------------------------------------------
ax_u  = subplot(313);
du = 0.1*(u_max-u_min);
set( ax_u,...
    'Tag','ax_u',...
    'Box','on',...
    'XGrid','on',...
    'YGrid','on',...
    'FontSize',12,...
    'FontName','Times New Roman',...
    'YLim',[u_min-du,u_max+du],...
    'XLim',[0,t_end]);
ylabel('$ u \rm \; (N) $',...
    'FontSize',12,...
    'Interpreter','latex');
xlabel('Time (s)',...
    'FontSize',12,...
    'Interpreter','latex');
patch(gca,...
    'FaceColor','g',...
    'FaceAlpha',0.05,...
    'EdgeColor','g',...
    'EdgeAlpha',0.5,...
    'XData',[t_end,0,0,t_end],...
    'YData',[u_max,u_max,u_min,u_min]);

text( t_end,u_max,'$ \; u_{max} $',...
    'Interpreter','latex',...
    'HorizontalAlignment','left',...
    'VerticalAlignment','middle',...
    'FontSize',14)
text( t_end,u_min,'$ \; u_{min} $',...
    'Interpreter','latex',...
    'HorizontalAlignment','left',...
    'VerticalAlignment','middle',...
    'FontSize',14)
%--------------------------------------------------------------------------
for i=1:N_lines
    line_color = rand(1,3);
    if norm( line_color )>1.7
        line_color = 0.5*line_color;
    end
    line_x1{i} = animatedline('Parent',ax_x1,...
        'Color',line_color,...
        'LineWidth',1);
    line_x3{i} = animatedline('Parent',ax_x3,...
        'Color',line_color,...
        'LineWidth',1);
    line_u{i} = animatedline('Parent',ax_u,...
        'Color',line_color,...
        'LineWidth',1);
    point_x1{i} = animatedline('Parent',ax_x1,...
        'LineStyle','none',...
        'Marker','.',...
        'MarkerEdgeColor',line_color,...
        'MarkerFaceColor',line_color,...
        'MarkerSize',14 );
    point_x3{i} = animatedline('Parent',ax_x3,...
        'LineStyle','none',...
        'Marker','.',...
        'MarkerEdgeColor',line_color,...
        'MarkerFaceColor',line_color,...
        'MarkerSize',14 );
    point_u{i} = animatedline('Parent',ax_u,...
        'LineStyle','none',...
        'Marker','.',...
        'MarkerEdgeColor',line_color,...
        'MarkerFaceColor',line_color,...
        'MarkerSize',14 );
end
drawnow limitrate
end  % END: function Creat_Graphic_Objects

%% === CB_pb_Run ==========================================================
function CB_pb_Run(src,event)
global ploting
global end_point
if ploting==1
    ploting=0;
    set(findobj('Tag','pb_Run'),'string','Continue')
else
    ploting=1;
    set(findobj('Tag','pb_Run'),'string','Pause')
    if ~isempty( end_point )
        clearpoints( end_point )
    end
end
end  % END: function CB_pb_Run(src,event)

%% === CB_pop_SetViewPoint ================================================
function CB_pop_SetViewPoint(src,event)
view_point = get( findobj('Tag','pop_SetViewPoint'),'Value');
SetViewPoint( view_point )
drawnow limitrate
end  % END: CB_pop_SetViewPoint(src,event)

%% === SetViewPoint(view_point) ===========================================
function SetViewPoint(view_point)
global factor
switch view_point
    case 1  % Top left view
        view(-50,30)
        set(findobj('Tag','fig1_ax1'),...
            'OuterPosition',[-0.1, -0.1, 1, 1]);
        zoom(1/factor)
        factor = 2;
        zoom(factor)
    case 2  % Top view
        view(0,90)
        set(findobj('Tag','fig1_ax1'),...
            'OuterPosition',[-0.18, -0.025, 1, 1]);
        zoom(1/factor)
        factor = 1;
        zoom(factor)
    case 3  % Front view
        view(-90,0)
        set(findobj('Tag','fig1_ax1'),...
            'OuterPosition',[-0.02, -0.2, 1, 1]);
        zoom(1/factor)
        factor = 1.25;
        zoom(factor)
    case 4  
        view(-90,0)
        set(findobj('Tag','fig1_ax1'),...
            'OuterPosition',[-0.06, -0.3, 1, 1]);
        zoom(1/factor)
        factor = 2;
        zoom(factor)
end   
end   
 function patch_obj=CreateCuboid(...
    Oxyz,Lx,Ly,Lz,FaceColor,visible,EdgeColor)
 if nargin<7  
    EdgeColor = 'none';
    if nargin<6   
        visible = 'on';
        if nargin<5   
            FaceColor = ones(1,3);   
        end
    end
end
x0 = Oxyz(1);                        
y0 = Oxyz(2);                       
z0 = Oxyz(3);                       
X  = [0, 0,Lx,Lx, 0, 0,Lx,Lx]'+x0;  
Y  = [0,Ly,Ly, 0, 0,Ly,Ly,0 ]'+y0;  
Z  = [0, 0, 0, 0,Lz,Lz,Lz,Lz]'+z0;  
F  = [...
    1 2 3 4;
    5 1 4 8;
    5 6 2 1;
    2 6 7 3;
    4 3 7 8;
    5 6 7 8];                       
patch_obj = patch('Faces',F,'Vertices',[X,Y,Z],...
    'FaceColor',FaceColor,'EdgeColor',EdgeColor,'Visible',visible );
end  % END: function patch_obj=CreateCuboid(...

function patch_obj=CreateCylinder(...
    Oxyz,radius,high,N_sides,FaceColor,visible,EdgeColor)
if nargin<7                        
    EdgeColor = 'none';
    if nargin<6                      
        visible = 'on';
        if nargin<5                 
            FaceColor = ones(1,3);   
            if nargin<4            
                N_sides = 32;
            end
        end
    end
end
Nh = length( high );               
N  = Nh +1;                         
x0 = Oxyz(1);                     
y0 = Oxyz(2);                        
z0 = Oxyz(3);                     
dth = 2*pi/N_sides;          
THETA = (0:dth:(N_sides-1)*dth)';  
Nxm = N*N_sides;
X = zeros(Nxm,1);                   
Y = zeros(Nxm,1);                  
Z = zeros(Nxm,1);                   
F = zeros(Nxm-N_sides,4);  
X(1:N_sides) = radius(1)*cos(THETA) +x0; 
Y(1:N_sides) = radius(1)*sin(THETA) +y0;  
Z(1:N_sides) = z0*ones(N_sides,1);
for i=2:N
    ixm = i*N_sides;
    X(ixm-N_sides+1:ixm) = radius(i)*cos(THETA) +x0; 
    Y(ixm-N_sides+1:ixm) = radius(i)*sin(THETA) +y0;  
    Z(ixm-N_sides+1:ixm) = ( z0 +sum(high(1:i-1)) )*ones(N_sides,1); 
    F(ixm-N_sides-N_sides+1:ixm-N_sides,:) = [ (1:N_sides-1)', (2:N_sides)', (N_sides+2:N_sides+N_sides)', (N_sides+1:N_sides+N_sides-1)';...
        N_sides, 1, N_sides+1, N_sides+N_sides] +ixm-N_sides-N_sides;  
end
patch_obj = patch('Faces',F,'Vertices',[X,Y,Z],...
    'FaceColor',FaceColor,'EdgeColor',EdgeColor,'Visible',visible );
end   