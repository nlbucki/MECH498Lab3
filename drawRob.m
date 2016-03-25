function [ handles ] = drawRob( joint_angles, rob )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    Plot a graphical representation of Rob with attached coordinate 
%    frames.
%
%    rob is a structure generated by robInit()
%
%    handles is a vector of graphics handles corresponding to the moving
%    frames attached to the robot
%
      

% Create structure of Rob forward kinematics transforms
[~,rob_T] = robFK(joint_angles,rob);

% Shorten variable names
l_1 = rob.parameters.l1; % [mm]
l_2 = rob.parameters.l2; % [mm]
l_3 = rob.parameters.l3; % [mm]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot scaling properties
origin_size = 20;
marker_size = 10;
vector_size = 0.05*max(abs(diff(reshape(rob.workspace,2,3))));

% Create figure window
figure('Color','w');

% Create axes object
ax = axes('XLim',rob.workspace(1:2),'YLim',rob.workspace(3:4),...
   'ZLim',rob.workspace(5:6));
vw = [31.3,22.8];
set(gca,'View',vw);
grid on;
axis equal;
xlabel('X (mm)','FontSize',16);
ylabel('Y (mm)','FontSize',16);
zlabel('Z (mm)','FontSize',16);

% Create frames and links

% Draw object in starting and final position
Obj(1) = hggroup('Parent',ax);
O = line(783.2878,-783.2878,224.3548,'Color',[0,0,0],'Marker','.','MarkerSize',50);
set(O,'Parent',Obj(1));
Obj(3) = hggroup('Parent',ax);
O = line(923.6,923.6,1350,'Color',[0,0,0],'Marker','.','MarkerSize',50);
set(O,'Parent',Obj(3));
set(Obj(3),'Visible','off');

% Base frame
h = drawRobotFrame([0,0,0]);
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
circ = linspace(0,2*pi,50);
L_0 = line(100*cos(circ),100*sin(circ),...
     0*ones(length(circ)),...
    'Color','k','LineWidth',1.5);
set(L_0,'Parent',hg);
T_0 = hgtransform('Parent',ax,'Matrix',makehgtform('translate',[0,0,0]));
set(hg,'Parent',T_0);

% Outline Worskpace
ws = rob.workspace;
line([ws(1:2),fliplr(ws(1:2)),ws(1)],[ws(3)*[1,1],ws(4)*[1,1],ws(3)],ws(5)*ones(1,5),'Color',[0,0,0]);
line([ws(1:2),fliplr(ws(1:2)),ws(1)],[ws(3)*[1,1],ws(4)*[1,1],ws(3)],ws(6)*ones(1,5),'Color',[0,0,0]);

% Robot FK frames
h = drawRobotFrame(rob.colors{1});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_1 = line([0,0],[0,0],[-l_1,0],...
    'Color',rob.colors{1},'LineWidth',1.5);
set(L_1,'Parent',hg);
T_1 = hgtransform('Parent',T_0,'Matrix',rob_T{1});
set(hg,'Parent',T_1);

h = drawRobotFrame(rob.colors{2});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_2 = line([0,l_2],[0,0],[0,0],...
    'Color',rob.colors{2},'LineWidth',1.5);
set(L_2,'Parent',hg);
T_2 = hgtransform('Parent',T_1,'Matrix',rob_T{2});
set(hg,'Parent',T_2);

h = drawRobotFrame(rob.colors{3});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_3 = line([0,0],[0,-l_3],[0,0],...
    'Color',rob.colors{3},'LineWidth',1.5);
set(L_3,'Parent',hg);
T_3 = hgtransform('Parent',T_2,'Matrix',rob_T{3});
set(hg,'Parent',T_3);

h = drawRobotFrame(rob.colors{4});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
Obj(2) = hggroup('Parent',ax);
O = line(0,0,0,'Color',[0,0,0],'Marker','.','MarkerSize',50);
set(O,'Parent',Obj(2));
set(Obj(2),'Visible','off');
T_4 = hgtransform('Parent',T_3,'Matrix',rob_T{4});
set(hg,'Parent',T_4);
set(Obj(2),'Parent',T_4);

set(gcf,'Renderer','openGL');
drawnow;

% Return hgtransform handles
handles = [T_1,T_2,T_3,T_4,Obj];

    function h = drawRobotFrame( color )
         
        % Plot reference frame
        X_b = [vector_size,0,0,1]';
        Y_b = [0,vector_size,0,1]';
        Z_b = [0,0,vector_size,1]';
        h(1) = line(0,0,0,'Marker','.','MarkerSize',origin_size,'Color',color);
        h(2) = line([0,X_b(1)],[0,X_b(2)],[0,X_b(3)],'LineWidth',1.5,'Color',color);
        h(3) = line([0,Y_b(1)],[0,Y_b(2)],[0,Y_b(3)],'LineWidth',1.5,'Color',color);
        h(4) = line([0,Z_b(1)],[0,Z_b(2)],[0,Z_b(3)],'LineWidth',1.5,'Color',color);
        h(5) = line(X_b(1),X_b(2),X_b(3),'LineWidth',1.5,'Marker','x','MarkerSize',marker_size,'Color',color);
        h(6) = line(Y_b(1),Y_b(2),Y_b(3),'LineWidth',1.5,'Marker','o','MarkerSize',marker_size,'Color',color);
        h(7) = line(Z_b(1),Z_b(2),Z_b(3),'LineWidth',1.5,'Marker','d','MarkerSize',marker_size,'Color',color);
    end


end

