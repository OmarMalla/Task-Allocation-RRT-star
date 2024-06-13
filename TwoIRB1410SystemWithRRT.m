%% Task Allocation for a Multi-Robot system with two IRB1410 robots using RRT*: 
% Size of teh Workspace:
x_max = 1200;
y_max = 1200;
z_max = 1200;

%% Robots' Specs (Dimensions are in mm):
L1 = 475; % length of link 1 : d1
a1 = 150;
L2 = 600;
L3 = 120;
L4 = 720; % length of link 2
L5 = 85; % length of link 3

fprintf('IRB 1410: \n\n');
% Joints' limits:
qmin =  [-170 , -70, -140, 0, -150, -115, 0, -300];
qmax =  [+170 , +70, +140, 0, +150, +115, 0, +300];

% Joint Limits Radians:
theta1_min = deg2rad(qmin(1)); % minimum joint angle 1
theta1_max = deg2rad(qmax(1)); % maximum joint angle 1
theta2_min = deg2rad(qmin(2)); % minimum joint angle 2
theta2_max = deg2rad(qmax(2)); % maximum joint angle 2
theta3_min = deg2rad(qmin(3)); % minimum joint angle 3
theta3_max = deg2rad(qmax(3)); % maximum joint angle 3
theta4_min = deg2rad(qmin(4)); % This joint is extra % No need to control
theta4_max = deg2rad(qmax(4)); % This joint is extra % No need to control
theta5_min = deg2rad(qmin(5)); % minimum joint angle 4
theta5_max = deg2rad(qmax(5)); % maximum joint angle 4
theta6_min = deg2rad(qmin(6)); % minimum joint angle 5
theta6_max = deg2rad(qmax(6)); % maximum joint angle 5
theta7_min = deg2rad(qmin(7)); % This joint is extra % No need to control
theta7_max = deg2rad(qmax(7)); % This joint is extra % No need to control
theta8_min = deg2rad(qmin(8)); % minimum joint angle 6
theta8_max = deg2rad(qmax(8)); % maximum joint angle 6

% Defining robot base relative to workspace origin point "Camera":
Tbase1 = [1 0 0 -1250; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Tbase2 = trotz(deg2rad(180))*Tbase1;
% fprintf('Robot 1 base transformation from workspace base point: \n');
% disp(Tbase1);

% Discretizing the joint rotations:
SampleRate = 0.1;

n1 = ((qmax(1)-qmin(1))/SampleRate)+1; % 180/SampelRate = 1800 points
n2 = ((qmax(2)-qmin(2))/SampleRate)+1;
n3 = ((qmax(3)-qmin(3))/SampleRate)+1;
n4 = ((qmax(5)-qmin(5))/SampleRate)+1; % 180/SampelRate = 1800 points
n5 = ((qmax(6)-qmin(6))/SampleRate)+1;
n6 = ((qmax(8)-qmin(8))/SampleRate)+1;

q1 = linspace(theta1_min,theta1_max,n1);
q2 = linspace(theta2_min,theta2_max,n2);
q3 = linspace(theta3_min,theta3_max,n3);
q4 = linspace(theta5_min,theta5_max,n1);
q5 = linspace(theta6_min,theta6_max,n2);
q6 = linspace(theta7_min,theta7_max,n3);

% DH Method:
% Link([0,d,a,theta])
%%%%%%%%%%%%%%%%%%%%% q4 not to be modified %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% q7 not to be modified %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

IRB1410M_1(1)= Link([0 , L1  , a1 , -pi/2]);
IRB1410M_1(2)= Link([0 ,  0  , L2 , 0     ]);
IRB1410M_1(3)= Link([0 ,  0  , L3 , 0     ]);
IRB1410M_1(4)= Link([0 ,  0  , 0  , -pi/2 ]);
IRB1410M_1(5)= Link([0 ,  L4 , 0  , pi/2  ]);
IRB1410M_1(6)= Link([0 ,  0  , L5 , 0     ]);
IRB1410M_1(7)= Link([0 ,  0 , 0   , -pi/2 ]);
IRB1410M_1(8)= Link([0 ,  0 , 0  , 0  ]);

IRB1410M_2(1)= Link([0 , L1  , a1 , -pi/2]);
IRB1410M_2(2)= Link([0 ,  0  , L2 , 0     ]);
IRB1410M_2(3)= Link([0 ,  0  , L3 , 0     ]);
IRB1410M_2(4)= Link([0 ,  0  , 0  , -pi/2 ]);
IRB1410M_2(5)= Link([0 ,  L4 , 0  , pi/2  ]);
IRB1410M_2(6)= Link([0 ,  0  , L5 , 0     ]);
IRB1410M_2(7)= Link([0 ,  0 , 0   , -pi/2 ]);
IRB1410M_2(8)= Link([0 ,  0 , 0  , 0  ]);

%IRB1410M(1).offset = pi;
IRB1410M_1(1).qlim = [theta1_min,theta1_max];
IRB1410M_1(2).offset = -pi/2;
IRB1410M_1(2).qlim = [theta2_min,theta2_max];
%IRB1410M(3).offset = pi/2 - IRB1410M(2).theta;
IRB1410M_1(4).offset = 0;
IRB1410M_1(3).qlim = [theta3_min,theta3_max];
IRB1410M_1(4).qlim = [theta4_min,theta4_max];
IRB1410M_1(5).qlim = [theta5_min,theta5_max];
IRB1410M_1(5).offset = 0;
IRB1410M_1(6).qlim = [theta6_min,theta6_max];
IRB1410M_1(6).offset = pi/2;
IRB1410M_1(7).offset = -pi/2;
IRB1410M_1(7).qlim = [theta7_min,theta7_max];
IRB1410M_1(8).offset = pi;

Rob1 = SerialLink(IRB1410M_1,'name','R1','base',Tbase1);

%IRB1410M(1).offset = pi;
IRB1410M_2(1).qlim = [theta1_min,theta1_max];
IRB1410M_2(2).offset = -pi/2;
IRB1410M_2(2).qlim = [theta2_min,theta2_max];
%IRB1410M(3).offset = pi/2 - IRB1410M(2).theta;
IRB1410M_2(4).offset = 0;
IRB1410M_2(3).qlim = [theta3_min,theta3_max];
IRB1410M_2(4).qlim = [theta4_min,theta4_max];
IRB1410M_2(5).qlim = [theta5_min,theta5_max];
IRB1410M_2(5).offset = 0;
IRB1410M_2(6).qlim = [theta6_min,theta6_max];
IRB1410M_2(6).offset = pi/2;
IRB1410M_2(7).offset = -pi/2;
IRB1410M_2(7).qlim = [theta7_min,theta7_max];
IRB1410M_2(8).offset = pi;

Rob2 = SerialLink(IRB1410M_2,'name','R2','base',Tbase2);

Q0 = [0,0,0,0,0,0,0,0];
Q01 = [0,deg2rad(-12.5),deg2rad(12.5),0,0,0,0,0];
Q1 = [deg2rad(25),deg2rad(12.5),deg2rad(-12.5),0,deg2rad(0),deg2rad(0),0,deg2rad(0)];
Q2 = [deg2rad(25),deg2rad(12.5),deg2rad(-12.5),0,deg2rad(0),deg2rad(0),0,deg2rad(0)];

figure;
set(gca,'color',[0.5,0.6,0.7]);
% Rob1.plot(Q0,'view',[40 20],'wrist','jaxes','arrow','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,0.5,0.1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);
% hold on
% Rob2.plot(Q0,'view',[40 20],'wrist','jaxes','arrow','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,0.5,0.1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);

Rob1.plot(Q01,'view',[40 20],'wrist','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,0.5,0.1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);
hold on
Rob2.plot(Q01,'view',[40 20],'wrist','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,0.5,0.1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);

%% Obstacles: 
obstacles = []; % Initialize obstacle array

% Generate random obstacles in studied space
% Setting the seed value to reproduce similar obstacles if needed
seed_value = 17; 
rng(seed_value);
rnc = [randi([100, 900]),randi([100, 900]),randi([100, 900])]; % random_number_coord 
o_dim = [randi([100, 300]),randi([100, 300]),randi([100, 300])];
obstacle1.coord = rnc;                  % Example coordinates
obstacle1.width = o_dim(1);             % Width of the obstacle
obstacle1.length = o_dim(2);            % Length of the obstacle
obstacle1.height = o_dim(3);            % Height of the obstacle

seed_value = 13; 
rng(seed_value);
rnc = [randi([-500, 900]),randi([100, 900]),randi([100, 900])]; % random_number_coord 
o_dim = [randi([100, 300]),randi([100, 300]),randi([100, 300])];
obstacle2.coord = rnc;                  % Example coordinates
obstacle2.width = o_dim(1);             % Width of the obstacle
obstacle2.length = o_dim(2);            % Length of the obstacle
obstacle2.height = o_dim(3);            % Height of the obstacle

obstacle22.coord = [-rnc(1),rnc(2),rnc(3)];                  % Example coordinates
obstacle22.width = o_dim(1);             % Width of the obstacle
obstacle22.length = o_dim(2);            % Length of the obstacle
obstacle22.height = o_dim(3);            % Height of the obstacle

seed_value = 19; 
rng(seed_value);
rnc = [randi([-500, 900]),randi([100, 900]),randi([100, 900])]; % random_number_coord 
o_dim = [randi([100, 300]),randi([100, 300]),randi([100, 300])];
obstacle3.coord = rnc;                  % Example coordinates
obstacle3.width = o_dim(1);             % Width of the obstacle
obstacle3.length = o_dim(2);            % Length of the obstacle
obstacle3.height = o_dim(3);            % Height of the obstacle

seed_value = 29; 
rng(seed_value);
rnc = [randi([100, 900]),randi([100, 900]),randi([100, 900])]; % random_number_coord 
o_dim = [randi([100, 300]),randi([100, 300]),randi([100, 300])];
obstacle4.coord = rnc;                  % Example coordinates
obstacle4.width = o_dim(1);             % Width of the obstacle
obstacle4.length = o_dim(2);            % Length of the obstacle
obstacle4.height = o_dim(3);            % Height of the obstacle

seed_value = 31; 
rng(seed_value);
rnc = [randi([100, 900]),randi([100, 900]),randi([100, 900])]; % random_number_coord 
o_dim = [randi([100, 300]),randi([100, 300]),randi([100, 300])];
obstacle5.coord = rnc;                  % Example coordinates
obstacle5.width = o_dim(1);             % Width of the obstacle
obstacle5.length = o_dim(2);            % Length of the obstacle
obstacle5.height = o_dim(3);            % Height of the obstacle

% Obstacle 6 is near the goal point:
seed_value = 37; 
rng(seed_value);
rnc = [randi([200, 300]),randi([200, 300]),randi([200, 300])]; % random_number_coord 
o_dim = [randi([20, 100]),randi([20, 100]),randi([20, 100])];
obstacle6.coord = rnc;                  % Example coordinates
obstacle6.width = o_dim(1);             % Width of the obstacle
obstacle6.length = o_dim(2);            % Length of the obstacle
obstacle6.height = o_dim(3);            % Height of the obstacle

% Obstacle 7 is near the goal point:
seed_value = 41; 
rng(seed_value);
rnc = [randi([-200, 0]),randi([0, 200]),randi([0, 50])]; % random_number_coord 
o_dim = [randi([20, 100]),randi([20, 100]),randi([20, 100])];
obstacle7.coord = rnc;                  % Example coordinates
obstacle7.width = o_dim(1);             % Width of the obstacle
obstacle7.length = o_dim(2);            % Length of the obstacle
obstacle7.height = o_dim(3);            % Height of the obstacle

% Add obstacles to the obstacles array
obstacles = [obstacles obstacle1 obstacle2 obstacle3 obstacle4 obstacle5 obstacle6 obstacle7 obstacle22];

% Create vertices and faces for all obstacles
all_vertices = [];
all_faces = [];
vertex_count = 0;
for i = 1:length(obstacles)
    obstacle = obstacles(i);
    [vertices, faces] = create_obstacle(obstacle);
    % Adjust faces indices for the current obstacle
    faces = faces + vertex_count;
    all_vertices = [all_vertices; vertices];
    all_faces = [all_faces; faces];
    % Update vertex count
    vertex_count = size(all_vertices, 1);
end


% %% Acceptable points
% step_size = 3;
% 
% % Create grid points
% [X, Y, Z] = meshgrid(0:step_size:x_max, 0:step_size:y_max, 0:step_size:z_max);
% 
% % Reshape the grid points into a single matrix
% reachable1 = [X(:), Y(:), Z(:)];
% reachable2 = [X(:), Y(:), Z(:)];

%% Path planning

tic;

goal = [600 600 200];
rob1 = [-425 0 1181];
rob2 = [425 0 1181];
% Plot obstacles

%figure();

plot3(0,0,0);

%set(gca,'color',[0.2,0.2,0.7]);

xlabel('X');
ylabel('Y');
zlabel('Z');

hold on
patch('Vertices', all_vertices, 'Faces', all_faces, 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'black', 'FaceAlpha', 0.9);

% Plot the start point of first robot
plot3(rob1(1), rob1(2), rob1(3), 'x', 'Color', 'r','MarkerSize', 7,'MarkerFaceColor', 'b');
text(rob1(1)+25, rob1(2)+25, rob1(3)+25, 'Rob1', 'FontSize', 8, 'FontWeight', 'bold');

% Plot the start point of second robot
plot3(rob2(1), rob2(2), rob2(3), 'x', 'Color', 'b','MarkerSize', 7,'MarkerFaceColor', 'b');
text(rob2(1)+25, rob2(2)+25, rob2(3)+25, 'Rob2', 'FontSize', 8, 'FontWeight', 'bold');


% Plot the GOAL
plot3(goal(1), goal(2), goal(3), 'o', 'Color', 'g','MarkerSize', 7,'MarkerFaceColor', 'b');
text(goal(1)+25, goal(2)+25, goal(3)+25, 'GOAL', 'FontSize', 8, 'FontWeight', 'bold');


EPS = 20;
numNodes = 500;        

q_start.coord = rob1;
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = goal;
q_goal.cost = 0;

nodes(1) = q_start;


for i = 1:1:numNodes
    %rr = ((rand(1) - 0.5) * 2);
    q_rand = [((rand(1) - 0.5) * 2)*x_max, ((rand(1) - 0.5) * 2)*y_max, rand(1)*z_max];
    % Plot the randomly generated node
    %plot3(q_rand(1), q_rand(2), q_rand(3), 'x', 'Color',  [0.5 0.4470 0.2])

    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            break
        end
    end

    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist_3d(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    %q_new.coord = steer3d(q_rand, q_near.coord, val, EPS, obstacles, reachable1);
    q_new.coord = steer3d(q_rand, q_near.coord, val, EPS, obstacles);
    line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', [0.9,0.2,0.2], 'LineWidth', 0.4);
    drawnow
    hold on

    % Plot the obstacle

    q_new.cost = dist_3d(q_new.coord, q_near.coord) + q_near.cost;
    
    % Within a radius of r, find all existing nodes
    q_nearest = [];
    r = 50;
    neighbor_count = 1;
    for j = 1:1:length(nodes)
        if (dist_3d(nodes(j).coord, q_new.coord)) <= r
            q_nearest(neighbor_count).coord = nodes(j).coord;
            q_nearest(neighbor_count).cost = nodes(j).cost;
            neighbor_count = neighbor_count+1;
        end
    end
    
    % Initialize cost to currently known value
    q_min = q_near;
    C_min = q_new.cost;
    
    % Iterate through all nearest neighbors to find alternate lower
    % cost paths
    
    for k = 1:1:length(q_nearest)
        if q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord) < C_min
            q_min = q_nearest(k);
            C_min = q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord);
            line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], [q_min.coord(3), q_new.coord(3)], 'Color', 'g');            
            hold on
        end
    end
    
    % Update parent to least cost-from node
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_min.coord
            q_new.parent = j;
        end
    end
    
    % Append to nodes
    nodes = [nodes q_new];
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist_3d(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];

% Initialize total cost of the final path
total_cost_1 = 0;

while q_end.parent ~= 0
    start = q_end.parent;
    % Calculate the distance between the current node and its parent
    dist_to_parent = dist_3d(q_end.coord, nodes(start).coord);
    % Add the distance to the total cost
    total_cost_1 = total_cost_1 + dist_to_parent;
    % Plot the edge between the current node and its parent
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], [q_end.coord(3), nodes(start).coord(3)], 'Color', 'r', 'LineWidth', 4);
    hold on
    % Move to the parent node
    q_end = nodes(start);
end


% while q_end.parent ~= 0
%     start = q_end.parent;
%     line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], [q_end.coord(3), nodes(start).coord(3)], 'Color', 'r', 'LineWidth', 4);
%     hold on
%     q_end = nodes(start);
% end

final_path1 = [];
current_node = q_final;

while current_node.parent ~= 0
    % Add the current node to the path
    final_path1 = [current_node.coord; final_path1];
    % Move to the parent node
    current_node = nodes(current_node.parent);
end

% Add the start node to the path
final_path1 = [q_start.coord; final_path1];


% Second Manipulator: 
hold on

clearvars -except C_min total_cost_1 obstacles final_path1 reachable2 x_max y_max z_max EPS numNodes q_goal rob2

q_start.coord = rob2;
q_start.cost = 0;
q_start.parent = 0;
q_goal.cost = 0;

nodes(1) = q_start;
figure(1)

for i = 1:1:numNodes
    q_rand = [((rand(1) - 0.5) * 2)*x_max, ((rand(1) - 0.5) * 2)*y_max, rand(1)*z_max];
    %plot3(q_rand(1), q_rand(2), q_rand(3), 'x', 'Color',  [0 0.4470 0.7410])
    
    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            break
        end
    end
    
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist_3d(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    %q_new.coord = steer3d(q_rand, q_near.coord, val, EPS, obstacles, reachable2);
    q_new.coord = steer3d(q_rand, q_near.coord, val, EPS, obstacles);
    line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', [0.2,0.2,0.9], 'LineWidth', 0.4);
    drawnow
    hold on
    q_new.cost = dist_3d(q_new.coord, q_near.coord) + q_near.cost;
    
    % Within a radius of r, find all existing nodes
    q_nearest = [];
    r = 50;
    neighbor_count = 1;
    for j = 1:1:length(nodes)
        if (dist_3d(nodes(j).coord, q_new.coord)) <= r
            q_nearest(neighbor_count).coord = nodes(j).coord;
            q_nearest(neighbor_count).cost = nodes(j).cost;
            neighbor_count = neighbor_count+1;
        end
    end
    
    % Initialize cost to currently known value
    q_min = q_near;
    C_min2 = q_new.cost;
    
    % Iterate through all nearest neighbors to find alternate lower
    % cost paths
    
    for k = 1:1:length(q_nearest)
        if q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord) < C_min2
            q_min = q_nearest(k);
            C_min2 = q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord);
            line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], [q_min.coord(3), q_new.coord(3)], 'Color', 'g');            
            hold on
        end
    end
    
    % Update parent to least cost-from node
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_min.coord
            q_new.parent = j;
        end
    end
    
    % Append to nodes
    nodes = [nodes q_new];
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist_3d(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];

% while q_end.parent ~= 0
%     start = q_end.parent;
%     line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], [q_end.coord(3), nodes(start).coord(3)], 'Color', 'b', 'LineWidth', 4);
%     hold on
%     q_end = nodes(start);
% end

% Initialize total cost of the final path of second robot
total_cost_2 = 0;

while q_end.parent ~= 0
    start = q_end.parent;
    % Calculate the distance between the current node and its parent
    dist_to_parent = dist_3d(q_end.coord, nodes(start).coord);
    % Add the distance to the total cost
    total_cost_2 = total_cost_2 + dist_to_parent;
    % Plot the edge between the current node and its parent
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], [q_end.coord(3), nodes(start).coord(3)], 'Color', 'b', 'LineWidth', 4);
    hold on
    % Move to the parent node
    q_end = nodes(start);
end

final_path2 = [];
current_node = q_final;

while current_node.parent ~= 0
    % Add the current node to the path
    final_path2 = [current_node.coord; final_path2];
    % Move to the parent node
    current_node = nodes(current_node.parent);
end

% Add the start node to the path
final_path2 = [q_start.coord; final_path2];


if (total_cost_1 < total_cost_2)
    disp("Rob1 is better for the given task");
    disp(total_cost_1)
else
    disp("Rob2 is better for the given task");
    disp(total_cost_2)
end

toc;


function A = steer3d(qr, qn, val, eps, obstacles)
    qnew = [0 0 0];
    if val >= eps
        qnew(1) = qn(1) + ((qr(1)-qn(1))*eps)/dist_3d(qr,qn);
        qnew(2) = qn(2) + ((qr(2)-qn(2))*eps)/dist_3d(qr,qn);
        qnew(3) = qn(3) + ((qr(3)-qn(3))*eps)/dist_3d(qr,qn);
    else
        qnew(1) = qr(1);
        qnew(2) = qr(2);
        qnew(3) = qr(3);
    end

    % Check for collision with obstacles
    for i = 1:size(obstacles, 2)
        obstacle = obstacles(i);
        if qnew(1) >= obstacle.coord(1) - obstacle.width/2 && qnew(1) <= obstacle.coord(1) + obstacle.width/2 && ...
           qnew(2) >= obstacle.coord(2) - obstacle.length/2 && qnew(2) <= obstacle.coord(2) + obstacle.length/2 && ...
           qnew(3) >= obstacle.coord(3) - obstacle.height/2 && qnew(3) <= obstacle.coord(3) + obstacle.height/2
            % If collision detected, return the current node (qn)
            A = qn;
            return;
        end
    end

    A = qnew;
end

function [vertices, faces] = create_obstacle(obstacle)
    x_min = obstacle.coord(1) - obstacle.width/2;
    x_max = obstacle.coord(1) + obstacle.width/2;
    y_min = obstacle.coord(2) - obstacle.length/2;
    y_max = obstacle.coord(2) + obstacle.length/2;
    z_min = obstacle.coord(3) - obstacle.height/2;
    z_max = obstacle.coord(3) + obstacle.height/2;

    % Define obstacle vertices
    vertices = [
        x_min, y_min, z_min; % Vertex 1
        x_max, y_min, z_min; % Vertex 2
        x_max, y_max, z_min; % Vertex 3
        x_min, y_max, z_min; % Vertex 4
        x_min, y_min, z_max; % Vertex 5
        x_max, y_min, z_max; % Vertex 6
        x_max, y_max, z_max; % Vertex 7
        x_min, y_max, z_max; % Vertex 8
    ];

    % Define obstacle faces
    faces = [
        1, 2, 3, 4; % Front face
        5, 6, 7, 8; % Back face
        1, 2, 6, 5; % Bottom face
        4, 3, 7, 8; % Top face
        1, 4, 8, 5; % Left face
        2, 3, 7, 6; % Right face
    ];
end

function d = dist_3d(q1,q2)
    d = sqrt((q1(1)-q2(1))^2 + (q1(2)-q2(2))^2 + (q1(3)-q2(3))^2);
end