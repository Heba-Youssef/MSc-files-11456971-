clear; close all;

include_namespace_dq

vi = DQ_VrepInterface();
vi.disconnect_all();
vi.connect('127.0.0.1',19997);
vi.set_synchronous(true);
vi.start_simulation();

%% UR3e robot
jointnames={'UR3_joint1','UR3_joint2',...
            'UR3_joint3','UR3_joint4',...
            'UR3_joint5','UR3_joint6'};

%% UR3e params. Verified on 02-08-2024
UR3e_DH_theta =  [pi,            0,          0,                  0,              0,          pi/2];
UR3e_DH_d =      [0,             0,          0,                  0.11188,        0.08535,    0.0921]; 
UR3e_DH_a =      [0,             -0.24335,   -0.2132-0.00435,    0,              0,          0];
UR3e_DH_alpha =  [pi/2,          0,          0,                  pi/2,           -pi/2,      -pi/2];

UR3e_DH_type = double(repmat(DQ_JointType.REVOLUTE,1,6));
UR3e_DH_matrix = [UR3e_DH_theta;
                    UR3e_DH_d;
                    UR3e_DH_a;
                    UR3e_DH_alpha;
                    UR3e_DH_type];

robot_0 = DQ_SerialManipulatorDH(UR3e_DH_matrix); % No end-effector extension
robot_1 = DQ_SerialManipulatorDH(UR3e_DH_matrix); % SPRT added
robot_2 = DQ_SerialManipulatorDH(UR3e_DH_matrix); % Gripper tip 

%% Set location of end effector (end of SPRT)
robot_1.set_effector(  1+0.5*E_*(  (0)*i_ + (-0.47)*j_ + (0)*k_  )  ); % 30/08 Changed color of furnace and sprt
robot_2.set_effector(  1+0.5*E_*(  (0)*i_ + (-0.012)*j_ + (0)*k_  )  ); % 2/9 tip of gripper to avoid furnace top

n = robot_1.get_dim_configuration_space();

q0 = [-90; -100; -20; -120; 90; 90]*pi/180; % Initial configuration
vi.set_joint_positions(jointnames, q0);

try   
    %----------------------------------------------------------------%
    % Set robot ref and base frames
    xbase = vi.get_object_pose('UR3_joint1');
    robot_0.set_reference_frame(xbase);
    robot_0.set_base_frame(xbase);
    robot_1.set_reference_frame(xbase);
    robot_1.set_base_frame(xbase);
    robot_2.set_reference_frame(xbase);
    robot_2.set_base_frame(xbase);
    vi.set_object_pose('base', xbase);

    q =  vi.get_joint_positions(jointnames);

    %----------------------------------------------------------------%
    % Controller definition
    qp_solver = DQ_QuadprogSolver();
    controller = DQ_ClassicQPController(robot_1, qp_solver);
    
    % Joint angle limits
    qmin = [-2*pi; -2*pi; -2*pi; -2*pi; -2*pi; -inf];
    qmax = [2*pi; 2*pi; 2*pi; 2*pi; 2*pi; inf];
    
    % Joint velocity limits
    q_dot_max = [pi; pi; pi; 2*pi; 2*pi; 2*pi]; % when using datasheet velocity limits

    % end-effector velocity limits
    t_dot_max = [0.01;0.01;0.01]*3;
    t_dot_min = -[0.01;0.01;0.01]*3;

    gain = 0.8;
    damping = 0.00001; %0.0005;
    stab_threshold = 0.0001;


    controller.set_gain( gain );
    controller.set_damping( damping );
    controller.set_stability_threshold( stab_threshold );

    %----------------------------------------------------------------%
    %  Plane constraints
    safe_distance = 0.00375;

    furnace_plane = Adsharp(vi.get_object_pose('furnace_plane'), k_);
    wall_k = Adsharp(vi.get_object_pose('wall_k'), k_);         % wall_k (left wall)
    wall_nk = Adsharp(vi.get_object_pose('wall_nk'), -k_);      % wall_nk (right wall)
    wall_k2 = Adsharp(vi.get_object_pose('wall_k2'), k_);       % wall_k2 (back wall)
    wall_nk2 = Adsharp(vi.get_object_pose('wall_nk2'), -k_);    % wall_nk2 (front wall)

    walls = {wall_k, wall_nk, wall_k2, wall_nk2};

    %----------------------------------------------------------------%
    %  Goals
    goal_line = vi.get_object_pose('goal1'); % Location of line constraint

    goal_plane1 = vi.get_object_pose('goal_plane1'); % First insertion depth of 45 cm
    goal_plane1_d = Adsharp(goal_plane1 , k_);

    goal_plane1mm = vi.get_object_pose('goal_plane1mm'); % pull up by 1 mm
    goal_plane1mm_d = Adsharp(goal_plane1mm , k_);

    goal_plane1cm = vi.get_object_pose('goal_plane1cm'); % pull up by 1 cm
    goal_plane1cm_d = Adsharp(goal_plane1cm , k_);

    goal_plane2cm = vi.get_object_pose('goal_plane2cm'); % pull up by 2 cm
    goal_plane2cm_d = Adsharp(goal_plane2cm , k_);

    goal_plane3cm = vi.get_object_pose('goal_plane3cm'); % pull up by 3 cm
    goal_plane3cm_d = Adsharp(goal_plane3cm , k_);

    goal_plane4cm = vi.get_object_pose('goal_plane4cm'); % pull up by 4 cm
    goal_plane4cm_d = Adsharp(goal_plane4cm , k_);

    goal_plane6cm = vi.get_object_pose('goal_plane6cm'); % pull up by 6 cm
    goal_plane6cm_d = Adsharp(goal_plane6cm , k_);

    goal_plane8cm = vi.get_object_pose('goal_plane8cm'); % pull up by 8 cm
    goal_plane8cm_d = Adsharp(goal_plane8cm , k_);

    goals = [goal_line, goal_plane1_d, goal_plane1mm_d, goal_plane1cm_d, goal_plane2cm_d, goal_plane3cm_d, goal_plane4cm_d, goal_plane6cm_d, goal_plane8cm_d,furnace_plane];

    desired_distance_to_target_plane = 0;

    %% After implementing each control type separately, this part of the code implements the full process.
    j=1; % Data logging
    c=1; % Iterate goals

    while c <= length(goals)
        while ~controller.system_reached_stable_region()

            vi.trigger_next_simulation_step();
            vi.wait_for_simulation_step_to_end();

            % get joint positions
            q =  vi.get_joint_positions(jointnames);

            % Original end effector
            x_pose_0 = robot_0.fkm(q);
            p_0 = translation(x_pose_0); 
            J_pose_0 = robot_0.pose_jacobian(q);
            J_trans_0 = robot_0.translation_jacobian(J_pose_0,x_pose_0);
      
            % SPRT extended end effector
            x_pose = robot_1.fkm(q);
            p = translation(x_pose); 
            J_pose = robot_1.pose_jacobian(q);
            J_trans = robot_1.translation_jacobian(J_pose,x_pose);

            % Gripper tip end effector
            x_pose_2 = robot_2.fkm(q);
            p_2 = translation(x_pose_2); 
            J_pose_2 = robot_2.pose_jacobian(q);
            J_trans_2 = robot_2.translation_jacobian(J_pose_2,x_pose_2);

            vi.set_object_pose('orig_endeff',x_pose_0)
            vi.set_object_pose('SPRT_endeff',x_pose)
            vi.set_object_pose('gripper_endeff',x_pose_2)

            if c == 1
            % Line control at goal_line (previously, named xgoal1)

                % define task refernece (line)
                line_point = translation(goal_line);
                line_direction = k_;
                line_target = line_direction +  E_ * cross(line_point, line_direction);
                task_reference = vec8(line_target);

                % controller objective
                controller.set_control_objective(ControlObjective.Line)
                controller.set_primitive_to_effector(j_);

                % Only constraint is top of furnace
                Jdists = -robot_1.point_to_plane_distance_jacobian(J_trans, p, furnace_plane);
                dists  = DQ_Geometry.point_to_plane_distance(p, furnace_plane) - safe_distance;
                
                % Constraints 
                A = [Jdists; -eye(n);    eye(n);    eye(n);     -eye(n)];
                b = [dists'; (q-qmin); -(q-qmax); q_dot_max; q_dot_max];

                controller.set_inequality_constraint(A,b);

                u = controller.compute_setpoint_control_signal(q, task_reference);

            else
                % Plane-following with wall avoidance using two end effector points (top and end of SPRT)
                task_reference = goals(c);

                % controller objective
                controller.set_control_objective(ControlObjective.DistanceToPlane)
                controller.set_target_primitive(task_reference);  

                % Thermowell wall constraints
                for w = 1:size(walls,2)
                    Jdists_0(w,:) = -robot_0.point_to_plane_distance_jacobian(J_trans_0, p_0, walls{w});
                    dists_0(w)    = (DQ_Geometry.point_to_plane_distance(p_0, walls{w}) - safe_distance );      

                    Jdists(w,:) = -robot_1.point_to_plane_distance_jacobian(J_trans, p, walls{w});
                    dists(w)    = (DQ_Geometry.point_to_plane_distance(p, walls{w}) - safe_distance );
                    
                    % Tip of gripper avoid top of furnace
                    Jdists_2 = -robot_2.point_to_plane_distance_jacobian(J_trans_2, p_2, furnace_plane);
                    dists_2  = DQ_Geometry.point_to_plane_distance(p_2, furnace_plane) - safe_distance;
                end
    
                % Constraints
                A = [J_trans(2:end,:);     -J_trans(2:end,:);Jdists; Jdists_0 ; Jdists_2; -eye(n);  eye(n);     eye(n);     -eye(n)];
                b = [t_dot_max;   -t_dot_min; dists'; dists_0' ; dists_2'; (q-qmin); -(q-qmax); q_dot_max; q_dot_max];

                % % Constraints (without end-eff velocity limits)
                % A = [Jdists; Jdists_0 ; Jdists_2; -eye(n);  eye(n);     eye(n);     -eye(n)];
                % b = [dists'; dists_0' ; dists_2'; (q-qmin); -(q-qmax); q_dot_max; q_dot_max];

                controller.set_inequality_constraint(A,b);
                u = controller.compute_setpoint_control_signal(q, desired_distance_to_target_plane);

            end

            q_log(:,j) = q;

            % Simulate movement in coppeliasim
            q = q + u*0.05;
            vi.set_joint_positions(jointnames,q);

            norm(controller.get_last_error_signal())
            
            depth_d(j) = DQ_Geometry.point_to_plane_distance(p, furnace_plane); % log SPRT insertion depth w.r.t furnace top
            e(c,j) = norm(controller.get_last_error_signal()); % log errors for each goal
            p_0_log(j) = p_0; % log translation of original end-effector
            p_sprt_log(j) = p; % log translation of SPRT end-effector
           
            % end_eff0 distance from walls
            wall_left_0(j) = DQ_Geometry.point_to_plane_distance(p_0, walls{1}); 
            wall_right_0(j) = DQ_Geometry.point_to_plane_distance(p_0, walls{2});
            wall_back_0(j) = DQ_Geometry.point_to_plane_distance(p_0, walls{3});
            wall_front_0(j) = DQ_Geometry.point_to_plane_distance(p_0, walls{4});

            % gripper end_eff distance from furnace top
            gripper_d_furnace(j) = DQ_Geometry.point_to_plane_distance(p_2, furnace_plane);

            % sprt end_eff distance from walls
            wall_left_sprt(j) = DQ_Geometry.point_to_plane_distance(p, walls{1});
            wall_right_sprt(j) = DQ_Geometry.point_to_plane_distance(p, walls{2});
            wall_back_sprt(j) = DQ_Geometry.point_to_plane_distance(p, walls{3});
            wall_front_sprt(j) = DQ_Geometry.point_to_plane_distance(p, walls{4});

            % log control input and joint velocities
            u_log(:,j) = u;
            q_dot(:,j) = vi.get_joint_velocities(jointnames);
            
            % log end effector velocity
            q_dot_v = vi.get_joint_velocities(jointnames);
            v_log(:,j) = J_trans(2:end,:)*q_dot_v;

            j = j+1;

        end % End of while controller ~stable region

        % After goal is reached:
        if c == 1 % Save the point where the robot reached the furnace top
            plot_idx = j;
        else
            j_freeze = j+25; % Robot freeze time at each goal (to make plotting clearer)
            q_freeze = q;

            while j < j_freeze
                vi.trigger_next_simulation_step();
                vi.wait_for_simulation_step_to_end();
   
                q_log(:,j) = q_freeze;

                depth_d(j) = DQ_Geometry.point_to_plane_distance(p, furnace_plane); % log SPRT insertion depth w.r.t furnace top
                e(c,j) = norm(controller.get_last_error_signal()); % log errors for each goal
                p_0_log(j) = p_0;
                p_sprt_log(j) = p;
    
                % end_eff0 distance from walls
                wall_left_0(j) = DQ_Geometry.point_to_plane_distance(p_0, walls{1}); 
                wall_right_0(j) = DQ_Geometry.point_to_plane_distance(p_0, walls{2});
                wall_back_0(j) = DQ_Geometry.point_to_plane_distance(p_0, walls{3});
                wall_front_0(j) = DQ_Geometry.point_to_plane_distance(p_0, walls{4});

                % gripper end_eff distance from furnace top
                gripper_d_furnace(j) = DQ_Geometry.point_to_plane_distance(p_2, furnace_plane);
    
                % sprt end_eff distance from walls
                wall_left_sprt(j) = DQ_Geometry.point_to_plane_distance(p, walls{1});
                wall_right_sprt(j) = DQ_Geometry.point_to_plane_distance(p, walls{2});
                wall_back_sprt(j) = DQ_Geometry.point_to_plane_distance(p, walls{3});
                wall_front_sprt(j) = DQ_Geometry.point_to_plane_distance(p, walls{4});
    
                % log control input and joint velocities
                u_log(:,j) = u;
                q_dot(:,j) = vi.get_joint_velocities(jointnames);

                % log end effector velocity
                q_dot_v = vi.get_joint_velocities(jointnames);
                v_log(:,j) = J_trans(2:end,:)*q_dot_v;

                j = j+1;
            end
        end
        c = c+1;

        % After line control is successful, reset 'reached stable region' by setting a new control objective. 
        if c<=length(goals)
            controller.set_control_objective(ControlObjective.DistanceToPlane)
            controller.set_target_primitive( goals(c) );
        end

    end % End of goals while loop

    % Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
    vi.wait_for_simulation_step_to_end();

    % Save variables of this run with time stamp
    timenow = datetime('now','Format','dd_MM''_T''hhmm');
    save("sim_vars_"+ string(timenow)+ '.mat')

    catch ME
    vi.stop_simulation();
    vi.disconnect();
    rethrow(ME)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% PLOTTING FUNCTIONS %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Define time vector (sampling to real time) and radius of SPRT
t_max = (j-1)*0.05;          % J index is always +1 of variable sizes. 0.05 is the simulation time
t_plot = 0:0.05:t_max-0.05;  % Plotting time
sprt_radius = 0.0035; 

%% Plot q_dot (subplots)
figure(1);
% set(fig, 'Units', 'centimeters', 'Position', [0, 0, 17, 10]); % Adjust figure size
for i = 1:n % Plot for all joints
    subplot(2,3,i);
    plot(t_plot,u_log(i,:),'b', 'LineWidth',1);
    hold on
    plot(t_plot,q_dot(i,:),':r', 'LineWidth',1);
    yline(q_dot_max(i),'--',"$\dot{q}_{\lim}$", 'Interpreter','latex');
    yline(-q_dot_max(i),'--',"$-\dot{q}_{\lim}$", 'Interpreter','latex');
    xlim([0 t_max])
    xlabel('Time (s)')
    ylabel('Velocity (rad/s)')
    title("$\dot{q}$"+i, 'Interpreter','latex', 'Rotation', 0)
end
% legend('reference', 'measurement')
hold off

% Make sure that q_dot follows u accurately
err_u_qdot = immse( u_log , q_dot ) 
[err_max,max_idx] = max(abs(u_log-q_dot),[],'all') 

%% Plot q_dot (1 plot)
plot(t_plot,q_dot)
line1 = yline(q_dot_max(1),'--',"$\dot{q}_{\lim,\min}$", 'Interpreter','latex');line1.LabelHorizontalAlignment = 'right';
line2 =yline(-q_dot_max(1),'--',"$-\dot{q}_{\lim,\min}$", 'Interpreter','latex');line2.LabelHorizontalAlignment = 'right';
% yline(q_dot_max(4),'--',"$\dot{q}_{\lim,4,5,6}$", 'Interpreter','latex');
% yline(-q_dot_max(4),'--',"$-\dot{q}_{\lim,4,5,6}$", 'Interpreter','latex');
legend("$\dot{q}_{1}$", "$\dot{q}_{2}$" , "$\dot{q}_{3}$","$\dot{q}_{4}$","$\dot{q}_{5}$","$\dot{q}_{6}$",'Interpreter','latex','Location','northeastoutside')
xlabel('Time (s)')
ylabel('Velocity (rad/s)')
xlim([0 t_max])
title("$\dot{q}$ within constraints", 'Interpreter','latex', 'Rotation', 0)
name = "1_q_dot";

%% Visualize SPRT insertion depths (working, cylinder)
figure(2)  
%%%%%%%%%% old%%%%%%%%%% old%%%%%%%%%% old%%%%%%%%%% old (Plots single point only)
% in_furn_idx = find(depth_d < 0, 1, 'first'); % Obtain idx for when SPRT is inside the furnace (depth < 0)
% plot3(wall_front_0(in_furn_idx:end), wall_left_0(in_furn_idx:end), depth_d(in_furn_idx:end)) 
% hold on
% % Modified wall distances from each other. This is the realistic distance (8 mm)
% fill3([0.008,0.008,0.008,0.008], [0,0.008,0.008,0] , [0,0,-0.45,-0.45], 'r', FaceAlpha=0.1) % Right wall
% fill3([0,0,0,0], [0,0.008,0.008,0] , [0,0,-0.45,-0.45], 'b',FaceAlpha=0.1) % left wall
% fill3([0,0.008,0.008,0], [0.008,0.008,0.008,0.008] , [0,0,-0.45,-0.45], 'g',FaceAlpha=0.1) % back wall
% fill3([0,0.008,0.008,0], [0,0,0,0] , [0,0,-0.45,-0.45], 'k',FaceAlpha=0.1) % front wall
% xlabel('front wall')
% ylabel('left wall')
% zlabel('depth')
% hold off
%%%%%%%%%% old%%%%%%%%%% old%%%%%%%%%% old%%%%%%%%%% old

in_furn_idx = find(depth_d < 0, 1, 'first'); % Obtain idx for when SPRT is inside the furnace (depth < 0)
up_furn_idx = find(diff(depth_d(in_furn_idx:end)) > 0, 1); % Obtain value when SPRT is being withdrawn

% Plot the trajectory of the mid-point of the thermometer
plot3(wall_front_sprt(in_furn_idx:up_furn_idx), wall_left_sprt(in_furn_idx:up_furn_idx), depth_d(in_furn_idx:up_furn_idx), 'LineWidth', 1.5)
hold on

% Modified wall distances from each other. This is the realistic distance (8 mm)
fill3([0.008,0.008,0.008,0.008], [0,0.008,0.008,0] , [0,0,-0.45,-0.45], 'r', 'FaceAlpha', 0.1) % Right wall
fill3([0,0,0,0], [0,0.008,0.008,0] , [0,0,-0.45,-0.45], 'b', 'FaceAlpha', 0.1) % Left wall
fill3([0,0.008,0.008,0], [0.008,0.008,0.008,0.008] , [0,0,-0.45,-0.45], 'g', 'FaceAlpha', 0.1) % Back wall
fill3([0,0.008,0.008,0], [0,0,0,0] , [0,0,-0.45,-0.45], 'k', 'FaceAlpha', 0.1) % Front wall

% Parameters for the thermometer
thermo_radius = 0.0035; % Radius of thermometer (half of the 7 mm diameter)
theta = linspace(0, 2*pi, 100); % Angle for the circular cross-section of the thermometer

for i = in_furn_idx:up_furn_idx
    % X and Y coordinates of the circular cross-section of the thermometer
    x = wall_front_sprt(i) + thermo_radius * cos(theta);
    y = wall_left_sprt(i) + thermo_radius * sin(theta);
    
    % Plot the cylinder at each depth point
    fill3(x, y, depth_d(i) * ones(size(x)), 'm', 'FaceAlpha', 0.5)
end

plot3(wall_front_sprt(up_furn_idx:end), wall_left_sprt(up_furn_idx:end), depth_d(up_furn_idx:end), 'LineWidth', 1.5)
for i = up_furn_idx:length(depth_d)
    % X and Y coordinates of the circular cross-section of the thermometer
    x = wall_front_sprt(i) + thermo_radius * cos(theta);
    y = wall_left_sprt(i) + thermo_radius * sin(theta);
    
    % Plot the cylinder at each depth point
    fill3(x, y, depth_d(i) * ones(size(x)), 'g', 'FaceAlpha', 0.5)
end

xlabel('front wall')
ylabel('left wall')
zlabel('depth')
title('Top view of SPRT occupation of thermowell')
view(0,90)  % XY
grid on
hold off
name = "2_cylinder";

%% Plot one circle (min distance)
figure(3)
[front,frontidx] = min(wall_front_sprt(in_furn_idx:end));
[left,leftidx] = min(wall_left_sprt(in_furn_idx:end));
if front < left
    circle_idx = frontidx;
    'front'
else
    circle_idx = leftidx;
    'left'
end

% Parameters for the thermometer
thermo_radius = 0.0035; % Radius of thermometer (half of the 7 mm diameter)
theta = linspace(0, 2*pi, 100); % Angle for the circular cross-section of the thermometer

x = wall_front_sprt(circle_idx) + thermo_radius * cos(theta);
y = wall_left_sprt(circle_idx) + thermo_radius * sin(theta);

fill(x, y, 	[1 0 1], 'FaceAlpha', 1)
hold on
fill([0, 0.008, 0.008, 0], [0, 0, 0.008, 0.008], [0.95 0.95 0.95], 'FaceAlpha', 0.1) % Prism walls
xlabel('front wall')
ylabel('left wall')
[~,min_idx] = min(y);
text(3e-3,1.1e-3,"$d_{\min} = $"+num2str(min(y)),'Interpreter','latex','Color',[1 1 1],'FontSize',15,'FontWeight','bold')
% legend("$d_{\min} = $"+num2str(min(y)),'Interpreter','latex')
% xlim([-0.001 8.001e-3])
% ylim([-0.001 8.001e-3])
title("SPRT nearest to wall at depth of "+num2str(depth_d(circle_idx),2))
grid on
name = "3_min_circ"

%% Plot depth with color gradient (working)
figure(3)
%%%%%%%%%% old%%%%%%%%%% old%%%%%%%%%% old%%%%%%%%%% old
% plot(t_plot(plot_idx:end),depth_d(plot_idx:end),'LineWidth', 2)
% hold on
% yline(0,'--','furnace top')
% G1 = yline(-0.45,'--','Goal #1: -45 cm');
% G2 = yline(-0.45+0.001,'--','Goal #2: +1 mm');    G2.LabelHorizontalAlignment = 'left';
% G3=yline(-0.45+0.01,'--','Goal #3: +1 cm'); G3.LabelHorizontalAlignment = 'left';
% G4=yline(-0.45+0.02,'--','Goal #4: +2 cm'); G4.LabelHorizontalAlignment = 'left';
% G5=yline(-0.45+0.03,'--','Goal #5: +3 cm'); G5.LabelHorizontalAlignment = 'left';
% G6=yline(-0.45+0.04,'--','Goal #6: +4 cm'); G6.LabelHorizontalAlignment = 'left';
% G7=yline(-0.45+0.06,'--','Goal #7: +6 cm'); G7.LabelHorizontalAlignment = 'left';
% G8=yline(-0.45+0.08,'--','Goal #8: +8 cm'); G8.LabelHorizontalAlignment = 'left';
% ylim([min(depth_d(plot_idx:end))-0.01,max(depth_d(plot_idx:end))])

% plot(t_plot(plot_idx:end),depth_d(plot_idx:end),'LineWidth', 2)
% hold on
% yline(0, '--','furnace top','FontSize',12, 'Color', [0.5, 0.5, 0.5]); % Gray for furnace top
% yline(-0.45, '--', 'Color', [1, 0, 0]);    % Red for Goal #1
% yline(-0.45 + 0.001, '--', 'Color', [0, 1, 0]); % Green for Goal #2
% yline(-0.45 + 0.01, '--', 'Color', [0, 0, 1]);  % Blue for Goal #3
% yline(-0.45 + 0.02, '--', 'Color', [1, 0.5, 0]); % Orange for Goal #4
% yline(-0.45 + 0.03, '--', 'Color', [0.5, 0, 0.5]); % Purple for Goal #5
% yline(-0.45 + 0.04, '--', 'Color', [0, 0.75, 0.75]); % Cyan for Goal #6
% yline(-0.45 + 0.06, '--', 'Color', [0.75, 0, 0.75]); % Magenta for Goal #7
% yline(-0.45 + 0.08, '--', 'Color', [0.5, 0.5, 0]);   % Olive for Goal #8
% ylim([min(depth_d(plot_idx:end))-0.01,max(depth_d(plot_idx:end))])
% % Add labels
% xlabel('Time (s)')
% ylabel('Distance from furnace top (m)')
% title('SPRT insertion depths')

%%%%%%%%%% old%%%%%%%%%% old%%%%%%%%%% old%%%%%%%%%% old
%%
hold on

% Define the data
t_plot_segment = t_plot(plot_idx:end); % Time (x-axis)
depth_segment = depth_d(plot_idx:end); % Depth (y-axis)

% Create a custom color map that transitions from red (low values) to blue (high values)
nPoints = length(depth_segment);
cmap = [linspace(1, 0, nPoints)', zeros(nPoints, 1), linspace(0, 1, nPoints)']; 
% This creates a gradient from red (1,0,0) to blue (0,0,1)

depth_min = min(depth_segment);
depth_max = max(depth_segment);

% Plot the line using 'surface' trick to get gradient coloring based on depth
for i = 1:nPoints-1
    % Normalize the depth to get a corresponding color from the custom colormap
    colorValue = cmap(round(((depth_segment(i) - depth_min) / (depth_max - depth_min)) * (nPoints-1)) + 1, :);
    % Plot each line segment
    line(t_plot_segment(i:i+1), depth_segment(i:i+1), 'Color', colorValue, 'LineWidth', 2);
end

% Plot horizontal goal lines with different colors
yline(0, '--','furnace top','FontSize',12, 'Color', [0.5, 0.5, 0.5]); % Gray for furnace top
yline(-0.45, '--', 'Color', [1, 0, 0]);    % Red for Goal #1
yline(-0.45 + 0.001, '--', 'Color', [0, 1, 0]); % Green for Goal #2
yline(-0.45 + 0.01, '--', 'Color', [0, 0, 1]);  % Blue for Goal #3
yline(-0.45 + 0.02, '--', 'Color', [1, 0.5, 0]); % Orange for Goal #4
yline(-0.45 + 0.03, '--', 'Color', [0.5, 0, 0.5]); % Purple for Goal #5
yline(-0.45 + 0.04, '--', 'Color', [0, 0.75, 0.75]); % Cyan for Goal #6
yline(-0.45 + 0.06, '--', 'Color', [0.75, 0, 0.75]); % Magenta for Goal #7
yline(-0.45 + 0.08, '--', 'Color', [0.5, 0.5, 0]);   % Olive for Goal #8

% Set limits for the plot
ylim([min(depth_segment)-0.01, max(depth_segment)]);
xlim([min(t_plot_segment), max(t_plot_segment)]);

% Add labels
xlabel('Time (s)')
ylabel('Distance from furnace top (m)')
title('SPRT insertion depths')

% Add a colorbar to show the gradient based on depth (y-axis)
colormap(cmap);
% colorbar;
caxis([depth_min, depth_max]); % Set the color axis limits to the depth range
hold off
name="4_depthgradi"

%% Unmodified point distances from walls 
figure(4)
subplot(2,1,1)
plot(t_plot(plot_idx:end),wall_left_0(plot_idx:end))
hold on
plot(t_plot(plot_idx:end),wall_right_0(plot_idx:end))
plot(t_plot(plot_idx:end),wall_back_0(plot_idx:end))
plot(t_plot(plot_idx:end),wall_front_0(plot_idx:end))
hold off
    xlim([t_plot(plot_idx) t_plot(end)])
xlabel('Time (s)')
ylabel('Distance (m)')
title('Original end-effector point-to-plane distance from walls')
legend('left','right','back','front','Location','northeastoutside')

subplot(2,1,2)
plot(t_plot(plot_idx:end),wall_left_sprt(plot_idx:end))
hold on
plot(t_plot(plot_idx:end),wall_right_sprt(plot_idx:end))
plot(t_plot(plot_idx:end),wall_back_sprt(plot_idx:end))
plot(t_plot(plot_idx:end),wall_front_sprt(plot_idx:end))
hold off
xlim([t_plot(plot_idx) t_plot(end)])
xlabel('Time (s)')
ylabel('Distance (m)')
title('SPRT point-to-plane distance from walls')
legend('left','right','back','front','FontSize',8,'Location','northeastoutside')
name="5_unmod_dist"
% figure_property.Width= '17'; % Figure width on canvas
% figure_property.Height= '8'; % Figure height on canvas

%% Constrained (with safe_dist) point distance from walls 
figure(5)

%%%%%%%%%%%
subplot(2,1,1)
plot(t_plot(plot_idx:end),wall_left_0(plot_idx:end)-safe_distance)
hold on
plot(t_plot(plot_idx:end),wall_right_0(plot_idx:end)-safe_distance)
plot(t_plot(plot_idx:end),wall_back_0(plot_idx:end)-safe_distance)
plot(t_plot(plot_idx:end),wall_front_0(plot_idx:end)-safe_distance)

hold off
title('Original end-effector point-to-plane distance from constraints')
legend('left','right','back','front','Location','northeastoutside')
xlim([t_plot(plot_idx) t_plot(end)])
xlabel('Time (s)')
ylabel('Distance (m)')

subplot(2,1,2)
plot(t_plot(plot_idx:end),wall_left_sprt(plot_idx:end)-safe_distance)
hold on
plot(t_plot(plot_idx:end),wall_right_sprt(plot_idx:end)-safe_distance)
plot(t_plot(plot_idx:end),wall_back_sprt(plot_idx:end)-safe_distance)
plot(t_plot(plot_idx:end),wall_front_sprt(plot_idx:end)-safe_distance)
hold off
title('SPRT point-to-plane distance from constraints')
legend('left','right','back','front','Location','northeastoutside')    
xlim([t_plot(plot_idx) t_plot(end)])
xlabel('Time (s)')
ylabel('Distance (m)')
name ="6_mod_dist_dsafe"


%% (With radius) point distance from walls 
figure(6)
subplot(2,1,1)
plot(t_plot(plot_idx:end),wall_left_0(plot_idx:end)-sprt_radius)
hold on
plot(t_plot(plot_idx:end),wall_right_0(plot_idx:end)-sprt_radius)
plot(t_plot(plot_idx:end),wall_back_0(plot_idx:end)-sprt_radius)
plot(t_plot(plot_idx:end),wall_front_0(plot_idx:end)-sprt_radius)
yline(0,'--');
yline(0.008,'--');
hold off
title('Original end-effector realistic distance from walls')
legend('left','right','back','front','Location','northeastoutside')
xlabel('Time (s)')
ylabel('Distance (m)')
xlim([t_plot(plot_idx) t_plot(end)])
ylim([0 0.0084])


subplot(2,1,2)
plot(t_plot(plot_idx:end),wall_left_sprt(plot_idx:end)-sprt_radius)
hold on
plot(t_plot(plot_idx:end),wall_right_sprt(plot_idx:end)-sprt_radius)
plot(t_plot(plot_idx:end),wall_back_sprt(plot_idx:end)-sprt_radius)
plot(t_plot(plot_idx:end),wall_front_sprt(plot_idx:end)-sprt_radius)
yline(0,'--');
yline(0.008,'--');
hold off
title('SPRT realistic distance from walls')
xlabel('Time (s)')
ylabel('Distance (m)')
legend('left','right','back','front','Location','northeastoutside')    
xlim([t_plot(plot_idx) t_plot(end)])
ylim([0 0.0084])
name ="7_mod_dist_radius"

%% Plot q  subplot (working); Add inf limit to last joint plot. Modify colors?
figure(7)
for i = 1:n % Plot for all joints
    subplot(2,3,i);
    plot(t_plot,q_log(i,:),'b', 'LineWidth',1);
    hold on
    yline(qmax(i),'--',"$q_{\lim}$", 'Interpreter','latex');
    yline(-qmax(i),'--',"$-q_{\lim}$", 'Interpreter','latex');
    xlim([0 t_max])
    xlabel('Time (s)')
    ylabel('Angle (rad)')
    title("$q$"+i, 'Interpreter','latex', 'Rotation', 0)
end
hold off
%% Plot q single plot
figure(7)
plot(t_plot,q_log);
hold on
line1 = yline(qmax(1),'--',"$q_{\lim,\min}$",'FontSize',12, 'Interpreter','latex');line1.LabelHorizontalAlignment = 'right';
line2 = yline(-qmax(1),'--',"$-q_{\lim,\min}$",'FontSize',12, 'Interpreter','latex');line2.LabelHorizontalAlignment = 'right';
xlim([0 t_max])
xlabel('Time (s)')
ylabel('Angle (rad)')
legend("$q_{1}$", "$q_{2}$" , "$q_{3}$","$q_{4}$","$q_{5}$","$q_{6}$",'Interpreter','latex','Location','northeastoutside')
hold off

name = "8_q"

%% Plot end effector velocity (subplots)
figure(8)
subplot(3,1,1)
plot(t_plot(in_furn_idx:end),v_log(1,in_furn_idx:end))
hold on 
yline(0.03,'--',"$t_{\lim}$", 'Interpreter','latex');
yline(-0.03,'--',"$-t_{\lim}$", 'Interpreter','latex');
hold off
legend('x-velocity')
    xlabel('Time (s)')
    ylabel('Velocity (m/s)')
xlim([t_plot(in_furn_idx) t_plot(end)])

subplot(3,1,2)
plot(t_plot(in_furn_idx:end),v_log(2,in_furn_idx:end))
hold on 
yline(0.03,'--',"$t_{\lim}$", 'Interpreter','latex');
yline(-0.03,'--',"$-t_{\lim}$", 'Interpreter','latex');
hold off
legend('y-velocity')
    xlabel('Time (s)')
    ylabel('Velocity (m/s)')
xlim([t_plot(in_furn_idx) t_plot(end)])

subplot(3,1,3)
plot(t_plot(in_furn_idx:end),v_log(3,in_furn_idx:end))
hold on 
yline(0.03,'--',"$t_{\lim}$", 'Interpreter','latex');
yline(-0.03,'--',"$-t_{\lim}$", 'Interpreter','latex');
hold off
legend('z-velocity')
    xlabel('Time (s)')
    ylabel('Velocity (m/s)')
xlim([t_plot(in_furn_idx) t_plot(end)])

%% plot end effector velocity (all on one plot) At furnace entry
figure(9)
plot(t_plot(in_furn_idx-50:end),v_log(:,in_furn_idx-50:end))
hold on
line1 = yline(0.03,'--',"$t_{\lim}$", 'Interpreter','latex','FontSize',13); line1.LabelHorizontalAlignment = 'center';
line2 = yline(-0.03,'--',"$-t_{\lim}$", 'Interpreter','latex','FontSize',13); line2.LabelHorizontalAlignment = 'center';
xline(t_plot(in_furn_idx),'-', 'SPRT entered furnace')
hold off
legend('x-velocity','y-velocity','z-velocity','FontSize',10)
xlabel('Time (s)')
ylabel('Velocity (m/s)')
xlim([t_plot(in_furn_idx-50) t_plot(end)])


%% plot end effector velocity (all on one plot) starting 1 sec 
figure(10)
plot(t_plot(30:end),v_log(:,30:end),'LineWidth',1)
hold on
line1 = yline(0.03,'--',"$t_{\lim}$", 'Interpreter','latex','FontSize',13); line1.LabelHorizontalAlignment = 'center';
line2 = yline(-0.03,'--',"$-t_{\lim}$", 'Interpreter','latex','FontSize',13); line2.LabelHorizontalAlignment = 'center';
line3 = xline(t_plot(236),'-','Control Objective Switch','FontSize',12.5); line2.LabelHorizontalAlignment = 'center';
hold off
legend('x-velocity','y-velocity','z-velocity','FontSize',11,'Location','southeast')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
xlim([t_plot(30) t_plot(end)])
title('SPRT translational velocity')
name="9_veloctiyendeff"
%% plot line-control 


%% Plot single circle (not working yet)
% figure(10)
% 
% % Calculate the minimum distance to any wall for each data point
% min_distances = min([wall_left_0, wall_right_0, wall_front_0, wall_back_0], [], 2);
% 
% % Find the index where the thermometer is closest to any wall
% [~, closest_idx] = min(min_distances);
% 
% % Plot the prism walls (8 mm apart)
% fill([0, 0.008, 0.008, 0], [0, 0, 0.008, 0.008], 'k', 'FaceAlpha', 0.1) % Prism walls
% 
% hold on
% 
% % Parameters for the thermometer
% thermo_radius = 0.0035; % Radius of the thermometer (7 mm diameter)
% theta = linspace(0, 2*pi, 100); % Angle for the circular cross-section of the thermometer
% 
% % X and Y coordinates of the circular cross-section of the thermometer at the closest point
% x = wall_front_0(closest_idx) + thermo_radius * cos(theta);
% y = wall_left_0(closest_idx) + thermo_radius * sin(theta);
% 
% % Plot the thermometer's cross-section (circle)
% fill(x, y, 'm', 'FaceAlpha', 0.5)
% 
% % % Annotate the minimum distance on the plot
% % min_distance_value = min_distances(closest_idx);
% % text(wall_front_0(closest_idx), wall_left_0(closest_idx), ...
% %     sprintf('Min dist = %.4f mm', min_distance_value), ...
% %     'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', 'FontSize', 12, 'Color', 'r')
% 
% % Add axis labels and grid
% xlabel('Front wall (m)')
% ylabel('Left wall (m)')
% axis equal
% grid on
% hold off


%% Plot SPRT and orig end effector cylinders (works but unnecessary)
% in_furn_idx = find(depth_d < 0, 1, 'first'); % Obtain idx for when SPRT is inside the furnace (depth < 0)
% % up_furn_idx = find(diff(depth_d(in_furn_idx:end)) > 0, 1); % Obtain value when SPRT is being withdrawn
% 
% % Plot the trajectory of the mid-point of the thermometer
% plot3(wall_front_0(in_furn_idx:up_furn_idx), wall_left_0(in_furn_idx:up_furn_idx), depth_d(in_furn_idx:up_furn_idx), 'LineWidth', 1.5)
% hold on
% 
% % Plot the prism walls (8 mm apart)
% fill3([0.008,0.008,0.008,0.008], [0,0.008,0.008,0] , [0,0,-0.45,-0.45], 'r', 'FaceAlpha', 0.1) % Right wall
% fill3([0,0,0,0], [0,0.008,0.008,0] , [0,0,-0.45,-0.45], 'b', 'FaceAlpha', 0.1) % Left wall
% fill3([0,0.008,0.008,0], [0.008,0.008,0.008,0.008] , [0,0,-0.45,-0.45], 'g', 'FaceAlpha', 0.1) % Back wall
% fill3([0,0.008,0.008,0], [0,0,0,0] , [0,0,-0.45,-0.45], 'k', 'FaceAlpha', 0.1) % Front wall
% 
% % Parameters for the thermometer
% thermo_radius = 0.0035; % Radius of thermometer (half of the 7 mm diameter)
% theta = linspace(0, 2*pi, 100); % Angle for the circular cross-section of the thermometer
% 
% for i = in_furn_idx:up_furn_idx
%     % X and Y coordinates of the circular cross-section of the thermometer at the bottom
%     x_bottom = wall_front_0(i) + thermo_radius * cos(theta);
%     y_bottom = wall_left_0(i) + thermo_radius * sin(theta);
% 
%     % X and Y coordinates of the circular cross-section of the thermometer at the top
%     x_top = wall_front_0(i) + thermo_radius * cos(theta);
%     y_top = wall_left_0(i) + thermo_radius * sin(theta);
% 
%     % Plot the bottom circle of the thermometer
%     fill3(x_bottom, y_bottom, depth_d(i) * ones(size(x_bottom)), 'm', 'FaceAlpha', 0.5)
% 
%     % Plot the top circle of the thermometer
%     fill3(x_top, y_top, depth_d(i) * ones(size(x_top)), 'c', 'FaceAlpha', 0.5)
% 
%     % Connect the top and bottom circles with a line
%     plot3([wall_front_0(i), wall_front_0(i)], ...
%           [wall_left_0(i), wall_left_0(i)], ...
%           [depth_d(i), depth_d(i)], 'k--', 'LineWidth', 1.5)
% end
% 
% xlabel('Front wall')
% ylabel('Left wall')
% zlabel('Depth')
% grid on
% hold off



%% (ARCHIVED) Emailed dr using this plot
 
% subplot(2,1,1)
% plot(t_plot(plot_idx:end),q_dot(2,plot_idx:end))
% hold on
% plot(t_plot(plot_idx:end),q_dot(4,plot_idx:end))
% title("$\dot{q}$ VS. Time", 'Interpreter','latex', 'Rotation', 0, 'FontSize',12)
% legend('$\dot{q}_2$','$\dot{q}_4$','interpreter', 'latex', 'FontSize',12)
% xlim([t_plot(plot_idx),t_plot(end)])
% ylabel("$\dot{q}$",'interpreter', 'latex', 'FontSize',12)
% xlabel('Time (s)', 'FontSize',12)
% 
% subplot(2,1,2)
% plot(t_plot(plot_idx:end),wall_left_0(plot_idx:end)-sprt_radius)
% hold on
% plot(t_plot(plot_idx:end),wall_right_0(plot_idx:end)-sprt_radius)
% plot(t_plot(plot_idx:end),wall_back_0(plot_idx:end)-sprt_radius)
% plot(t_plot(plot_idx:end),wall_front_0(plot_idx:end)-sprt_radius)
% legend('left wall','right wall','back wall','front wall', 'FontSize',12)
% title("$\tilde{d}$ from thermowell walls",'interpreter', 'latex', 'FontSize',12)
% ylabel("$\tilde{d}$",'interpreter', 'latex', 'FontSize',12)
% xlabel('Time (s)', 'FontSize',12)
% xlim([t_plot(plot_idx),t_plot(end)])