clear all
close all
%% Attempt at kinematics for quadcopter
%Testing the angular stuff.
current_angles = [0 0 0];
angles = [0 0 0];
velocity_euler = [deg2rad(90); deg2rad(90); deg2rad(90)];

wb = body_ang_vel(current_angles, velocity_euler);
wf = fixed_ang_vel(current_angles, velocity_euler);

for i = .001:.001:1
    current_angles = current_angles + (wb * .001).';
    angles = vertcat(angles, current_angles);
end
print_angular_change(angles, 'If angular change is applied to Body Frame');

current_angles = [0 0 0];
angles = [0 0 0];
for i = .001:.001:1
    current_angles = current_angles + (wf * .001).';
    angles = vertcat(angles, current_angles);
end
print_angular_change(angles, 'If angular change is applied to Reference Frame');

%% Angular Velocity in terms of BodyFrame Coodinates
function print_angular_change(angles, tittle)
    figure()
    for i = 1:10:length(angles)
        title(tittle);
        xlabel('X-axis');
        xlim([-1 1]);
        ylabel('Y-axis');
        ylim([-1 1]);
        zlabel('Z-axis');
        zlim([-1 1]);
        one_rotation = eul2rotm(angles(i,:),'ZYZ');
        scatter3(one_rotation(1,:),one_rotation(2,:),one_rotation(3,:));
        pause(.1)
        hold on
    end
    hold off
    eul2rotm(angles(length(angles),:),'ZYZ')
end

function wb = body_ang_vel(current_euler,velocity_euler)
    theta = current_euler(2);
    sinn = current_euler(3);
    Trans = [-sin(theta)*cos(sinn) sin(sinn) 0; 
              sin(theta)*sin(sinn) cos(sinn) 0; 
              cos(theta)            0        1]
    wb = Trans * velocity_euler;
end

%%Angular Velocity in terms of InertialFrame Coordinates

function wf = fixed_ang_vel(current_euler,velocity_euler)
    phi = current_euler(1);
    theta = current_euler(2);
    Trans = [0     -sin(phi)      -sin(theta)*cos(phi); 
             0      cos(phi)       sin(theta)*sin(phi); 
             1         0               cos(theta)]
    wf = Trans * velocity_euler;
end