%% New attempt with built in libraries

clear all
close all

init_frame = [1 0 0; 0 1 0; 0 0 1;]

% Euler angles defining orientation of local axes
yaw = 90;
pitch = 90;
roll = 90;

% Create orientation matrix from Euler angles using quaternion class
% q = quaternion([yaw pitch roll],'eulerd','zyz','frame');
% myRotationMatrix = rotmat(q,'frame');

for i = 0:1:360
    
    q = quaternion([i i i],'eulerd','xyz','frame');
    myRotationMatrix = rotmat(q,'frame');
    
    child_frame = myRotationMatrix * init_frame
    
    title('Testing');
    xlabel('X-axis');
    xlim([-1 1]);
    ylabel('Y-axis');
    ylim([-1 1]);
    zlabel('Z-axis');
    zlim([-1 1]);
    scatter3(child_frame(1,:),child_frame(2,:),child_frame(3,:));
    pause(.1)
    hold on
    
end
