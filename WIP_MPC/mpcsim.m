% % animation plot

disp('Animation Over Prediction phase started')
% Extract states
xHistory1=xHistory';
postion_model = xHistory1(:, 1);
angle_model = xHistory1(:, 3); %.*(180/pi);
% postion_sindy = xSINDYc(:, 1);
% angle_sindy = xHistory(:, 3); %.*(180/pi);
L  = 0.58; 
% Set up the figure
figure(5);
% axis tight manual;
% ax = gca;
% ax.NextPlot = 'replaceChildren';
i=1;


% Animation loop
for i = 1:5:Nt  % Pendulum position
    p_xm = postion_model(i) + L*sin(angle_model(i));
    p_ym = L*cos(angle_model(i));
    p_zm = 0;
    % p_xs = postion_sindy(i) + L*sin(angle_sindy(i));
    % p_ys= L*cos(angle_sindy(i));
    % p_zs = 0;

    % Wheel position
    w_xm = postion_model(i);
    w_ym = 0;
    w_zm = 0;
    % w_xs = postion_sindy(i);
    w_ys = 0;
    % w_zs = 0;


    %  2D plot
    plot([w_xm p_xm], [w_ym p_ym],  'r','LineWidth', 2);
    hold on;
    % plot([w_xs p_xs], [w_ys p_ys], 'b','LineWidth', 2);
    
    plot(w_xm, w_ym, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    plot(p_xm, p_ym, 'bo', 'MarkerSize', 10, 'LineWidth', 2);    
    % plot(w_xs, w_ys, 'r.', 'MarkerSize', 20, 'LineWidth', 2);
    % plot(p_xs, p_ys, 'b.', 'MarkerSize', 20, 'LineWidth', 2);
    % plot([postion_model, p_xm], [0.1, p_ym], '--r');
    hold off;

    % Set axis limits
    newLimits= [w_xm - 2, w_xm + 2];
    xlim(newLimits);
    ylim([-1 1]);

    % Title and labels
    
    xlabel('Position X asix (m)');
    ylabel('Height (m)');

%     % % Plot the pendulum and wheel
%     % plot3([w_x p_x], [w_y p_y], [w_z p_z], 'LineWidth', 2);
%     % hold on;
%     % plot3(w_x, w_y, w_z, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
%     % plot3(p_x, p_y, p_z, 'bo', 'MarkerSize', 20, 'LineWidth', 2);
%     % hold off;
% 
%     % Set axis limits
%     % xlim([-100 20]);
%     % ylim([-1 5]);
%     % zlim([-0.5 0.5]);
% 
%     % % Title and labels
%     % title('Wheeled Inverted Pendulum Animation');
%     % xlabel('Position (m)');
%     % ylabel('Position (m)');
%     % zlabel('Z Position (m)');
% 
%     % View angle
%     % view(30, 30);
% 
% disp('frame updated')

legend('Model');
    % % Pause for animation
    title('t: ',num2str(i*Ts))
    % hold on
    pause(Ts);
    % hold on
    
end
disp('Animation Over Prediction phase ended')