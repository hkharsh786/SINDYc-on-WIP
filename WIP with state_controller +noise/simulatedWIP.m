% % animation plot

disp('Animation Over Prediction phase started')
% Extract states
postion_model = x(:, 1);
angle_model = x(:, 3); %.*(180/pi);
postion_sindy = xSINDYc(:, 1);
angle_sindy = xSINDYc(:, 3); %.*(180/pi);
L  = 0.58; 
% Set up the figure
figure();
% Create a VideoWriter object
% videoFile = VideoWriter('animation_video.mp4', 'MPEG-4');
% videoFile.FrameRate = 30; % Set the frame rate (adjust as needed)
% open(videoFile);
% Animation loop
for i = 1:20:length(tspan)  % Pendulum position
    p_xm = postion_model(i) + L*sin(angle_model(i));
    p_ym = L*cos(angle_model(i));
    p_zm = 0;
    p_xs = postion_sindy(i) + L*sin(angle_sindy(i));
    p_ys= L*cos(angle_sindy(i));
    % p_zs = 0;

    % Wheel position
    w_xm = postion_model(i);
    w_ym = 0;
    w_zm = 0;
    w_xs = postion_sindy(i);
    w_ys = 0;
    % w_zs = 0;


    %  2D plot
    plot([w_xm p_xm], [0.1 p_ym],  'r-','LineWidth', 1);
    hold on;
    plot([w_xs p_xs], [0.1 p_ys], 'b-','LineWidth', 1);
    
    viscircles([w_xm,0.1], 0.1);
    % plot(w_xm, w_ym, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    plot(p_xm, p_ym, 'ro', 'MarkerSize', 10, 'LineWidth', 2);    
    % plot(w_xs, w_ys, 'ro', 'MarkerSize', 20, 'LineWidth', 2);
    viscircles([w_xs,0.1], 0.1,'EdgeColor', 'b');
    plot(p_xs, p_ys, 'bo', 'MarkerSize', 10, 'LineWidth', 2);
    x_line = linspace(-5000, 5000, 100);
    plot(x_line, zeros(size(x_line)), 'k-', 'LineWidth', 0.5);
    % y_tilted_line = -0.02 + tand(45) * x_line;
    % plot(x_line, y_tilted_line, 'g--', 'LineWidth', 1);
    % Plot tilted lines
    % for x_int = -w_xm:1:w_xm + 4
    %     y_start = -0.12;
    %     y_end = 0;
    %     x_tilted_line = [x_int, x_int + (y_end - y_start) / tand(45)];
    %     y_tilted_line = [y_start, y_end];
    %     plot(x_tilted_line, y_tilted_line, 'k--', 'LineWidth', 1);
    % end
    hold off;

    % Set axis limits
    newLimits= [w_xm - 2, w_xm + 2];
    xlim(newLimits);
    ylim([-0.4 1]);
    grid on;
    axis equal;

    % Title and labels
    
    xlabel('Position X asix (m)');
    ylabel('Height (m)');



legend('Model', 'Sindy');
    % % Pause for animation
    title('t: ',num2str(i*dt))
    % hold on
    pause(dt);
    % hold on
    
end
% close(videoFile);
disp('Animation Over Prediction phase ended')


