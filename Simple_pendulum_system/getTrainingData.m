%% Execute this file to collect training data for model identification

% Parameters: Model
n = 2; %no of states
x0=[0;0];  % Initial condition

dt = 0.01;  % Time step


if ONLY_TRAINING_LENGTH == 1
    % Without noise-corruption
    tspan=[0:dt:20];
    Ntrain = (length(tspan)-1)/2+1;
else
    % If Noise-corruption
    tspan=[0:dt:40];
    Ntrain = 3*(length(tspan)-1)/4+1;
end


options = odeset('RelTol',1e-10,'AbsTol',1e-10*ones(1,n));

switch InputSignalType
    case 'torque'
        A = 100;
        forcing = @(x,t) [(A*(sin(1*pi*t)))];
        [t,x]=ode45(@(t,x) pendulum_system(t,x,forcing(x,t)),tspan,x0,options);
        u = forcing(0,tspan);

    case 'input2'
        A=5;
        B = 10;
        forcing = @(x,t) [(A +B*(sin(1*pi*t)))];
        [t,x]=ode45(@(t,x) pendulum_system(t,x,forcing(x,t)),tspan,x0,options);
        u = forcing(0,tspan);
end
%%  Split into training and validation data set

    
    xv = x(Ntrain+1:end,:);
    x = x(1:Ntrain,:);
    
    uv = u(Ntrain+1:end);
    u = u(1:Ntrain);
    
    tv = t(Ntrain+1:end);
    t = t(1:Ntrain);
    
    tspanv = tspan(Ntrain+1:end);
    tspan = tspan(1:Ntrain);

    figure;
    plot(tspan,u)
    title('Applied Input');
    xlabel('Time (in Sec)')
    ylabel('Input Applied ')
    
    figure;
    plot(t,x,'LineWidth',1.5)
    xlabel('Time')
    ylabel('xi')
    title('Original System Behaviour From Applied Input');
    legend('Angle','Angular velocity')
    set(gca,'LineWidth',1, 'FontSize',14)
    set(gcf,'Position',[100 100 300 200])
    set(gcf,'PaperPositionMode','auto')
    