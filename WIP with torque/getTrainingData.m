%% Execute this file to collect training data for model identification
disp('Training started')
% Parameters: Model
n = 4; %no of states
x0=[0.1;0;0.5;0];  % Initial condition
% x0=[0;0;0.3;1];  % Initial condition
% x0=[0;0;0;0];  % Initial condition

dt = 0.001;  % Time step


if ONLY_TRAINING_LENGTH == 1
    % Without noise-corruption
    tspan=[0:dt:200];
    Ntrain = (length(tspan)-1)/2+1;
else
    % If Noise-corruption
    tspan=[0:dt:800];
    Ntrain = 3*(length(tspan)-1)/4+1;
end


options = odeset('RelTol',1e-10,'AbsTol',1e-10*ones(1,n));

switch InputSignalType
    case 'torque'
        A = 50; 
        forcing = @(x,t) [-1.5.^(-t)+(A*(sin(3*t) -sin(0.5*t)))]; 
        [t,x]=ode45(@(t,x) WIP_CT_ode(t,x,forcing(x,t)),tspan,x0,options);
        u = forcing(0,tspan);

    case 'sine2'
        A = 200;
        forcing = @(x,t) [-0.1+(A*(sin(1*t)+sin(.1*t))).^2];
        [t,x]=ode45(@(t,x) WIP_CT_ode(t,x,forcing(x,t)),tspan,x0,options);
        u = forcing(0,tspan);
    case 'sine3'
            forcing = @(x,t) (.5*sin(5*t).*sin(.5*t)+0.1).^3;
            [t,x]=ode45(@(t,x) WIP_CT_ode(t,x,forcing(x,t)),tspan,x0,options);
            u = forcing(0,tspan);
    % case 'chirp'
    %     A = .5;
    %     forcing = @(x,t) A*chirp(t,[],max(tspan),0.1).^2;
    %     [t,x]=ode45(@(t,x) WIP_CT_ode(t,x,forcing(x,t)),tspan,x0,options);
    %     u = forcing(0,tspan);
    % 
    % case 'prbs'
    %     A = 0.05236; 
    %     taulim = [1 8];
    %     states = [-0.5:0.25:0.5];
    %     Nswitch = 4000;
    %     forcing = @(x,t) A*prbs(taulim, Nswitch, states, t,0);
    % 
    %     [t,x]=ode45(@(t,x) WIP_CT_ode(t,x,forcing(x,t)),tspan,x0,options);
    % 
    %     u = zeros(size(tspan));
    %     for i = 1:length(tspan)
    %         u(i) = forcing(0,tspan(i));
    %     end
    %     figure,plot(tspan,u)
    % 
    % case 'sphs'
    %     Pf = 10;% Fundamental period
    %     K = 2; 
    %     A = 0.1;
    %     forcing = @(x,t) A*sphs(Pf,K,t);
    %     [t,x]=ode45(@(t,x) WIP_CT_ode(t,x,forcing(x,t)),tspan,x0,options);
    %     u = forcing(0,tspan);
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
    disp('Training complete')
%% Grphs

    figure(1);
    plot(tspan,u)
    xlabel('Time')
    ylabel('Input Torque')
    title('Input Applied');

    figure(2);
    plot(t,x,'LineWidth',1.5)
    xlabel('Time')
    ylabel('xi')
    legend('Wheel Position', 'Wheel Velocity','Angle', 'Angular Velocity')
    title('Original ODE states');
    set(gca,'LineWidth',1, 'FontSize',14)
    set(gcf,'Position',[100 100 300 200])
    set(gcf,'PaperPositionMode','auto')