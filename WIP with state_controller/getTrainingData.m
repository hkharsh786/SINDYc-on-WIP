%% Execute this file to collect training data for model identification
disp('Training started')
% Parameters: Model
n = 4; %no of states
% x0=[0.1;0;0.5;0];  % Initial condition
% x0=[0;0;0.3;1];  % Initial condition
x0=[0;0;0;0];  % Initial condition

dt = 0.002;  % Time step 

if ONLY_TRAINING_LENGTH == 1
    % Without noise-corruption
    tspan=[0:dt:20];
    Ntrain = (length(tspan)-1)/2+1;
else
    % If Noise-corruption
    tspan=[0:dt:800];
    Ntrain = 3*(length(tspan)-1)/4+1;
end


options = odeset('RelTol',1e-10,'AbsTol',1e-10*ones(1,n));

switch InputSignalType
    case 'torque'
        disp('Continious Time torque input Training Running')
        A = 15;
        forcing = @(x,t) [(A*(sin(5*t) -sin(0.5*t)))];
        [t,x]=ode45(@(t,x) WIP_CT_ode(t,x,forcing(x,t)),tspan,x0,options);
        u = forcing(0,tspan);



    case 'type4ct'
        disp('Continious Time Angle input Training Running')
        uk=[];
        u = @sim_controller;
       [t,x]=ode45(@(t,x) WIP_CT_ode(t,x,u(x,t)),tspan,x0,options);
        for i = 1:length(tspan)
            t1 = tspan(i);
            u_val = u(x, t1);
            uk = [uk, u_val];
        end
        u=uk;

    case 'type4dtv2'
        disp('DiscreteTime Angle input Training Running')
        uk=0;
        uk1=[];
        x(:,1) = x0;
        g = 150;
        f1=20;
        f2=4;
        % for i= 1:length(tspan)
        %     if i>2
        %         uk = g*((x(3,k)-10*pi/180)*f1 + x(4)*f2);%170;
        %     end
        %     if i>length(tspan)/10
        %         uk = g*((x(3,k)+10*pi/180)*f1 + x(4)*f2);%170;
        %     end
        %     if i>2*length(tspan)/10
        %         uk = g*((x(3)-10*pi/180)*f1 + x(4)*f2);%170;
        %     end
        %     if i>3*length(tspan)/10
        %         uk = g*((x(3)+20*pi/180)*f1 + x(4)*f2);%170;
        %     end
        %     if i>4*length(tspan)/10
        %         uk = g*((x(3)-20*pi/180)*f1 + x(4)*f2);%170;
        %     end
        %     if i>5*length(tspan)/10
        %         uk = g*((x(3)+30*pi/180)*f1 + x(4)*f2);%170;
        %     end
        %     if i>6*length(tspan)/10
        %         uk = g*((x(3)-30*pi/180)*f1 + x(4)*f2);%170;
        %     end
        %     if i>7*length(tspan)/10
        %         uk = g*((x(3)-20*pi/180)*f1 + x(4)*f2);%170;
        %     end
        %     if i>8*length(tspan)/10
        %         uk = g*((x(3)-0*pi/180)*f1 + x(4)*f2);%170;
        %     end
        %     if i>9*length(tspan)/10
        %         uk = g*((x(3)-10*pi/180)*f1 + x(4)*f2);%170;
        %     end 
        %     % uk=15*(sin(0.05*i) +sin(0.5*i));
        % 
        %     [x(:,i+1), yk]= WIP_De(x(:,i), uk, dt);   
        set_pitch=0;
        for k = 1:length(tspan)
            if k>100
            set_pitch = 10*pi/180;
            end
            if k>2000
            set_pitch = 30*pi/180;
            end
            if k>3000
            set_pitch = 15*pi/180;
            end
            if k>4000
            set_pitch = -5*pi/180;
            end
            if k>5000
            set_pitch = -30*pi/180;
            end
            if k>6000
            set_pitch = -10*pi/180;
            end
            if k>7000
            set_pitch = 10*pi/180;
            end
            if k>8000
            set_pitch = 15*pi/180;
            end
            if k>9000
            set_pitch = 0*pi/180;
            end
            uk = g*((-set_pitch + x(3,k))*f1 + x(4,k)*f2);
            
            [x(:,k+1), yk] = WIP_De(x(:,k), uk, dt);

            uk1 = [uk1, uk];
        end
        x=x';
        u=uk1;
        t=tspan';


    case 'type4dt'
        disp('DiscreteTime Angle input Training Running')
        uk = g*((-set_pitch + x(3,k))*10 + x(4,k)*1);
             
                
        for i = 1:length(tspan)
            t1 = tspan(i);
            u_val = u(x, t1);
            x=x + dt*WIP_CT_odeDe(x,u_val);
            xk=[xk,x];
            uk = [uk, u_val];
        end
        x=xk';
        u=uk;
        t=tspan';


    case 'type4'
        
        tic;
        disp('DiscreteTime Frame Training Running')
        x=x0;
        uk=[];
        ts=0.002;
        xk=[];
        set_pitch_req=0*pi/180;
        setK=[];
        gain = 600;
        f1=0.1;
        f2=0.01;
        u = @sim_controller;
        for i = 1:length(tspan)
            
            % if i >= length(tspan)/10 && i < 2*length(tspan)/10
            %     set_pitch_req = 10*pi/180;
            % elseif i >= 2*length(tspan)/10 && i < 3*length(tspan)/10
            %     set_pitch_req =-10*pi/180;
            % elseif i >= 3*length(tspan)/10 && i < 4*length(tspan)/10
            %     set_pitch_req = 0*pi/180;
            % elseif i >= 4*length(tspan)/10 && i < 5*length(tspan)/10
            %     set_pitch_req = 20*pi/180;
            % elseif i >= 5*length(tspan)/10 && i < 6*length(tspan)/10
            %     set_pitch_req =-20*pi/180;
            % elseif i >= 6*length(tspan)/10 && i < 7*length(tspan)/10
            %     set_pitch_req = -30*pi/180;
            % elseif i >= 7*length(tspan)/10 && i < 8*length(tspan)/10
            %     set_pitch_req = 0;
            % elseif i >= 8*length(tspan)/10 && i < 9*length(tspan)/10
            %     set_pitch_req =30*pi/180;
            % elseif i >=9*length(tspan)/10
            %     set_pitch_req =0;
            % end
                
            % u1 = (gain*((x(3)-set_pitch_req)*f1-x(4)*f2));
            t1 = tspan(i);
            u1 = u(x, t1);
            x=x + ts*WIP_CT_odeDe(x,u1);
            % forcing(x, tspan)
            xk=[xk,x];
            uk = [uk,u1];
            % setK=[setK,set_pitch_req];
            % disp('DiscreteTime Frame Training Running')
            % loop_left=length(tspan)-i
            
        end
        elapsedTime = toc
        x=xk';
        t=tspan';
        u=uk;
        setK=setK';
   


    case 'type4a'
        tic;
        x=x0;
        u_last=0;
        u=[];
        ts=0.002;
        xk=[];
        set_pitch_req=pi/6;
        setK=[];
        gain = 750;
        f1=1;
        f2=0.1;
        for i = 1:length(tspan)
            if i > length(tspan)/10
                set_pitch_req = pi/6;
            elseif i >= 2*length(tspan)/10
                set_pitch_req = 0;
            elseif i >= 3*length(tspan)/10 
                set_pitch_req =-pi/6;          
            elseif i >=4*length(tspan)/10
                set_pitch_req =0;
            end
            % set_pitch_req=0.001;     
            max_inc = ts*45*pi/180; 
            set_pitch = max(min(set_pitch_req,u_last + max_inc),u_last - max_inc); 
            u_last = set_pitch;
            u1 = (gain*((x(3)-set_pitch_req)*f1-x(4)*f2));
            x=x + ts*WIP_CT_odeDe(x,u1);            
            xk=[xk,x];
            u = [u,u1];
            setK=[setK,set_pitch_req];
            % disp('DiscreteTime Frame Training Running')
            % loop_left=length(tspan)-i
            
        end
        elapsedTime = toc
        x=xk';
        t=tspan';
        setK=setK';



        
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

    % if exist('type4') == 1
    %     setKv = setK(Ntrain+1:end);
    %     setK = setK(1:Ntrain);
    % end
    % 
    tspanv = tspan(Ntrain+1:end);
    tspan = tspan(1:Ntrain);
    disp('Training complete')
%% Grphs

    figure(1);
    plot(tspan,u)
    xlabel('Time')
    ylabel('Input ')
    title('Input Applied');

    figure(2);
    plot(t,x,'LineWidth',1.5)
    ylim([-25 25]);
    xlabel('Time')
    ylabel('xi')
    legend('Wheel Position', 'Wheel Velocity','Angle', 'Angular Velocity')
    title('Original ODE states');
    set(gca,'LineWidth',1, 'FontSize',14)
    set(gcf,'Position',[100 100 300 200])
    set(gcf,'PaperPositionMode','auto')
    % 
    
    % if exist('type4') == 1
    %     figure(3);
    %     plot(tspan,setK)
    %     xlabel('Time')
    %     ylabel('set angle Required ')
    %     title('setAngle');
    % end

function u = sim_controller(x,t)
        g = 600;
        f1=1;
        f2=0.1;
    %     u = g*(x(3)*1000 + x(4)*100);%170;
        u = g*(x(3)*f1 + x(4)*f2);%170;
    %     u = g*(x(3)*10 + x(4)*1 + x(2));%170;
    
    %     v_cog = x(4)*0.55 * cos(x(3)) + x(2);
    %     s = 0.2*v_cog ;
    %     if t>1
    %     u = g*((x(3) + s)*10 + x(4)*1);%170;
    %     end
    
    %     v_cog = x(4)*0.55 * cos(x(3)) + x(2);
    %     if t>1
    %     u = g*(x(3)*10 + x(4)*1 + (v_cog - 1)*1);%170;
    %     end
    
        if t>0
        u = g*((x(3)-10*pi/180)*f1 + x(4)*f2);%170;
        end
        if t>2
        u = g*((x(3)+10*pi/180)*f1 + x(4)*f2);%170;
        end
        if t>4
        u = g*((x(3)-10*pi/180)*f1 + x(4)*f2);%170;
        end
        if t>6
        u = g*((x(3)+20*pi/180)*f1 + x(4)*f2);%170;
        end
        if t>8
        u = g*((x(3)-20*pi/180)*f1 + x(4)*f2);%170;
        end
        if t>10
        u = g*((x(3)+30*pi/180)*f1 + x(4)*f2);%170;
        end
        if t>12
        u = g*((x(3)-30*pi/180)*f1 + x(4)*f2);%170;
        end
        if t>14
        u = g*((x(3)-20*pi/180)*f1 + x(4)*f2);%170;
        end
        if t>16
        u = g*((x(3)-0*pi/180)*f1 + x(4)*f2);%170;
        end
        if t>18
        u = g*((x(3)-10*pi/180)*f1 + x(4)*f2);%170;
        end
end