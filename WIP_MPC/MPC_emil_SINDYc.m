% MPC applied to WIP system using a SINDYc model.
% Function for TESTING
%load final_results.mat mpc result directly from data folder

clear all, close all, clc
figpath = './FIGURES/emil_mpc/'; mkdir(figpath)
datapath = './DATA/emil_generalized/'; 

SystemModel = 'emil_generalized';
ModelName = 'SINDYc';

%% Load Model
Nvar = 4;
InputSignalTypeModel = 'type4ct'; % prbs; chirp; noise; sine2; sphs; mixed
load(fullfile(datapath,['EX_',SystemModel,'_SI_',ModelName,'_',InputSignalTypeModel,'.mat'])) 



%% Apply Model predictive control to system using SINDYc model
select_model = 'SINDYc';
pest.ahat = Model.Xi(:,1:Nvar); % Use Xi0(:,1:Nvar) to execute true model parameters
pest.polyorder = Model.polyorder;
pest.usesine = Model.usesine;
pest.dt = 0.002; 

% Parameters MPC
options = optimoptions('fmincon','Algorithm','sqp','Display','none', ...
    'MaxIterations',100);

Duration = 5;             % Run for 'Duration' time units
x_B= 45; %goal x position
angleref=10;
Ton = 0;                         % Time units when control is turned on   
x0n = [0,0,0,0]'; % Initial condition
Ts  = pest.dt;                   % Sampling time [sec]
Tcontrol = 0;
getMPCparams    

% xref=[0,0, 0, 0];


% Initialize variables
Nt = (Duration/Ts)+1;
uopt0    = 0.00;
xhat     = x0n;
uopt     = uopt0.*ones(Nu,1);
xHistory = zeros(Nvar,Nt); xHistory(:,1) = xhat;
uHistory = zeros(1,Nt); uHistory(1)   = uopt(1);
tHistory = zeros(1,Nt); tHistory(1)   = Tcontrol;
rHistory = zeros(Nvar,Nt);

%%
% Start simulation
fprintf('Simulation started.  It might take a while...\n')
tic
for ct = 1:(Duration/Ts)   % For each iteration: take measurements & optimize control input & apply control input
    % Set references over optimization horizon
    % tref = (ct:ct+N-1).*Ts;
    % [xref, x_trajectory]=x_ref(tref,Duration, x_B);
    if (ct*Ts)> (3*Duration/8)
        angleref=0;
    end
    if (ct*Ts)> (5*Duration/8)
        angleref=-10;
    end
    if (ct*Ts)> (7*Duration/8)
        angleref=0;
    end   
    xref=[x_B,0,angleref*pi/180,0];
    
    % NMPC with full-state feedback
    COSTFUN = @(u) ObjectiveFCN(u,xhat,N,Nu,xref,uHistory(:,ct),pest,diag(Q),R,Ru,select_model);
    % CONSFUN = @(u) ConstraintFCN(u,uHistory(:,ct),xhat,N,LBo,UBo,LBdu,UBdu,pest);
    CONSFUN = @(u) ConstraintFCN_models(u,uHistory(:,ct),xhat,N,LBo,UBo,LBdu,UBdu,pest,select_model);
    uopt = fmincon(COSTFUN,uopt,[],[],[],[],LB,UB,CONSFUN,options);
    
    
    % Integrate system
    xhat = rk4u(@WIP_CT_ode,xhat,uopt(1),Ts/2,2,[],0); 
    xHistory(:,ct+1) = xhat;
    uHistory(:,ct+1) = uopt(1);
    tHistory(:,ct+1) = ct*Ts+Tcontrol; 
    rHistory(:,ct+1) = xref;
    
    if mod(ct,1000) == 0
        disp(['PROGRESS: ',num2str(100*ct/(Duration/Ts)),'%'])
    end
    
    
end
fprintf('Simulation finished!\n')
toc
objectiveFCNscore=evalObjectiveFCN(uHistory, xHistory,rHistory,diag(Q), R, Ru);

%% Show results
clear ph

figure();
plot(tHistory, xHistory(1, :), tHistory, rHistory(1, :));
title('Position in meters');
xlabel('Time (seconds)');
ylabel('Position (meters)');
legend('Actual Position', 'Reference Position');

figure();
%velocity
plot(tHistory, xHistory(2, :), tHistory, rHistory(2, :));
title('Velocity in meters/sec');
xlabel('Time (seconds)');
ylabel('Velocity (m/s)');
legend('Actual Velocity', 'Reference Velocity');

%Angle
figure();
plot(tHistory, xHistory(3, :)*180/pi, tHistory, rHistory(3, :)*180/pi);
title('Angle in Deg');
xlabel('Time (seconds)');
ylabel('Angle (Deg)');
legend('Actual Angle', 'Reference Angle');

%Angluar Velocity
figure();
plot(tHistory, xHistory(4, :)*180/pi, tHistory, rHistory(4, :)*180/pi);
title('Anglular Velocity in Deg/sec');
xlabel('Time (seconds)');
ylabel('Anglular Velocity (Deg/Sec)');
legend('Actual Anglular Velocity', 'Reference Anglular Velocity');

%Applied Input
figure();
plot(tHistory, uHistory);
title('MPC Generated Optimal Input');
xlabel('Time (seconds)');
ylabel('Torque');


%Applied cost
figure();
plot(tHistory, objectiveFCNscore');
title('Anglular Velocity in Deg/sec');
xlabel('Time (seconds)');
ylabel('Tracking Cost');



% 
%% Save Results
Results.t = tHistory;
Results.x = xHistory;
Results.u = uHistory;
Results.J = evalObjectiveFCN(uHistory,xHistory,rHistory,diag(Q),R,Ru);

