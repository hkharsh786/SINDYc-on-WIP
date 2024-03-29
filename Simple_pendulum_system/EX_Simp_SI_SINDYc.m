clear, close all, clc
figpath = './FIGURES/simP/';
datapath = './DATA/simP/';
mkdir(figpath)
mkdir(datapath)

SystemModel = 'pendulum_system dt0.02';

%% Generate Data 
InputSignalType = 'input2'; %torque or input2
ONLY_TRAINING_LENGTH = 1;
getTrainingData
u = u'; uv=uv';

%% SINDYc
% Parameters
ModelName = 'SINDYc'; 
% iModel = 2;
Nvar = 2;
polyorder = 1;
usesine = 1;
lambda = 1;
eps = 0;

%Add noise
% eps = 0.05*std(x(:,1));
% x = x + eps*randn(size(x));
            
trainSINDYcSP
%% 

%%Prediction  over training phase 


disp('Prediction  over training phase started')

if any(strcmp(InputSignalType,{'sine2', 'chirp','prbs', 'sphs'})==1)
    [tSINDYc,xSINDYc]=ode45(@(t,x)sparseGalerkinControl(t,x,forcing(x,t),Xi(:,1:Nvar),polyorder,usesine),tspan,x0,options);

else
    p.ahat = Xi(:,1:Nvar);
    p.polyorder = polyorder;
    p.usesine = usesine;
    p.dt = dt;
    [N,Ns] = size(x);
    xSINDYc = zeros(Ns,N); xSINDYc(:,1) = x0';
    for ct=1:N-1
        xSINDYc(:,ct+1) = rk4u(@sparseGalerkinControl_Discrete,xSINDYc(:,ct),u(ct),dt,1,[],p);
    end
    xSINDYc = xSINDYc';
end

disp('Prediction  over training phase completed')

disp('Showing validation Over Prediction phase started')

clear ph
figure(4),box on,
ccolors = get(gca,'colororder');
ccolors_valid = [ccolors(1,:)-[0 0.2 0.2];ccolors(2,:)-[0.1 0.2 0.09]];
for i = 1:Nvar
    ph(i) = plot(tspan,x(:,i),'-','Color',ccolors(i,:),'LineWidth',1); hold on
end
for i = 1:Nvar
    ph(Nvar+i) = plot(tspan,xSINDYc(:,i),'.-.','Color',ccolors_valid(i,:),'LineWidth',2);
end
ylim([ -40, 60]);
xlabel('Time')
ylabel('xi')
title('System Prediction over Training phase');
legend(ph([1,2,3,4]),'M-Angle', 'M-Angular Velocity','S-Angle', 'S-Angular Velocity')
%% future prediction
disp('Showing  Prediction over future')


tspanV   = [10:dt:20];
xA      = xv;
tA      = tv;

% Model
if any(strcmp(InputSignalType,{'sine2','chirp','prbs', 'sphs'})==1)
    [tB,xB]=ode45(@(t,x)sparseGalerkinControl(t,x,forcing(x,t),Xi(:,1:Nvar),polyorder,usesine),tspanV,x(end,:),options);  % approximate
    xB = xB(2:end,:);
    tB = tB(2:end); 
else
    [N,Ns] = size(xA);
    xB = zeros(Ns,N); xB(:,1) = x(end,:)';
    for ct=1:N
        xB(:,ct+1) = rk4u(@sparseGalerkinControl_Discrete,xB(:,ct),uv(ct),dt,1,[],p);
    end
    xB = xB(:,1:N+1)';
    tB = tspanV(1:end);
end
u = u';
VIZ_SI_Validation

%% Data saving

Model.name = 'SINDYc';
Model.polyorder = polyorder;
Model.usesine = usesine;
Model.Xi = Xi;
Model.dt = dt;
save(fullfile(datapath,['EX_',SystemModel,'_SI_',ModelName,'_',InputSignalType,'.mat']),'Model')