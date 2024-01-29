clear all, close all 
clc
figpath = './FIGURES/wipsimplified/';mkdir(figpath)
datapath = './DATA/wipsimplified/'; mkdir(datapath)


SystemModel = 'wipts0.01';
% global Ad Bd Cd Dd; %used for linear system in latter stage

%% Generate Data 
InputSignalType = 'torque'; %'torque'; %sphs'; %prbs; chirp; noise; sine2; sphs; mixed
ONLY_TRAINING_LENGTH = 1; % since no nise corruption
ENSEMBLE_DATA = 0;
getTrainingData
u = u'; uv=uv';

%% SINDYc
% Parameters
ModelName = 'SINDYc'; iModel = 2;
Nvar = 4;
polyorder = 1;
usesine = 1;
% lambda=10;
lambda_vec = [0.1,1,0.1,1]; %can be defined as vec tor as per no of states
eps = 0;  

%Add noise
%eps = 0.05*std(x(:,1));
%x = x + eps*randn(size(x));
            
trainSINDYcEmil


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

% Show validation -------- shifted at end of annimation
clear ph
figure,box on,
ccolors = get(gca,'colororder');
ccolors_valid = [ccolors(1,:)-[0 0.2 0.2];ccolors(2,:)-[0.1 0.2 0.09];ccolors(3,:)-[0.1 0.2 0.09];ccolors(4,:)-[0.1 0.1 0.2]];
for i = 1:Nvar
    ph(i) = plot(tspan,x(:,i),'-','Color',ccolors(i,:),'LineWidth',1); hold on
end
for i = 1:Nvar
    ph(Nvar+i) = plot(tspan,xSINDYc(:,i),'.-.','Color',ccolors_valid(i,:),'LineWidth',2);
end
ylim([ -50, 50]);
xlabel('Time')
ylabel('xi')
legend(ph([1,2,3,4,5]),'Wheel Position', 'Wheel Velocity', 'Angle', 'Angular Velocity',ModelName)
set(gca,'LineWidth',1, 'FontSize',14)
set(gcf,'Position',[100 100 300 200])
set(gcf,'PaperPositionMode','auto')

% print('-depsc2', '-loose', '-cmyk', [figpath,'EX_',SystemModel,'_SI_',ModelName,'_',InputSignalType,'.eps']);

%% 

%% animation plot

% simulatedWIP


%% Show validation
disp('Showing validation Over Prediction phase started')

clear ph
figure(),box on,
ccolors = get(gca,'colororder');
ccolors_valid = [ccolors(1,:)-[0 0.2 0.2];ccolors(2,:)-[0.1 0.2 0.09];ccolors(3,:)-[0.1 0.2 0.09];ccolors(4,:)-[0.1 0.1 0.2]];
for i = 1:Nvar
    ph(i) = plot(tspan,x(:,i),'-','Color',ccolors(i,:),'LineWidth',1); hold on
end
for i = 1:Nvar
    ph(Nvar+i) = plot(tspan,xSINDYc(:,i),'.-.','Color',ccolors_valid(i,:),'LineWidth',2);
end
ylim([ -50, 50]);
xlabel('Time')
ylabel('xi')
legend(ph([1,2,3,4,5]),'Wheel Position', 'Wheel Velocity', 'Angle', 'Angular Velocity',ModelName)



% 

%% Prediction

disp('Showing  Prediction over future')


tspanV   = [100:dt:200];
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
