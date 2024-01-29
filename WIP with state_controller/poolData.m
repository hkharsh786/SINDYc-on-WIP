%% 
%% 
function yout = poolData(yin,nVars,polyorder,usesine)
% Copyright 2015, All Rights Reserved
% Code by Steven L. Brunton
% For Paper, "Discovering Governing Equations from Data: 
%        Sparse Identification of Nonlinear Dynamical Systems"
% by S. L. Brunton, J. L. Proctor, and J. N. Kutz
% parameters

%%%the useline library has beeen modified by Kumar Harsh
%%the part is done as a part of Master thesis: Nonlinear System Identification and
% Control using Sparse Identification of Nonlinear Dynamics for Model Predictive Control (SINDYc) with Evaluation on EvoBot


Jr = 0.003836154286559; % Jr

m1 = 1.5;       % wheel mass

m2 = 14.5;      % pendulum mass

r  = 0.1;       % radius wheel

g  = 9.81;      % gravity of earth

L  = 0.58;      % pendulum length

d1 = 0.2;      % cart damping

d2 = 0.2;     % pend damping

C1= (Jr/(2*pi*r)) + m1 ;
C2= (m1+m2+Jr/(2*pi*r));


n = size(yin,1);
% yout = zeros(n,1+nVars+(nVars*(nVars+1)/2)+(nVars*(nVars+1)*(nVars+2)/(2*3))+11);

ind = 1;



% poly order 0
yout(:,ind) = ones(n,1);
ind = ind+1;

% poly order 1
for i=1:nVars
    yout(:,ind) = yin(:,i);
    ind = ind+1;
    
end

if(polyorder>=2)
    % poly order 2
    for i=1:nVars
        for j=i:nVars
            yout(:,ind) = yin(:,i).*yin(:,j);
            ind = ind+1;
        end
    end
end

if(polyorder>=3)
    % poly order 3
    for i=1:nVars
        for j=i:nVars
            for k=j:nVars
                yout(:,ind) = yin(:,i).*yin(:,j).*yin(:,k);
                ind = ind+1;
            end
        end
    end
end

if(polyorder>=4)
    % poly order 4
    for i=1:nVars
        for j=i:nVars
            for k=j:nVars
                for l=k:nVars
                    yout(:,ind) = yin(:,i).*yin(:,j).*yin(:,k).*yin(:,l);
                    ind = ind+1;
                end
            end
        end
    end
end

if(polyorder>=5)
    % poly order 5
    for i=1:nVars
        for j=i:nVars
            for k=j:nVars
                for l=k:nVars
                    for m=l:nVars
                        yout(:,ind) = yin(:,i).*yin(:,j).*yin(:,k).*yin(:,l).*yin(:,m);
                        ind = ind+1;
                    end
                end
            end
        end
    end
end



if(usesine)

    % for i=[2, 4]
    %     yout(:,ind)=sin(yin(:,i));
    %       ind = ind+1;
    % end
    % for i=[2, 3 ,4]
    %     yout(:,ind)=cos(yin(:,i));
    %     ind = ind+1;
    % end
    % 
    % 
    % 
    % for i=[2, 4]
    %     yout(:,ind)=sin(yin(:,i)).^2;
    %     ind = ind+1;
    % end
    % 
    % % for i=[2, 4]
    % %     yout(:,ind)=cos(yin(:,i)).^2;
    % %     ind = ind+1;
    % % end
    % 
    % for i=[2, 4]
    %     for j=1:nVars
    %         yout(:,ind)=yin(:,i).^2.*sin(yin(:,j)).*cos(yin(:,j));
    %         ind = ind+1;
    %     end
    % end
    % 
    % for i=4:4
    %     for j=[2, 4]
    %         yout(:,ind)=yin(:,i).^2.*sin(yin(:,j));
    %         ind = ind+1;
    %     end
    % end
    % 
    % for i=4:4
    %     for j=1:nVars
    %         yout(:,ind)=yin(:,i).^2.*cos(yin(:,j));
    %         ind = ind+1;
    %     end
    % end
    % %
    % for i=4:4
    %     for j=[2, 4]
    %         yout(:,ind)=sin(yin(:,i)).*cos(yin(:,j));
    %         ind = ind+1;
    %     end
    % end




    % 
    % 
    % 
    % 


    % 
    % for k=1:1
    %     for p=3:3
    % 
    %             yout = [yout...
    %                 yin(:,p-1)./(C1+ (m2*sin(k*yin(:,p)).*sin(k*yin(:,p))))...
    %                 yin(:,p-1)./(m2*L*(cos(k*yin(:,p)).*cos(k*yin(:,p))) - C2*L)... 
    %                 ...
    %                 sin((k*yin(:,p))).*cos(k*yin(:,p))./(C1+ (m2*sin(k*yin(:,p)).*sin(k*yin(:,p))))...
    %                 sin((k*yin(:,p))).*yin(:,p+1).*yin(:,p+1)./(C1+ (m2*sin(k*yin(:,p)).*sin(k*yin(:,p))))...
    %                 cos(k*yin(:,p)).*yin(:,p+1)./(C1+ (m2*sin(k*yin(:,p)).*sin(k*yin(:,p))))...
    %                 sin((k*yin(:,p)))./(m2*L*(cos(k*yin(:,p)).*cos(k*yin(:,p))) - C2*L)...
    %                 sin((k*yin(:,p))).*cos(k*yin(:,p)).*yin(:,p+1).*yin(:,p+1)./(m2*L*(cos(k*yin(:,p)).*cos(k*yin(:,p))) - C2*L)...
    %                 ...
    %                 yin(:,p+1)./(m2*L*(cos(k*yin(:,p)).*cos(k*yin(:,p))) - C2*L)...
    %                 ...
    %                 yin(:,p+2)./(C1+ (m2*sin(k*yin(:,p)).*sin(k*yin(:,p))))...
    %                 yin(:,p+2).*cos(k*yin(:,p))./(m2*L*(cos(k*yin(:,p)).*cos(k*yin(:,p))) - C2*L)...
    %             ];
    %     end
    % end





    %% type2


    for k=1:1
        for p=2:3

                yout = [yout...
                    yin(:,p-1)./(C1+ (m2.*sin(k*yin(:,p)).*sin(k*yin(:,p))))... %x2/D1
                    yin(:,p-1)./(m2.*L.*(cos(k*yin(:,p))) - (C2.*L./(cos(k*yin(:,p)))))... %x2/D2
                    ...
                    sin((k*yin(:,p))).*cos(k*yin(:,p))./(C1+ (m2.*sin(k*yin(:,p)).*sin(k*yin(:,p))))... %sincos/D1
                    sin((k*yin(:,p))).*yin(:,p+1).*yin(:,p+1)./(C1+ (m2.*sin(k*yin(:,p)).*sin(k*yin(:,p))))...%sinx3*x4^2/D1
                    cos(k*yin(:,p)).*yin(:,p+1)./(C1+ (m2.*sin(k*yin(:,p)).*sin(k*yin(:,p))))... %cosx3.x4/D1
                    (sin((k*yin(:,p)))./cos(k*yin(:,p)))./(m2.*L.*(cos(k*yin(:,p))) - (C2.*L./(cos(k*yin(:,p)))))... %(sinx3/cosx3)/D2
                    sin((k*yin(:,p))).*yin(:,p+1).*yin(:,p+1)./(m2.*L.*(cos(k*yin(:,p))) - (C2.*L./(cos(k*yin(:,p)))))... %sinx3*x4^2/D2
                    ...
                    (yin(:,p+1)./cos(k*yin(:,p)))./(m2.*L.*(cos(k*yin(:,p))) - (C2.*L./(cos(k*yin(:,p)))))...%(x4/cosx3)/D2
                    ...
                    yin(:,p+2)./(C1+ (m2.*sin(k*yin(:,p)).*sin(k*yin(:,p))))... %U/D1
                    yin(:,p+2)./(m2.*L.*(cos(k*yin(:,p))) - (C2.*L./(cos(k*yin(:,p)))))... %U/D2
                ];
        end
    end


end









