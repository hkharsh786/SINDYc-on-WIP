%% modified with sindypi tech
%% 
function yout = poolData1(yin,nVars,polyorder,usesine)

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

for states= 1:nVars
    if(states==1)
        % poly order 0
        % yout(:,ind) = ones(n,1);
        % ind = ind+1;

        % poly order 1
        for i=2:2
            yout(:,ind) = yin(:,i);
            ind = ind+1;
        end

    end
%% 

    if(states==2)
        % poly order 0
        % yout(:,ind) = ones(n,1);
        % ind = ind+1;
                % poly order 1 for u and x2
        for i=[2,5]
            yout(:,ind) = yin(:,i);
            ind = ind+1;
        end
        %additing trignometry:

        %for cos(x3)*x4, sin(x3).x4^2, sin(x3).cos(x3)
        for i=3:3
            for j=4:4
                yout(:,ind) = cos(yin(:,i)).*yin(:,j);
                ind = ind+1;
                yout(:,ind) = sin(yin(:,i)).*(yin(:,j).^2);
                ind = ind+1;
       
            end
        end
        
       % for sin^2 in denominator
       for i=3:3
           yout(:,ind) = sin(yin(:,i)).*cos(yin(:,i));
           ind = ind+1;
           yout(:,ind) = sin(yin(:,i)).^2;
           ind = ind+1;
        end

    end
%% 

    if(states==3)
        % poly order 0
        % yout(:,ind) = ones(n,1);
        % ind = ind+1;

        % poly order 1
        for i=4:4 %1:nVars
            yout(:,ind) = yin(:,i);
            ind = ind+1;
        end

    end
%% 

    if(states==4)
        % poly order 1
        for i=[2,5]
            yout(:,ind) = yin(:,i);
            ind = ind+1;
        end
        
        %additing trignometry:

        %for cos(x3)*x4, sin(x3).x4^2, sin(x3).cos(x3)
        for i=3:3 %1:nVars
            for j=4:4
                %nummerator part
                yout(:,ind) = sin(yin(:,i)).*(yin(:,j).^2);
                ind = ind+1;
                yout(:,ind) = yin(:,j)./(cos(yin(:,i)));
                ind = ind+1;                
            end
        end
        %denominator part
        for i=3:3
            yout(:,ind) = sin(yin(:,i))./cos(yin(:,i));
            ind = ind+1;
            yout(:,ind) = cos(yin(:,i));
            ind = ind+1;
            yout(:,ind) = 1./cos(yin(:,i));
            ind = ind+1;       
        end
    end
end



%% From here, we add the dX*Theta elements in our data.
% if Highest_dPoly_Order>=1
%     for k=1:pin1
%         Index=Index+1;
%         Data(:,Index)=dX(:,1).*Data(:,k);
%         Sym_Struct{1,Index}=Symbol_dX(iter,1)*(Sym_Struct{1,k});
%     end
% end





end






