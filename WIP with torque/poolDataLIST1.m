function yout = poolDataLIST(yin,ahat,nVars,polyorder,usesine)
% Copyright 2015, All Rights Reserved
% Code by Steven L. Brunton
% For Paper, "Discovering Governing Equations from Data: 
%        Sparse Identification of Nonlinear Dynamical Systems"
% by S. L. Brunton, J. L. Proctor, and J. N. Kutz

n = size(yin,1);

ind = 1;
% poly order 0
yout{ind,1} = ['1'];
ind = ind+1;

% poly order 1
for i=[2,4]
    yout(ind,1) = yin(i);
    ind = ind+1;
end

if(polyorder>=2)
    % poly order 2
    for i=1:nVars
        for j=1:nVars
            yout{ind,1} = [yin{i},yin{j}];
            ind = ind+1;
        end
    end
end

if(polyorder>=3)
    % poly order 3
    for i=1:nVars
        for j=i:nVars
            for k=j:nVars
                yout{ind,1} = [yin{i},yin{j},yin{k}];
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
                    yout{ind,1} = [yin{i},yin{j},yin{k},yin{l}];
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
                        yout{ind,1} = [yin{i},yin{j},yin{k},yin{l},yin{m}];
                        ind = ind+1;
                    end
                end
            end
        end
    end
end
% disp('nVAR')
% nVars
% disp('length(yin)-2')
% length(yin)-2

if(usesine)
    %TO BE UPDATED FOR PRINTING
    for k=1:100
    %     for i=1:nVars
    % 
    %         yout{ind,1} = ['sin(', num2str(k), '*', yin{i},')'];
    %         ind = ind+1;
    %     end
    %     for i=1:nVars
    % 
    %         yout{ind,1} = ['cos(', num2str(k), '*', yin{i},')'];
    %         ind = ind+1;
    %     end
    % 
    % 
    % 
    %     for i=1:nVars
    %         yout{ind,1} = ['sin(', num2str(k), '*', yin{i},'^2)'];
    %         ind = ind+1;
    %     end
    % 
    %     for i=1:nVars
    %         yout{ind,1} = ['cos(', num2str(k), '*', yin{i},'^2)'];
    %         ind = ind+1;
    %     end
    % 
    %     for i=1:nVars
    %         for j=1:nVars
    %             yout{ind,1} = [ yin{i},'^2*sin(', num2str(k), '*', yin{j},')cos(', num2str(k), '*', yin{j},')'];
    %             ind = ind+1;
    %         end
    %     end
    % end
    
    %     for i=1:nVars
    %         for j=1:nVars
    %             ind = ind+1;
    %             yout(:,ind)=yin(:,i).^2.*sin(yin(:,j));
    %         end
    %     end
    %     %
    %     for i=1:nVars
    %         for j=1:nVars
    %             ind = ind+1;
    %             yout(:,ind)=yin(:,i).^2.*cos(yin(:,j));
    %         end
    %     end
    %     %
    %     for i=1:nVars
    %         for j=1:nVars
    %             ind = ind+1;
    %             yout(:,ind)=sin(yin(:,i)).*cos(yin(:,j));
    %         end
    %     end
        for state=3:3
            yout{ind,1} = [yin{state-1},'./(C1+ (m2*sin(', num2str(k), '*', yin{state}, ')).*sin(', num2str(k), '*', yin{state},')'];
            ind = ind + 1;
            yout{ind,1} = [yin{state-1},'*', yin{state-1},'./(m2*L*(cos(', num2str(k), '*', yin{state}, ')'];
            ind = ind + 1;
            yout{ind,1} = ['sinx3/c1'];
            ind = ind + 1;

            yout{ind,1} = ['-m2L'];
            ind = ind + 1;
            yout{ind,1} = ['d2/L'];
            ind = ind + 1;
            yout{ind,1} = ['c2*g'];
            ind = ind + 1;
            yout{ind,1} = ['m2L'];
            ind = ind + 1;


            yout{ind,1} = ['C2d2/M2L'];
            ind = ind + 1;
            yout{ind,1} = ['1'];
            ind = ind + 1;
            yout{ind,1} = ['1'];
            ind = ind + 1;


        end


    end
end


output = yout;
newout(1) = {''};
for k=1:length(yin)
    newout{1,1+k} = [yin{k},'dot'];
end
%size(ahat,1)
% newout = {'','xdot','ydot','udot'};
for k=1:size(ahat,1)
    newout(k+1,1) = output(k);
    for j=1:length(yin)
        newout{k+1,1+j} = ahat(k,j);
    end
end
newout