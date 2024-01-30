disp('Sindy Training started')
if eps == 0
    DERIV_NOISE = 0;
end


if size(x,1)==size(u,2)
    u = u';
end
% compute derivative using fourth order central difference
dx = zeros(length(x)-5,Nvar);

    for i=3:length(x)-3
        for k=1:size(x,2)
            dx(i-2,k) = (1/(12*dt))*(-x(i+2,k)+8*x(i+1,k)-8*x(i-1,k)+x(i-2,k));
        end
    end


% Concatenate states, input, and time derivatives
xaug = [x(3:end-3,:) u(3:end-3,:)];
dx(:,size(x,2)+1) = 0*dx(:,size(x,2));
n = size(dx,2);
% disp('xaug=');
%  size
% disp('dx= ');
% dx
disp('data pooling stared')
clear Theta Xi
Theta = poolData(xaug,n,polyorder,usesine);

% Normalize library columns
disp('data normalizing stared')
Theta_norm = zeros(size(Theta,2),1); 

for i = 1:size(Theta,2)
    Theta_norm(i) = norm(Theta(:,i));
    Theta(:,i) = Theta(:,i)./Theta_norm(i);
end

disp('data sparcifying stared')
m = size(Theta,2);
if exist('lambda_vec') == 1
    Xi = sparsifyDynamicsIndependent(Theta,dx,lambda_vec,n-1);
else
    Xi = sparsifyDynamics(Theta,dx,lambda,n-1);
end
% Rescale identified coefficients to account for normalization
for i = 1:size(Theta,2)
    Xi(i,:) = Xi(i,:)./Theta_norm(i);
end
% Xi
str_vars = {'x1','x2','x3','x4','u'};
disp('Output SINDy Matrix Data :')
yout = poolDataLIST(str_vars,Xi,n,polyorder,usesine);