if eps == 0
    DERIV_NOISE = 0;
end


if size(x,1)==size(u,2)
    u = u';
end
dx = zeros(length(x)-5,Nvar);
size(dx)
size(x,2)
for i=3:length(x)-3
    for k=1:Nvar
        dx(i-2,k) = (1/(12*dt))*(-x(i+2,k)+8*x(i+1,k)-8*x(i-1,k)+x(i-2,k));
    end
end
% Concatenate states, input, and time derivatives
    xaug = [x(3:end-3,:) u(3:end-3,:)];
    dx(:,size(x,2)+1) = 0*dx(:,size(x,2));
n = size(dx,2);


clear Theta Xi
Theta = poolData(xaug,n,polyorder,usesine);

% Normalize library columns
Theta_norm = zeros(size(Theta,2),1); 

for i = 1:size(Theta,2)
    Theta_norm(i) = norm(Theta(:,i));
    Theta(:,i) = Theta(:,i)./Theta_norm(i);
end

m = size(Theta,2);
Xi = sparsifyDynamics(Theta,dx,lambda,n-1);
% Rescale identified coefficients to account for normalization
for i = 1:size(Theta,2)
    Xi(i,:) = Xi(i,:)./Theta_norm(i);
end
Xi
if n == 3
    str_vars = {'x','y','u'};
elseif n == 5
    str_vars = {'x1','x2','x3','x4','u'};
end
yout = poolDataLIST(str_vars,Xi,n,polyorder,usesine);