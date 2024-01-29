disp('Sindy Training started')
if eps > 0
    disp('Smoothing dataset')
    x(:,1) = sgolayfilt(x(:,1),filter_order,filter_framelen);
    x(:,1) = smoothdata(x(:,1), 'gaussian', filter_framelen);
    x(:,2) = sgolayfilt(x(:,2),filter_order,filter_framelen);
    x(:,2) = smoothdata(x(:,2), 'gaussian', filter_framelen);
    x(:,3) = sgolayfilt(x(:,3),filter_order,filter_framelen);
    x(:,3) = smoothdata(x(:,3), 'gaussian', filter_framelen);
    x(:,4) = sgolayfilt(x(:,4),filter_order,filter_framelen);
    x(:,4) = smoothdata(x(:,4), 'gaussian', filter_framelen);

    x(:,1) = sgolayfilt(x(:,1),filter_order,filter_framelen);
    x(:,1) = smoothdata(x(:,1), 'loess', filter_framelen);
    x(:,2) = sgolayfilt(x(:,2),filter_order,filter_framelen);
    x(:,2) = smoothdata(x(:,2), 'loess', filter_framelen);
    x(:,3) = sgolayfilt(x(:,3),filter_order,filter_framelen);
    x(:,3) = smoothdata(x(:,3), 'loess', filter_framelen);
    x(:,4) = sgolayfilt(x(:,4),filter_order,filter_framelen);
    x(:,4) = smoothdata(x(:,4), 'loess', filter_framelen);

    x(:,1) = sgolayfilt(x(:,1),filter_order,filter_framelen);
    x(:,1) = smoothdata(x(:,1), 'rloess', filter_framelen);
    x(:,2) = sgolayfilt(x(:,2),filter_order,filter_framelen);
    x(:,2) = smoothdata(x(:,2), 'rloess', filter_framelen);
    x(:,3) = sgolayfilt(x(:,3),filter_order,filter_framelen);
    x(:,3) = smoothdata(x(:,3), 'rloess', filter_framelen);
    x(:,4) = sgolayfilt(x(:,4),filter_order,filter_framelen);
    x(:,4) = smoothdata(x(:,4), 'rloess', filter_framelen);

    x(:,1) = sgolayfilt(x(:,1),filter_order,filter_framelen);
    x(:,1) = smoothdata(x(:,1), 'sgolay', filter_framelen);
    x(:,2) = sgolayfilt(x(:,2),filter_order,filter_framelen);
    x(:,2) = smoothdata(x(:,2), 'sgolay', filter_framelen);
    x(:,3) = sgolayfilt(x(:,3),filter_order,filter_framelen);
    x(:,3) = smoothdata(x(:,3), 'sgolay', filter_framelen);
    x(:,4) = sgolayfilt(x(:,4),filter_order,filter_framelen);
    x(:,4) = smoothdata(x(:,4), 'sgolay', filter_framelen);

    x = sgolayfilt(x,filter_order,filter_framelen);
    x = smoothdata(x, 'loess', filter_framelen);
    x = sgolayfilt(x,filter_order,filter_framelen);
    x = smoothdata(x, 'rloess', filter_framelen);
    x = smoothdata(x, 'sgolay', filter_framelen);
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