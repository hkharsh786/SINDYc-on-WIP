function [c, ceq] = ConstraintFCN(u,uold,x,N,LBo,UBo,LBdu,UBdu,p)
%% Constraint function of nonlinear MPC for F8 system
%
% Inputs:
%   u:      optimization variable, from time k to time k+N-1 
%   x:      current state at time k
%   Ts:     controller sample time
%   N:      prediction horizon length
%   uold:   latest applied control input
%   LBo:    Lower bound of output x
%   UBo:    Upper bound of output x
%   LBdu:   Lower bound for input difference uk - uk-1
%   UBdu:   Upper bound for input difference uk - uk-1
%   p:      Parameters for model
%
% Output:
%   c:      inequality constraints applied across prediction horizon
%   ceq:    equality constraints  (empty)
%

%% Nonlinear MPC design parameters
% range of angle of attack
zMin = LBo(3);
zMax = UBo(3);
%% Inequality constraints calculation
% c = zeros(2*N,1);
% c1 = zeros(2*N,1);
% c2 = zeros(2*N,1);
% c3 = zeros(2*N,1);
% c4 = zeros(2*N,1);

% Apply 2N constraints across prediction horizon, from time k+1 to k+N
xk = x;
uk = u(1);
duk = u(1)-uold;

%
Ns = size(x,1);
xk = zeros(Ns,N+1); xk(:,1) = x;
    for ct=1:N
        % Obtain plant state at next prediction step.
        xk(:,ct+1) = rk4u(@sparseGalerkinControl_Discrete,xk(:,ct),u(ct),p.dt,1,[],p);
    end
xk = xk(:,2:N+1);
% for ct=1:N
%     % -z + zMin < 0 % lower bound 
%     c(2*ct-1) = -duk+LBdu; 
%     c1(2*ct-1) = -xk(1,ct)+zMin(1); 
%     c2(2*ct-1) = -xk(2,ct)+zMin(2); 
%     c3(2*ct-1) = -xk(3,ct)+zMin(3);
%     c4(2*ct-1) = -xk(4,ct)+zMin(4);
% 
%     % z - zMax < 0 % upper bound
%     c(2*ct) = duk-UBdu;
%     c1(2*ct) = xk(1,ct)-zMax(1);
%     c2(2*ct) = xk(2,ct)-zMax(2); 
%     c3(2*ct) = xk(3,ct)-zMax(3);
%     c4(2*ct) = xk(4,ct)-zMax(4);
% 
%     % Update plant state and input for next step
% 
%     if ct<N
%         uk = u(ct+1);
%         duk = u(ct+1)-u(ct);
%     end
% end

%% Inequality constraints calculation
c = zeros(N,1);
% Apply N population size constraints across prediction horizon, from time
% k+1 to k+N

for ct=1:N
    % -z + zMin < 0 % lower bound
    c(ct) = -xk(1,ct)+zMin;

end


% c = [c;c1;c2;c3;c4];

%% No equality constraints
ceq = [];
