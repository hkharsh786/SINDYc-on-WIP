function [xk1, yk] = WIP_De(xk, uk, Ts)
[dxdt, ~,~,~,~,~] = WIP_CT_odeDe(xk,uk);
xk1 = xk + Ts*dxdt;
yk = xk1;
end
