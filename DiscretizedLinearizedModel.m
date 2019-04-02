function [Ad, Bd, gd] = DiscretizedLinearizedModel(Xbar_k, Ubar_k, ModelParams, Ts)
m = 1400;
g = 9.806;
Nw = 2;
f = 0.01;
Iz = 2667;
a = 1.35;
b = 1.45;
By = 0.27;
Cy = 1.2;
Dy = 0.7;
Ey = -1.6;
Fzf = b/(a+b)*m*g;
Fzr = a/(a+b)*m*g;
sx = 6;
su = 2;

u = Xbar_k(2);
r = Xbar_k(6);
v = Xbar_k(4);
deltaf = Ubar_k(1);
Fx = Ubar_k(2);
psi = Xbar_k(5);

dafddeltaf = 180.0/pi;
dafdv = rad2deg(-1/(u*((v + a*r)^2/u^2 + 1)));
dafdr = rad2deg(-a/(u*((v + a*r)^2/u^2 + 1)));
dafdu = rad2deg((v + a*r)/(u^2*((v + a*r)^2/u^2 + 1)));

dardv = rad2deg(-1/(u*((v - b*r)^2/u^2 + 1)));
dardr = rad2deg(b/(u*((v - b*r)^2/u^2 + 1)));
dardu = rad2deg((v - b*r)/(u^2*((v - b*r)^2/u^2 + 1)));

af = rad2deg(deltaf - atan2(v+a*r, u));
ar = rad2deg(-atan2(v-b*r, u));

phiyf = (1 - Ey)*af + Ey/By*atan(By*af);
phiyr = (1 - Ey)*ar + Ey/By*atan(By*ar);

dphiyfdaf = Ey/(By^2*af^2 + 1) - Ey + 1;
dphiyrdar = Ey/(By^2*ar^2 + 1) - Ey + 1;

Fyf = Fzf*Dy*sin(Cy*atan(By*phiyf));
Fyr = Fzr*Dy*sin(Cy*atan(By*phiyr));

dFyfdphiyf = (By*Cy*Dy*Fzf*cos(Cy*atan(By*phiyf)))/(By^2*phiyf^2 + 1);
dFyrdphiyr = (By*Cy*Dy*Fzr*cos(Cy*atan(By*phiyr)))/(By^2*phiyr^2 + 1);
dFyfdaf = dFyfdphiyf*dphiyfdaf;
dFyrdar = dFyrdphiyr*dphiyrdar;

dFyfdu = dFyfdaf*dafdu;
dFyrdu = dFyrdar*dardu;

dFyfdv = dFyfdaf*dafdv;
dFyrdv = dFyrdar*dardv;

dFyfdr = dFyfdaf*dafdr;
dFyrdr = dFyrdar*dardr;
dFyfddeltaf = dFyfdaf*dafddeltaf;

Ac = [ 0, 0, 0, 0, 0, 0; ...
    cos(psi), -1.0/m*dFyfdu*sin(deltaf), sin(psi), 1.0/m*(dFyfdu*cos(deltaf) + dFyrdu) - r, 0, 1.0/Iz*(a*dFyfdu*cos(deltaf) - b*dFyrdu); ...
    0, 0, 0, 0, 0, 0; ...
    -sin(psi), -1.0/m*dFyfdv*sin(deltaf) + r, cos(psi), 1.0/m*(dFyfdv*cos(deltaf) + dFyrdv), 0, 1.0/Iz*(a*dFyfdv*cos(deltaf) - b*dFyrdv); ...
    -u*sin(psi) - v*cos(psi), 0, u*cos(psi) - v*sin(psi), 0, 0, 0; ...
    0, -1.0/m*dFyfdr*sin(deltaf) + v, 0, 1.0/m*(dFyfdr*cos(deltaf) + dFyrdr) - u, 1.0, 1.0/Iz*(a*dFyfdr*cos(deltaf) - b*dFyrdr)]';

Bc = [ 0, -1.0/m*(dFyfddeltaf*sin(deltaf) + Fyf*cos(deltaf)), 0, 1.0/m*(dFyfddeltaf*cos(deltaf) - Fyf*sin(deltaf)), 0, a/Iz*(dFyfddeltaf*cos(deltaf) - Fyf*sin(deltaf)); ...
    0, Nw/m, 0, 0, 0, 0]';

f = [u*cos(psi)-v*sin(psi);
    1/m*(-f*m*g+Nw*Fx-Fyf*sin(deltaf))+v*r;
    u*sin(psi)+v*cos(psi);
    1/m*(Fyf*cos(deltaf)+Fyr)-u*r;
    r;
    1/Iz*(a*Fyf*cos(deltaf)-b*Fyr)];

gc=f-Ac*Xbar_k(1:sx)-Bc*Ubar_k(1:su);

Bc_aug=[Bc gc];

%discretize

% see report for proof of following method
tmp = expm([Ac Bc_aug; zeros(su+1,sx+su+1)]*Ts);

Ad = zeros(sx+1,sx+1);
Bd = zeros(sx+1,su+1);
gd = zeros(sx+1,1);
Ad(1:sx,1:sx) =tmp(1:sx,1:sx);
Bd(1:sx,1:su) =tmp(1:sx,sx+1:sx+su);
gd(1:sx) =tmp(1:sx,sx+su+1);

% following to avoid numerical errors
Ad(end,end)=1;
Bd(end,end)=Ts;


end

