function dx = car_model(t, state,input)
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
Shy = 0;
Svy = 0;
Fzf = b/(a+b)*m*g;
Fzr = a/(a+b)*m*g;

x = state(1);
u = state(2);
y = state(3);
v = state(4);
psi = state(5);
r = state(6);
deltaf = input(1);
Fx = input(2);

af = rad2deg(deltaf - atan2(v+a*r, u));
ar = rad2deg(-atan2(v-b*r, u));

phiyf = (1-Ey)*(af+Shy)+Ey/By*atan(By*(af+Shy));
phiyr = (1-Ey)*(ar+Shy)+Ey/By*atan(By*(ar+Shy));
Fyf = Fzf*Dy*sin(Cy*atan(By*phiyf))+ Shy;
Fyr = Fzr*Dy*sin(Cy*atan(By*phiyr))+ Shy;

F_total=sqrt((Nw*Fx)^2+(Fyr^2));
F_max=0.7*m*g;

if F_total>F_max
    Fx=F_max/F_total*Fx;
    Fyr=F_max/F_total*Fyr;
end

%disp(['Fyf is ', Fyf])
%fprintf('%f %f %f %f %f %f\n', af, ar, phiyf, phiyr, Fyf, Fyr);



dx1 = u*cos(psi)-v*sin(psi);
dx2 = 1/m*(-f*m*g+Nw*Fx-Fyf*sin(deltaf))+v*r;
dx3 = u*sin(psi)+v*cos(psi);
dx4 = 1/m*(Fyf*cos(deltaf)+Fyr)-u*r;
dx5 = r;
dx6 = 1/Iz*(a*Fyf*cos(deltaf)-b*Fyr);

dx = [dx1;dx2;dx3;dx4;dx5;dx6; input(3)];
end




