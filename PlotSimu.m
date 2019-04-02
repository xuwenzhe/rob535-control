function [ ] = PlotSimu(X, U, Y, track, track2, Ts, Xobs)

% simulate a ode45 first with zero input


figure(10)
clf
hold on
set(gcf,'Position',[1000,10,800,800])
plot(track.br(1,:),track.br(2,:),'r')
plot(track.bl(1,:),track.bl(2,:),'r')
plot(track2.br(1,:),track2.br(2,:),'k')
plot(track2.bl(1,:),track2.bl(2,:),'k')
    
ode45_options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
x0 = X(1:7,1);
%for i = 1:size(X(:,X(1,:)~=0),2)
    %[~, sim_x] = ode45(@(t, x) car_model(t, x, U(1:3,i)), [0,Ts], x0, ode45_options);
%     size(sim_x)
    %x0 = sim_x(end,:)';
    %scatter(X(1,i), X(3,i),'ob')
%end
totalSteps = size(X(:, X(1,:)~=0), 2);
extend = 1;
U(1:3, totalSteps+1:totalSteps+extend) = repmat([0;1000; U(3, totalSteps)], 1, extend);
input = @(t) U(1:3, floor(t/Ts)+1);
[~, sim_x] = ode45(@(t, x) car_model(t, x, input(t)), [0, (totalSteps+extend)*Ts], x0, ode45_options);
plot(sim_x(:,1), sim_x(:,3), '-g');
scatter(X(1,1:totalSteps), X(3,1:totalSteps), 'ob');

for i = 1:numel(Xobs)
    thisOb = Xobs{i};
    plot(thisOb(:,1), thisOb(:,2), '--b');
    plot(thisOb([end, 1], 1), thisOb([end, 1], 2), '--b')
end

axis equal
end

