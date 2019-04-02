% Copyright (C) 2018, ETH Zurich, D-ITET, Kenneth Kuchera, Alexander Liniger
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [  ] = PlotPrediction( x,u,b,Y,track,track2,traj,MPC_vars,ModelParams, Xobs)
    
    N = MPC_vars.N;
    Ts = MPC_vars.Ts;
    
    tl = traj.ppx.breaks(end);
    
    figure(1);
    set(gcf,'Position',[10,10,800,800])
    plot(track.br(1,:),track.br(2,:),'r')
    hold on
    plot(track.bl(1,:),track.bl(2,:),'r')
    plot(track2.br(1,:),track2.br(2,:),'k')
    plot(track2.bl(1,:),track2.bl(2,:),'k')
    plot(b(:,1),b(:,2),'g.')
    plot(b(:,3),b(:,4),'g.')
    plot(ppval(traj.ppx,mod(x(7,:),tl)),ppval(traj.ppy,mod(x(7,:),tl)),':k')
    plot(x(1,:),x(3,:),'-ob')
    
    for i = 1:numel(Xobs)
    thisOb = Xobs{i};
    plot(thisOb(:,1), thisOb(:,2), '--b');
    plot(thisOb([end, 1], 1), thisOb([end, 1], 2), '--b')
end
    carBox(x(:,1))
    %scatter(x(1,1), x(3,1))

    if ~isempty(Y)
        for i=1:size(Y,2)
            carBox(Y(:,i))
        end
    end
    xlabel('X [m]')
    ylabel('Y [m]')
    axis equal
    
%     xlim([200, 500]);
%     ylim([-200, 200]);
    
%     xmin = min(x(1,:));
%     xmax = max(x(1,:));
%     ymin = min(x(3,:));
%     ymax = max(x(3,:));
%     xlim([xmin-5, xmax+5])
%     ylim([ymin-5, ymax+5])
%     
%     xlim([270, 292]);
%     ylim([-185, -166]);
%     axis equal
    hold off

%     figure(2)
%     subplot(4,1,1)
%     plot([0:N-1]*Ts,u(1,:))
%     xlabel('time [s]')
%     ylabel('\delta [rad]')
%     subplot(4,1,2)
%     plot([0:N-1]*Ts,u(2,:))
%     xlabel('time [s]')
%     ylabel('Fx [N]')
%     subplot(4,1,3)
%     plot([0:N-1]*Ts,u(3,:))
%     xlabel('time [s]')
%     ylabel('v_{\Theta} [m/s]')
%     subplot(4,1,4)
%     plot([0:N]*Ts,x(2,:))
%     xlabel('time [s]')
%     ylabel('ux [m/s]')
%     figure(3)
%     subplot(3,1,1)
%     plot([0:N]*Ts,x(5,:))
%     xlabel('time [s]')
%     ylabel('\phi [rad]')
%     subplot(3,1,2)
%     plot([0:N]*Ts,x(6,:))
%     xlabel('time [s]')
%     ylabel('\omega [rad/s]')
%     subplot(3,1,3)
%     plot([0:N]*Ts,x(7,:))
%     xlabel('time [s]')
%     ylabel('\Theta [m]')

%     pause(0.001)

end

