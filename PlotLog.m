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

function [  ] = PlotLog( X,U,Y,track,track2,simN,Ts)
    
    figure(1);
    set(gcf,'Position',[1000,10,800,800])
    plot(track.br(1,:),track.br(2,:),'r')
    hold on
    plot(track.bl(1,:),track.bl(2,:),'r')
    plot(track2.br(1,:),track2.br(2,:),'k')
    plot(track2.bl(1,:),track2.bl(2,:),'k')
    
    plot(X(1,X(1,:)~=0),X(3,X(3,:)~=0),'b')
    if ~isempty(Y)
        for i=1:size(Y,2)
            carBox(Y(:,i));
        end
    end
    xlabel('X [m]')
    ylabel('Y [m]')
    axis equal
    hold off

%     figure(2)
%     subplot(3,1,1)
%     plot([0:simN-1]*Ts,U(1,:))
%     xlabel('time [s]')
%     ylabel('\delta [rad]')
%     subplot(3,1,2)
%     plot([0:simN-1]*Ts,U(2,:))
%     xlabel('time [s]')
%     ylabel('Fx [N]')
%     subplot(3,1,3)
%     plot([0:simN-1]*Ts,U(3,:))
%     xlabel('time [s]')
%     ylabel('v_{\Theta} [m/s]')
% 
%     figure(3)
%     subplot(3,1,1)
%     plot([0:simN-1]*Ts,X(5,:))
%     xlabel('time [s]')
%     ylabel('\phi [rad]')
%     subplot(3,1,2)
%     plot([0:simN-1]*Ts,X(6,:))
%     xlabel('time [s]')
%     ylabel('\r [rad/s]')
%     subplot(3,1,3)
%     plot([0:simN-1]*Ts,180/pi*atan2(X(4,:),X(2,:)))
%     xlabel('time [s]')
%     ylabel('\beta [deg]')
% 
%     pause(0.001)
end

