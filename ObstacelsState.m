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

function Y = ObstacelsState(track,traj,trackWidth,n_cars)

if n_cars == 1
    Y = [];
elseif n_cars == 5
    startIdx_Op1 = 50; 
    vx0_Op1 = 0; 
    % find theta that coresponds to the 10th point on the clineline
    [theta_Op1, ~] = findTheta([track.cline(1,startIdx_Op1),track.cline(2,startIdx_Op1)],track.cline,traj.ppx.breaks,trackWidth,startIdx_Op1);

    x0_Op1 = [track.cline(1,startIdx_Op1), vx0_Op1,   track.cline(2,startIdx_Op1)+trackWidth(startIdx_Op1)/2,... % point on clineline
       0,  atan2(ppval(traj.dppy,theta_Op1),ppval(traj.dppx,theta_Op1)),... % aligned with clineline
       0,theta_Op1]'; %driving straight with vx0, and correct theta progress

    startIdx_Op2 = 150; 
    vx0_Op2 = 0; 
    % find theta that coresponds to the 10th point on the clineline
    [theta_Op2, ~] = findTheta([track.cline(1,startIdx_Op2),track.cline(2,startIdx_Op2)],track.cline,traj.ppx.breaks,trackWidth,startIdx_Op2);

     x0_Op2 = [track.cline(1,startIdx_Op2), vx0_Op2,   track.cline(2,startIdx_Op2)+trackWidth(startIdx_Op2)/4,... % point on clineline
       0,  atan2(ppval(traj.dppy,theta_Op2),ppval(traj.dppx,theta_Op2)),... % aligned with clineline
       0,theta_Op2]'; %driving straight with vx0, and correct theta progres

    startIdx_Op3 = 250; 
    vx0_Op3 = 0; 
    % find theta that coresponds to the 10th point on the clineline
    [theta_Op3, ~] = findTheta([track.cline(1,startIdx_Op3),track.cline(2,startIdx_Op3)],track.cline,traj.ppx.breaks,trackWidth,startIdx_Op3);

    
    x0_Op3 = [track.cline(1,startIdx_Op3), vx0_Op3,   track.cline(2,startIdx_Op3),... % point on clineline
       0,  atan2(ppval(traj.dppy,theta_Op3),ppval(traj.dppx,theta_Op3)),... % aligned with clineline
       0,theta_Op3]'; %driving straight with vx0, and correct theta progress
    
    startIdx_Op4 = 300; 
    vx0_Op4 = 0; 
    % find theta that coresponds to the 10th point on the clineline
    [theta_Op4, ~] = findTheta([track.cline(1,startIdx_Op4),track.cline(2,startIdx_Op4)],track.cline,traj.ppx.breaks,trackWidth,startIdx_Op4);

    x0_Op4 = [track.cline(1,startIdx_Op4), vx0_Op4,   track.cline(2,startIdx_Op4)-trackWidth(startIdx_Op4)/3,... % point on clineline
       0,  atan2(ppval(traj.dppy,theta_Op4),ppval(traj.dppx,theta_Op4)),... % aligned with clineline
       0,theta_Op4]'; %driving straight with vx0, and correct theta progress
    
    Y = [x0_Op1,x0_Op2,x0_Op3,x0_Op4];
else
    error('only 0 or 4 obstacles are pre programmed settings, feel free to change the obstacle constellation')
end
