function track = obstacleAdjust(track, Xobs, traj, borders, trackWidth)
nInsert = 10;
nObs = numel(Xobs);
for i = 1:nObs
    %% center
    ob = Xobs{i};
    center = sum(ob, 1)/4.0;
    %% center theta
    startIdx = 1;
    [theta, ~] = findTheta(center, track.cline, traj.ppx.breaks, trackWidth, startIdx);
    %% check if to adjust left or right
    leftRef = [ppval(borders.pplx, theta), ppval(borders.pply, theta)];
    rightRef = [ppval(borders.pprx, theta), ppval(borders.ppry, theta)];
    N = size(track.br, 2);
    if  (sum(sum((ob - repmat(leftRef, 4, 1)).^2, 2)) > sum(sum((ob - repmat(rightRef, 4, 1)).^2, 2)))
        
        %% adjust right use point No.1 No.4
        [theta1, ~] = findTheta(ob(1,:), track.cline, traj.ppx.breaks, trackWidth, startIdx);
        [theta2, ~] = findTheta(ob(4,:), track.cline, traj.ppx.breaks, trackWidth, startIdx);
        right1 = [ppval(borders.pprx, theta1); ppval(borders.ppry, theta1)];
        right2 = [ppval(borders.pprx, theta2); ppval(borders.ppry, theta2)];
        
        %% find closest to point 1
        [~,  minIndex] = min(sum((track.br - repmat(right1, 1, N)).^2, 1));
        leftBound = minIndex - 2;
        [~, minIndex] = min(sum((track.br - repmat(right2, 1, N)).^2, 1));
        rightBound = minIndex + 2;
%         nInsert = rightBound - leftBound + 1;
        p1 = ob(1,:);
        p2 = ob(4,:);

        while norm(p1 - p2) < 3.5
            temp1 = 1.5*p1 - 0.5*p2;
            temp2 = 1.5*p2 - 0.5*p1;
            p1 = temp1;
            p2 = temp2;
        end

        %px = spline(1:4, [track.br(1, leftBound), ob(1,1), ob(4,1), track.br(1, rightBound)]);
        %py = spline(1:4, [track.br(2, leftBound), ob(1,2), ob(4,2), track.br(2, rightBound)]);
        px = spline(1:4, [track.br(1, leftBound), p1(1), p2(1), track.br(1, rightBound)]);
        py = spline(1:4, [track.br(2, leftBound), p1(2), p2(2), track.br(2, rightBound)]);
        
        source = [ ppval(px, linspace(1,4, nInsert)); ppval(py, linspace(1,4, nInsert))];
        track.br(:, leftBound:rightBound) = [];
        track.br = insertCols(track.br, source, leftBound);
        
        [cTheta_leftBound, ~] = findTheta(track.cline(:,leftBound), track.cline, traj.ppx.breaks, trackWidth, 1);
        [cTheta_rightBound, ~] = findTheta(track.cline(:,rightBound), track.cline, traj.ppx.breaks, trackWidth, 1);
        thetaInterp = linspace(cTheta_leftBound, cTheta_rightBound, nInsert);
        source = [ppval(borders.pplx, thetaInterp); ppval(borders.pply, thetaInterp)];
        track.bl(:, leftBound:rightBound) = [];
        track.bl = insertCols(track.bl, source, leftBound);
        source = 0.5*( track.bl(:, leftBound:leftBound + nInsert - 1) + track.br(:, leftBound:leftBound+nInsert - 1));
        track.cline(:, leftBound:rightBound) = [];
        track.cline = insertCols(track.cline, source, leftBound);
        
      
        [traj, borders] = splinify(track);
        
    else
      
        
        %% adjust left
        %% adjust right use point No.2 No.3
        [theta1, ~] = findTheta(ob(2,:), track.cline, traj.ppx.breaks, trackWidth, startIdx);
        [theta2, ~] = findTheta(ob(3,:), track.cline, traj.ppx.breaks, trackWidth, startIdx);
        right1 = [ppval(borders.pplx, theta1); ppval(borders.pply, theta1)];
        right2 = [ppval(borders.pplx, theta2); ppval(borders.pply, theta2)];
        
        %% find closest to point 1
        [~, minIndex] = min(sum((track.bl - repmat(right1, 1, N)).^2, 1));
        leftBound = minIndex - 2;
        [~, minIndex] = min(sum((track.bl - repmat(right2, 1, N)).^2, 1));
        rightBound = minIndex + 2;
%         nInsert = rightBound - leftBound + 1;
        p1 = ob(2,:);
        p2 = ob(3,:);

        while norm(p1 - p2) < 3.5
            temp1 = 1.5*p1 - 0.5*p2;
            temp2 = 1.5*p2 - 0.5*p1;
            p1 = temp1;
            p2 = temp2;
        end

        
        px = spline(1:4, [track.bl(1, leftBound), p1(1), p2(1), track.bl(1, rightBound)]);
        py = spline(1:4, [track.bl(2, leftBound), p1(2), p2(2), track.bl(2, rightBound)]);
        
        source = [ ppval(px, linspace(1,4, nInsert)); ppval(py, linspace(1,4, nInsert))];
        track.bl(:, leftBound:rightBound) = [];
        track.bl = insertCols(track.bl, source, leftBound);
        
        [cTheta_leftBound, ~] = findTheta(track.cline(:,leftBound), track.cline, traj.ppx.breaks, trackWidth, startIdx-5);
        [cTheta_rightBound, ~] = findTheta(track.cline(:,rightBound), track.cline, traj.ppx.breaks, trackWidth, startIdx-5);
        thetaInterp = linspace(cTheta_leftBound, cTheta_rightBound, nInsert);
        source = [ppval(borders.pprx, thetaInterp); ppval(borders.ppry, thetaInterp)];
        track.br(:, leftBound:rightBound) = [];
        track.br = insertCols(track.br, source, leftBound);
        source = 0.5*( track.bl(:, leftBound:leftBound + nInsert - 1) + track.br(:, leftBound:leftBound+nInsert - 1));
            

        track.cline(:, leftBound:rightBound) = [];
        track.cline = insertCols(track.cline, source, leftBound);   
    
        [traj, borders] = splinify(track);
    end
end
end




