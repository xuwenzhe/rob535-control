nCases = 100;
%fail =false;
nfail = 1;
Nobs = 25;
load Tracks/TestTrack.mat;

for caseI = 1:nCases
    Xobs = generateRandomObstacles(Nobs, TestTrack);
    fprintf('Testing case No .%d .....\n', caseI);
    sol_2 = ROB599_ControlsProject_part2_Team3(TestTrack, Xobs);
    [Y, ~] = forwardIntegrateControlInput2(sol_2);
    figure(10)
    clf
    hold on
    set(gcf,'Position',[1000,10,800,800])
    plot(TestTrack.br(1,:),TestTrack.br(2,:),'k')
    plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'k')
    plot(Y(:,1), Y(:,3), '-ob')
    for i = 1:numel(Xobs)
        thisOb = Xobs{i};
        plot(thisOb(:,1), thisOb(:,2), '-b');
        plot(thisOb([end, 1], 1), thisOb([end, 1], 2), '-b')
    end

    info = getTrajectoryInfo(Y(:,[1,3]), sol_2, Xobs)

    if info.percent_of_track_completed < 1
        failname = sprintf('testSet/obs%d.mat', nfail);
        nfail = nfail + 1;
        save(failname, 'Xobs')
        %break
    else
        fprintf('Finish time is %f\n', info.t_end);
    end
end
