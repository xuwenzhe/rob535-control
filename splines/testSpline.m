track = load('TestTrack.mat');
track = track.TestTrack;
[traj, borders] = splinify(track);

figure
hold on

plot(track.cline(1,:), track.cline(2,:), '--')
plot(track.bl(1,:), track.bl(2,:), '--')
plot(track.br(1,:), track.br(2,:), '--')

s = 0:5:traj.ppy.breaks(end);

xc = ppval(traj.ppx, s);
yc = ppval(traj.ppy, s);
xl = ppval(borders.pplx, s);
yl = ppval(borders.pply, s);
xr = ppval(borders.pprx, s);
yr = ppval(borders.ppry, s);

scatter(xc, yc, 'o')
scatter(xl, yl, 'x')
scatter(xr, yr, 'x')
