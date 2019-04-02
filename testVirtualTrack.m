load Tracks/TestTrack.mat;

[track2] = append_virtual_track(TestTrack);

figure(10),hold
set(gcf,'Position',[1000,10,800,800])
plot(track2.br(1,:),track2.br(2,:),'ro')
plot(track2.bl(1,:),track2.bl(2,:),'ro')
plot(track2.cline(1,:),track2.cline(2,:),'go')
plot(TestTrack.br(1,:),TestTrack.br(2,:),'ko')
plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'ko')