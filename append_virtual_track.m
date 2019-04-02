function [track2] = append_virtual_track(track3)

track2 = track3;
virtual_discrete_size = 15;
virtual_discrete_distX = 7;
virtual_angle = track3.theta(end-7);

track3length = length(track3.br);
for i = 1:virtual_discrete_size
    track2.br(1,track3length+i) = track3.br(1,end) + i * virtual_discrete_distX;
    track2.br(2,track3length+i) = track3.br(2,end) + i * virtual_discrete_distX * tan(virtual_angle);
    track2.bl(1,track3length+i) = track3.bl(1,end) + i * virtual_discrete_distX;
    track2.bl(2,track3length+i) = track3.bl(2,end) + i * virtual_discrete_distX * tan(virtual_angle);
    
    track2.cline(1,track3length+i) = 0.5 * (track2.br(1,track3length+i) + track2.bl(1,track3length+i));
    track2.cline(2,track3length+i) = 0.5 * (track2.br(2,track3length+i) + track2.bl(2,track3length+i));
    
    track2.theta(track3length+i) = virtual_angle;
end
end

