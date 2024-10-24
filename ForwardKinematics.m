[plyData, properties] = plyread('bed.ply', 'tri');
verticies = [plyData(1,:), plyData(2), plyData(3)];
faces =plyData.face.vertex_indices +1;

figure;
trisurf(faces, vertices(:,1), vertices(:,2), vertices(:,3), 'FaceVertexCData', colours, 'EdgeColour', 'none', 'FaceColour', 'interp')




% igus = IGUSReBel;
% 
% T1 = transl(0.3, 0.3, 0)
% 
% T2 = transl(-0.3, -0.3, 0)
% q1 = igus.model.ikine(T1, 'q0', [0 0 0 0 0 0], 'mask', [1 1 1 0 0 0], 'forceSoln')
% q2 = igus.model.ikine(T2, 'q0', [0 0 0 0 0 0], 'mask', [1 1 1 0 0 0], 'forceSoln')
% 
% QMatrix = jtraj(q1, q2, 20)
% 
% igus.model.plot(QMatrix, 'trail', '-r');
% 
% igus.model.animate()
