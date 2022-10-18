L(1) = Revolute('d', 0.27,        'a', 0.069, 'alpha', -pi/2);
L(2) = Revolute('d', 0,           'a', 0, 'alpha', pi/2, 'offset', pi/2);

leftprong =  SerialLink(L, 'name', 'Baxter LEFT');
rightprong = SerialLink(L, 'name', 'Baxter RIGHT');

leftprong.base = transl(0.064614, 0.25858, 0.119) * rpy2tr(0, 0, pi/4);
rightprong.base = transl(0.063534, -0.25966, 0.119) * rpy2tr(0, 0, -pi/4);

% define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%   qstretch   arm is stretched out in the X direction
%   qn         arm is at a nominal non-singular configuration
%
qz = [0 0]; % zero angles, L shaped pose
qr = [0 -pi/2]; % ready pose, arm up
qs = [0 0];
qn = [0 pi/4];
