eulerAngles = [(0:10:90).',zeros(numel(0:10:90),2)];
q = quaternion(eulerAngles,'eulerd','ZYX','frame');

dt = 1;
av = angvel(q,dt,'frame') % units in rad/s