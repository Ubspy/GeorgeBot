function exampleHelperVisualizeMapAndPoseGraph(omap, pGraph, ax)
% This helper function is useful for visualizing the built occupancy map 3D
% (omap) and pose graph (pGraph). The plot view is tuned for this example.

%   Copyright 2019 The MathWorks, Inc.

show(omap,'Parent',ax);
hold on;
pGraph.show('Parent',ax,"IDs","off");
xlim([-10 50]);
ylim([-20 50]);
zlim([-10 10]);
view([20,50]);
drawnow
hold off;
grid on;
end

