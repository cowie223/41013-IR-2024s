function [toolPatch, toolVertices, toolFaces] = CreateTool(tool)
    % Load PLY file
    pointCloud = pcread(tool);
    toolVertices = pointCloud.Location;
    toolFaces = convhull(toolVertices);    % Create faces if necessary

    % Create patch object for the tool
    toolPatch = patch('Vertices', toolVertices, 'Faces', toolFaces, ...
                  'FaceColor', [0.8 0.8 1.0], 'EdgeColor', 'none');

end
