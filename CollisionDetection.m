% Collision Check Robot and User-Defined Planes
function CollisionDetection()

   igus = IGUSReBel;
   
   hold on;

    
    % Define initial configuration
    q = zeros(1,7);  % Initial joint angles
    % igus.model.animate(q);
    hold on;
    igus.model.plot(q, 'workspace', [-2 2 -2 2 0 2]);
    hold on;



    % 2. Define user-specified planes as structures
    % Each plane is defined by a point on the plane and a normal vector.
    planes = {
        struct('point', [1, 0, 0], 'normal', [0, 0, 1], 'vertices', [[1, 0, 0]; [0.5, 0.5, 0]; [1.5, 0.5, 0]]),
        struct('point', [0, 1, 0], 'normal', [0, 1, 0], 'vertices', [[0, 1, 0]; [-0.5, 1, 0.5]; [0.5, 1, 0.5]])
    };

    % Plot each plane for visualization
    hold on;
    for i = 1:length(planes)
        fill3(planes{i}.vertices(:,1), planes{i}.vertices(:,2), planes{i}.vertices(:,3), 'c', 'FaceAlpha', 0.3);
    end

    % 3. Compute link transforms for collision check
    tr = GetLinkPoses(q, igus);

    % 4. Collision detection between robot links and each plane
    % For each link and each plane, check if there is an intersection.
    for i = 1:size(tr, 3) - 1
        for j = 1:length(planes)
            % Use the plane normal and point to check intersections with the link
            vertOnPlane = planes{j}.point;
            normal = planes{j}.normal;
            triangleVerts = planes{j}.vertices;

            % Calculate intersection between robot link and plane
            [intersectP, check] = LinePlaneIntersection(normal, vertOnPlane, tr(1:3, 4, i)', tr(1:3, 4, i + 1)');

            % If intersection exists and is within the triangle, plot it
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP, triangleVerts)
                plot3(intersectP(1), intersectP(2), intersectP(3), 'g*');
                disp(['Intersection detected with plane ', num2str(j)]);
            end
        end
    end

end

%% Utility Functions

% Function to compute transforms for each link in SerialLink robot
function [transforms] = GetLinkPoses(q, robot)
    links = robot.model.links;
    transforms = zeros(4, 4, length(links) + 1);
    transforms(:,:,1) = robot.model.base;

    for i = 1:length(links)
        L = links(1,i);
        current_transform = transforms(:,:,i);
        current_transform = current_transform * trotz(q(i) + L.offset) * transl(0, 0, L.d) * transl(L.a, 0, 0) * trotx(L.alpha);
        transforms(:,:,i + 1) = current_transform;
    end
end

% Function to check if an intersection point lies within the boundaries of a triangle
function result = IsIntersectionPointInsideTriangle(intersectP, triangleVerts)
    u = triangleVerts(2,:) - triangleVerts(1,:);
    v = triangleVerts(3,:) - triangleVerts(1,:);
    uu = dot(u,u);
    uv = dot(u,v);
    vv = dot(v,v);
    w = intersectP - triangleVerts(1,:);
    wu = dot(w,u);
    wv = dot(w,v);
    D = uv * uv - uu * vv;
    s = (uv * wv - vv * wu) / D;
    if (s < 0.0 || s > 1.0)
        result = 0
        return;
    end
    t = (uv * wu - uu * wv) / D;
    if (t < 0.0 || (s + t) > 1.0)
        result = 0
        return;
    end
    result = 1
end

% Function to find intersection between a line segment and a plane
function [intersectP, check] = LinePlaneIntersection(normal, pointOnPlane, point1, point2)
    lineVec = point2 - point1;
    planeVec = pointOnPlane - point1;
    d = dot(normal, planeVec) / dot(normal, lineVec);
    intersectP = point1 + d * lineVec
    check = (d >= 0 && d <= 1) % True if the intersection is within the segment
end
