classdef PlanningEnvironment
  properties
    obstacles = {};
    safe_regions = IRISRegion.empty();
    r;
    yaw_slices = linspace(-pi, pi, 20);
    lb = [-5;-5;-pi];
    ub = [10;5;pi];
    x0;
    goal_pos = [];
  end

  methods
    function obj = PlanningEnvironment(r)
      if nargin < 1
        addpath(fullfile(getDrakePath, 'examples', 'Atlas'));
        warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
        warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
        r = Atlas(fullfile(getDrakePath(), 'examples', 'Atlas', 'urdf', 'atlas_minimal_contact.urdf'));
        r = removeCollisionGroupsExcept(r,{'heel','toe'});
        r = compile(r);
      end
      obj.r = r;
      obj.x0 = r.loadFixedPoint();
    end

    function obj = addBox(obj, box_size, xyz, rpy)
      obs = {[xyz(1) - box_size(1)/2, xyz(1) + box_size(1)/2;
              xyz(2) + box_size(2)/2, xyz(2) + box_size(2)/2;
              repmat(xyz(3) + box_size(3)/2, 1, 2)],...
             [xyz(1) - box_size(1)/2, xyz(1) + box_size(1)/2;
              xyz(2) - box_size(2)/2, xyz(2) - box_size(2)/2;
              repmat(xyz(3) + box_size(3)/2, 1, 2)],...
             [xyz(1) + box_size(1)/2, xyz(1) + box_size(1)/2;
              xyz(2) - box_size(2)/2, xyz(2) + box_size(2)/2;
              repmat(xyz(3) + box_size(3)/2, 1, 2)],...
             [xyz(1) - box_size(1)/2, xyz(1) - box_size(1)/2;
              xyz(2) - box_size(2)/2, xyz(2) + box_size(2)/2;
              repmat(xyz(3) + box_size(3)/2, 1, 2)]};
      obj.obstacles = [obj.obstacles, obs];
      s = RigidBodyBox(box_size, xyz, rpy);
      s.c = [0.8, 0.3, 0.3];
      obj.r = addShapeToBody(obj.r, 'world', s);
      obj.r = addContactShapeToBody(obj.r, 'world', s);
      obj.r = compile(obj.r);
    end

    function obj = addRegion(obj, pose)
      kinsol = doKinematics(obj.r, obj.x0(1:obj.r.getNumPositions()));
      origin = pose(1:3);
      origin(3) = origin(3) + 10;
      distance = collisionRaycast(obj.r, kinsol, origin, pose(1:3));
      distance(distance == -1) = 10;
      pose(3) = pose(3) + 10 - distance;

      c_obs = {};
      for j = 1:length(obj.obstacles)
        bot = obj.get_bot_border(max(obj.obstacles{j}(3,:)) - pose(3));
        c_obs = [c_obs, iris.cspace.cspace3(obj.obstacles{j}(1:2,:), bot, obj.yaw_slices)];
      end
      A_bounds = [-eye(3); eye(3)];
      b_bounds = [-obj.lb; obj.ub];
      [A,b,C,d,results] = iris.inflate_region(c_obs, A_bounds, b_bounds, pose([1,2,6]), struct('require_containment', true));
      obj.safe_regions(end+1) = IRISRegion(A,b,pose(1:3),rpy2rotmat(pose(4:6)) * [0;0;1]);
    %   iris.drawing.animate_results(results);
    end
    
    function v = draw(obj)
      v = obj.r.constructVisualizer();
      v.draw(0, obj.x0);
      lc = lcm.lcm.LCM.getSingleton();
      lcmgl = drake.util.BotLCMGLClient(lc, 'Footstep_Planning');
      for j = 1:length(obj.safe_regions)
        obj.safe_regions(j).draw_lcmgl(lcmgl);
      end
%       lcmgl.switchBuffers();
      
%       lcmgl = drake.util.BotLCMGLClient(lc, 'Goal');
      lcmgl.glColor3f(0,0,0);
      lcmgl.glLineWidth(5)
      if ~isempty(obj.goal_pos)
        obj.goal_pos.center = mean([obj.goal_pos.right, obj.goal_pos.left], 2);
        r = 0.25;
        lcmgl.circle(obj.goal_pos.center(1), obj.goal_pos.center(2), obj.goal_pos.center(3), r);
        lcmgl.glBegin(lcmgl.LCMGL_LINES)
        v1 = obj.goal_pos.center(1:3);
        v2 = [v1(1:2) + rotmat(obj.goal_pos.center(6)) * [r; 0]; v1(3)];
        lcmgl.glVertex3f(v1(1), v1(2), v1(3));
        lcmgl.glVertex3f(v2(1), v2(2), v2(3));
        lcmgl.glEnd();
      end
      lcmgl.switchBuffers();
    end
      
    function v = constructVisualizer(obj)
      v = obj.r.constructVisualizer();
    end

    function bot = get_bot_border(obj)
      bot = [-0.01, 0.01, 0.01, -0.01;
             -0.01, -0.01, 0.01, 0.01];
    end
  end
end