safe_regions = struct('A', {}, 'b', {}, 'point', {}, 'normal', {});
V = [-0.15, 0.2 .2, -0.15; 0.2, 0.2, -0.2, -0.2];
[A, b] = poly2lincon(V(1,:), V(2,:));
safe_regions(end+1) = struct('A', A, 'b', b, 'point', [.3;0;1], 'normal', [0;0;1]);

V = [0.25, 0.4 .4, 0.25; 0, 0, -0.3, -0.3];
[A, b] = poly2lincon(V(1,:), V(2,:));
safe_regions(end+1) = struct('A', A, 'b', b, 'point', [.3;0;1], 'normal', [0;0;1]);

V = [.59, 2, 2, .59; 1, 1, -1, -1];
[A, b] = poly2lincon(V(1,:), V(2,:));
safe_regions(end+1) = struct('A', A, 'b', b, 'point', [.8;0;1], 'normal', [0;0;1]);

V = [.5,.51,.5,.51; -.05, -.05, -.06, -.06];
[A, b] = poly2lincon(V(1,:), V(2,:));
safe_regions(end+1) = struct('A', A, 'b', b, 'point', [.5;-.05;1], 'normal', [0;0;1]);

V = [.5,.51,.51,.5; .05, .05, .06, .06];
[A, b] = poly2lincon(V(1,:), V(2,:));
safe_regions(end+1) = struct('A', A, 'b', b, 'point', [.5;.05;1], 'normal', [0;0;1]);

V = [.45,.46,.46,.45; 0, 0, .01, .01];
[A, b] = poly2lincon(V(1,:), V(2,:));
safe_regions(end+1) = struct('A', A, 'b', b, 'point', [.5;.05;1], 'normal', [0;0;1]);


start = struct('body', [0;0], 'rf', [0.1;-0.05],...
                              'lf', [0.1;0.05],...
                              'rh', [-0.1;-0.05],...
                              'lh', [-0.1;0.05]);
goal = struct('body', [1;0]);

gait = struct('rf', {}, 'lf', {}, 'rh', {}, 'lh', {});
feet = fieldnames(gait)';

v = SimpleGaitVisualizer(feet);
v.safeRegions = safe_regions;

gait(1) = struct('rf', true, 'lf', true, 'rh', true, 'lh', true);
gait(2) = struct('rf', true, 'lf', true, 'rh', false, 'lh', true);
gait(3) = struct('rf', true, 'lf', false, 'rh', false, 'lh', true);
gait(4) = struct('rf', true, 'lf', false, 'rh', true, 'lh', true);
gait(5) = struct('rf', true, 'lf', true, 'rh', true, 'lh', true);
gait(6) = struct('rf', true, 'lf', true, 'rh', true, 'lh', false);
gait(7) = struct('rf', false, 'lf', true, 'rh', true, 'lh', false);
gait(8) = struct('rf', false, 'lf', true, 'rh', true, 'lh', true);

num_periods = 8;
frames_in_gait = length(gait);
for j = 2:num_periods
  gait = [gait, gait(1:frames_in_gait)];
end
gait(end+1) = struct('rf', true, 'lf', true, 'rh', true, 'lh', true);
nsteps = length(gait);

body_pos = sdpvar(2, nsteps, 'full');
feet_pos = struct('rf', sdpvar(2, nsteps, 'full'), 'lf', sdpvar(2, nsteps, 'full'), 'rh', sdpvar(2, nsteps, 'full'), 'lh', sdpvar(2, nsteps, 'full'));
dt = sdpvar(1, nsteps, 'full');
nr = length(safe_regions);
region = struct('rf', binvar(nr, nsteps, 'full'), 'lf', binvar(nr, nsteps, 'full'), 'rh', binvar(nr, nsteps, 'full'), 'lh', binvar(nr, nsteps, 'full'));
sin_yaw = sdpvar(1, nsteps, 'full');
cos_yaw = sdpvar(1, nsteps, 'full');
rot_sector = binvar(4, nsteps, 'full');

foci = struct('rf', struct('v', {[0.1; -0.05]}, 'r', {0.05}),...
              'lf', struct('v', {[0.1; 0.05]}, 'r', {0.05}),...
              'rh', struct('v', {[-0.1; -0.05]}, 'r', {0.05}),...
              'lh', struct('v', {[-0.1; 0.05]}, 'r', {0.05}));
SWING_SPEED = 1;
BODY_SPEED = 0.25;
MAX_DISTANCE = 30;
ROTATION_RATE = 1;

constraints = [body_pos(:,1) == start.body,...
               dt >= 0,...
               start.body(1) - MAX_DISTANCE <= body_pos(1,:) <= start.body(1) + MAX_DISTANCE,...
               start.body(2) - MAX_DISTANCE <= body_pos(2,:) <= start.body(2) + MAX_DISTANCE,...
               sum(rot_sector, 1) == 1,...
               -1 <= sin_yaw <= 1,...
               -1 <= cos_yaw <= 1,...
%                cos_yaw(end) >= 0.9,... % Turn this off to let the robot finish with arbitrary orientation
               ];
for f = feet
  foot = f{1};
  constraints = [constraints, feet_pos.(foot)(:,1) == start.(foot), ...
                 sum(region.(foot), 1) == 1,...
%                  region.(foot)(1,:) == 1,...
                 start.(foot)(1) - MAX_DISTANCE <= feet_pos.(foot)(1,:) <= start.(foot)(1) + MAX_DISTANCE,...
                 start.(foot)(2) - MAX_DISTANCE <= feet_pos.(foot)(2,:) <= start.(foot)(2) + MAX_DISTANCE,...
                 ];
end
constraints = [constraints, body_pos(:,end) == goal.body];


objective = sum(dt);

for j = 1:nsteps
  for f = feet
    foot = f{1};
    
    % Enforce reachability
    for k = 1:length(foci.(foot))
      c = foci.(foot)(k);
      constraints = [constraints, cone(feet_pos.(foot)(:,j) - (body_pos(:,j) + [cos_yaw(j), -sin_yaw(j); sin_yaw(j), cos_yaw(j)] * c.v), c.r)];
    end
    
    % Enforce region membership
    for r = 1:nr
      constraints = [constraints, implies(region.(foot)(r,j), safe_regions(r).A * feet_pos.(foot)(:,j) <= safe_regions(r).b)];
    end
    
    if j < nsteps
      % Enforce fixed feet and swing speed for free feet
      if gait(j).(foot)
        constraints = [constraints, feet_pos.(foot)(:,j) == feet_pos.(foot)(:,j+1),...
                       region.(foot)(:,j) == region.(foot)(:,j+1)];
      else
        constraints = [constraints, cone(feet_pos.(foot)(:,j+1) - feet_pos.(foot)(:,j), dt(j) * SWING_SPEED)];
      end
      objective = objective + norm(feet_pos.(foot)(:,j+1) - feet_pos.(foot)(:,j))...
                            + norm(body_pos(:,j+1) - body_pos(:,j));
    end
  end
  if j < nsteps
    constraints = [constraints, cone(body_pos(:,j+1) - body_pos(:,j), dt(j) * BODY_SPEED),...
                   cone([sin_yaw(j+1); cos_yaw(j+1)] - [sin_yaw(j); cos_yaw(j)], dt(j) * ROTATION_RATE),...
                   ];
  end
  
  % Enforce sin/cos sectors
  constraints = [constraints, implies(rot_sector(1, j), sin_yaw(j) == 1 - cos_yaw(j)),...
                              implies(rot_sector(2, j), sin_yaw(j) == 1 + cos_yaw(j)),...
                              implies(rot_sector(3, j), sin_yaw(j) == -1 - cos_yaw(j)),...
                              implies(rot_sector(4, j), sin_yaw(j) == -1 + cos_yaw(j))];
end

if 1
  solvesdp(constraints, objective, sdpsettings('solver', 'gurobi'));

  body_pos = double(body_pos);
  for f = feet
    foot = f{1};
    feet_pos.(foot) = double(feet_pos.(foot));
  end
  dt = double(dt);
  t = [0, cumsum(dt(1:end-1))];

  % figure(1);
  % clf
  % hold on
  % plot(t, body_pos(1,:), 'ko')
  % % t, feet_pos.rf(1,:), t, feet_pos.lf(1,:), t, feet_pos.rh(1,:), t, feet_pos.lh(1,:))
  % % legend('body', 'rf', 'lf', 'rh', 'lh')
  % colors = {'r', 'g', 'b', 'm'};
  % for f = 1:length(feet)
  %   foot = feet{f};
  %   color = colors{f};
  %   for j = 1:nsteps-1
  %     if gait(j).(foot)
  %       plot([t(j), t(j+1)], [feet_pos.(foot)(1,j), feet_pos.(foot)(1,j+1)], '-', 'Color', color)
  %     else
  %       plot([t(j), t(j+1)], [feet_pos.(foot)(1,j), feet_pos.(foot)(1,j+1)], 'o', 'Color', color)
  %     end
  %   end
  % end
  % 
  % figure(2)
  % clf
  % hold on
  % for j = 1:nsteps-1
  %   for f = 1:length(feet)
  %     foot = feet{f};
  %     color = colors{f};
  %     if gait(j).(foot)
  %       plot([t(j), t(j+1)], [f, f], '-', 'Color', color, 'LineWidth', 5)
  %     end
  %   end
  % end
  % legend(feet{:})

  yaw = atan2(double(sin_yaw), double(cos_yaw));
  z = ones(size(yaw));
  x = [body_pos; z; yaw; 
       feet_pos.(feet{1}); z; yaw;
       feet_pos.(feet{2}); z; yaw; 
       feet_pos.(feet{3}); z; yaw;
       feet_pos.(feet{4}); z; yaw];
  xtraj = PPTrajectory(foh(t, x));
else
  load('xtraj.mat', 'xtraj');
end
xtraj = setOutputFrame(xtraj, v.inputFrame);

v.playback(xtraj, struct('slider', true));