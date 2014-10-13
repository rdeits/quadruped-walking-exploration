function [footsteps, stance_times] = convertFootstepPlan(plan,robot)
  if nargin < 2
    foot_frame_id.lf = 0;
    foot_frame_id.rf = 0;
    foot_frame_id.lh = 0;
    foot_frame_id.rh = 0;
  else
    foot_frame_id.lf = robot.parseBodyOrFrameID('front_left_foot_center');
    foot_frame_id.rf = robot.parseBodyOrFrameID('front_right_foot_center');
    foot_frame_id.lh = robot.parseBodyOrFrameID('back_left_foot_center');
    foot_frame_id.rh = robot.parseBodyOrFrameID('back_right_foot_center');
  end

gait_graph = plan.full_gait;
footsteps = [];
stance_times = [];
for field = fields(gait_graph)'
  [stance_times_leg,footsteps_leg] = ...
    getStancePhases(plan.t,[gait_graph.(field{1})],plan.pose.(field{1}));
  stance_times = [stance_times, stance_times_leg];
  footsteps = [footsteps; footsteps_leg];
end
end

function [stance_times,footsteps] = getStancePhases(t,in_stance,pose)
  stance_times = [];
  footsteps = [];
  currently_in_stance = false;
  for i = 1:numel(t)
    if ~currently_in_stance && in_stance(i)
      stance_times(1,end+1) = t(i);
      footsteps = [footsteps; Footstep([pose(1:3,i);zeros(3,1)],[],true,true,[],[],[],[])];
      currently_in_stance = true;
    elseif currently_in_stance && ~in_stance(i)
      stance_times(2,end) = t(i);
      currently_in_stance = false;
    end
  end
  if in_stance(end)
    stance_times(2,end) = t(end);
  end
end
