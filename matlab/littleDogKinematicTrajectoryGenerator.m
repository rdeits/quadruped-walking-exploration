function x_traj = littleDogKinematicTrajectoryGenerator(sol,little_dog_dir,num_knots)
  % Compute trajectory based on foot poses of 

  [t,unique_t_indices] = unique(sol.t);
  if nargin < 3, num_knots = 2*numel(t); end

  t_knot = linspace(0,1,num_knots);
  for field = fields(sol.pose)'
    pose.(field{1}) = sol.pose.(field{1})(:,unique_t_indices);
    pose_knots.(field{1}) = ppval(pchip(t/t(end),pose.(field{1})),t_knot);
  end
  addpath(little_dog_dir);
  robot = LittleDog();
  S = load(fullfile(little_dog_dir,'data','littleDog_fp.mat'));
  q_nom = S.qstar;
  q_nom(3) = 1.1;
  nq = robot.getNumPositions();

  foot_frame_id.lf = robot.parseBodyOrFrameID('front_left_foot_center');
  foot_frame_id.rf = robot.parseBodyOrFrameID('front_right_foot_center');
  foot_frame_id.lh = robot.parseBodyOrFrameID('back_left_foot_center');
  foot_frame_id.rh = robot.parseBodyOrFrameID('back_right_foot_center');

  for leg = {'lh','rh','lf','rf'}
    foot_position_function_struct.(leg{1}) = drakeFunction.kinematic.WorldPosition(robot,foot_frame_id.(leg{1}));
  end

  q_seed = zeros(nq,num_knots);
  com_seed = zeros(3,num_knots);
  A_seed = zeros(6,nq,num_knots);
  ik_prog = NonlinearProgram(nq,robot.getPositionFrame().coordinates);
  ik_prog = ik_prog.addCost(QuadraticConstraint(-Inf,Inf,eye(nq),-q_nom));

  for i = 1:num_knots
    display(i);
    ik_prog_i = ik_prog.addConstraint(ConstantConstraint(pose_knots.body([1:2,4],i)),[1:2,6]);
    for leg_cell = {'rf','lf','rh','lh'}
      leg = leg_cell{1};
      ik_prog_i = ik_prog_i.addConstraint(DrakeFunctionConstraint(pose_knots.(leg)(1:3,i),pose_knots.(leg)(1:3,i),foot_position_function_struct.(leg)));
    end
    q_seed(:,i) = ik_prog_i.solve(q_nom);
    kinsol = robot.doKinematics(q_seed(:,i));
    com_seed(:,i) = robot.getCOM(kinsol);
    A_seed(:,:,i) = robot.getCMM(kinsol);
  end

  x_traj = PPTrajectory(foh(t_knot,[q_seed;0*q_seed]));
  x_traj = x_traj.setOutputFrame(robot.getStateFrame());
end

