classdef QuadrupedVisualizer < Visualizer
  properties
    safe_regions = [];
  end
  
   methods
    function obj = QuadrupedVisualizer(plant, safe_regions)
      obj = obj@Visualizer(plant.getOutputFrame);
      obj.playback_speed = .2;
      obj.display_dt = 0;
      obj.safe_regions = safe_regions;
    end
    
    function draw(obj,t,x)
      persistent hFig;

      if (isempty(hFig))
        hFig = figure(25);
        set(hFig,'DoubleBuffer', 'on');
      end
      
      bot = [-0.1, -0.1, 0.1, 0.1; -0.05, 0.05, 0.05, -0.05];
      yaw = x(3);
      bot = rotmat(yaw) * bot;
      bot = bsxfun(@plus, bot, x(1:2));
      
      figure(25); clf; hold on;
      
      for j = 1:length(obj.safe_regions)
        V = iris.thirdParty.polytopes.lcon2vert(obj.safe_regions(j).A, obj.safe_regions(j).b);
        V = V';
        V = V(1:2, convhull(V(1,:), V(2,:)));
        patch(V(1,:), V(2,:), 'k', 'FaceColor', [0.8,0.8,0.8])
      end
      
      patch(bot(1,:), bot(2,:), 'k');
      
      plot(x(4), x(5), 'ro', 'MarkerSize', 10)
      plot(x(6), x(7), 'go', 'MarkerSize', 10)
      plot(x(8), x(9), 'bo', 'MarkerSize', 10)
      plot(x(10), x(11), 'mo', 'MarkerSize', 10)
      
      
      axis equal
      xlim([-0.1; 1.1])
      ylim([-0.6; 0.6])
%       grid on
    end
   end
end
