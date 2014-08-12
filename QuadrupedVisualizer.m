classdef QuadrupedVisualizer < Visualizer
   methods
    function obj = QuadrupedVisualizer(plant)
      obj = obj@Visualizer(plant.getOutputFrame);
      obj.playback_speed = .2;
      obj.display_dt = 0;
    end
    
    function draw(obj,t,x)
      persistent hFig;

      if (isempty(hFig))
        hFig = sfigure(25);
        set(hFig,'DoubleBuffer', 'on');
      end
      
      bot = [-0.1, -0.1, 0.1, 0.1; -0.05, 0.05, 0.05, -0.05];
      bot = bsxfun(@plus, bot, x(1:2));
      
      sfigure(25); clf; hold on;
      
      patch(bot(1,:), bot(2,:), 'k');
      
      plot(x(3), x(4), 'ro', 'MarkerSize', 10)
      plot(x(5), x(6), 'go', 'MarkerSize', 10)
      plot(x(7), x(8), 'bo', 'MarkerSize', 10)
      plot(x(9), x(10), 'mo', 'MarkerSize', 10)
      
      axis equal
      xlim([-0.1; 1.1])
      ylim([-0.6; 0.6])
      grid on
    end
   end
end
