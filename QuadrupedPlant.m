classdef QuadrupedPlant < DrakeSystem
   methods
    function obj = QuadrupedPlant()
      obj = obj@DrakeSystem(10,0,10,10);
      obj = setOutputFrame(obj,getStateFrame(obj));
    end
   end
end