classdef QuadrupedPlant < DrakeSystem
   methods
    function obj = QuadrupedPlant()
      obj = obj@DrakeSystem(11,0,11,11);
      obj = setOutputFrame(obj,getStateFrame(obj));
    end
   end
end