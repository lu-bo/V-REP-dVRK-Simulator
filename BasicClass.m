
classdef BasicClass
   properties
      Value
   end
   methods
      function r = roundOff(self)
         r = round([self.Value],2);
      end
      function r = multiplyBy(self,n)
         r = self.Value * n;
      end
   end
end