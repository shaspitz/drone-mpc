function [J] = computeJ(s,m)

J = zeros(3);
for i = 1:size(s,2)
   J = J + m*oneOTto2OT(s(:,i))'*oneOTto2OT(s(:,i));
end

end