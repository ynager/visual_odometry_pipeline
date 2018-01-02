function [isclose] = isClose(query,reference,delta)
%ISCLOSE function replacing the previes "ismember" fct.
% ismember checks if it's exactly the same entry, isClose checks instead
% if the input query (Mx2) is close to reference (Nx2) up to a certain
% delta
%output: validity (Mx1) 1 if is valid, 0 if a similiar entry was found

query_l = size(query,1);
reference_l = size(reference,1);

isclose = zeros(query_l,1);

for i=1:query_l
    for j=1:reference_l
        
        isclose(i) = (query(i,1)-reference(j,1))^2 + (query(i,2)-reference(j,2))^2 < delta^2;
        
        if isclose(i)
            break; 
        end
        
    end
end

