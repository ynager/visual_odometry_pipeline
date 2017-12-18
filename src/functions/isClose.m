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
    query_curr = query(i,:);
    for j=1:reference_l
        reference_curr = reference(j,:);
        
        not_valid = norm(query_curr-reference_curr) < delta; 
        
        if not_valid
            isclose(i)=1;
        end
    end

end

