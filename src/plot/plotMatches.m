function plotMatches(matchedPoints_0, matchedPoints_1)

%scatter(matchedPoints_1(:,1),matchedPoints_1(:,1)); 


for i = 1:matchedPoints_0.Count
    x_from = matchedPoints_0.Location(i,1); 
    x_to = matchedPoints_1.Location(i, 1);
    y_from = matchedPoints_0.Location(i, 2);
    y_to = matchedPoints_1.Location(i, 2);
    
    plot([x_from; x_to], [y_from; y_to], 'r-', 'Linewidth', 3);
    hold on; 
end

scatter(matchedPoints_0.Location(:,1),matchedPoints_0.Location(:,2), 'gx');