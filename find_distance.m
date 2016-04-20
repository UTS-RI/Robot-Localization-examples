function [shortest_distance, x_found, y_found] = find_distance(max_range,linedata, x3, y3, theta)

% this function compute the intersection of a laser beam with all the line
% segments and find the intersection with shortest distance
% Shoudong Huang, 2016 April

    line_size = size(linedata,1); % the number of line segments   
      
    shortest_distance = max_range; % Initialise shortest distance;

    %hold on;
    %plot(x3,y3,'g.-');      
    %[x3 y3 (theta*(180/pi))]
    
    for j = 1:line_size
        
        x1 = linedata(j,1);
        y1 = linedata(j,3);
        x2 = linedata(j,2);
        y2 = linedata(j,4);   
        
        %plot([x1,x2],[y1,y2],'-k'); %Plot line

        m3 = tan(theta);
        u = [(y3-y1) + m3*(x1-x3)] / [(y2-y1) - m3*(x2-x1)];
        c = y3 - m3*x3;

        x = x1 + u*(x2-x1);
        y = y1 + u*(y2-y1);
        
        %Check if the intersection point is in the wrong direction%
       
            Heading_wrong = wrap((atan2([y-y3],[x-x3])) + pi);
       
        if (Heading_wrong>theta-0.001)&&(Heading_wrong<theta+0.001)
           distance = max_range;
        else     
           %Check bound of intecept 
            if (x>=x1&&x<=x2)||(x<=x1&&x>=x2) % Within X bounds
                if (y>=y1&&y<=y2)||(y<=y1&&y>=y2) % Within Y bounds
                    distance = sqrt((x-x3)^2+(y-y3)^2);
                 else
                    distance = max_range;
                end
            else
                distance = max_range;
            end
        end
        
        % Update shortest distance
        if distance < shortest_distance
            shortest_distance = distance;
            x_found = x;
            y_found = y;
        end
    end
    
    if shortest_distance == max_range
        x_found = x3; y_found = y3;          
    end
    
   %  plot([x_found],[y_found],'.r'); %plot intercept
             
    %answer = struct('laser',{[shortest_distance;x_found;y_found]});
%     hold off;
end