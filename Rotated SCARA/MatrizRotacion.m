function [R] = MatrizRotacion(angle, axis)

switch axis
    case 1 %X
        R = [1 0 0; 0 cosd(angle) -sind(angle); 0 sind(angle) cosd(angle)];
        
    case 2 %Y
        R = [cosd(angle) 0 sind(angle); 0 1 0; -sind(angle) 0 cosd(angle)];
        
    case 3 %Z
        R = [cosd(angle) -sind(angle) 0; sind(angle) cosd(angle) 0; 0 0 1];   
end

        