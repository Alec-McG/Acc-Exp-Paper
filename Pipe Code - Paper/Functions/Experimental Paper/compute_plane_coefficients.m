%--------------------------------------------------------------------------
% Description - From 3 points compute the equation of the plane. Compute 
%               the normal from the vectors defined by the points then 
%               solve for the constant d
%
% Parameters  - p = [x x x
%                    y y y
%                    z z z]
%--------------------------------------------------------------------------

function [c, n] = compute_plane_coefficients(p)

P1 = p(:,1);
P2 = p(:,2);
P3 = p(:,3);

n  = cross(P1-P2, P1-P3)';
n = n/norm(n);

% Solve for constant d
d = -n(1)*P1(1)-n(2)*P1(2)-n(3)*P1(3);
c = [n d];
