% q2d extracts a direction-cosine matrix from a quaternion
% 
% Usage:  Q=q2d(q)
%
% q2d expects a 4x1 column (symbolic or complex is fine, but who would ever use
%         a complex quaternion??)
%
function [Q]=q2d(q)
%
Q=(q(4)^2-q(1:3).'*q(1:3))*eye(3)+2*q(1:3)*q(1:3).'-2*q(4)*crs(q(1:3));
%