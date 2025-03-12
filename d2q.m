%
% d2q extracts a quaternion from a direction cosine matrix without singularities
% 
% Usage:  q=d2q(Q,method)
%
%         where method=1 uses the singularity-ridden classical formula
%               method=2 uses Davenport's q method with Mason's interpretation.
%         The default is method=1, which is faster.
%         For the case where method 1 is singular (within sqrt(eps)), the 
%         function uses method 2.

function [q]=d2q(Q,method)
%
qt=(1+trace(Q));
%
if method==2|abs(qt)<=sqrt(eps),
    r=[1 0 0].';
    s=[0 1 0].';
    b=Q(:,1);
    c=Q(:,2);
    B=b*r.'+c*s.';
    K=[    B+B.'-eye(3)*trace(B)   cross(b,r)+cross(c,s)
       (cross(b,r)+cross(c,s)).'                trace(B)];
    [V,lam]=eig(K);
    lam=diag(lam);
    im=find(lam==max(lam));
    q=V(:,im(1));
else
    if method==1,
        q=[Q(2,3)-Q(3,2); Q(3,1)-Q(1,3); Q(1,2)-Q(2,1); qt];
        q=q/norm(q);
    else
        disp('Method must be =1 or =2')
    end
end
%

