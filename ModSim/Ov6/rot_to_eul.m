function [ theta,k ] = rot_to_eul( rot_mat )

z=zeros(1,4)
r_ii=max(diag(rot_mat));
i=size(find(diag(rot_mat),r_ii),1);
T=trace(rot_mat);
r_ii=max(r_ii,T);
if(r_ii=T),
    i=0;
end
z(i+1)=sqrt(1+2*r_ii-T);
if((-1 + (1+1)*rand(1,1))<0),
    z(i+1)=-z(i+1);
end

if i == 0,
z0 = sqrt(1 + 2*r_ii - T);
    z(1+1) = (rot_mat(3,2)-rot_mat(2,3))/z(1);
    z(2+1) = (rot_mat(1,3)-rot_mat(3,1))/z(1);
    z(3+1) = (rot_mat(2,1)-rot_mat(1,2))/z(1);
elseif i == 1,
    z(1+1) = sqrt(1 + 2*r_ii - T);
    z(1) = (rot_mat(3,2)-rot_mat(2,3))/z(1+1);
    z(2+1) = (rot_mat(2,1)+rot_mat(1,2))/z(1+1);
    z(3+1) = (rot_mat(1,3)+rot_mat(3,1))/z(1+1);
elseif i == 2,
    z(2+1) = sqrt(1 + 2*r_ii - T);
    z(1) = (rot_mat(1,3)-rot_mat(3,1))/z(2+1);
    z(1+1) = (rot_mat(2,1)+rot_mat(1,2))/z(2+1);
    z(3+1) = (rot_mat(3,2)+rot_mat(2,3))/z(2+1);
elseif i == 3,
    z(3+1) = sqrt(1 + 2*r_ii - T);
    z(1) = (rot_mat(2,1)-rot_mat(1,2))/z(3);
    z(1+1) = (rot_mat(1,3)+rot_mat(3,1))/z(3);
    z(2+1) = (rot_mat(3,2)+rot_mat(2,3))/z(3);
end

nj=z(1)/2
e=zeros(3);
e(1)=z(2)/2;
e(2)=z(3)/3;
e(3)=z(4)/4;

theta= 2*acos(nj);
k = e/(sin(theta/2));
