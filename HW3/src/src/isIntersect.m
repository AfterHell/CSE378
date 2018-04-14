function intrsct = isIntersect(p1, p2, p3, p4)
% Check if the segment connecting p1 and p2 intesect with the segment connecting p3 and p4
% Copied from: 
% https://stackoverflow.com/questions/27928373/how-to-check-whether-two-lines-intersect-or-not?rq=1

xy = cat(2, p1, p2, p3, p4);
x = xy(1,:);
y = xy(2,:);
dt1=det([1,1,1;x(1),x(2),x(3);y(1),y(2),y(3)])*det([1,1,1;x(1),x(2),x(4);y(1),y(2),y(4)]);
dt2=det([1,1,1;x(1),x(3),x(4);y(1),y(3),y(4)])*det([1,1,1;x(2),x(3),x(4);y(2),y(3),y(4)]);

if (dt1<=0 && dt2<=0)
    intrsct = 1;   %If lines intesect
else
    intrsct = 0;
end
