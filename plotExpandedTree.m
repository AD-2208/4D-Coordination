function plotExpandedTree(world,tree,dim)
numNodes = size(tree,1); % no of nodes
N = 100;
theta = 0:2*pi/N:2*pi;
%axis([world.origincorner(1), world.endcorner(1), world.origincorner(2), world.endcorner(2)]);
hold on
for i=1:world.NumObstacles
    X = world.radius(i)*sin(theta) + world.cx(i);
    Y = world.radius(i)*cos(theta) + world.cy(i);
    fill(Y,X,'blue');
end
while numNodes > 0
    branch = []; % stores node coords
    node = tree(numNodes,:); % extract node info from tree
    branch = [branch; node]; % append node info to list
    parent_node = node(dim+3); % parent node info 
    while parent_node > 1 % move up the tree to the root
        cur_parent = parent_node;
        branch = [branch; tree(parent_node,:)];
        parent_node = tree(parent_node,dim+3);
    end
    numNodes = numNodes - 1;
    
    if dim == 2
        X = branch(:,1);
        Y = branch(:,2);
        p = plot(Y,X);
        set(p,'Color','r','LineWidth',0.5,'Marker','.','MarkerEdgeColor','g');
        hold on;
    end
end
end