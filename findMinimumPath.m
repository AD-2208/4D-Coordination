function [rovPath, pathDistance] = findMinimumPath(tree, end_node, dim)

% Initialize path distance
pathDistance = 0;

% find nodes that connect to end_node
connectingNodes = [];
for i=1:size(tree,1)
    if tree(i,dim+1)==1
        connectingNodes = [connectingNodes ; tree(i,:)];
    end
end

if size(connectingNodes, 1) > 0
    
    % find minimum cost last node
    [~, idx] = min(connectingNodes(:,dim+2));
    
    % construct lowest cost path
    rovPath = [connectingNodes(idx,:); end_node];
    parent_node = connectingNodes(idx,dim+3);
    while parent_node>1
        parentNodeIndex = parent_node;  % Store the current parent node index
        parent_node = tree(parent_node,dim+3);
        rovPath = [tree(parent_node,:); rovPath];
        
        % Calculate distance between current node and its parent
        currentNode = tree(parentNodeIndex, 1:dim);
        parentNode = tree(parent_node, 1:dim);
        pathDistance = pathDistance + norm(currentNode - parentNode);
    end
else
    rovPath = [];
end

end
