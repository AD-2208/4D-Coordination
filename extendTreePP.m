function [new_tree,flag] = extendTreePP(tree,end_node,segmentLength,radius,world,flag_chk,dim)

flag1 = 0;
while flag1==0
    % select a random point
    randomPoint = ones(1,dim);
    for i=1:dim
        randomPoint(1,i) = (world.endcorner(i)-world.origincorner(i))*rand;
    end
    
    % find leaf on node that is closest to randomPoint
    tmp = tree(:,1:dim)-ones(size(tree,1),1)*randomPoint;
    sqrd_dist = sqr_eucl_dist(tmp,dim);
    [min_dist,idx] = min(sqrd_dist);
    min_parent_idx = idx;
    
    new_point = (randomPoint-tree(idx,1:dim));
    new_point = tree(idx,1:dim)+(new_point/norm(new_point))*segmentLength;
    
    min_cost  = cost_np(tree(idx,:),new_point,dim);
    new_node  = [new_point, 0, min_cost, idx];
    
    if collision(new_node, tree(idx,:), world, dim) == 0      
        tmp_dist = tree(:,1:dim)-(ones(size(tree,1),1)*new_point);
        dist = sqr_eucl_dist(tmp_dist,dim);
        near_idx = find(dist <= radius^2);
        
        if size(near_idx,1)>1
            size_near = size(near_idx,1);
            
            for i = 1:size_near
                if collision(new_node, tree(near_idx(i),:), world,dim)==0
                    
                    cost_near = tree(near_idx(i),dim+2)+line_cost(tree(near_idx(i),:),new_point,dim);
                    
                    if  cost_near < min_cost
                        min_cost = cost_near;
                        min_parent_idx = near_idx(i);
                    end
                    
                end
            end
        end
        
        new_node = [new_point, 0 , min_cost, min_parent_idx];
        new_tree = [tree; new_node];
        new_node_idx = size(new_tree,1);
        
        if size(near_idx,1)>1
            reduced_idx = near_idx;
            for j = 1:size(reduced_idx,1)
                near_cost = new_tree(reduced_idx(j),dim+2);
                lcost = line_cost(new_tree(reduced_idx(j),:),new_point,dim);
                if near_cost > min_cost + lcost && collision(new_tree(reduced_idx(j),:),new_node,world,dim)
                    before = new_tree(reduced_idx(j),dim+3);
                    new_tree(reduced_idx(j),dim+3) = new_node_idx;
                    after = new_tree(reduced_idx(j),dim+3);
                end
                
            end
        end
        flag1=1;
    end
end


if flag_chk == 0
    % check to see if new node connects directly to end_node
    if ( (norm(new_node(1:dim)-end_node(1:dim))<segmentLength ) && (collision(new_node,end_node,world,dim) == 0) )
        flag = 1;
        new_tree(end,dim+1)=1;  % mark node as connecting to end.
    else
        flag = 0;
    end
    
else 
    flag = 1;
end
end