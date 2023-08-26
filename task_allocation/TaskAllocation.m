%% Task allocation based on Hungarian Algorithm
function [goal_set,IND] = TaskAllocation(goalset,cost)
dim = size(cost,1);
[zero_primeM,Contain_zero_row,Contain_zero_col] = reset_zero(dim);
flag = true;
markedM = zero_primeM;
% Step1: For each row of the matrix, find the smallest element and subtract it from every element in its row. Go to Step 2.
C_sub = zeros(dim,dim);
for i = 1:dim
    C_sub(i,:) = cost(i,:) - min(cost(i,:));
end
% Step2: Find a zero (Z) in the resulting matrix. If there is no starred zero in its row or column, star (Z). Repeat for each element in the matrix. Go to Step 3.
markedM = find_zero(dim,C_sub,markedM);
step = [1 0 0 0 0];
while step(5) == 0
    % Step3: Cover each column containing a starred zero. If K columns are covered the starred zeros describe a complete set of unique assignments. In this case, Go to DOWN, otherwise, Go to Step 4.
    if step(1) == 1
        for j = 1:dim
            if ~isempty(find(markedM(:,j),1))
                Contain_zero_col(j) = 1;
            end
        end
        flag_col_all = length(find(Contain_zero_col==1)) == dim;
        if flag_col_all
            step = [0 0 0 0 1];
        else
            step = [0 1 0 0 0];
        end
    end
    % Step4: Find a noncovered zero and prime it. If there is no starred zero in the row containing this primed zero. Go to Step 5. Otherwise,
    % cover this row and uncover the column containing the starred zero.
    % Continue in this manner until there are no uncovered zeros left. Save the smallest uncovered value and Go to Step 6.
    if step(2) == 1
        contain_zero_mr = Contain_zero_row * ones(1,dim);
        contain_zero_mc = ones(dim,1) * Contain_zero_col;
        C_temp = contain_zero_mr + contain_zero_mc + C_sub;
        [zind_r, zind_c] = find(C_temp == 0);
        for j = 1:length(zind_r)
            zind_r_prime = zind_r(j); zind_c_prime = zind_c(j);
            zero_primeM(zind_r_prime, zind_c_prime) = 1;
            if ~isempty(find(markedM(zind_r_prime,:)))
                Contain_zero_row(zind_r_prime) = 1;
                Contain_zero_col(find(markedM(zind_r_prime,:) == 1)) = 0;
            else
                % Go to Step 5:
                step = [0 0 1 0 0];
                break;
            end
        end
        if step(3) == 0
            Contain_row_M = fill_zero(Contain_zero_row,dim);
            Contain_col_M = fill_zero(Contain_zero_col,dim);
            C_temp_min = min(min(C_sub + Contain_row_M + Contain_col_M));
            % Go to Step 6:
            step = [0 0 0 1 0];
        end
    end
    if step(3) == 1
        % Step5: Construct a series of alternating primed and starred zeros as follows. Let Z0 represent the uncovered primed zero found in Step 4. 
        % Let Z1 denote the starred zero in the column of Z0 (if any). Let Z2 denote the primed zero in the row of Z1 (there will always be one). 
        % Continue until the series terminates at a primed zero that has no starred zero in its column. Unstar each starred zero of the series, star each primed zero of the series, erase
        % all primes and uncover every line in the matrix. Return to Step 3.
        Z = [zind_r_prime,zind_c_prime];
        while flag
            mark_ind_r = find(markedM(:,zind_c_prime) == 1);
            markedM(zind_r_prime, zind_c_prime) = 1;
            if ~isempty(mark_ind_r)
                markedM(mark_ind_r(1),zind_c_prime) = 0;
                zind_r_prime =mark_ind_r(1);
                Z = [Z;[mark_ind_r(1),zind_c_prime]];
                zind_c_prime = find(zero_primeM(zind_r_prime,:) == 1);
                zind_c_prime = zind_c_prime(1);
                Z = [Z;[zind_r_prime, zind_c_prime]];
            else
                flag = false;
            end
        end
        [zero_primeM,Contain_zero_row,Contain_zero_col] = reset_zero(dim);
        % Return to Step 3:
        step = zeros(1,5);
        step(1) = 1;
    end
    if step(4) == 1
        % Step6: Add the value found in Step 4 to every element of each covered row, and subtract it from every element of each uncovered column. Return to Step 4 without altering any stars, primes, or covered lines.
        % Return to Step 4:
        step = [0 1 0 0 0];
        C_temp_raw = Contain_zero_row * ones(1,dim) * C_temp_min;
        C_sub = C_sub + C_temp_raw;
        C_temp_col = ones(dim,1) * (ones(1,dim) - Contain_zero_col) * C_temp_min;
        C_sub = C_sub - C_temp_col;
    end
end
% DONE: Assignment pairs are indicated by the positions of the starred zeros in the cost matrix. If C(i,j) is a starred zeros, then the element associated with row i is assigned to the element associated with column j.
% [idx1,idx2] = find(markedM == 1);
% Cost = 0;
% for j = 1:length(idx1)
%     Cost = Cost + cost(idx1(j),idx2(j));
% end
for i = 1:size(markedM,1)
    IND(i) = find(markedM(i,:)==1);
end
for i = 1:size(goalset,1)
    goal_set(i,1:3) = goalset(IND(i),1:3);
end
end
function markedM = find_zero(dim,C_sub,markedM)
for i = 1:dim
    for j = 1:dim
        if C_sub(i,j) == 0 && isempty(find(markedM(i,:),1)) && isempty(find(markedM(:,j),1))
            markedM(i,j) = 1;
        end
    end
end
end
function Contain_M = fill_zero(Contain_zero,dim)
if size(Contain_zero,2) == 1
    Contain_M = Contain_zero * ones(1,dim) * inf;
else
    Contain_M = ones(dim,1) * Contain_zero * inf;
end
Contain_M(find(isnan(Contain_M)==1)) = 0;
end
function [zero_primeM,Contain_zero_row,Contain_zero_col] = reset_zero(dim)
zero_full = zeros(dim,dim);
Contain_zero_row = zeros(dim,1);
Contain_zero_col = zeros(1,dim);
zero_primeM = zero_full;
end
% function [goal_set,IND] = TaskAllocation(goalset,cost)
% dim = size(cost,1);
% markedM = zeros(dim,dim);
% zero_primeM = zeros(dim,dim);
% Contain_zero_row = zeros(dim,1);
% Contain_zero_col = zeros(1,dim);
% % Step1: For each row of the matrix, find the smallest element and subtract it from every element in its row. Go to Step 2.
% C_sub = zeros(dim,dim);
% parfor i = 1:dim
%     raw_min = min(cost(i,:));
%     C_sub(i,:) = cost(i,:) - raw_min;
% end
% % Step2: Find a zero (Z) in the resulting matrix. If there is no starred zero in its row or column, star (Z). Repeat for each element in the matrix. Go to Step 3.
% for i = 1:dim
%     for j = 1:dim
%         if C_sub(i,j) == 0 && isempty(find(markedM(i,:),1)) && isempty(find(markedM(:,j),1))
%             markedM(i,j) = 1;
%         end
%     end
% end
% step = zeros(1,5);
% step(1) = 1;
% while step(5) == 0
%     % Step3: Cover each column containing a starred zero. If K columns are covered the starred zeros describe a complete set of unique assignments. In this case, Go to DOWN, otherwise, Go to Step 4.
%     if step(1) == 1
%         parfor j = 1:dim
%             if ~isempty(find(markedM(:,j),1))
%                 Contain_zero_col(j) = 1;
%             end
%         end
%         if length(find(Contain_zero_col==1)) == dim
%             step = zeros(1,5);
%             step(5) = 1;
%         else
%             step = zeros(1,5);
%             step(2) = 1;
%         end
%     end
%     % Step4: Find a noncovered zero and prime it. If there is no starred zero in the row containing this primed zero. Go to Step 5. Otherwise,
%     % cover this row and uncover the column containing the starred zero.
%     % Continue in this manner until there are no uncovered zeros left. Save the smallest uncovered value and Go to Step 6.
%     if step(2) == 1
%         C_temp = Contain_zero_row * ones(1,dim) + ones(dim,1) * Contain_zero_col + C_sub;
%         [zind_r, zind_c] = find(C_temp == 0);
%         for j = 1:length(zind_r)
%             zind_r_prime = zind_r(j);
%             zind_c_prime = zind_c(j);
%             zero_primeM(zind_r_prime, zind_c_prime) = 1;
%             if isempty(find(markedM(zind_r_prime,:)))
%                 % Go to Step 5:
%                 step = zeros(1,5);
%                 step(3) = 1;
%                 break;
%             else
%                 Contain_zero_row(zind_r_prime) = 1;
%                 ind_r = find(markedM(zind_r_prime,:) == 1);
%                 Contain_zero_col(ind_r) = 0;
%             end
%         end
%         if step(3) == 0
%             Contain_row_M = Contain_zero_row * ones(1,dim) * inf;
%             Contain_row_M(find(isnan(Contain_row_M)==1)) = 0;
%             Contain_col_M = ones(dim,1) * Contain_zero_col * inf;
%             Contain_col_M(find(isnan(Contain_col_M)==1)) = 0;
%             C_temp = C_sub + Contain_row_M + Contain_col_M;
%             C_temp_min = min(min(C_temp));
%             [C_temp_min_r, C_temp_min_c] = find(C_temp == C_temp_min);
%             C_temp_min_r = C_temp_min_r(1);
%             C_temp_min_c = C_temp_min_c(1);
%             % Go to Step 6:
%             step = zeros(1,5);
%             step(4) = 1;
%         end
%     end
%     if step(3) == 1
%         % Step5: Construct a series of alternating primed and starred zeros as follows. Let Z0 represent the uncovered primed zero found in Step 4. 
%         % Let Z1 denote the starred zero in the column of Z0 (if any). Let Z2 denote the primed zero in the row of Z1 (there will always be one). 
%         % Continue until the series terminates at a primed zero that has no starred zero in its column. Unstar each starred zero of the series, star each primed zero of the series, erase
%         % all primes and uncover every line in the matrix. Return to Step 3.
%         flag = true;
%         Z = [zind_r_prime,zind_c_prime];
%         while flag
%             mark_ind_r = find(markedM(:,zind_c_prime) == 1);
%             markedM(zind_r_prime, zind_c_prime) = 1;
%             if isempty(mark_ind_r)
%                 flag = false;
%             else
%                 Z_temp = [mark_ind_r(1),zind_c_prime];
%                 Z = [Z;Z_temp];
%                 markedM(Z_temp(1),Z_temp(2)) = 0;
%                 zind_r_prime = Z_temp(1);
%                 zind_c_prime = find(zero_primeM(zind_r_prime,:) == 1);
%                 zind_c_prime = zind_c_prime(1);
%                 Z_temp = [zind_r_prime, zind_c_prime];
%                 Z = [Z;Z_temp];
%             end
%         end
%         zero_primeM = zeros(dim,dim);
%         Contain_zero_row = zeros(dim,1);
%         Contain_zero_col = zeros(1,dim);
%         % Return to Step 3:
%         step = zeros(1,5);
%         step(1) = 1;
%     end
%     if step(4) == 1
%         % Step6: Add the value found in Step 4 to every element of each covered row, and subtract it from every element of each uncovered column. Return to Step 4 without altering any stars, primes, or covered lines.
%         C_sub = C_sub + Contain_zero_row * ones(1,dim) * C_temp_min;
%         Uncontain_zero_col = ones(1,dim) - Contain_zero_col;
%         C_sub = C_sub - ones(dim,1) * Uncontain_zero_col * C_temp_min;
%         % Return to Step 4:
%         step = zeros(1,5);
%         step(2) = 1;
%     end
% end
% % DONE: Assignment pairs are indicated by the positions of the starred zeros in the cost matrix. If C(i,j) is a starred zeros, then the element associated with row i is assigned to the element associated with column j.
% % [idx1,idx2] = find(markedM == 1);
% % Cost = 0;
% % for j = 1:length(idx1)
% %     Cost = Cost + cost(idx1(j),idx2(j));
% % end
% for i = 1:size(markedM,1)
%     IND(i) = find(markedM(i,:)==1);
% end
% for i = 1:size(goalset,1)
%     goal_set(i,1:3) = goalset(IND(i),1:3);
% end
% end