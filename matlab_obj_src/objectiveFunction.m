function J =  objectiveFunction(t, z, desired_pos, p)

% desired_pos = [xd, yd] as a row vector

% t_star = t(end);
% for k = 1:p.N
%     current_state = z(p.nstates*(k-1)+1 : p.nstates*k);
%     pos = p.model.kinematics(current_state);
%     current_pos = pos(end,:);
%     if norm(current_pos - desired_pos) < p.eps
%         t_star = min(t_star, t(k));
%     end
% end
% J = t_star;


J = 0;
for k = 1:p.N
    current_state = z(p.nstates*(k-1)+1 : p.nstates*k);
    pos = p.model.kinematics(current_state);
    current_pos = pos(end,:);
    J = J + 0.05*norm(current_pos - desired_pos);
end

end