function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G: =q
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).
global K HOVER
 J = zeros(K,1);
 u = 5*ones(K,1);
 loop = true;
 while loop
     J_old = J;
     for i = 1:K
         %VALUE UPDATE
         sum_part_val = 0;
         for j = 1:K
             sum_part_val = sum_part_val + P(i,j,u(i))*J(j); 
         end
         J(i) = G(i,u(i))+ sum_part_val;
         
         %POLICY IMPROVEMENT
         u_pos_val = zeros(5,1);
         for u_count = 1:5
             sum_part_policy =0;
             for j = 1:K
                 sum_part_policy = sum_part_policy + P(i,j,u_count)*J(j);
             end
             u_pos_val(u_count) = G(i,u_count) + sum_part_policy;
         end
         %u(i) = find(min(u_pos_val)==u_pos_val);
         [~,u(i)]= min(u_pos_val);
     end
     %check if we are withing error threshhold
     J_diff = abs(J - J_old);
     loop = max(J_diff) > 10^(-5);
     
 end
 u_opt_ind = u;
 J_opt= J;


%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
u_opt_ind(TERMINAL_STATE_INDEX)= HOVER;

end