function [ J_opt, u_opt_ind ] = PolicyIteration(P, G)
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

% Initialization: Always Hover
u_opt_ind = HOVER*ones(K, 1);

iter = 0;
while 1
    % Stage 1: Policy Evaluation
    q = G(sub2ind(size(G), (1:K)', u_opt_ind(:)));
    
    p = zeros(K, K);
    for i = 1:K % go over rows
        p(i,:) = P(i,:,u_opt_ind(i));
        if i == TERMINAL_STATE_INDEX
            p(i,:) = zeros(1, K);
        end
    end
    
    J_opt = (eye(K, K) - p)\q;
    % Stage 2: Policy Improvements
    u_opt_before = u_opt_ind;
    for i = 1:K % for every row in u_opt_ind
        costs = G(i,:)' + squeeze(P(i,:,:))'*J_opt; % 1 by 5 vector
        [val, u_opt_ind(i)] = min(costs);
    end
    
    % Terminate if poliicy didn't change
    if u_opt_before == u_opt_ind
        disp('Finished in ' + string(iter) + ' iterations.')
        break;
    end
    iter = iter + 1;
end

