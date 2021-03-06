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
J_opt = zeros(K,1);
J_diff = ones(K,1);
u_opt_ind = 5*ones(K,1);

while max(J_diff) > 10^(-5)
    J_old = J_opt;
    %VALUE UPDATE
    [repeat_onenumber, repeat_numbers] = meshgrid(1:K);
    u_mesh= meshgrid(u_opt_ind,1:K);
    J_opt = G(sub2ind(size(G), (1:K)', u_opt_ind(:)))+ reshape(P(sub2ind(size(P), repeat_onenumber(:),repeat_numbers(:), u_mesh(:))), [K,K]).'*J_opt; 
    %POLICY IMPROVEMENT
    [~,u_opt_ind(:)]= min(G + reshape(pagemtimes(P(:,:,:),J_opt),[K,5]), [],2);
    %Calculate difference for threshhold
    J_diff = abs(J_opt - J_old);
end
%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
u_opt_ind(TERMINAL_STATE_INDEX)= HOVER;
end