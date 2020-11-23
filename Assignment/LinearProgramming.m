function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
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

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
% the x vector is simply the J(i)s. the f is going to be -1*ones(length(J))
% for the constraint we have to stack all the different pages for the
% inputs.
% the inequality constraint directly follows from the 
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
%%
f = -1*ones(K,1);

P_rs = [P(:,:,1);P(:,:,2);P(:,:,3);P(:,:,4);P(:,:,5)];
G_rs = [G(:,1);G(:,2);G(:,3);G(:,4);G(:,5)];
G_rs(G_rs==Inf) = 10e12;

P_aug = [eye(K);eye(K);eye(K);eye(K);eye(K)]-P_rs;

b_eq = 0;
A_eq = zeros(K,1); A_eq(TERMINAL_STATE_INDEX) = 1;

J = linprog(f, P_aug, G_rs, A_eq', b_eq);
u_all = zeros(length(J),5);
for l = 1:5
    u_all(:,l) = G(:,l)+P(:,:,l)*J;
end 
J_opt = J;
[~,u_opt_ind] = min(u_all,[],2);
u_opt_ind(TERMINAL_STATE_INDEX) = HOVER;
end

