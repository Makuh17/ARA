function P = ComputeTransitionProbabilities(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
%
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

L = 5;

no_wind = 1 - P_WIND;
wind = P_WIND/4;

[m_pick_up, n_pick_up] = find(map == PICK_UP);
id_pick_up_nopack = getStateSpaceIndex(stateSpace , m_pick_up,n_pick_up,0);
id_pick_up_pack = getStateSpaceIndex(stateSpace , m_pick_up,n_pick_up,1);

[m_drop_off, n_drop_off] = find(map == DROP_OFF);
id_drop_off_pack = getStateSpaceIndex(stateSpace , m_drop_off,n_drop_off,1);
id_drop_off_nopack = getStateSpaceIndex(stateSpace , m_drop_off,n_drop_off,0);

[m_shooters,n_shooters] = find(map == SHOOTER);
amount_shooters = length(m_shooters);

[m_base, n_base]= find(map == BASE);
id_base = getStateSpaceIndex(stateSpace , m_base,n_base,0);

P = zeros(K,K,L);
for k_current = 1:K
    m_cur = stateSpace(k_current,1);
    n_cur = stateSpace(k_current,2);
    package_cur = stateSpace(k_current,3);

        
    for l = 1:L
        %Base Move
        m_move = 0;
        n_move = 0;
        if l == NORTH
            n_move = 1;
        elseif l == SOUTH
            n_move = -1;
        elseif l == EAST
            m_move = 1;
        elseif l == WEST
            m_move = -1;
        end
        
        m_hop = m_cur + m_move;
        n_hop = n_cur + n_move;
        
        id_hop = getStateSpaceIndex(stateSpace, m_hop , n_hop,package_cur);  
        
        
        if ~isempty(id_hop)
            probability = 0;
            for wind_dir = 1:5
                m_dest = m_hop;
                n_dest = n_hop;
                if wind_dir == NORTH
                    n_dest = n_dest + 1;
                    probability = wind;
                elseif wind_dir == SOUTH
                    n_dest = n_dest - 1;
                    probability = wind;
                elseif wind_dir == EAST
                    m_dest = m_dest + 1;
                    probability = wind;
                elseif wind_dir == WEST
                    m_dest = m_dest - 1;
                    probability = wind;
                elseif wind_dir == HOVER
                    probability = no_wind;
                end
                id_dest = getStateSpaceIndex(stateSpace, m_dest , n_dest,package_cur);
                %Case where we move into a tree or out of the map
                if isempty(id_dest)
                    id_dest = id_base;
                    
                else
                %loop through shooters
                    for shooter = 1:amount_shooters
                        d = abs(m_dest - m_shooters(shooter)) + abs(n_dest - n_shooters(shooter));
                        if (d <= R && d >= 0)
                            probability_shot = GAMMA/(d+1);
                            probability_shotandwind = probability *probability_shot;
                            probability = probability - probability_shotandwind;
                            P(k_current,id_base,l) = P(k_current,id_base,l) + probability_shotandwind;
                        end
                    end
                    %DELIVERY OR PICKUP
                    if (id_dest == id_pick_up_nopack)
                        id_dest = id_pick_up_pack;
                    elseif (id_dest == id_drop_off_pack)
                        id_dest = id_drop_off_nopack;
                    end
                    
                end
                
                P(k_current,id_dest,l) =  P(k_current,id_dest,l) +  probability;
            end
                 
        end

        
    end

   
    
end



end

function id = getStateSpaceIndex(stateSpace, m, n, package)


    id = find(stateSpace(:, 1) == m & stateSpace(:, 2) == n & stateSpace(:, 3) == package);
end
