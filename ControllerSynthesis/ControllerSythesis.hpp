#pragma once
#include <array>
#include <vector>
#include "../robotControl/dynamics.hpp"
#include "../mission-rules/labeling&MissionRules.hpp"
#include "../robotControl/state2coord.hpp"
#include "../robotControl/coord2state.hpp"
#include "../Reachablity/reachability.hpp"
#include "../Safety/safety_synthesis.hpp"
#include <algorithm>
#include <cstdint>

using namespace dyn;

struct Controller{
    std::vector<std::array<int,5>> rank;
    std::vector<std::array<int,5>> command;
};

/*
for each cell we will define an array of 5 commands and an array of 5 ranks
we call them by rank[cell][0] , rank[cell][1]... and command[cell][1], command[cell][2] ...
but why do we have 5 rank entries and 5 entries for each cell ?
because it depends of what already was visited ("START","VISITED_R1"...)
the rank is defined as the number of movement needed to be accept 
rank -1 : Completion is not certified 
that means tha we haven't find any way to guarantee success
rank 0 : No more movement needed
rank 1 : At most 1 movement left
rank 2 : At most 2 movements left
and same for rank 3, 4 ....

for the command here we consider it as the correct command the robot should 
take now.
*/

using Transitions = std::vector<std::array<int32_t,2>>;
// the transitional table (the concretisation of the initial f function ) above stores for each (cell,command) two corner 
// cell ID describing the box of possibel next cells 

/*
our synthetize_controller function takes a trasional table and return a
command structure which is a list of ranks and a list commands (each rank and command)
depend on the mission state of the cell)
*/
inline Controller synthesize_controller(const Transitions& g){
    const int n_states = n_x[0] * n_x[1] * n_x[2]; // total number of cells
    Controller controller;
    controller.rank.resize(n_states + 1);
    controller.command.resize(n_states + 1);   
    // allocating the size to both vectors  
    for(int cell = 0; cell <= n_states; cell ++){
        controller.rank[cell].fill(-1);
        controller.command[cell].fill(0);
    }

    const int success = static_cast<int>(Mission::SUCCESS) ;// its index 3 
    for(int cell = 1; cell <= n_states; cell ++ ){
        controller.rank[cell][success] = 0;
        // we set the rank of each success state which has index 3 to 0
    }

    const int failure = static_cast<int>(Mission::FAILURE);
    for(int cell = 1 ; cell <= n_states ; cell ++){
        controller.rank[cell][failure] = -1 ;
        // unnecessary because we already set all elements to -1 in line 46 but just
        // for clarity 
    }

    const int n_commands = n_u[0] * n_u[1];
    const Transitions g_safe = filter_safe_transitions(g);
    // we use the safety algorithm to avoid the forbidden zone which is R4
    // we give it a transitional table (discretisation of the original function f)
    // and it returns a new transitional table that for each command that leads to
    // the forbidden zone it returns {0,0}  

    for(int k = 1; ; k++){
        const auto previous_rank = controller.rank;
        // previous snapshots of the ranks of each cell (depending on the previous mission state)
        // that's why we will loop over mission state indices (m) {START, VISITED_R1, VISITED_R2}
        bool changed = false ;
        for(int m = 0 ; m < 3 ; m++){
            std::vector<int> A;
            for(int next_cell = 1 ; next_cell <= n_states ; next_cell ++){
                // we loop throught every cell
                Mission next_mission = update_mission(static_cast<Mission>(m) , label(next_cell));
                // then we find the next mission by first finding which reagion the next_cell belongs to
                // and the current mission which is m 
                int next_m = static_cast<int>(next_mission);
                // we convert the mission to an int (index)
                if(previous_rank[next_cell][next_m] >= 0){
                    A.push_back(next_cell);
                }
                // we populate our vector A by the next_cells that are certified 
                // (all the cells that doesn't have a negative rank)
            }
            const auto predecessors = reach::pred(g_safe,A);
            /*
            So we know that A have only next cells that have positive ranks 
            positive rank means that from those cells we have a strategy (set of commands) to 
            guaretee the success 
            predecessors function return a vector of (cell,command)
            in which applying command starting from cell will lead us to A  
            */
            for(const auto& choice : predecessors){
                int cell = choice.state; // state here is the starting cell
                if(previous_rank[cell][m] != -1){
                    continue;
                    // we skip the cell if we already certified that cell (we assigned a positive rank to it)
                }
                controller.rank[cell][m] = k;
                // we assign the rank k to the cells that belongs to the predecessors of the certified cell
                controller.command[cell][m] = choice.command;
                // we save also the command guaranteeing going to the certified set of cells
                changed = true;
            }

        }
        if(!changed){
            break;
        }
        // here if we loop through the ranks of all the cells and nothing changes
        // we stop because all cells have their correct ranks and dont have to be incremented further
        // -1 for uncertified , 0 for success cells 
        // 1 , 2 , 3... : completion is guaranteed within that many movements 
     }
    return controller;
}

inline int choose_command(const Controller& controller, int cell, Mission mission){
    return controller.command[cell][static_cast<int>(mission)];
}
