#pragma once 
#include "../dynamics/dynamics.hpp"
#include "../abstraction/abstraction.hpp"

/*
Assume every room starts between 17°C and 22°C.
this function tests whether all room's temperature will stay
between 17°C and 22°C after applying the same command (heater)
to all the heaters in each room 

This function answer this question : Is there a heater 
setting that keeps the entire building safe
and we choose abstractly 3 rooms with range 17 and 22 and we test if applying
a command will keep the system safe 
*/

inline bool keeps_whole_building_safe(double heater){
    // let lower and upper the coldest and highest temperatures
    // for the current 3 rooms we're working on 
    const VEC_N lower(3,dyn::T_lo);
    const VEC_N upper(3,dyn::T_hi);

    // let heaters the vector filled by our command (heater)
    const VEC_N heaters(3, heater);
    // we calculate the next tempurature bounds by applying the commands (heaters)
    const Bounds next = dyn::reachable_bounds(lower,upper,heaters);
    
    return next.first[1]  >= dyn::T_lo &&
       next.second[1] <= dyn::T_hi;
    // test if the bounds are within the range 17°C and 22°C
}

/*
the next function find a command id that keeps the whole system safe
*/
inline int find_safe_uniform_level(){
    for(int level = 0 ; level < dyn :: nu ; level ++){
        const double heater = abstraction::decode_command(level,1)[0];
        if(keeps_whole_building_safe(heater)){
            return level;
        }
    }
    return -1;
}

/*
this function answer this question 
If I apply this command, is every cell I could land in still allowed? 
*/

inline bool all_successors_in_candidate(const abstraction::Successors& box,const std::vector<bool>& candidate){
    if(box.leaves_safe_set) return false;
    // loops through every possible combination and check if we our resulting cell still safe 
    auto bins = box.lower;
    while(true){
        const auto cell = abstraction::encode_cell(bins);
        if(!candidate[cell]) return false;

        std::size_t room = 0;
        while(room < bins.size() && bins[room] == box.upper[room]){
            bins[room] = box.lower[room];
            ++room;
        }
        if(room == bins.size()) return true;
        ++bins[room];
    }
}

/*
this function for each cell it return a command that guarentees that all successors
are in the safe set
*/

inline abstraction::Id find_command_in_candidate(const abstraction::Abstraction& A, abstraction::Id cell, const std::vector<bool>& candidate){
    for(abstraction::Id u = 0 ; u < A.num_commands; u++){
        const auto& next = A.transitions[cell*A.num_commands + u];
        if(all_successors_in_candidate(next,candidate)){
            return u;
        }
    }
    return -1 ;
}

/*
this function simply returns a list such as for each cell has a boolean value
that tells us wether there is at least one command that do no take outside 
the safety set (current for now ) 
*/

inline std::vector<bool> next_safety_pass(const abstraction::Abstraction& A,const std::vector<bool>& current){
    std::vector<bool> next(A.num_states,false);
    for(abstraction::Id cell = 0 ; cell < A.num_states; cell ++){
        if(!current[cell]) continue;
        const auto command = find_command_in_candidate(A,cell,current);
        next[cell] = (command !=-1);
    }
    return next;
}

/*
this function is the application of the safety algorithm so it returns for
each cell id whether its a winning cell or not (winning means that there are at least one command that keeps us in the safe set)
*/

inline std::vector<bool> compute_winning_set(const abstraction::Abstraction& A){
    std::vector<bool> current(A.num_states,true);
    while(true){
        auto next = next_safety_pass(A,current);
        if(next == current) return current;
        current.swap(next);
 
    }
}

/*
Given a cell and the winning set already computed (by the algorithm above)
we return a list of safe command we can apply without leaving the safe set
*/

inline std::vector<abstraction::Id> safe_commands_for_cell(const abstraction::Abstraction& A, abstraction::Id cell, const std::vector<bool>& winning){
    std::vector<abstraction::Id> commands;
    if(!winning[cell]) return commands;
    for(abstraction::Id u = 0 ; u < A.num_commands ; u++){
        const auto& box = A.transitions[cell*A.num_commands +u];
        if(all_successors_in_candidate(box,winning)){
            commands.push_back(u);
        }
    }
    return commands;
}

/*
FINALLY, we will synthetise our controller so for every cell id it gives us
a list of safe commands (and an empty list for the none winning cells )
*/

inline std::vector<std::vector<abstraction::Id>> synthesize_safety(const abstraction::Abstraction& A){
    const auto winning = compute_winning_set(A);
    std::vector<std::vector<abstraction::Id>> controller(A.num_states);
    for(abstraction::Id cell = 0 ; cell < A.num_states; cell++){
        controller[cell] = safe_commands_for_cell(A,cell,winning);
    }
    return controller;
}