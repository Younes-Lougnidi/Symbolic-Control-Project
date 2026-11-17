#pragma once

#include <vector>

inline std::vector<int> state2coord(long long i , const std::vector<long long>& p_T){
    std::vector<int> I(p_T.size()-1);
    for (size_t k = 0; k < p_T.size()-1; ++k) I[k] = ((i - 1) % p_T[k + 1]) / p_T[k] + 1;
    return I;
}


// same as the RObot this goes from the cell ID , and gives us 
// the combination that generated it