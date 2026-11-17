#pragma once 

#include <vector>

inline long long coord2state(const std::vector<int>& I,const std::vector<long long>& p_T){
    long long i = 1;
    size_t N = I.size();
    for (size_t k = 0; k < N; ++k) i += (I[k] - 1) * p_T[k];
    return i;
}

// same thing here as the robot, we will get (i1,i2,i3) cell 
// each stemming from (T1,T2,T3) respectivelly (in the case of 3).
