#pragma once

#include <vector>
#include <cmath>

// we will set some tools we will need for simplicity and readability
using VEC_N = std::vector<double>;
using Bounds = std::pair<VEC_N,VEC_N>;


namespace dyn {
// let's define the system's constants
const int Te = -1;
const int Th = 50;
const double alpha = 0.45;
const double beta = 0.045;
const double gamma = 0.09;

// let's set the system dynamics:
inline VEC_N f(const VEC_N& T, const VEC_N& u){
    // the returned result instantiation
    int N = T.size();
    VEC_N result(N);

    for (int i = 0 ; i<N; i++){
        double newT = T[i] + alpha*(T[(i+1)%N]+T[(i-1+N)%N]-2*T[i]) + beta*(Te-T[i]) + gamma*(Th-T[i])*u[i];
        result[i] = newT;
    }

    return result;

}

// The reachable_bounds, we will compute it from the corners (Check report for proof):
inline Bounds reachable_bounds(const VEC_N& T_low, const VEC_N& T_high, const VEC_N& u){
    return {f(T_low,u),f(T_high,u)};
}

// some helpful constants (similar to Robot) useful for discretization:
const double T_lo = 17.0;
const double T_hi = 22.0;
const double u_lo = 0;
const double u_hi = 0.6;
const int cpr = 10; // cells per room (coming from the requirements)
const int nu = 3; // input value per room (also // // //)

// bin's width aka we work with intervals:
const double d_T = (T_hi-T_lo)/cpr ;

// we work with point and so as we saw previously we divide by nu-1
const double d_u = (u_hi-u_lo)/(nu-1);

// now we are going to add in the state multipliers


inline std::vector<long long> multiplier(int N, int base){
    std::vector<long long> result(N+1);
    for (int i = 0 ; i<N+1 ; i++){
        result[i] = std::llround(std::pow(base, i));
    }
    return result;
}
/*
These two are indespensable , but unfortunately since N is a local variable in main
 they can't be initilized here and so this is a copy paste code
to use in a main.cpp file upon inputing the N we want to use in the experiment


// p_T:
std::vector<long long> p_T = multiplier(N,cpr); // N has to be passed as a global param

// p_u:
std::vector<long long> p_u = multiplier(N,nu);
*/

}