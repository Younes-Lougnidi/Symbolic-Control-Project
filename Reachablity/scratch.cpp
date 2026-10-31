// STEP 1 : given a position x and a arrival position f in the continuous settings we compute the correspondant sets cells
// STEP 2 : we loop through every possible command (15 commands) and find the set of evalutions of g using the initial cell index .
// STEP 3 : Make a function that calculate the distance between two cells 
// STEP 4: and we choose the command to minimize the distance at each iteration by choosing the closest cell to the arrival station
// and we stop until we reach 0 

// Implementation:
// first we implement a function that given a state it will return a set of couples for each one the first coordinate corresponds
// to a set of couples (pred,control) and the second one is the states of A : pred(std::array<int,N> A)  
// second we implement an iterative algo to find the maximal set of states from which we can reach the target using finite (maximal) number steps (number of inputs)
// Max_reach(std::array<int,N> Qs) (use template)


// pseudo-code for : pred
// we will brute force through the cells while applying for each one all the controls, we append when one reaches the target with a certain input. 
// to do that we will use the sym abstraction we already implemented so that we know the dynamics of our mechanism
// whenever, we find one, we construct the couples we talked about. and we do this for each of the states in the input A.

/*
Entrées : Q_a ⊆ Ξ
R_0 = Q_a
répéter
    R_{k+1} = Q_a ∪ Pre(R_k)
jusqu'à R_{k+1} = R_k ;
Sorties : R* = R_k
*/