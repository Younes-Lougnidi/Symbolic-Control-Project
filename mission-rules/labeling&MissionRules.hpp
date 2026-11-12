#include <vector>
#include "../robotControl/dynamics.hpp"
#include "../robotControl/state2coord.hpp"
#include "../robotControl/coord2state.hpp"
#include <algorithm>

using namespace dyn;

// the enum class containing all the region in the specifications:
enum class Region{NONE,R1,R2,R3,R4};

// The labeling method

// what does it do?
/*
it takes a discrete cell ID and returns the Region
it belongs to according to the specifications.
*/

// what are the specifications?

/*
R_1: [4,5] × [8.5,9.5] × [– π, π]
R_2: [8.5,9.5] × [2,3] × [– π, π]
R_3: [2,3] × [0.5,1.5] × [– π, π]
R_4: [3,7] × [3,7] × [– π, π]
*/

// First logical step is to concretisize each region:
/*
we need to find the discrete representation of each region
R_{i}, build Dis(R_{i}) being the descretization of R_{i}. 
*/

// turns a real life box into cells:
std::vector<int> boxToCells(double lb1,double ub1,double lb2,double ub2){
    std::vector<int> result;

    // we need to find the bottom left in the first slice:
    int cellLowX1 = floor(lb1/0.1)+1;
    int cellLowX2 = floor(lb2/0.1)+1;
    int cellBottomLeft = 1+cellLowX1-1 + (cellLowX2-1)*100 + 10000*0; // 10000*0 just to illustrate that we are in the first layer and we don't need to multiply.

    // we need to find the top right in the 30th slice:
    int cellUpperX1 = ceil(ub1/0.1);
    int cellUpperX2 = ceil(ub2/0.1);
    int cellTopRight = 1+cellUpperX1-1 + (cellUpperX2-1)*100 + 10000*29;

    // we need to recover all the cells in that interval:
    // let's get the bounds in coords of cells
    const std::array<int, 4> p_x = {1, n_x[0], n_x[0] * n_x[1], n_x[0] * n_x[1] * n_x[2]};
    Idx3 lo = state2coord<3>(cellBottomLeft,p_x); // this should return (41,86,1)
    Idx3 hi = state2coord<3>(cellTopRight,p_x); // this should return (50,95,30)

    for (int i = lo[2]; i<=hi[2]; i++){
        for (int j = lo[1]; j<= hi[1]; j++){
            for (int k = lo[0]; k<=hi[0]; k++){
                result.push_back(coord2state<3>({k,j,i},p_x));
            }
        }
    }
    return result;
}


// label method checks all regions
Region label(int cell){

    std::vector<int> R1 = boxToCells(4,5,8.5,9.5);
    std::vector<int> R2 = boxToCells(8.5,9.5,2,3);
    std::vector<int> R3 = boxToCells(2,3,0.5,1.5);
    std::vector<int> R4 = boxToCells(3,7,3,7);

    if (std::binary_search(R4.begin(), R4.end(), cell)) return Region::R4;
    if (std::binary_search(R1.begin(), R1.end(), cell)) return Region::R1;
    if (std::binary_search(R2.begin(), R2.end(), cell)) return Region::R2;
    if (std::binary_search(R3.begin(), R3.end(), cell)) return Region::R3;

    return Region::NONE;

}

// the mission table
enum class Mission { START, VISITED_R1, VISITED_R2, SUCCESS, FAILURE };

Mission update_mission(Mission previous, Region current){

    if (previous == Mission::SUCCESS || previous == Mission::FAILURE) return previous;

    if (current == Region::R4) return Mission::FAILURE;

    switch (previous) {
        case Mission::START:
            if (current == Region::R1) return Mission::VISITED_R1;
            if (current == Region::R2) return Mission::VISITED_R2;
            return Mission::START;              

        case Mission::VISITED_R1:
            if (current == Region::R2) return Mission::FAILURE;   
            if (current == Region::R3) return Mission::SUCCESS;
            return Mission::VISITED_R1;          

        case Mission::VISITED_R2:
            if (current == Region::R1) return Mission::FAILURE;   
            if (current == Region::R3) return Mission::SUCCESS;
            return Mission::VISITED_R2;         

        default:
            return previous;
    }
} 