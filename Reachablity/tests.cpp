#include <cassert>
#include <iostream>
#include "reachability.hpp"
int main() {
    using namespace reach;
    Transitions g(static_cast<std::size_t>(n_states)*n_commands, {0,0});
    auto set = [&](int s, int u, int lo, int hi) {
        g[(static_cast<std::size_t>(s)-1)*n_commands+u-1] = {lo,hi};
    };
    set(2,2,1,1); // Must skip invalid command 1.
    set(3,1,1,2); // Both successors must win.
    set(4,1,1,5); // Has a path to target, but cannot guarantee arrival.
    set(5,1,5,5); // Self-loop cannot establish progress.
    auto r = Max_reach(g, std::array<int,2>{1,1});
    assert(r.winning.size() == 3);
    assert(r.rank[1] == 0 && r.rank[2] == 1 && r.rank[3] == 2);
    assert(r.rank[4] == -1 && r.rank[5] == -1);
    assert(r.h[1] == 0 && r.h[2] == 2 && r.h[3] == 1);
    assert(pred(g, std::array<int,2>{1,2}).size() == 2);
    for (int s : r.winning) if (r.rank[s] > 0) {
        Membership earlier(n_states+1, false);
        for (int t : r.winning) earlier[t] = r.rank[t] < r.rank[s];
        assert(successors_in(g[(s-1)*n_commands+r.h[s]-1], earlier));
    }
    assert(Max_reach(g, std::vector<int>{}).winning.empty());
    auto id = [](int x, int y, int a) { return coord2state<3>({x,y,a}, p_x); };
    Membership A(n_states+1, false);
    for (int a : {30,1}) for (int y : {2,3}) for (int x : {2,3}) A[id(x,y,a)] = true;
    std::array<int32_t,2> box{id(2,2,30), id(3,3,1)};
    assert(successors_in(box,A)); // Wrapped Cartesian box, eight successors.
    A[id(2,3,1)] = false;
    assert(!successors_in(box,A)); // Reject missing non-corner successor.
    assert(!successors_in({0,0},A));
    bool rejected = false;
    try { Max_reach(g, std::array<int,1>{0}); } catch (const std::out_of_range&) { rejected = true; }
    assert(rejected);
    std::cout << "All reachability tests passed\n";
}
