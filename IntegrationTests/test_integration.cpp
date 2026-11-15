#include <iostream>
#include <string>
#include <vector>

#include "run_tests.hpp"
#include "../Simulation/controller_io.hpp"

// Integration entry point: loads the abstraction, synthesizes (or loads) the
// controller ONCE, audits the table, then runs the full simulation tests.
//
// Build and run: IntegrationTests/run_integration.ps1 (or see the g++ commands inside it).

int main() {
    std::cout << "======================================================================\n";
    std::cout << "        UNICYCLE SYMBOLIC CONTROL - INTEGRATION AND TEST RUNNER        \n";
    std::cout << "======================================================================\n";

    // 1. Load the abstraction (symb_abs.bin is written by robotControl/sym_abs.cpp)
    Transitions g;
    std::vector<Vec2> val_u;
    std::string abs_path = "symb_abs.bin";
    if (!sim_io::load_abstraction(abs_path, g, val_u)) {
        abs_path = "../symb_abs.bin";
        if (!sim_io::load_abstraction(abs_path, g, val_u)) {
            std::cerr << "Error: symb_abs.bin not found! Please compile and run robotControl/sym_abs.cpp first.\n";
            return 1;
        }
    }
    std::cout << "Abstraction loaded from " << abs_path << ": " << g.size() << " transitions, "
              << val_u.size() << " commands.\n";

    // 2. Synthesize or load the controller ONCE
    Controller controller = sim_io::get_or_synthesize_controller(g, "controller_cache.bin");

    // 3. Audit the table, then run every simulation test
    bool ok = integ::check_controller_table(g, controller);
    ok = integ::run_tests(controller, val_u) && ok;

    return ok ? 0 : 1;
}
