#include "util/solver.h"

int main() {
    LunarLander::Util::Solver solver(
        0,
        10000,
        0,
        3,
        1000,
        1000,
        5,
        5,
        2000,
        2,
        3,
        3
    );
    bool res = solver.solve();
    if (res) {
        solver.log();
    } else {
        std::cout << "No solution\n";
    }
    return 0;
}
