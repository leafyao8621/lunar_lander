#ifndef UTIL_SOLVER_H_
#define UTIL_SOLVER_H_

#include <memory>
#include <vector>
#include <tuple>
#include <cstdint>
#include <ortools/linear_solver/linear_solver.h>

using namespace operations_research;

namespace LunarLander {
    namespace Util {
        class Solver {
            std::unique_ptr<MPSolver> solver;
            std::vector<MPVariable*> l, c, r;
            MPObjective *objective;
        public:
            Solver(
                double x0,
                double y0,
                double vx0,
                double vy0,
                uint64_t time,
                uint64_t fuel,
                double a,
                uint64_t precision,
                double x,
                double tolerance,
                double vx,
                double vy);
            bool solve();
            std::tuple<
                std::vector<uint64_t>,
                std::vector<uint64_t>,
                std::vector<uint64_t> > get_solution();
            void log();
        };
    }
}

#endif
