#include <sstream>

#include "solver.h"

using namespace LunarLander::Util;

Solver::Solver(
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
    double vy) {
    this->solver = std::unique_ptr<MPSolver>(MPSolver::CreateSolver("SCIP"));
    this->l.reserve(time);
    this->c.reserve(time);
    this->r.reserve(time);
    for (uint64_t i = 0; i < time; ++i) {
        {
            std::ostringstream oss;
            oss << 'l' << i;
            this->l.push_back(
                this->solver->MakeIntVar(0, precision, oss.str())
            );
        }
        {
            std::ostringstream oss;
            oss << 'c' << i;
            this->c.push_back(
                this->solver->MakeIntVar(0, precision, oss.str())
            );
        }
        {
            std::ostringstream oss;
            oss << 'r' << i;
            this->r.push_back(
                this->solver->MakeIntVar(0, precision, oss.str())
            );
        }
    }
    this->objective = this->solver->MutableObjective();
    for (const auto &i : this->l) {
        objective->SetCoefficient(i, 1);
    }
    for (const auto &i : this->c) {
        objective->SetCoefficient(i, 1);
    }
    for (const auto &i : this->r) {
        objective->SetCoefficient(i, 1);
    }
    objective->SetMinimization();
    const double scale = a / precision;
    const double v_gravity = 1.62 * time;
    MPConstraint* const c_vx =
        this->solver->MakeRowConstraint(-vx - vx0, vx - vx0);
    for (uint64_t i = 0; i < time; ++i) {
        c_vx->SetCoefficient(this->l[i], scale);
        c_vx->SetCoefficient(this->r[i], -scale);
    }
    MPConstraint* const c_vy =
        this->solver->MakeRowConstraint(
            -vy - v_gravity - vy0,
            vy - v_gravity - vy0
        );
    for (auto const &i : this->c) {
        c_vy->SetCoefficient(i, -scale);
    }
    const double s_vx0 = vx0 * time;
    MPConstraint* const c_x =
        this->solver->MakeRowConstraint(
            x - tolerance - s_vx0 - x0,
            x + tolerance - s_vx0 - x0
        );
    for (uint64_t i = 0; i < time; ++i) {
        c_x->SetCoefficient(this->l[i], scale * (time - i - 0.5));
        c_x->SetCoefficient(this->r[i], -scale * (time - i - 0.5));
    }
    const double s_vy0 = vy0 * time;
    const double s_g = 0.5 * 1.62 * time * time;
    MPConstraint* const c_y =
        this->solver->MakeRowConstraint(
            y0 - s_vy0 - s_g,
            y0 - s_vy0 - s_g
        );
    for (uint64_t i = 0; i < time; ++i) {
        c_y->SetCoefficient(this->c[i], -scale * (time - i - 0.5));
    }
}

bool Solver::solve() {
    const MPSolver::ResultStatus result_status = this->solver->Solve();
    return result_status == MPSolver::OPTIMAL;
}

void Solver::log() {
    std::cout << "Solution:\n";
    std::cout << "Objective value = " << this->objective->Value() << '\n';
    uint64_t idx = 0;
    for (auto const &i : this->l) {
        std::cout << "l" << idx << " = " << i->solution_value() << '\n';
        ++idx;
    }
    idx = 0;
    for (auto const &i : this->c) {
        std::cout << "c" << idx << " = " << i->solution_value() << '\n';
        ++idx;
    }
    idx = 0;
    for (auto const &i : this->r) {
        std::cout << "r" << idx << " = " << i->solution_value() << '\n';
        ++idx;
    }
    std::cout << "Problem solved in " << solver->wall_time() <<
        " milliseconds\n";
    std::cout << "Problem solved in " << solver->iterations() <<
        " iterations\n";
    std::cout << "Problem solved in " << solver->nodes() <<
        " branch-and-bound nodes\n";
}
