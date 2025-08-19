#pragma once
#include <osqp/osqp.h>
#include "Y2Matrix/YMatrix.hpp"
#include <vector>
#include <iostream>
#include <memory>
#include <cstring>

class QPSolver {
private:
    // Convert YMatrix to CSC (Compressed Sparse Column) format for OSQP
    struct CSCMatrix {
        std::vector<OSQPFloat> data;
        std::vector<OSQPInt> indices;
        std::vector<OSQPInt> indptr;
        OSQPInt nrows, ncols, nnz;
    };
    
    CSCMatrix convertToCSC(const YMatrix& matrix, double tolerance = 1e-12);
    CSCMatrix convertToCSC_full(const YMatrix& matrix, double tolerance = 1e-12);  // For constraint matrices

public:
    struct QPResult {
        std::vector<double> solution;
        bool success;
        OSQPInt status;
        OSQPFloat objective_value;
    };
    
    QPResult solve(const YMatrix& H, const std::vector<double>& f,
                   const YMatrix& A_eq, const std::vector<double>& b_eq,
                   const std::vector<double>& lb, const std::vector<double>& ub,
                   double tolerance = 1e-4);
};