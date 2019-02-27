/*
 * PolyUtils.h
 *
 *  Created on: Feb 6, 2019
 *      Author: pses
 */

#ifndef PSES_CONTROL_MPC_SRC_POLYUTILS_H_
#define PSES_CONTROL_MPC_SRC_POLYUTILS_H_

#include <eigen3/Eigen/src/Core/DenseCoeffsBase.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Core/PlainObjectBase.h>
#include <eigen3/Eigen/src/Core/Solve.h>
#include <eigen3/Eigen/src/QR/HouseholderQR.h>
#include <stddef.h>
#include <cmath>
#include <vector>

/*
 *  Example code for fitting a polynomial to sample data (using Eigen 3)
 *
 *  Copyright (C) 2014  RIEGL Research ForschungsGmbH
 *  Copyright (C) 2014  Clifford Wolf <clifford@clifford.at>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

void polyfit(const std::vector<double> &xv, const std::vector<double> &yv, std::vector<double> &coeff, int order) {
	Eigen::MatrixXd A(xv.size(), order+1);
	Eigen::VectorXd yv_mapped = Eigen::VectorXd::Map(&yv.front(), yv.size());
	Eigen::VectorXd result;

	// create matrix
	for (size_t i = 0; i < xv.size(); i++)
	for (size_t j = 0; j < order+1; j++)
		A(i, j) = pow(xv.at(i), j);

	// solve for linear least squares fit
	result = A.householderQr().solve(yv_mapped);

	coeff.resize(order+1);
	for (size_t i = 0; i < order+1; i++)
		coeff[i] = result[i];
}

// Evaluate a polynomial.
double polyeval(const std::vector<double> coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

#endif /* PSES_CONTROL_MPC_SRC_POLYUTILS_H_ */
