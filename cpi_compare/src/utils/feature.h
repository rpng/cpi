/**
 * MIT License
 * Copyright (c) 2018 Kevin Eckenhoff
 * Copyright (c) 2018 Patrick Geneva
 * Copyright (c) 2018 Guoquan Huang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#ifndef FEATURE_H
#define FEATURE_H

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


class feature {

public:

    /// Default constructor
    feature(){}

    /// Optimized position in the global
    Eigen::Vector3d pos_FinG;

    /// Optimized position in the anchor frame (inverse depth representation, left camera)
    Eigen::Vector3d pos_FinA_inv;

    /// Triangulated position in the global
    Eigen::Vector3d pos_FinA_trianglated;

    /// Optimized position in the global
    Eigen::Vector3d pos_FinA_optimized;

    /// State IDs that each UV coordinate should be linked too
    std::vector<size_t> leftstateids;
    std::vector<size_t> rightstateids;

    /// UV coordinates of the features
    std::vector<Eigen::Vector2d> leftuv;
    std::vector<Eigen::Vector2d> rightuv;

    /// Simulation IDs used for matching of features (theses should all be the same)
    std::vector<uint> leftids;
    std::vector<uint> rightids;


};



#endif // FEATURE_H