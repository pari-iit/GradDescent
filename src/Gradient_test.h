/*
BSD 3-Clause License

Copyright (c) 2020, Parikshit Dutta
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once

#include "gtest/gtest.h"
#include "gradientDescent.h"
#include <memory>



class GradientTest:public ::testing::Test{
    std::unique_ptr<GradientDescent<double> > _gd;
public:
    std::vector<double> evaluateSOSGradient(const std::vector<double>& ip,const double& mult,const double & ssize, const double& ep){
        std::unique_ptr<Function<double> > fptr= std::make_unique<SumOfSquares<double> >(mult);
        _gd =  std::make_unique<GradientDescent<double> >(ssize,ep,std::move(fptr));
        return _gd->runGD(ip);
    }

    std::vector<double> evaluateRosenBrockGradient(const std::vector<double>& ip,const double& mult,const double & ssize, const double& ep){
        std::unique_ptr<Function<double> > fptr= std::make_unique<RosenBrock<double> >(mult);
        _gd =  std::make_unique<GradientDescent<double> >(ssize,ep,std::move(fptr));
        return _gd->runGD(ip);
    }
};


TEST_F(GradientTest,evaluateSOSGradienttest){
    double tol = 1e-5;
    auto val = evaluateSOSGradient( {1.0,2.0},10,0.001,tol);
    EXPECT_NEAR(0.0,val[0],tol*10);
    EXPECT_NEAR(0.0,val[1],tol*10);
}

TEST_F(GradientTest,evaluateRosenBrockGradienttest){
    double tol = 1e-5;
    auto val = evaluateRosenBrockGradient( {5.0,2.0},100,0.0001,1e-10);
    EXPECT_NEAR(1.0,val[0],tol*10);
    EXPECT_NEAR(1.0,val[1],tol*10);
}