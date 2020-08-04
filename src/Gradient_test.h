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