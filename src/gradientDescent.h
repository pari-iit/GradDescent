/*
 This implements gradient descent method.
*/
//Copyright  2020 (C) Parikshit Dutta



#pragma once

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <cmath>
#include <type_traits>
#include <cassert>
#include <algorithm>
#include <limits>
#include <numeric>

//Find out if something is a vector
template<typename T> struct is_vector : public std::false_type {};

template<typename T, typename A>
struct is_vector<std::vector<T, A>> : public std::true_type {};

template< class T >
inline constexpr bool is_vector_v = is_vector<T>::value;
//end here

template<typename T>
class Function{
public:
    virtual std::vector<double> grad(const std::vector<double>& pt) = 0;
    virtual ~Function(){};
    //Rule of five to follow
};

template<typename T>
//F = a^2x^2 + b^2y^2
class SumOfSquares: public Function<T>{
    const T _mult;
public:
    SumOfSquares(const T& mult):_mult(mult){};
    std::vector<double> grad(const std::vector<double>& pt){
        if constexpr (std::is_floating_point_v<T>){
            return {2*_mult*_mult*pt[0], 2*_mult*_mult*pt[1]};
        }
        else if constexpr (is_vector_v<T>){
            return {2*_mult[0]*_mult[0]*pt[0], 2*_mult[1]*_mult[1]*pt[1]};
        }        
        throw("Wrong templated parameter\n");
        return {0.0,0.0};
    }
};


template<typename T>
//F =  (1-x)^2 + mult*(y-x^2)^2
class RosenBrock: public Function<T>{
    const T _mult;
public:
    RosenBrock(const T& mult):_mult(mult){};
    std::vector<double> grad(const std::vector<double>& pt){
        if constexpr (std::is_floating_point_v<T>){
            return { 2*( 2*_mult*pow(pt[0],3) - 2*_mult*pt[0]*pt[1] + pt[0] -1 ),
                    2*_mult*(pt[1]-pt[0]*pt[0]) };
        }     
        throw("Wrong templated parameter\n");
        return {0.0,0.0};
    }
};

/*
Add your function
template<typename T>
class Myfunc: public Function<T>{
    const T _params;
public:
    Myfunc(T params):_params(params);
    std::vector<double> grad(std::vector<double>& pt){
        // return grad
    }
};
*/

template<typename T>
class GradientDescent{
    const double _step_sz;
    std::unique_ptr<Function<T> > _fnptr;
    const double _ep;

    double norm(const std::vector<double>& v1, const std::vector<double>& v2){
        assert(v1.size() == v2.size());
        std::vector<double> diff;
        std::transform(v1.begin(),v1.end(),v2.begin(), std::back_inserter(diff),[](const double& a, const double& b){
            return a-b;
        } );
        double nr = std::accumulate(diff.begin(),diff.end(),0.0,[](double s,const double& d){
            return std::move(s) + d*d;
        });
        return sqrt(nr);
    }

public:
    GradientDescent(const double& step_sz, const double& ep, std::unique_ptr<Function<T> > fnptr):_step_sz(step_sz),_ep(ep), _fnptr(std::move(fnptr)){}

    std::vector<double> runGD(const std::vector<double>& ip ){
        double vnorm = DBL_MAX;
        std::vector<double> pt = ip;
        while(vnorm > _ep){
            std::vector<double> gradpt = _fnptr->grad(pt);
            std::vector<double> newpt;
            std::transform(pt.begin(), pt.end(),gradpt.begin(),std::back_inserter(newpt),[this](const auto& v1, const auto& v2){
                return v1-_step_sz*v2;
            } );            
            vnorm = norm(newpt,pt);
            pt = std::move(newpt);
        }
    return pt;
    }

};