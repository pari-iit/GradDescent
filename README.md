# GradDescent
Gradient Descent Based Optimization with known Derivatives


## Dependencies for Running Locally

* cmake >= 3.7
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* Third party:
  * For testing purposes we use [Google Test](https://github.com/google/googletest)

## Basic Build Instructions

1. Clone this repo using `https://github.com/pari-iit/GradDescent.git`
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./graddestest`.

## Using Your Own Function. 

1. Open `gradientDescent.h`
2. Create class using the following template:
    ```template<typename T>
       class Myfunc: public Function<T>{
            const T _params;
        public:
            Myfunc(T params):_params(params);
            std::vector<double> grad(std::vector<double>& pt){
            // return grad
            }
        };
        
## License

The project is distributed under the BSD-3 clause license.