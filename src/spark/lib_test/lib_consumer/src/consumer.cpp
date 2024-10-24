#include <iostream>
#include "the_library/lib_header.hpp"

int main() {

    TestClass TestObj;
    int res = TestObj.add_two_numbers(3,5);
    std::cout <<std::endl << res << std::endl;

    return 0;
}