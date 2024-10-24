#include "lib_header.hpp"


int main()
{
    TestClass TestObj;
    int res = TestObj.add_two_numbers(1,2);
    std::cout <<std::endl << res << std::endl;

    return 0;

}