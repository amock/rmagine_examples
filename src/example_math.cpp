#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>

#include <rmagine/math/linalg.h>


namespace rm = rmagine;

int main(int argc, char** argv)
{
    
    rm::Transform T1;
    T1.t = {1.0, 2.0, 3.0};
    T1.R = rm::EulerAngles{0.0, 0.2, 0.3};

    rm::Vector3 S1 = {1.0, 1.0, 1.0};

    std::cout << "orig:" << std::endl;
    std::cout << T1 << std::endl;
    std::cout << S1 << std::endl;


    rm::Matrix4x4 M = rm::compose(T1, S1);
    
    rm::Transform T2;
    rm::Vector S2;
    rm::decompose(M, T2, S2);

    std::cout << "composed, decomposed:" << std::endl;

    std::cout << T2 << std::endl;
    std::cout << S2 << std::endl;

    return 0;
}