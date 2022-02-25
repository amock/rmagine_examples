#include <iostream>

#include <rmagine/util/synthetic.h>

#include <rmagine/simulation/SphereSimulatorOptix.hpp>

#include "rmagine_examples/models.h"
#include "rmagine_examples/helper.h"

using namespace rmagine;

int main(int argc, char** argv)
{
    std::cout << "Rmagine Examples: CPU change map" << std::endl;

    Transform Tsb;
    Tsb.setIdentity();

    

    SphereSimulatorOptix sim2;


    auto sphere = genSphere(50, 50);
    OptixMapPtr map1(new OptixMap(&sphere) );


    SphereSimulatorOptix sim;

    

    sim.setTsb(Tsb);

    sim.setMap(map1);

    auto model = example_spherical_model();
    sim.setModel(model);

    // TODO: cube
    // auto cube = genCube();
    // OptixMapPtr map2(new OptixMap(&cube));

    Memory<Transform, RAM> Tbm(1);
    Tbm[0].setIdentity();
    Memory<Transform, VRAM_CUDA> Tbm_;
    Tbm_ = Tbm;

    // sim.setMap(map1);

    Memory<float, RAM> ranges_sphere;
    ranges_sphere = sim.simulateRanges(Tbm_);
    
    saveRangesAsXYZ(ranges_sphere, *model, "points_gpu_cm_sphere");

    // sim.setMap(map2);

    // Memory<float, RAM> ranges_cube;
    // ranges_cube = sim.simulateRanges(Tbm_);

    // saveRangesAsXYZ(ranges_cube, *model, "points_gpu_cm_cube");
}