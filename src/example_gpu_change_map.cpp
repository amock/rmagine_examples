#include <iostream>

#include <rmagine/util/synthetic.h>

#include <rmagine/simulation/SimulatorOptix.hpp>

#include "rmagine_examples/models.h"
#include "rmagine_examples/helper.h"

using namespace rmagine;

int main(int argc, char** argv)
{
    std::cout << "Rmagine Examples: GPU change map" << std::endl;


    Simulator<SphericalModel, Optix> sim;


    OptixContextPtr optix_ctx = OptixContext::create(0);


    auto model = example_spherical_model();
    sim.setModel(model);

    Transform Tsb;
    Tsb.setIdentity();
    sim.setTsb(Tsb);

    Memory<Transform, RAM> Tbm(1000);
    for(int i=0; i<1000; i++)
    {
        Tbm[i].setIdentity();
    }
    

    Memory<Transform, VRAM_CUDA> Tbm_;
    Tbm_ = Tbm;
    
    // sphere

    size_t Niter = 100;

    Memory<float, RAM> ranges_sphere;
    for(size_t i=0; i<Niter; i++)
    {
        std::cout << i << "/" << Niter << std::endl;
        auto sphere = genSphere(200, 200);
        OptixMapPtr map1(new OptixMap(&sphere, optix_ctx) );

        sim.setMap(map1);
        
        ranges_sphere = sim.simulateRanges(Tbm_);
    }

    saveRangesAsXYZ(ranges_sphere, *model, "points_cm_sphere");

    // cube
    // auto cube = genCube();
    // OptixMapPtr map2(new OptixMap(&cube));
    // sim->setMap(map2);
    // Memory<float, RAM> ranges_cube;
    // ranges_cube = sim->simulateRanges(Tbm_);
    // saveRangesAsXYZ(ranges_cube, *model, "points_cm_cube");

    // Tbm_.free();



    return 0;
}