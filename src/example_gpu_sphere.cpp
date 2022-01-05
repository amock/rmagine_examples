#include <iostream>
#include <fstream>
// #include <imagine/simulation/SphereSimulatorOptix.hpp>
// #include <imagine/util/StopWatch.hpp>
// #include <imagine/math/math.h>

#include <imagine/simulation.h>
#include <imagine/util/StopWatch.hpp>

// predefined models
#include "imagine_examples/models.h"
#include "imagine_examples/helper.h"

using namespace imagine;

int main(int argc, char** argv)
{
    std::cout << "Example GPU: Simulate spherical model." << std::endl;

    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " [meshfile] " << std::endl;
        return 0;
    }

    // Load Map
    OptixMapPtr map = importOptixMap(argv[1]);
    std::cout << "Loaded file '" << argv[1] << "'" << std::endl; 

    // std::cout << "- Meshes: " << map->meshes.size() << std::endl;

    size_t N = 10;

    // Create Simulator in map
    SphereSimulatorOptix sim_sphere(map);

    // Define sensor model
    Memory<LiDARModel, RAM> model = example_spherical_model();
    sim_sphere.setModel(model);

    // Define Sensor to base transform (offset between simulated pose and scanner)
    Memory<Transform, RAM> Tsb(1);
    Tsb->setIdentity();
    sim_sphere.setTsb(Tsb);

    // Define poses to simulate from
    Memory<Transform, RAM> Tbm_(N);
    for(size_t i=0; i<N; i++)
    {
        // for simplicity take the identity
        Tbm_[i] = Tsb[0];
    }

    // upload to gpu
    Memory<Transform, VRAM_CUDA> Tbm;
    Tbm = Tbm_;

    // simulate ranges and measure time
    StopWatch sw;
    sw();
    Memory<float, VRAM_CUDA> ranges_ = sim_sphere.simulateRanges(Tbm);
    double el = sw();
   
    // Download
    Memory<float, RAM> ranges;
    ranges = ranges_;

    
    std::cout << "Simulation Statistics: " << std::endl;
    std::cout << "- Sensors: " << N << std::endl;
    std::cout << "- Rays per sensor: " << model->size() << std::endl;
    std::cout << "- Total rays: " << ranges.size() << std::endl;
    std::cout << "- Runtime: " << el << "s" << std::endl;


    saveRangesAsXYZ(ranges, *model, "points_gpu_sphere");
    
    return 0;
}