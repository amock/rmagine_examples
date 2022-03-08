#include <iostream>
#include <fstream>
#include <rmagine/simulation/SimulatorOptix.hpp>
#include <rmagine/util/StopWatch.hpp>

// Generic Interface
#include <rmagine/simulation/SimulationResults.hpp>
#include <rmagine/types/Bundle.hpp>
#include <rmagine/util/prints.h>

// Predefined models
#include "rmagine_examples/models.h"
#include "rmagine_examples/helper.h"

using namespace rmagine;

int main(int argc, char** argv)
{
    std::cout << "Example GPU: O1Dn Model Simulation" << std::endl;

    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " [meshfile] " << std::endl;
        return 0;
    }

    // Load Map
    OptixMapPtr map = importOptixMap(argv[1]);
    std::cout << "Loaded file '" << argv[1] << "'" << std::endl; 

    std::cout << "- Meshes: " << map->meshes.size() << std::endl;

    // Create Simulator in map
    Simulator<O1DnModel, Optix> sim(map);

    // Define sensor model
    
    auto model = example_o1dn_model();
    sim.setModel(model);

    // Define Sensor to base transform (offset between simulated pose and scanner)
    Memory<Transform, RAM> Tsb(1);
    Tsb->setIdentity();
    sim.setTsb(Tsb);

    size_t N = 10;

    // Define poses to simulate from
    Memory<Transform, RAM> Tbm(N);
    for(size_t i=0; i<N; i++)
    {
        // for simplicity take the identity
        Tbm[i] = Tsb[0];
    }

    Memory<Transform, VRAM_CUDA> Tbm_;
    Tbm_ = Tbm;

    Memory<float, VRAM_CUDA> ranges_;
    Memory<float, RAM> ranges;

    // simulate ranges and measure time
    StopWatch sw;
    sw();
    ranges_ = sim.simulateRanges(Tbm_);
    double el = sw();
    ranges = ranges_;


    std::cout << "Simulation Statistics: " << std::endl;
    std::cout << "- Sensors: " << N << std::endl;
    std::cout << "- Rays per sensor: " << model.size() << std::endl;
    std::cout << "- Total rays: " << ranges.size() << std::endl;
    std::cout << "- Runtime: " << el << "s" << std::endl;

    saveRangesAsXYZ(ranges, model, "points_gpu_o1dn");

    return 0;
}