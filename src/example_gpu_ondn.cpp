#include <iostream>
#include <fstream>
#include <imagine/simulation/OnDnSimulatorOptix.hpp>
#include <imagine/util/StopWatch.hpp>

// Generic Interface
#include <imagine/simulation/SimulationResults.hpp>
#include <imagine/types/Bundle.hpp>

// Predefine models
#include "imagine_examples/models.h"
#include "imagine_examples/helper.h"

using namespace imagine;

int main(int argc, char** argv)
{
    std::cout << "Example GPU: O1Dn Model Simulation" << std::endl;

    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " [meshfile] " << std::endl;
    }

    // Load Map
    OptixMapPtr map = importOptixMap(argv[1]);
    std::cout << "Loaded file '" << argv[1] << "'" << std::endl; 

    // std::cout << "- Meshes: " << map->meshes.size() << std::endl;

    // Create Simulator in map
    OnDnSimulatorOptix sim(map);

    // Define sensor model
    
    OnDnModel<RAM> model = example_ondn_model();
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

    std::cout << "Simulate OnDn Model" << std::endl;

    Memory<float, VRAM_CUDA> ranges_;

    // simulate ranges and measure time
    StopWatch sw;
    sw();
    ranges_ = sim.simulateRanges(Tbm_);
    double el = sw();
    std::cout << "Simulated " << N << " sensors in " << el << "s" << std::endl;

    Memory<float, RAM> ranges;
    ranges = ranges_;

    saveRangesAsXYZ(ranges, model, "points_gpu_ondn");

    return 0;
}