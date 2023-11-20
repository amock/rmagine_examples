#include <iostream>
#include <fstream>
#include <rmagine/simulation/SimulatorEmbree.hpp>
#include <rmagine/util/StopWatch.hpp>

// Generic Interface
#include <rmagine/simulation/SimulationResults.hpp>
#include <rmagine/types/Bundle.hpp>

// models
#include "rmagine_examples/models.h"
#include "rmagine_examples/helper.h"

using namespace rmagine;

int main(int argc, char** argv)
{
    std::cout << "Example CPU: Simulate pinhole model." << std::endl;

    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " [meshfile] " << std::endl;
        return 0;
    }

    // Load Map
    EmbreeMapPtr map = import_embree_map(argv[1]);
    std::cout << "Loaded file '" << argv[1] << "'" << std::endl;

    // Create Simulator in map
    Simulator<PinholeModel, Embree> sim_pinhole(map);

    // Define sensor model
    Memory<PinholeModel, RAM> model = example_pinhole_model();
    sim_pinhole.setModel(model);

    // Define Sensor to base transform (offset between simulated pose and scanner)
    Memory<Transform, RAM> Tsb(1);
    Tsb->setIdentity();
    sim_pinhole.setTsb(Tsb);

    size_t N = 10;

    // Define poses to simulate from
    Memory<Transform, RAM> Tbm(N);
    for(size_t i=0; i<N; i++)
    {
        // for simplicity take the identity
        Tbm[i] = Tsb[0];
    }

    StopWatch sw;
    double el;

    // define intersection attributes
    using IntAttr = Bundle<
        Ranges<RAM> 
    >;

    sw();
    IntAttr res = sim_pinhole.simulate<IntAttr>(Tbm);
    el = sw();
    
    std::cout << "Simulation Statistics: " << std::endl;
    std::cout << "- Sensors: " << N << std::endl;
    std::cout << "- Rays per sensor: " << model->size() << std::endl;
    std::cout << "- Total rays: " << res.ranges.size() << std::endl;
    std::cout << "- Runtime: " << el << "s" << std::endl;

    saveRangesAsXYZ(res.ranges, *model, "points_cpu_pinhole");

    return 0;
}