#include <iostream>
#include <fstream>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <rmagine/util/StopWatch.hpp>

#include <rmagine/simulation/SimulatorEmbree.hpp>

// Generic Interface
#include <rmagine/simulation/SimulationResults.hpp>
#include <rmagine/types/Bundle.hpp>

#include <rmagine/noise/GaussianNoise.hpp>

// models
#include "rmagine_examples/models.h"
#include "rmagine_examples/helper.h"

using namespace rmagine;

int main(int argc, char** argv)
{
    std::cout << "Example CPU: Simulate spherical model." << std::endl;

    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " [meshfile] " << std::endl;
        return 0;
    }

    // Load Map
    EmbreeMapPtr map = import_embree_map(argv[1]);
    std::cout << "Loaded file '" << argv[1] << "'" << std::endl;

    // Create Simulator in map

    Simulator<SphericalModel, Embree> sim_sphere(map);
    // SphereSimulatorEmbree sim_sphere(map);

    // Define sensor model
    Memory<SphericalModel, RAM> model = example_spherical_model();
    sim_sphere.setModel(model);

    // Define Sensor to base transform (offset between simulated pose and scanner)
    Memory<Transform, RAM> Tsb(1);
    Tsb->setIdentity();
    sim_sphere.setTsb(Tsb);

    size_t N = 10;

    // Define poses to simulate from
    Memory<Transform, RAM> Tbm(N);
    for(size_t i=0; i<N; i++)
    {
        // for simplicity take the identity
        Tbm[i] = Tsb[0];
    }

    // define intersection attributes
    using IntAttr = Bundle<
        Ranges<RAM> 
    >;

    // simulate ranges and measure time
    StopWatch sw;
    sw();
    IntAttr res = sim_sphere.simulate<IntAttr>(Tbm);
    double el = sw();

    std::cout << "Simulation Statistics: " << std::endl;
    std::cout << "- Sensors: " << N << std::endl;
    std::cout << "- Rays per sensor: " << model->size() << std::endl;
    std::cout << "- Total rays: " << res.ranges.size() << std::endl;
    std::cout << "- Runtime: " << el << "s" << std::endl;

    // apply noise
    sw();
    GaussianNoise(0.0, 0.01).apply(res.ranges);
    el = sw();
    std::cout << "- Runtime Noise: " << el << "s" << std::endl;

    saveRangesAsXYZ(res.ranges, *model, "points_cpu_sphere");

    return 0;
}