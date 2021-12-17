#include <iostream>
#include <fstream>
#include <imagine/simulation/O1DnSimulatorEmbree.hpp>
#include <imagine/util/StopWatch.hpp>

// Generic Interface
#include <imagine/simulation/SimulationResults.hpp>
#include <imagine/types/Bundle.hpp>

// Predefine models
#include "imagine_examples/models.h"

using namespace imagine;

int main(int argc, char** argv)
{
    std::cout << "Example CPU: O1Dn Model Simulation" << std::endl;

    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " [meshfile] " << std::endl;
    }

    // Load Map
    EmbreeMapPtr map = importEmbreeMap(argv[1]);
    std::cout << "Loaded file '" << argv[1] << "'" << std::endl; 

    std::cout << "- Meshes: " << map->meshes.size() << std::endl;

    // Create Simulator in map
    O1DnSimulatorEmbree sim(map);

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

    std::cout << "Simulate O1Dn Model" << std::endl;

    // simulate ranges and measure time
    StopWatch sw;
    sw();
    Memory<float, RAM> ranges = sim.simulateRanges(Tbm);
    double el = sw();
    std::cout << "Simulated " << N << " sensors in " << el << "s" << std::endl;

    std::ofstream out("points_cpu_o1dn.xyz", std::ios_base::binary);

    if(out.good())
    {
        for(unsigned int vid=0; vid<model.getHeight(); vid++)
        {
            for(unsigned int hid=0; hid<model.getWidth(); hid++)
            {
                const unsigned int loc_id = model.getBufferId(vid, hid);
                Vector orig = model.getOrigin(vid, hid);
                Vector ray = model.getRay(vid, hid);
                
                // std::cout << "Ray: " << ray.x << " " << ray.y << " " << ray.z << std::endl;
                float range = ranges[loc_id];
                if(range >= model.range.min && range <= model.range.max)
                {
                    Point p = orig + ray * range;
                    // std::cout << "Intersection: " << p.x << " " << p.y << " " << p.z << std::endl;
                    out << p.x << " " << p.y << " " << p.z << "\n";
                }
            }
        }

        out.close();
    }

    return 0;
}