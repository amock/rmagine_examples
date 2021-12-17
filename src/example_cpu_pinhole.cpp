#include <iostream>
#include <fstream>
#include <imagine/simulation/PinholeSimulatorEmbree.hpp>
#include <imagine/util/StopWatch.hpp>

// Generic Interface
#include <imagine/simulation/SimulationResults.hpp>
#include <imagine/types/Bundle.hpp>

// models
#include "imagine_examples/models.h"

using namespace imagine;

int main(int argc, char** argv)
{
    std::cout << "Example CPU: Simulate pinhole model." << std::endl;

    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " [meshfile] " << std::endl;
    }

    // Load Map
    EmbreeMapPtr map = importEmbreeMap(argv[1]);
    std::cout << "Loaded file '" << argv[1] << "'" << std::endl; 

    std::cout << "- Meshes: " << map->meshes.size() << std::endl;

    // Create Simulator in map
    PinholeSimulatorEmbree sim_pinhole(map);

    // Define sensor model
    Memory<PinholeModel, RAM> pinhole_model = example_pinhole_model();
    sim_pinhole.setModel(pinhole_model);

    // Define Sensor to base transform (offset between simulated pose and scanner)
    Memory<Transform, RAM> Tsb(1);
    Tsb->R.x = 0.0;
    Tsb->R.y = 0.0;
    Tsb->R.z = 0.0;
    Tsb->R.w = 1.0;
    Tsb->t.x = 0.0;
    Tsb->t.y = 0.0;
    Tsb->t.z = 0.0;
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

    std::cout << "Simulate Pinhole Model" << std::endl;

    sw();
    Memory<float, RAM> pinhole_ranges = sim_pinhole.simulateRanges(Tbm);
    el = sw();
    std::cout << "Simulated " << N << " sensors in " << el << "s" << std::endl;

    std::ofstream out_pinhole("points_cpu_pinhole.xyz", std::ios_base::binary);

    if(out_pinhole.good())
    {
        for(unsigned int vid=0; vid<pinhole_model->height; vid++)
        {
            for(unsigned int hid=0; hid<pinhole_model->width; hid++)
            {
                const unsigned int loc_id = pinhole_model->getBufferId(vid, hid);
                Vector ray = pinhole_model->getRay(vid, hid);


                // std::cout << "Ray: " << ray.x << " " << ray.y << " " << ray.z << std::endl;
                float range = pinhole_ranges[loc_id];
                if(range >= pinhole_model->range.min && range <= pinhole_model->range.max)
                {
                    Point p = ray * range;
                    // std::cout << "Intersection: " << p.x << " " << p.y << " " << p.z << std::endl;
                    out_pinhole << p.x << " " << p.y << " " << p.z << "\n";
                }
            }
        }

        out_pinhole.close();
    }

    return 0;
}