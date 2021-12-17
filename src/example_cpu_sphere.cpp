#include <iostream>
#include <fstream>
#include <imagine/simulation/SphereSimulatorEmbree.hpp>
#include <imagine/util/StopWatch.hpp>

// Generic Interface
#include <imagine/simulation/SimulationResults.hpp>
#include <imagine/types/Bundle.hpp>

// models
#include "imagine_examples/models.h"

using namespace imagine;

int main(int argc, char** argv)
{
    std::cout << "Example CPU" << std::endl;

    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " [meshfile] " << std::endl;
    }

    // Load Map
    EmbreeMapPtr map = importEmbreeMap(argv[1]);
    std::cout << "Loaded file '" << argv[1] << "'" << std::endl; 

    std::cout << "- Meshes: " << map->meshes.size() << std::endl;

    // Create Simulator in map
    SphereSimulatorEmbree sim_sphere(map);

    // Define sensor model
    Memory<LiDARModel, RAM> sphere_model = example_spherical_model();
    sim_sphere.setModel(sphere_model);

    // Define Sensor to base transform (offset between simulated pose and scanner)
    Memory<Transform, RAM> Tsb(1);
    Tsb->R.x = 0.0;
    Tsb->R.y = 0.0;
    Tsb->R.z = 0.0;
    Tsb->R.w = 1.0;
    Tsb->t.x = 0.0;
    Tsb->t.y = 0.0;
    Tsb->t.z = 0.0;
    sim_sphere.setTsb(Tsb);

    size_t N = 10;

    // Define poses to simulate from
    Memory<Transform, RAM> Tbm(N);
    for(size_t i=0; i<N; i++)
    {
        // for simplicity take the identity
        Tbm[i] = Tsb[0];
    }

    std::cout << "Simulate Spherical Model" << std::endl;

    // simulate ranges and measure time
    StopWatch sw;
    sw();
    Memory<float, RAM> ranges = sim_sphere.simulateRanges(Tbm);
    double el = sw();
    std::cout << "Simulated " << N << " sensors in " << el << "s" << std::endl;

    std::ofstream out("points_cpu_sphere.xyz", std::ios_base::binary);

    if(out.good())
    {
        for(unsigned int vid=0; vid<sphere_model->phi.size; vid++)
        {
            for(unsigned int hid=0; hid<sphere_model->theta.size; hid++)
            {
                const unsigned int loc_id = sphere_model->getBufferId(vid, hid);
                Vector ray = sphere_model->getRay(vid, hid);
                
                // std::cout << "Ray: " << ray.x << " " << ray.y << " " << ray.z << std::endl;
                float range = ranges[loc_id];
                if(range >= sphere_model->range.min && range <= sphere_model->range.max)
                {
                    Point p = ray * range;
                    // std::cout << "Intersection: " << p.x << " " << p.y << " " << p.z << std::endl;
                    out << p.x << " " << p.y << " " << p.z << "\n";
                }
            }
        }

        out.close();
    }

    return 0;
}