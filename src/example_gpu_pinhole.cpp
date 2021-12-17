#include <iostream>
#include <fstream>
#include <imagine/simulation/PinholeSimulatorOptix.hpp>
#include <imagine/util/StopWatch.hpp>
#include <imagine/math/math.h>

#include "imagine_examples/models.h"

using namespace imagine;

int main(int argc, char** argv)
{
    std::cout << "Example 1" << std::endl;

    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " [meshfile] " << std::endl;
    }

    // Load Map
    OptixMapPtr map = importOptixMap(argv[1]);
    std::cout << "Loaded file '" << argv[1] << "'" << std::endl; 

    // std::cout << "- Meshes: " << map->meshes.size() << std::endl;

    size_t N = 10;

    // Create Simulator in map
    PinholeSimulatorOptix sim_pinhole(map);

    // Define sensor model
    Memory<PinholeModel, RAM> model_pinhole = example_pinhole_model();
    sim_pinhole.setModel(model_pinhole);

    // Define Sensor to base transform (offset between simulated pose and scanner)
    Memory<Transform, RAM> Tsb(1);
    Tsb->setIdentity();
    sim_pinhole.setTsb(Tsb);

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
    double el;
    

    Memory<float, RAM> ranges;
    Memory<float, VRAM_CUDA> ranges_;
    
    sw();
    ranges_ = sim_pinhole.simulateRanges(Tbm);
    el = sw();

    // Download
    ranges = ranges_;
    std::cout << "Simulated " << N << " pinholes in " << el << "s" << std::endl;

    std::ofstream out_pinhole("points_gpu_pinhole.xyz", std::ios_base::binary);

    if(out_pinhole.good())
    {
        for(unsigned int vid=0; vid<model_pinhole->getHeight(); vid++)
        {
            for(unsigned int hid=0; hid<model_pinhole->getWidth(); hid++)
            {
                const unsigned int loc_id = model_pinhole->getBufferId(vid, hid);
                Vector ray = model_pinhole->getRay(vid, hid);
                float range = ranges[loc_id];
                if(range >= model_pinhole->range.min && range <= model_pinhole->range.max)
                {
                    Point p = ray * range;
                    out_pinhole << p.x << " " << p.y << " " << p.z << "\n";
                }
            }
        }

        out_pinhole.close();
    }
    

    return 0;
}