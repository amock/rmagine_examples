#include <iostream>
#include <fstream>
#include <imagine/simulation/O1DnSimulatorEmbree.hpp>
#include <imagine/util/StopWatch.hpp>

// Generic Interface
#include <imagine/simulation/SimulationResults.hpp>
#include <imagine/types/Bundle.hpp>

using namespace imagine;

Memory<LiDARModel, RAM> velodyne_model()
{
    Memory<LiDARModel, RAM> model(1);
    model->theta.min = -M_PI;
    model->theta.max = M_PI; 
    model->theta.size = 440;
    model->theta.computeStep();
    
    model->phi.min = -0.261799;
    model->phi.max = 0.261799;
    model->phi.size = 16;
    model->phi.computeStep();
    
    model->range.min = 0.5;
    model->range.max = 130.0;
    return model;
}

O1DnModel<RAM> custom_model()
{
    // represent spherical model as custom model to compare results
    // build model out of two velo models
    auto velo_model = velodyne_model();

    O1DnModel<RAM> model;
        
    size_t W = velo_model->getWidth();
    size_t H = velo_model->getHeight() * 2;

    model.width = W;
    model.height = H;
    model.range = velo_model->range;

    model.orig.x = 0.0;
    model.orig.y = 0.0;
    model.orig.z = 0.0;
    model.rays.resize(W * H);

    for(size_t vid=0; vid<velo_model->getHeight(); vid++)
    {
        for(size_t hid=0; hid<velo_model->getWidth(); hid++)
        {
            const Vector ray = velo_model->getRay(vid, hid);
            unsigned int loc_id_1 = model.getBufferId(vid, hid);
            model.rays[loc_id_1] = ray;

            const Vector ray_flipped = {ray.x, ray.z, ray.y};
            unsigned int loc_id_2 = model.getBufferId(vid + velo_model->getHeight(), hid);
            model.rays[loc_id_2] = ray_flipped;
        }
    }

    return model;
}

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
    
    auto model = custom_model();
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

    std::cout << "Simulate Custom Model" << std::endl;

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