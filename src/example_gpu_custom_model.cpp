#include <iostream>
#include <fstream>
#include <imagine/simulation/O1DnSimulatorOptix.hpp>
#include <imagine/util/StopWatch.hpp>

// Generic Interface
#include <imagine/simulation/SimulationResults.hpp>
#include <imagine/types/Bundle.hpp>
#include <imagine/util/prints.h>

using namespace imagine;


Memory<LiDARModel, RAM> velodyne_model()
{
    Memory<LiDARModel, RAM> model(1);
    model->theta.min = -M_PI;
    model->theta.max = M_PI; 
    model->theta.size = 440;
    model->theta.computeStep();
    // model->theta.step = (model->theta.max - model->theta.min) / ( static_cast<float>(model->theta.size - 1) );
    
    model->phi.min = -0.261799;
    model->phi.max = 0.261799;
    model->phi.size = 16;
    model->phi.computeStep();
    // automate this somehow?
    
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
    std::cout << "Example GPU: Custom Models" << std::endl;

    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " [meshfile] " << std::endl;
    }

    // Load Map
    OptixMapPtr map = importOptixMap(argv[1]);
    std::cout << "Loaded file '" << argv[1] << "'" << std::endl; 

    std::cout << "- Meshes: " << map->meshes.size() << std::endl;

    // Create Simulator in map
    O1DnSimulatorOptix sim(map);

    // Define sensor model
    
    auto model = custom_model();
    sim.setModel(model);

    // Define Sensor to base transform (offset between simulated pose and scanner)
    Memory<Transform, RAM> Tsb(1);
    Tsb->R.x = 0.0;
    Tsb->R.y = 0.0;
    Tsb->R.z = 0.0;
    Tsb->R.w = 1.0;
    Tsb->t.x = 0.0;
    Tsb->t.y = 0.0;
    Tsb->t.z = 0.0;
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

    std::cout << "Simulate Custom Model" << std::endl;

    // simulate ranges and measure time
    StopWatch sw;
    sw();
    Memory<float, VRAM_CUDA> ranges_ = sim.simulateRanges(Tbm_);
    double el = sw();
    std::cout << "Simulated " << N << " sensors in " << el << "s" << std::endl;

    Memory<float, RAM> ranges;
    ranges = ranges_;

    std::ofstream out("example_gpu_custom_model.xyz", std::ios_base::binary);

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