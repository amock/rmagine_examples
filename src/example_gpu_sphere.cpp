#include <iostream>
#include <fstream>
#include <imagine/simulation/SphereSimulatorOptix.hpp>
#include <imagine/simulation/PinholeSimulatorOptix.hpp>
#include <imagine/util/StopWatch.hpp>
#include <imagine/math/math.h>

using namespace imagine;

Memory<LiDARModel, RAM> velodyne_model()
{
    Memory<LiDARModel, RAM> model(1);
    model->theta.min = -M_PI;
    model->theta.max = M_PI; 
    model->theta.size = 440;
    model->theta.step = (model->theta.max - model->theta.min) / ( static_cast<float>(model->theta.size - 1) );
    
    model->phi.min = -0.261799;
    model->phi.max = 0.261799;
    model->phi.size = 16;
    // automate this somehow?
    model->phi.step = (model->phi.max - model->phi.min) / ( static_cast<float>(model->phi.size - 1) );
    
    model->range.min = 0.5;
    model->range.max = 130.0;
    return model;
}

Memory<PinholeModel, RAM> camera_model()
{
    Memory<PinholeModel, RAM> model(1);
    model->width = 400;
    model->height = 300;
    model->c[0] = 200.0;
    model->c[1] = 150.0;
    model->f[0] = 1000.0;
    model->f[1] = 1000.0;
    model->range.min = 0.0;
    model->range.max = 100.0;
    return model;
}

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
    SphereSimulatorOptix sim_sphere(map);
    PinholeSimulatorOptix sim_pinhole(map);

    // Define sensor model
    Memory<LiDARModel, RAM> model_sphere = velodyne_model();
    sim_sphere.setModel(model_sphere);

    Memory<PinholeModel, RAM> model_pinhole = camera_model();
    sim_pinhole.setModel(model_pinhole);

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
    sw();
    Memory<float, VRAM_CUDA> ranges_ = sim_sphere.simulateRanges(Tbm);
    double el = sw();
    std::cout << "Simulated " << N << " spheres in " << el << "s" << std::endl;

    // Download
    Memory<float, RAM> ranges;
    ranges = ranges_;

    std::ofstream out("example_gpu_sphere.xyz", std::ios_base::binary);

    if(out.good())
    {
        for(unsigned int vid=0; vid<model_sphere->getHeight(); vid++)
        {
            for(unsigned int hid=0; hid<model_sphere->getWidth(); hid++)
            {
                const unsigned int loc_id = model_sphere->getBufferId(vid, hid);
                Vector ray = model_sphere->getRay(vid, hid);
                float range = ranges[loc_id];
                if(range >= model_sphere->range.min && range <= model_sphere->range.max)
                {
                    Point p = ray * range;
                    out << p.x << " " << p.y << " " << p.z << "\n";
                }
            }
        }

        out.close();
    }
    
    sw();
    ranges_.resize(0);
    ranges_ = sim_pinhole.simulateRanges(Tbm);
    el = sw();

    ranges = ranges_;
    std::cout << "Simulated " << N << " pinholes in " << el << "s" << std::endl;

    std::ofstream out_pinhole("example_gpu_pinhole.xyz", std::ios_base::binary);

    if(out_pinhole.good())
    {
        for(unsigned int vid=0; vid<model_pinhole->getHeight(); vid++)
        {
            for(unsigned int hid=0; hid<model_pinhole->getWidth(); hid++)
            {
                const unsigned int loc_id = model_pinhole->getBufferId(vid, hid);
                Vector ray = model_pinhole->getRay(vid, hid);
                float range = ranges[loc_id];
                if(range >= model_sphere->range.min && range <= model_sphere->range.max)
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