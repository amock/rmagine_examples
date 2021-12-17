#include <iostream>
#include <fstream>
#include <imagine/simulation/SphereSimulatorEmbree.hpp>
#include <imagine/simulation/PinholeSimulatorEmbree.hpp>
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
    PinholeSimulatorEmbree sim_pinhole(map);

    // Define sensor model
    Memory<LiDARModel, RAM> sphere_model = velodyne_model();
    sim_sphere.setModel(sphere_model);
    
    Memory<PinholeModel, RAM> pinhole_model = camera_model();
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
    sim_sphere.setTsb(Tsb);
    sim_pinhole.setTsb(Tsb);

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

    std::ofstream out("example_cpu_sphere.xyz", std::ios_base::binary);

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
    

    std::cout << "Simulate Pinhole Model" << std::endl;

    sw();
    Memory<float, RAM> pinhole_ranges = sim_pinhole.simulateRanges(Tbm);
    el = sw();
    std::cout << "Simulated " << N << " sensors in " << el << "s" << std::endl;

    std::ofstream out_pinhole("example_cpu_pinhole.xyz", std::ios_base::binary);

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

    // // 2. Generic Interface

    // // Scenario: We want to check which face of which object was intersected
    // using ResultT = Bundle<Hits<RAM>, ObjectIds<RAM>, FaceIds<RAM> >;

    // ResultT res = sim.simulate<ResultT>(Tbm);

    // // res.hits : Memory<uint8_t, RAM>
    // // res.object_ids : Memory<uint32_t, RAM>
    // // res.face_ids: Memory<uint32_t, RAM>

    // for(unsigned int vid=0; vid<model->phi.size; vid++)
    // {
    //     for(unsigned int hid=0; hid<model->theta.size; hid++)
    //     {
    //         const unsigned int loc_id = model->getBufferId(vid, hid);

    //         bool hit = res.hits[loc_id];
    //         unsigned int object_id = res.object_ids[loc_id];
    //         unsigned int face_id = res.face_ids[loc_id];

    //         // if(hit)
    //         // {
    //         //     std::cout << "(object, face): " << object_id << ", " << face_id << std::endl;
    //         // }
            
    //     }
    // }
    


    return 0;
}