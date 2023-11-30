#include <iostream>

#include <rmagine/util/synthetic.h>

#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <rmagine/simulation/SimulatorEmbree.hpp>
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/map/embree/EmbreeScene.hpp>

// example geometries
#include <rmagine/map/embree/embree_shapes.h>

#include "rmagine_examples/models.h"
#include "rmagine_examples/helper.h"

#include <unordered_map>

#include <omp.h>



namespace rm = rmagine;

rm::EmbreeMapPtr make_map()
{
    aiScene sphere = rm::genSphere(50, 50);
    rm::EmbreeScenePtr scene1 = rm::make_embree_scene(&sphere);
    scene1->commit();
    rm::EmbreeMapPtr map1 = std::make_shared<rm::EmbreeMap>(scene1);

    return map1;
}


int main(int argc, char** argv)
{
    // Example which fails. 
    // TODO: put something like this into offical tests
    std::cout << "Rmagine Examples: CPU change sim" << std::endl;

    auto mesh_map = make_map();

    std::unordered_map<unsigned int, rm::SphereSimulatorEmbreePtr> sims;


    int num_sims = 8;

    // construct simulators
    for(size_t i=0; i<num_sims; i++)
    {
        rm::SphereSimulatorEmbreePtr sim = std::make_shared<rm::SphereSimulatorEmbree>(mesh_map);
        rm::Transform Tsb;
        Tsb.setIdentity();
        sim->setTsb(Tsb);

        auto model = rm::example_spherical_model();
        model->range.min = 0.0;
        sim->setModel(model);

        sims[i] = sim;
    }

    rm::Memory<rm::Transform, rm::RAM> Tbm(1);
    Tbm[0].setIdentity();

    #pragma omp parallel for
    for(size_t i=0; i<1000; i++)
    {   
        int tid = 0;
        {
            // threaded. get sim that belong to current thread
            tid = omp_get_thread_num();
        }
        #pragma omp critical
        std::cout << "Thread: " << tid << std::endl;

        // get simulator
        rm::SphereSimulatorEmbreePtr sim;

        auto sims_it = sims.find(tid);
        if(sims.find(tid) != sims.end())
        {
            // TAKE SIMULATOR FROM CACHE
            sim = sims_it->second;
        } else {
            #pragma omp critical
            std::cout << "Create new simulator for thread " << tid << "..." << std::endl; 

            // NEW
            sim = std::make_shared<rm::SphereSimulatorEmbree>(mesh_map);
            sim->setTsb(rm::Transform::Identity());
            sims[tid] = sim;

            #pragma omp critical
            std::cout << "Created new simulator for thread " << tid << std::endl; 
        }

        // get simulator

        using IntAttr = rm::Bundle<
            rm::Ranges<rm::RAM> 
        >;

        IntAttr res = sim->simulate<IntAttr>(Tbm);
        
    }



    return 0;
}