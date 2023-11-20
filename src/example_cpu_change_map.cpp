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


namespace rm = rmagine;

rm::EmbreeMapPtr make_map1()
{
    aiScene sphere = rm::genSphere(50, 50);
    rm::EmbreeScenePtr scene1 = rm::make_embree_scene(&sphere);
    scene1->commit();
    rm::EmbreeMapPtr map1 = std::make_shared<rm::EmbreeMap>(scene1);

    return map1;
}

rm::EmbreeMapPtr make_map2()
{
    rm::EmbreeMeshPtr cube = std::make_shared<rm::EmbreeCube>();
    {
        auto T = rm::Transform::Identity();
        T.t.z = 2.0;
        cube->setTransform(T);
        cube->setScale(rm::Vector{10.0, 10.0, 10.0});
        cube->apply(); // apply transform
        cube->commit(); // commit element
    }
    
    rm::EmbreeScenePtr scene2 = std::make_shared<rm::EmbreeScene>();
    scene2->add(cube);
    scene2->commit();
    rm::EmbreeMapPtr map2 = std::make_shared<rm::EmbreeMap>(scene2);

    return map2;
}

int main(int argc, char** argv)
{
    std::cout << "Rmagine Examples: CPU change map" << std::endl;

    

    auto map1 = make_map1();
    auto map2 = make_map2();

    
    rm::Simulator<rm::SphericalModel, rm::Embree> sim;
    rm::Transform Tsb;
    Tsb.setIdentity();


    sim.setTsb(Tsb);
    auto model = rm::example_spherical_model();
    model->range.min = 0.0;
    sim.setModel(model);

    rm::Memory<rm::Transform, rm::RAM> Tbm(1);
    Tbm[0].setIdentity();

    
    sim.setMap(map1);
    auto ranges_sphere = sim.simulateRanges(Tbm);
    saveRangesAsXYZ(ranges_sphere, *model, "points_cm_sphere");


    sim.setMap(map2);
    auto ranges_cube = sim.simulateRanges(Tbm);
    saveRangesAsXYZ(ranges_cube, *model, "points_cm_cube");

    return 0;
}