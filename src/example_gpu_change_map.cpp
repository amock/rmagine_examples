#include <iostream>

#include <rmagine/util/synthetic.h>

#include <rmagine/simulation/SimulatorOptix.hpp>
#include <rmagine/map/OptixMap.hpp>
// collection of commonly used shapes
#include <rmagine/map/optix/optix_shapes.h>

#include "rmagine_examples/models.h"
#include "rmagine_examples/helper.h"

namespace rm = rmagine;

rm::OptixMapPtr make_map1()
{
    aiScene sphere = rm::genSphere(50, 50);
    rm::OptixScenePtr scene1 = rm::make_optix_scene(&sphere);
    scene1->commit();
    rm::OptixMapPtr map1 = std::make_shared<rm::OptixMap>(scene1);

    return map1;
}

rm::OptixMapPtr make_map2()
{
    rm::OptixMeshPtr cube = std::make_shared<rm::OptixCube>();
    {
        auto T = rm::Transform::Identity();
        T.t.z = 2.0;
        cube->setTransform(T);
        cube->setScale(rm::Vector{10.0, 10.0, 10.0});
        cube->apply(); // apply transform
        cube->commit(); // commit element
    }
    
    rm::OptixScenePtr scene2 = std::make_shared<rm::OptixScene>();
    scene2->add(cube);
    scene2->commit();
    rm::OptixMapPtr map2 = std::make_shared<rm::OptixMap>(scene2);

    return map2;
}


int main(int argc, char** argv)
{
    std::cout << "Rmagine Examples: GPU change map" << std::endl;
    
    std::cout << "-- TODO: THIS APP CAUSES DOUBLE FREE. FIX THIS" << std::endl;

    auto map1 = make_map1();
    auto map2 = make_map2();


    rm::Simulator<rm::SphericalModel, rm::Optix> sim;

    auto model = rm::example_spherical_model();
    sim.setModel(model);

    rm::Transform Tsb;
    Tsb.setIdentity();
    sim.setTsb(Tsb);

    rm::Memory<rm::Transform, rm::RAM> Tbm(1000);
    for(int i=0; i<1000; i++)
    {
        Tbm[i].setIdentity();
    }

    rm::Memory<rm::Transform, rm::VRAM_CUDA> Tbm_;
    Tbm_ = Tbm;

    using IntAttr = rm::Bundle<
        rm::Ranges<rm::VRAM_CUDA> 
    >;

    rm::Memory<float, rm::RAM> ranges;
    
    sim.setMap(map1);
    ranges = sim.simulate<IntAttr>(Tbm).ranges;
    saveRangesAsXYZ(ranges, *model, "points_cm_sphere");

    sim.setMap(map2);
    ranges = sim.simulate<IntAttr>(Tbm).ranges;
    saveRangesAsXYZ(ranges, *model, "points_cm_cube");

    return 0;
}