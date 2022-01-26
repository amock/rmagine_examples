#include <iostream>

#include <imagine/util/synthetic.h>

#include <imagine/simulation/SphereSimulatorEmbree.hpp>

#include "imagine_examples/models.h"
#include "imagine_examples/helper.h"

using namespace imagine;

int main(int argc, char** argv)
{
    std::cout << "Imagine Examples: CPU change map" << std::endl;

    auto sphere = genSphere(50, 50);
    EmbreeMapPtr map1(new EmbreeMap(&sphere) );

    auto cube = genCube();
    EmbreeMapPtr map2(new EmbreeMap(&cube));

    Transform Tsb;
    Tsb.setIdentity();

    SphereSimulatorEmbree sim;

    sim.setTsb(Tsb);
    auto model = example_spherical_model();
    sim.setModel(model);

    Memory<Transform, RAM> Tbm(1);
    Tbm[0].setIdentity();

    sim.setMap(map1);

    auto ranges_sphere = sim.simulateRanges(Tbm);

    saveRangesAsXYZ(ranges_sphere, *model, "points_cm_sphere");

    sim.setMap(map2);

    auto ranges_cube = sim.simulateRanges(Tbm);
    saveRangesAsXYZ(ranges_cube, *model, "points_cm_cube");


    return 0;
}