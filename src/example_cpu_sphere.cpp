#include <iostream>
#include <fstream>

// Map & Simulation
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/map/embree/embree_shapes.h>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <rmagine/noise/GaussianNoise.hpp>

// Generic Interface
#include <rmagine/simulation/SimulationResults.hpp>
#include <rmagine/types/Bundle.hpp>

// Utilities
#include <rmagine/util/StopWatch.hpp>

// Models
#include "rmagine_examples/models.h"
#include "rmagine_examples/helper.h"


namespace rm = rmagine;

/**
 * This functions 
 */
rm::EmbreeMapPtr load_map_or_dummy(int argc, const char** argv)
{
  rm::EmbreeMapPtr map;
  if(argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " [meshfile] " << std::endl;
    std::cout << "No meshfile provided: Loading simple cube instead." << std::endl;

    rm::EmbreeScenePtr scene = std::make_shared<rm::EmbreeScene>();    
    rm::EmbreeGeometryPtr mesh = std::make_shared<rm::EmbreeCube>();
    mesh->commit();
    scene->add(mesh);
    scene->commit();
    map = std::make_shared<rm::EmbreeMap>(scene);

  } else {
    // Load map from drive
    map = rm::import_embree_map(argv[1]);
    std::cout << "Loaded file '" << argv[1] << "'" << std::endl;
  }
  return map;
}

int main(int argc, const char** argv)
{
  std::cout << "Example CPU: Simulate spherical model." << std::endl;

  rm::EmbreeMapPtr map = load_map_or_dummy(argc, argv);

  // Create simulator in a map
  rm::SphereSimulatorEmbree sim_lidar(map);

  // Define sensor model
  rm::SphericalModel model;
  {
    model.theta.min = -M_PI;
    model.theta.inc = 0.4 * M_PI / 180.0;
    model.theta.size = 900;
    model.phi.min = -15.0 * M_PI / 180.0;
    model.phi.inc = 2.0 * M_PI / 180.0;
    model.phi.size = 16;
    model.range.min = 0.0;
    model.range.max = 130.0;
  }
  sim_lidar.setModel(model);

  // Define sensor to base transform, ie, offset between simulated pose and scanner
  rm::Transform Tsb = rm::Transform::Identity();
  sim_lidar.setTsb(Tsb);

  // Define a poses to simulate from, ie, transform from base to map
  rm::Transform Tbm = rm::Transform::Identity();

  // define intersection attributes
  using IntAttr = rm::Bundle<
    rm::Ranges<rm::RAM> 
  >;

  // simulate ranges and measure time
  rm::StopWatch sw;
  sw();
  IntAttr res = sim_lidar.simulate<IntAttr>(Tbm);
  double el = sw();

  std::cout << "Simulation Statistics: " << std::endl;
  std::cout << "- Rays: " << res.ranges.size() << std::endl;
  std::cout << "- Runtime: " << el << "s" << std::endl;

  // apply noise
  sw();
  rm::GaussianNoise(0.0, 0.01).apply(res.ranges);
  el = sw();
  std::cout << "- Runtime Noise: " << el << "s" << std::endl;

  saveRangesAsXYZ(res.ranges, model, "points_cpu_sphere");

  return 0;
}