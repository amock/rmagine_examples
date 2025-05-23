#ifndef RMAGINE_EXAMPLES_MODELS_H
#define RMAGINE_EXAMPLES_MODELS_H

#include <rmagine/types/sensor_models.h>

namespace rmagine 
{

/**
 * @brief Example model: Velodyne laserscanner
 * 
 * @return SphericalModel 
 */
SphericalModel example_spherical_model()
{
  SphericalModel model;
  model.theta.min = -M_PI;
  model.theta.inc = 0.4 * M_PI / 180.0;
  model.theta.size = 900;

  model.phi.min = -15.0 * M_PI / 180.0;
  model.phi.inc = 2.0 * M_PI / 180.0;
  model.phi.size = 16;
  
  model.range.min = 0.0;
  model.range.max = 130.0;
  return model;
}

/**
 * @brief Example pinhole model. Some camera
 * 
 * @return Memory<PinholeModel, RAM> 
 */
Memory<PinholeModel, RAM> example_pinhole_model()
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



/**
 * @brief Example O1Dn Model. One origin n Directions
 * 
 * @return O1DnModel 
 */
O1DnModel example_o1dn_model()
{
    // represent spherical model as custom model to compare results
    // build model out of two velo models
    auto velo_model = example_spherical_model();

    O1DnModel model;
        
    size_t W = velo_model.getWidth();
    size_t H = velo_model.getHeight() * 2;

    model.width = W;
    model.height = H;
    model.range = velo_model.range;

    model.orig.x = 0.0;
    model.orig.y = 0.0;
    model.orig.z = 0.5;
    model.dirs.resize(W * H);

    for(size_t vid=0; vid<velo_model.getHeight(); vid++)
    {
        for(size_t hid=0; hid<velo_model.getWidth(); hid++)
        {
            const Vector ray = velo_model.getDirection(vid, hid);
            unsigned int loc_id_1 = model.getBufferId(vid, hid);
            model.dirs[loc_id_1] = ray;

            const Vector ray_flipped = {ray.x, ray.z, ray.y};
            unsigned int loc_id_2 = model.getBufferId(vid + velo_model.getHeight(), hid);
            model.dirs[loc_id_2] = ray_flipped;
        }
    }

    return model;
}

/**
 * @brief Example OnDn Model. n origins n directions
 * 
 * @return O1DnModel 
 */
OnDnModel example_ondn_model()
{
    OnDnModel model;

    model.height = 10;
    model.width = 100;

    model.range.min = 0.0;
    model.range.max = 100.0;
    
    model.origs.resize(model.width * model.height);
    model.dirs.resize(model.width * model.height);

    for(size_t vid=0; vid<model.getHeight(); vid++)
    {
        Vector orig = {0.0, 0.0, 0.0};
        // v equally distributed between -0.5 and 0.5
        float v = (static_cast<float>(vid) - static_cast<float>(model.getHeight() / 2) ) / static_cast<float>(model.getHeight());
        orig.z = v;
        for(size_t hid=0; hid<model.getWidth(); hid++)
        {
            // h from 0 to 2PI
            float h = static_cast<float>(hid) / static_cast<float>(model.getWidth()) * 2.0 * M_PI;
            Vector ray = {cos(h), sin(h), 0.0};
            unsigned int loc_id = model.getBufferId(vid, hid);
            model.origs[loc_id] = orig;
            model.dirs[loc_id] = ray;
        }
    }

    return model;
}


} // namespace rmagine


#endif // RMAGINE_EXAMPLES_MODELS_H