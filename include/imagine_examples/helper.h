#ifndef IMAGINE_EXAMPLES_HELPER_H
#define IMAGINE_EXAMPLES_HELPER_H

#include <imagine/types/Memory.hpp>
#include <fstream>
#include <sstream>

namespace imagine {

template<typename ModelT>
void saveRangesAsXYZ(const Memory<float, RAM>& ranges, const ModelT& model, std::string name)
{
    std::stringstream ss;

    ss << name << ".xyz";

    std::ofstream out(ss.str(), std::ios_base::binary);

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


}

} // namespace imagine

#endif // IMAGINE_EXAMPLES_HELPER_H