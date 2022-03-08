#ifndef RMAGINE_EXAMPLES_HELPER_H
#define RMAGINE_EXAMPLES_HELPER_H

#include <rmagine/types/Memory.hpp>
#include <fstream>
#include <sstream>

namespace rmagine {

template<typename ModelT>
void saveRangesAsXYZ(const Memory<float, RAM>& ranges, const ModelT& model, std::string name)
{
    std::stringstream ss;

    if(ranges.size() == 0)
    {
        std::cout << "saveRangesAsXYZ - emptyBuffers" << std::endl;
        return;
    }

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
                Vector dir = model.getDirection(vid, hid);
                
                // std::cout << "Ray: " << ray.x << " " << ray.y << " " << ray.z << std::endl;
                float range = ranges[loc_id];
                if(range >= model.range.min && range <= model.range.max)
                {
                    Point p = orig + dir * range;
                    // std::cout << "Intersection: " << p.x << " " << p.y << " " << p.z << std::endl;
                    out << p.x << " " << p.y << " " << p.z << "\n";
                }
            }
        }

        out.close();
    }

    std::cout << "Output written to '" << ss.str() << "'" << std::endl; 

}

} // namespace rmagine

#endif // RMAGINE_EXAMPLES_HELPER_H