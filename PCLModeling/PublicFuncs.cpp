#include "PublicFuncs.h"
#include <pcl/common/impl/common.hpp>
#include <lasreader.hpp>
#include <laswriter.hpp>
#include <fstream>
#include <iostream>
#include "dvprint.h"



PointT GetCenter(const PointCloudT cloud)
{
    PointT min, max, center;
    pcl::getMinMax3D(cloud, min, max);
    center.x = (min.x + max.x) / 2;
    center.y = (min.y + max.y) / 2;
    center.z = (min.z + max.z) / 2;
    return center;
}

vec PointT2Vec(const PointT &ptt)
{
    return vec(ptt.x, ptt.y, ptt.z);
}

PointT Vec2PointT(const vec &ptv)
{
    PointT ptt;
    ptt.x = ptv.x;
    ptt.y = ptv.y;
    ptt.z = ptv.z;
    return ptt;
}

void Transitionlocal(PointCloudT &cloud, IN vec const *centerptr/*=nullptr*/)
{
    vec center;
    if (!centerptr)
        center = PointT2Vec(GetCenter(cloud));
    else
        center = *centerptr;
    int cloudSize = cloud.size();
    for (int i = 0; i < cloudSize; ++i)
    {
        PointT &cur = cloud.at(i);
        cur.x -= center.x;
        cur.y -= center.y;
        cur.z -= center.z;
    }
}

//用double类型不丢失精度
void Xyz2Las(const std::string & infile, const std::string &outfile)
{
    LASwriteOpener laswriteopener;
    dd("%s", outfile.c_str());
    laswriteopener.set_file_name(outfile.c_str());
    if (!laswriteopener.active())
    {
        return;
    }
    LASheader lasheader;
    lasheader.x_offset = 0;
    lasheader.y_offset = 0;
    lasheader.z_offset = 0.0;
    lasheader.point_data_format = 3;	//1是xyzrgb		,颜色设置不上，未找到原因暂时屏蔽掉
    lasheader.point_data_record_length = 34;
    // init point 

    LASpoint laspoint;
    laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);

    // open laswriter
    LASwriter* laswriter = laswriteopener.open(&lasheader);
    if (laswriter == 0)
    {
        fprintf(stderr, "ERROR: could not open laswriter\n");
        return;
    }
    std::ifstream is(infile);
    std::string line;
    if (is.is_open())
    {
        double  x, y, z;
        u_short r, g, b;
        int scanfCount = 0;
        while (std::getline(is, line))
        {
            scanfCount = sscanf(line.c_str(), "%lf %lf %lf %d %d %d", &x, &y, &z, &r, &g, &b);

            if (6 == scanfCount)
            {
                laspoint.set_R(r * 256);
                laspoint.set_G(g * 256);
                laspoint.set_B(b * 256);
                laspoint.have_rgb = true;
            }
            else if (3 == scanfCount)
            {
                laspoint.set_R(0);
                laspoint.set_G(0);
                laspoint.set_B(0);
            }
            else
            {
                continue;
            }

            laspoint.set_x(x);
            laspoint.set_y(y);
            laspoint.set_z(z);
            laswriter->write_point(&laspoint);
        }
        is.close();
    }
    // update the header
    laswriter->update_header(&lasheader, TRUE);
    laswriter->close();
    delete laswriter;
}