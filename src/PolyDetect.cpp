#include"polygon_visu.hpp"

int main (int argc, char** argv)
{
    cout << "Usage: " << argv[0] << " file.ply  inlier_threshold nb_iter min_plane_size  alpha" << endl;
    if(argc<3) return 1;

    int i = 1;
    string file="";
    if (argc > i)
        file = argv[i++];

    


    float inlier_threshold = 0.03;
    int nb_iter=1000;
    int min_plane_size = 1000;
    float alpha=0.07;
    if (argc > i) inlier_threshold = atof(argv[i++]);
    if (argc > i) nb_iter = atoi(argv[i++]);
    if (argc > i) min_plane_size = atoi(argv[i++]);

    if (argc > i) alpha = atof(argv[i++]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(file, *cloud) == -1) //* load the file
    {
        //PCL_ERROR("Couldn't read file " + file + "\n");
        return (-1);
    }
    std::cout<<cloud->points.size ()<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCopy(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud<pcl::PointXYZ,pcl::PointXYZ>(*cloud, * cloudCopy);
    
    Planes_data result1 = plane_detection(cloudCopy, inlier_threshold,nb_iter ,min_plane_size,alpha);
    std::vector<std::vector<poly_holes>>P_H1;
    std::vector<std::vector<poly_holes>>P_H2;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>V1;

    for(int i=0;i<result1.Pl.size(); i++)
    {
        std::list<Polygon_with_holes> res =Alpha_shape(  result1.Pl[i], result1.In_cloud[i],alpha);
        int h=-1;
        for(typename std::list<Polygon_with_holes>::iterator it_ring = res.begin();
            it_ring != res.end(); it_ring++){

            {h++;
                pcl::PointCloud<pcl::PointXYZ> cl;

                pcl::PointCloud<pcl::PointXYZRGB> CL;

                int32_t red = rand ()>> 16 & 0xFF;
                int32_t green = rand () >> 8 & 0xFF;
                int32_t blue = rand ()& 0xFF;
                cl=Grid_polygon(*it_ring, result1.Pl[i]);
                for(int k=0; k<cl.points.size(); k++)
                {pcl::PointXYZRGB pt;
                    pt.x=cl.points[k].x;
                    pt.y=cl.points[k].y;
                    pt.z=cl.points[k].z;
                    pt.r=red;
                    pt.g=green;
                    pt.b=blue;
                    uint32_t rgb = (red << 16) | (green << 8) | (blue);
                    pt.rgb=rgb;
                    CL.points.push_back(pt);
                }
                pcl::io::savePLYFile("grid_cloud"+to_string(i)+"_"+to_string(h)+".ply",CL);
            }}}
    return 0;
}
