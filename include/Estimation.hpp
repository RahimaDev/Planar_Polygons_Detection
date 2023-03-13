# define MY_PI 3.14159265358979323846

#include <list>
#include <cmath>
#include <iostream>
#include<vector>
#include <numeric>
#include <iostream>
#include <cstring>
#include <boost/random.hpp>
#include <vector>
#include <random>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/geometry.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>

#include<pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/pcl_painter2D.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include "PlaneFitting.hpp"
using namespace std;
struct poly_holes
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr outer;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> inners;
};

struct Plane
{
    pcl::ModelCoefficients::Ptr Cof;
    Eigen::Vector3d normal;
    float d;
    int size;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud;
    std::vector<poly_holes>poly;
    bool processed=false;
};



struct Planes_data
{std::vector<pcl::ModelCoefficients::Ptr> Pl;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> In_cloud;
};

////////////// Angle between two plane normals

double Angle(Eigen::Vector3d V1,Eigen::Vector3d V2 )

{
    
    double  a=V1[0];
    double  b=V1[1];
    double  c=V1[2];

    double h=V2[0];
    double i=V2[1];
    double j=V2[2];






    double s=(a * h)+(b* i)+(c * j);

    //H=||normal1||
    double  t=sqrt((a * a)+(b* b)+(c * c));

    // L=||normal2||
    double u=sqrt((h * h)+(i*i)+(j * j ));
    s=s/(t*u);

    double A=(180/MY_PI)*(acos(s));
    // std::cout<<"Angle is:"<<A<<"degree"<<endl;
    
    return A;
}
///// input data generation for (RANSAC/MSAC)

void generateData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::vector<double> & x, 
                  std::vector<double> & y,
                  std::vector<double> & z)
{for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {x.push_back(cloud->points[i].x);
        y.push_back(cloud->points[i].y);
        z.push_back(cloud->points[i].z);}
    
}
////// plane set estimation using MSAC

Planes_data plane_detection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float thre, int nb_iter, int mi_size ,float alpha){
    Planes_data out_data;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);


    int nb=0, k=0;
    do{std::cout<<cloud->points.size()<<std::endl;
        std::cout<<"Plane:"<<" "<<out_data.Pl.size()<<std::endl;
        std::vector<double> x, y, z;
        generateData(cloud,x,y,z);

        auto planeFitting = std::make_shared<PlaneFittingProblem>();
        planeFitting->setData(cloud);


        robest::MSAC solver;
        //robest::RANSAC solver;
        solver.solve(planeFitting, thre, nb_iter);
        double res_a,res_b,res_c, res_d;
        planeFitting->getResult(res_a,res_b,res_c,res_d);
        double frac=solver.getInliersFraction();
        //std::cout<<frac<<" "<<frac*cloud->points.size()<<std::endl;
        std::cout<<res_a<<" "<<res_b<<" "<<res_c<<" "<<res_d<<std::endl;

        std::vector<int>inl=solver.getInliersIndices();
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::PointIndices::Ptr V (new pcl::PointIndices);
        inliers->indices.resize(inl.size());

        



        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

        for(int i=0; i<inl.size(); i++)
        {inliers->indices[i]=inl[i];}
        std::cout<<"nb_inliers="<<inl.size() <<std::endl;

        pcl::ExtractIndices<pcl::PointXYZ> extract, ext;
        std::vector<pcl::PointXYZ> vec;
        ext.setInputCloud (cloud);
        ext.setIndices (inliers);
        ext.setNegative (false);
        ext.filter (*cloud_out);

        ////////////////////////////////////////

        pcl::PointCloud<pcl::PointXYZRGB> CL;

        int32_t red = rand ()>> 16 & 0xFF;
        int32_t green = rand () >> 8 & 0xFF;
        int32_t blue = rand ()& 0xFF;

        for(int k=0; k<cloud_out->points.size(); k++)
        {pcl::PointXYZRGB pt;
            pt.x=cloud_out->points[k].x;
            pt.y=cloud_out->points[k].y;
            pt.z=cloud_out->points[k].z;
            pt.r=red;
            pt.g=green;
            pt.b=blue;
            uint32_t rgb = (red << 16) | (green << 8) | (blue);
            pt.rgb=rgb;
            CL.points.push_back(pt);
        }


        pcl::io::savePLYFile("inliers_cloud"+to_string(k)+".ply",CL);





        /////////////////////////////////////////////////////
        Eigen::Vector4f res=planeFitting->LeastSquaresFitting(cloud_out);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        coefficients->values.resize (4);
        coefficients->values[0]=res[0];
        coefficients->values[1]=res[1];
        coefficients->values[2]=res[2];
        coefficients->values[3]=res[3];
        std::cout<<res[0]<<" "<<res[1]<<" "<<res[2]<<" "<<res[3]<<std::endl;

        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        
        proj.setInputCloud (cloud_out);
        proj.setModelCoefficients (coefficients);
        proj.filter (*cloud_projected);


        out_data.In_cloud.push_back(cloud_projected);

        out_data.Pl.push_back(coefficients);



        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*cloud_in);
        *cloud=*cloud_in;
        nb=inliers->indices.size();
        k++;

    }
    while(nb>mi_size );
    return out_data;
}
//// plane coefficients normalization

pcl::ModelCoefficients::Ptr Normalize(pcl::ModelCoefficients::Ptr P)
{
    pcl::ModelCoefficients::Ptr  Result(new pcl::ModelCoefficients);
    Result->values.resize (4);


    const double & a = P->values[0];
    const double & b = P->values[1];
    const double & c = P->values[2];
    const double & d = P->values[3];
    double norm = sqrt(a*a+b*b+c*c);



    Result->values[0] = a / norm;
    Result->values[1]= b / norm;
    Result->values[2]= c / norm;
    Result->values[3] = d / norm;

    return Result;
}


