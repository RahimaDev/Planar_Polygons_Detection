#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

#include "MSAC.hpp"



typedef std::vector<Eigen::Vector3d> Point3Dvector;

class PlaneFittingProblem : public robest::EstimationProblem{

public:
    PlaneFittingProblem();
    ~PlaneFittingProblem();

    void setData(pcl::PointCloud<pcl::PointXYZ>::Ptr cl );

    double estimErrorForSample(int i);
    void estimModelFromSamples(const std::vector<int> & samplesIdx);

    int getTotalNbSamples() const{
        return (int) points.size();
    }
    void getResult(double & resa, double & resb, double & resc, double &resd){

        const double & a = this->A;
        const double & b = this->B;
        const double & c = this->C;
        const double & d = this->D;
        double norm = 1./sqrt(a*a+b*b+c*c);



        resa = a * norm;
        resb = b * norm;
        resc = c * norm;
        resd = d*norm;
    }

    Eigen::Vector4f LeastSquaresFitting(pcl::PointCloud<pcl::PointXYZ>::Ptr c)
    {
        // copy coordinates to  matrix in Eigen format
        size_t size = c->points.size();
        Eigen::Matrix<Eigen::Vector3f::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, size);
        for (size_t i = 0; i < size; ++i)
        {Eigen::Vector3f point=Eigen::Vector3f(c->points[i].x,c->points[i].y,c->points[i].z);

            coord.col(i) = point;}

        // calculate centroid
        Eigen::Vector3f centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

        // subtract centroid
        coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);


        auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Vector3f Ei=svd.singularValues();
        int imin = 0;
        //Eigen::Matrix3f U= svd.matrixU();
        if (abs(Ei[1]) < abs(Ei[0])) imin = 1;
        if (abs(Ei[2]) < abs(Ei[imin])) imin = 2;
        //Eigen::Vector3f plane_normal = svd.matrixU().rightCols<1>();
        Eigen::Vector3f plane_normal;
        plane_normal[0] = svd.matrixU()(0,imin);
        plane_normal[1] = svd.matrixU()(1,imin);
        plane_normal[2] = svd.matrixU()(2,imin);

        float d=-(plane_normal[0]*centroid[0]+plane_normal[1]*centroid[1]+plane_normal[2]*centroid[2]);

        auto length = plane_normal.norm();

        plane_normal/= length;
        d /= length;
        Eigen::Vector4f res;
        res[0]=plane_normal[0];
        res[1]=plane_normal[1];
        res[2]=plane_normal[2];
        res[3]=d;
        return res;
    }

private:
    bool isDegenerate(std::vector<int> samplesIdx);
    Point3Dvector points; // Data
    double A = 0.0;
    double B = 0.0;
    double C = 0.0;
    double D = 0.0;
};
