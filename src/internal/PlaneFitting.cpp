
#include "PlaneFitting.hpp"

PlaneFittingProblem::PlaneFittingProblem(){
    setNbParams(4);
    setNbMinSamples(3);
}

PlaneFittingProblem::~PlaneFittingProblem(){

}

void PlaneFittingProblem::setData(pcl::PointCloud<pcl::PointXYZ>::Ptr cl){

    points.clear();
    for (auto i = 0; i < cl->points.size(); i++){
        points.push_back(Eigen::Vector3d(cl->points[i].x,cl->points[i].y,cl->points[i].z));
    }
}

double PlaneFittingProblem::estimErrorForSample(int i)
{
    const Eigen::Vector3d & P = points[i];
    return std::fabs(A*P[0]+B*P[1]+C*P[2]+D)/sqrt(A*A+B*B+C*C);
}

void PlaneFittingProblem::estimModelFromSamples(const std::vector<int> & samplesIdx){

    if( !isDegenerate(samplesIdx)){

        const Eigen::Vector3d & P = points[samplesIdx[0]];
        const Eigen::Vector3d& V = points[samplesIdx[1]];
        const Eigen::Vector3d  & K = points[samplesIdx[2]];

        A =  (V[1]-P[1])*(K[2]-P[2]) - (K[1]-P[1])*(V[2]-P[2]);
        B =  (V[0]-P[0])*(K[2]-P[2]) + (V[2]-P[2])*(K[0]-P[0]);
        C =  (V[0]-P[0])*(K[1]-P[1]) - (V[1]-P[1])*(K[0]-P[0]);
        D =  (-1)*(A*P[0] + B*P[1] +C*P[2]);
    }
}
bool PlaneFittingProblem::isDegenerate(std::vector<int> samplesIdx)
{
    const Eigen::Vector3d & P = points[samplesIdx[0]];
    const Eigen::Vector3d & V = points[samplesIdx[1]];
    const Eigen::Vector3d & K = points[samplesIdx[2]];

    const Eigen::Vector3d u(V[0] - P[0], V[1] - P[1], V[2] - P[2]);
    const Eigen::Vector3d v(K[0] - P[0], K[1] - P[1], K[2] - P[2]);

    const auto dotProduct = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];

    return ( dotProduct < 1e-5 );
}






















