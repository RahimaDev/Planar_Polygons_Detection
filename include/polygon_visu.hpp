#include"Polygon.hpp"


///////////// grid points generation
pcl::PointCloud<pcl::PointXYZ>::Ptr Grid_cloud(Polygon_with_holes& p ,pcl::ModelCoefficients::Ptr pl)
{pcl::PointXYZ min_pt, max_pt;
    Lg::Point3f n(pl->values[0], pl->values[1],pl->values[2]);
    Lg::Point3f m = best_m(n);
    m.Normalize();
    Lg::Point3f k = n^m;
    k.Normalize();
    Lg::Point3f O=( - pl->values[3]/((pl->values[0]*pl->values[0])+(pl->values[1]*pl->values[1])+(pl->values[2]*pl->values[2])))*n;
    std::vector<Point> VEC;

    Polygon outer = p.outer_boundary();
    ///////
    for(typename Polygon::Vertex_const_iterator it_vertex =outer.vertices_begin();
        it_vertex != outer.vertices_end(); it_vertex++){
        VEC.push_back(Point( it_vertex->x(),it_vertex->y() ));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);



    cloud->width    = VEC.size();
    cloud->height   = 1;
    cloud->is_dense = false;
    for(int i=0; i<VEC.size(); i++)
    {Lg::Point3f A;
        A =O +CGAL::to_double( VEC[i].x())*m +CGAL::to_double( VEC[i].y())*k;

        pcl::PointXYZ B=pcl::PointXYZ(A.x(),A.y(), A.z());
        cloud->points.push_back(B);

    }
    pcl::getMinMax3D(*cloud, min_pt, max_pt);




    std::vector<pcl::PointXYZ>VEC1;
    float seed =0.02f;
    float M=(max_pt.z-min_pt.z)/2;

    for(float x=min_pt.x-1.; x<max_pt.x+1.; x+=0.01f)
    {
        for(float y=min_pt.y-1.; y<max_pt.y+1.; y+=0.01f)//
        {float z;
            if(pl->values[2]==0)
                z=M;



            else
                z=-(pl->values[3]+pl->values[0]*x+pl->values[1]*y)/pl->values[2];
            pcl::PointXYZ pt=pcl::PointXYZ(x,y,z);

            VEC1.push_back(pt);
        }}
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

    output->width    = VEC1.size();
    output->height   = 1;
    output->is_dense = false;

    for(int i=0;i<VEC1.size(); i++)
    {


        output->points.push_back(VEC1[i]);
    }

    return output;
}


//////////////  3D polygon creation for simplifing the visualization

pcl::PointCloud<pcl::PointXYZ> Grid_polygon( Polygon_with_holes& p ,pcl::ModelCoefficients::Ptr pl)
{    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


     std::vector<pcl::PointCloud<pcl::PointXYZ>> out_data;
      pcl::PointCloud<pcl::PointXYZ>polygon_hul ;
       pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZ>) ;
        pcl::PointCloud<pcl::PointXYZ> Out;

         Polygon p1,p2;
          Cloud=Grid_cloud(p,pl);

           // pcl::io::savePLYFile("Grid.ply",Cloud);
           p1= p.outer_boundary();



            Lg::Point3f n(pl->values[0], pl->values[1], pl->values[2]);

             //n.Normalize();
             Lg::Point3f m = best_m(n);
              m.Normalize();
               Lg::Point3f k = n^m;
                k.Normalize();
                 Lg::Point3f O=(-pl->values[3]/(n*n))*n;
                  std::vector<int>indices;
                   std::vector<K::Point_2> points_cgal(Cloud->points.size());
                    for(int i=0; i<Cloud->points.size(); i++)
                    {
                        Lg::Point3f P=Lg::Point3f(Cloud->points[i].x,Cloud->points[i].y,Cloud->points[i].z);
                        Lg::Point3f P0 = Lg::Point3f(P-O);
                        float xp=P0*m, yp=P0*k;
                        K::Point_2 pp=K::Point_2(xp, yp);
                        bool IsInside = (p1.bounded_side(pp) == CGAL::ON_BOUNDED_SIDE);
                        bool IsInside2 = (p1.bounded_side(pp) == CGAL::ON_BOUNDARY);
                        if((IsInside==true)||(IsInside2==true))

                            Out.points.push_back(Cloud->points[i]);
                    }




                     return Out;
}
