/** @file
 Light 3D distance class for use in MATIS.
 For each function we give the numbers of (+-,*,/,<>!=) operators used in worst case\n
 Lg::Point3::{+,-} (3,0,0,0)\n
 Lg::Point3::{.,Norm2()) (2,3,0,0)\n
 Lg::Point3::^ (3,6,0,0)\n
 WARNING: some distances are not implemented. Ask me if you need such a distance.\n
Author: Bruno Vallet 30/11/2009
*/

#ifndef LgDistance3_hpp
#define LgDistance3_hpp

#include <math.h>
#include "LgPoint3.hpp"
#include "LgLine3.hpp"
#include "LgSegment3.hpp"
#include "LgPlane3.hpp"

/// namespace for all LiteGeom objects
namespace Lg {

/// Point3/Point3 distance (5,3,0,0)
template <typename T> T SquaredDistance( const TPoint3<T> & P0, const TPoint3<T> & P1 )
{
    return (P1-P0).Norm2();
}

/// Point3/Line3 distance (10,12,1,0)
template <typename T> T SquaredDistance( const TPoint3<T> & P, const TLine3<T> & l )
{
    TPoint3<T> y = (P-l.A())^l.Vector(); // 6,6,0,0
    return y.Norm2()/T(l.Vector().Norm2()); // 4,6,1,0
}
/// Line3/Point3 distance (10,12,1,0)
template <typename T> T SquaredDistance( const TLine3<T> & l, const TPoint3<T> & P )
{
    return SquaredDistance<T>(P, l);
}
/// Line3/Line3 distance (18,23,1,0) TODO: optimize
template <typename T> T SquaredDistance( const TLine3<T> & l0, const TLine3<T> & l1)
{
    T v00 = l0.V().Norm2(), v11 = T(l1.V().Norm2()), v01 = l0.V()*l1.V(); // 6,9,0,0
    TPoint3<T> A01 = l0.A() - l1.A(); //3,0,0,0
    T a0 = A01*l0.V(), a1 = A01*l1.V(); //4,6,0,0
    T det=(v00*v11-v01*v01); //1,2,0,0
    T x0, x1;
    if(det)
    {
        T denom=T(1)/det; //0,0,1,0
        x0 = denom*(v01*a1-v11*a0); //1,3,0,0
        x1 = denom*(v00*a1-v01*a0); //1,3,0,0
        return (A01+x0*l0.V()-x1*l1.V()).Norm2(); //8,9,0,0
    }
    return (A01^l0.V()).Norm2()/v00; //2,9,1,0
}

/// Point3/Segment3 distance (17,15,1,2) TODO: optimize
template <typename T> T SquaredDistance( const TPoint3<T> & P, const TSegment3<T> & s )
{
    TPoint3<T> v = s.Vector(), AP = P - s.A(), y = v^AP; // 6,0,0,0
    T n2 = v.Norm2(), x = AP*v, y2 = y*y; // 8,12,0,0
    if(x<T(0)) { // 0,0,0,1
        return (x*x + y2)/n2; // not worst
    }
    x -= n2;
    if(x<T(0)) { // 0,0,0,1
        return y2/n2; // not worst
    }
    return (x*x + y2)/n2; // 3,3,1,0
}
/// Segment3/Point3 distance (17,15,1,2)
template <typename T> T SquaredDistance( const TSegment3<T> & s, const TPoint3<T> & P )
{
    return SquaredDistance<T>(P, s);
}

/// Line3/Segment3 distance TODO: optimize
template <typename T> T SquaredDistance( const TLine3<T> & l, const TSegment3<T> & s)
{
    T v00 = l.V().Norm2(), v11 = T(l.V().Norm2()), v01 = l.V()*s.V(); // 6,9,0,0
    TPoint3<T> A01 = l.A() - s.A(); //3,0,0,0
    T a0 = A01*l.V(), a1 = A01*s.V(); //4,6,0,0
    T det=(v00*v11-v01*v01); //1,2,0,0
    T x0, x1;
    if(det)
    {
        T denom=T(1)/det; //0,0,1,0
        x0 = denom*(v01*a1-v11*a0); //1,3,0,0
        x1 = denom*(v00*a1-v01*a0); //1,3,0,0
        if(x1>1.) x1=1.; else if(x1<0.) x1=0.;
        return (A01+x0*l.V()-x1*s.V()).Norm2(); //8,9,0,0
    }
    return (A01^l.V()).Norm2()/v00; //2,9,1,0
}
/// Segment3/Segment3 distance TODO: optimize
template <typename T> T SquaredDistance( const TSegment3<T> & s0, const TSegment3<T> & s1)
{
    T v00 = s0.V().Norm2(), v11 = T(s0.V().Norm2()), v01 = s0.V()*s1.V(); // 6,9,0,0
    TPoint3<T> A01 = s0.A() - s1.A(); //3,0,0,0
    T a0 = A01*s0.V(), a1 = A01*s1.V(); //4,6,0,0
    T det=(v00*v11-v01*v01); //1,2,0,0
    T x0, x1;
    if(det)
    {
        T denom=T(1)/det; //0,0,1,0
        x0 = denom*(v01*a1-v11*a0); //1,3,0,0
        x1 = denom*(v00*a1-v01*a0); //1,3,0,0
        if(x0>1.) x0=1.; else if(x0<0.) x0=0.;
        if(x1>1.) x1=1.; else if(x1<0.) x1=0.;
        return (A01+x0*s0.V()-x1*s1.V()).Norm2(); //8,9,0,0
    }
    // TODO: exact computation
    return std::min(std::min(SquaredDistance(s0.A(),s1.A()),SquaredDistance(s0.B(),s1.B())),
                    std::min(SquaredDistance(s0.A(),s1.B()),SquaredDistance(s0.B(),s1.A())));
}

// TODO: distances involving Triangles

/// Point3/Plane3 signed distance (3,3,0,0) the plane equation is OP.N=d, assumes N is normalized
template <typename T> T SignedDistance( const TPoint3<T> & P, const TPlane3<T> & plane )
{
    return P*plane.Normal()-plane.Dist(); // 3,3,0,0
}
/// Plane3/Point3 signed distance (3,3,0,0) the plane equation is OP.N=d, assumes N is normalized
template <typename T> T SignedDistance( const TPlane3<T> & plane, const TPoint3<T> & P )
{
    return SignedDistance(P, plane);
}

/// Point3/Plane3 distance (5,7,1,0) the plane equation is OP.N=d, even if N is not normalized
template <typename T> T SquaredDistance( const TPoint3<T> & P, const TPlane3<T> & plane )
{
    T d = P*plane.Normal()-plane.Dist(); // 3,3,0,0
    return (d*d)/T(plane.Normal().Norm2()); // 2,4,1,0
}
/// Plane3/Point3 distance (5,7,1,0) the plane equation is OP.N=d, even if N is not normalized
template <typename T> T SquaredDistance( const TPlane3<T> & plane, const TPoint3<T> & P )
{
    return SquaredDistance<T>(plane, P);
}

#ifndef LgDistance2_hpp
/// generic square root of a squared distance
template <class G1, class G2> typename G1::coord_type Distance( const G1 & g1, const G2 & g2 )
{
    typename G1::coord_type d = SquaredDistance(g1, g2);
    return sqrt(d);
}
#endif // LgDistance2_hpp

}; // namespace Lg

#endif // LgDistance3_hpp
