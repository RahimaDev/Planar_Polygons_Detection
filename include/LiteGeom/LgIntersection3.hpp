/** @file
 Light 3D intersection class for use in MATIS (fast but no degenerate cases handling).
 Tests if a primitive intersects another.\n
 If a third arg is given, it is used to return the result of the intersection.\n
Author: Bruno Vallet 30/11/2009
*/

#ifndef LgIntersection3_hpp
#define LgIntersection3_hpp

#include "LgMat3.hpp"

/// namespace for all LiteGeom objects
namespace Lg {

#ifdef LgPlane3_hpp
#ifdef LgLine3_hpp

    /// Line3/Plane3 intersection
    template<typename T> bool Intersects( const TLine3<T> & l, const TPlane3<T> & p )
    {
        return (p.Normal() * l.Vector() != T(0));
    }
    /// Plane3/Line3 intersection
    template<typename T> bool Intersects( const TPlane3<T> & p, const TLine3<T> & l )
    {
        return (p.Normal() * l.Vector() != T(0));
    }

    /// x coord of the intersection of a Line3/Plane3 intersection
    template<typename T> T xInter ( const TLine3<T> & l, const TPlane3<T> & p)
    {
        T vn = l.Vector()*p.Normal();
        if(vn == T(0)) return std::numeric_limits<T>::max(); // parallel
        return (p.Dist() - l.Point()*p.Normal())/vn;
    }
#ifdef LgRay3_hpp
    /// Ray3/Plane3 intersection
    template<typename T> bool Intersects( const TRay3<T> & r, const TPlane3<T> & p )
    {
        T vn = r.Vector()*p.Normal();
        if(vn == T(0)) return false; // parallel
        return (p.Dist() - r.Point()*p.Normal())>=0;
    }
    /// Plane3/Ray3 intersection
    template<typename T> bool Intersects( const TPlane3<T> & p, const TRay3<T> & r )
    {
        return Intersects(r, p);
    }
    /// Ray3/Plane3 intersection
    template<typename T> bool Intersects( const TRay3<T> & r, const TPlane3<T> & p, TPoint3<T> & I)
    {
        T vn = r.Vector()*p.Normal();
        if(vn == T(0)) return false; // parallel
        T x = p.Dist() - r.Point()*p.Normal();
        if(x<0) return false; // not in right direction
        I = r.Point() + x/vn * r.Vector();
        return true;
    }
    /// Plane3/Ray3 intersection
    template<typename T> bool Intersects( const TPlane3<T> & p, const TRay3<T> & r, TPoint3<T> & I)
    {
        return Intersects(r, p, I);
    }
#endif // LgRay3_hpp

    /// Line3/Plane3 intersection
    template<typename T> bool Intersects( const TLine3<T> & l, const TPlane3<T> & p, TPoint3<T> & I)
    {
        T x = xInter(l, p);
        I = l.Point() + x * l.Vector();
        return (x != std::numeric_limits<T>::max());
    }
    /// Plane3/Line3 intersection
    template<typename T> bool Intersects( const TPlane3<T> & p, const TLine3<T> & l, TPoint3<T> & I )
    {
        return Intersects(l, p, I);
    }
    /// Line3/Plane3 intersection
    template<typename T> TPoint3<T> Intersection( const TLine3<T> & l, const TPlane3<T> & p )
    {
        return l.Point() + xInter(l, p) * l.Vector();
    }
    /// Plane3/Line3 intersection
    template<typename T> TPoint3<T> Intersection( const TPlane3<T> & p, const TLine3<T> & l )
    {
        return l.Point() + xInter(l, p) * l.Vector();
    }
#endif // LgLine3_hpp

#ifdef LgSegment3_hpp
    /// x coord of the intersection of a Segment3/Plane3 intersection
    template<typename T> T xInter ( const TSegment3<T> & s, const TPlane3<T> & p)
    {
        return xInter<T>(s.Line(), p);
    }
    /// Segment3/Plane3 intersection
    template<typename T> bool Intersects(const TSegment3<T> & s, const TPlane3<T> & p)
    {
        T x = xInter(s, p);
        return (x > T(0) && x < T(1));
    }
    /// Plane3/Segment3 intersection
    template<typename T> bool Intersects(const TPlane3<T> & p, const TSegment3<T> & s)
    {
        return Intersects<T>(s, p);
    }
    /// Segment3/Plane3 intersection
    template<typename T> bool Intersects(const TSegment3<T> & s, const TPlane3<T> & p, TPoint3<T> & I)
    {
        T x = xInter(s, p);
        I = s.A() + x*s.Vector();
        return (x > T(0) && x < T(1));
    }
    /// Plane3/Segment3 intersection
    template<typename T> bool Intersects(const TPlane3<T> & p, const TSegment3<T> & s, TPoint3<T> & I)
    {
        return Intersects<T>(s, p, I);
    }
#endif // LgSegment3_hpp

    // TODO: Plane/Plane

#ifdef LgTriangle3_hpp
    /// Triangle3/Plane3 intersection
    template<typename T> bool Intersects(const TTriangle3<T> & t, const TPlane3<T> & p)
    {
        return Intersects<T>(t.Segment(0), p) ||
                Intersects<T>(t.Segment(1), p) ||
                Intersects<T>(t.Segment(2), p);
    }
    /// Plane3/Triangle3 intersection
    template<typename T> bool Intersects(const TPlane3<T> & p, const TTriangle3<T> & t)
    {
        return Intersects<T>(t, p);
    }
    /// Triangle3/Plane3 intersection
    template<typename T> bool Intersects(const TTriangle3<T> & t, const TPlane3<T> & p,
    std::vector< TPoint3<T> > & vI)
    {
        TPoint3<T> I0, I1, I2;
        if(Intersects<T>(t.Segment(0), p, I0)) vI.push_back(I0);
        if(Intersects<T>(t.Segment(1), p, I1)) vI.push_back(I1);
        if(Intersects<T>(t.Segment(2), p, I2)) vI.push_back(I2);
    }
    /// Plane3/Triangle3 intersection
    template<typename T> bool Intersects(const TPlane3<T> & p, const TTriangle3<T> & t,
    std::vector< TPoint3<T> > & vI)
    {
        return Intersects<T>(t, p, vI);
    }

#endif // LgTriangle3_hpp

#endif // LgPlane3_hpp

#ifdef LgLine3_hpp
    /// returns point minimizing the sum of squared distances to a vector of lines
    template<typename T> TPoint3<T> BestIntersection( std::vector< TLine3<T> > const & vl)
    {
        TPoint3<T> I, sumW2A;
        TMat3<T> sumW2;
        for(size_t i=0; i<vl.size(); i++) {
            TMat3<T> W = CrossProductMatrix(vl[i].V().Normalized());
            TMat3<T> W2 = W*W;
            sumW2+=W2;
            sumW2A+=W2*vl[i].A();
        }
        T det = sumW2.Determinant();
        if(!det) return I;
        I = sumW2.Comat()*sumW2A;
        return I/det;
    }

#ifdef LgTriangle3_hpp

    /// (surfacic) Triangle3/Line3 intersection
    template<typename T> bool Intersects(const TTriangle3<T> & t, const TLine3<T> & l, TPoint3<T> & I)
    {
        TPoint3<T> n = t.WeightedNormal();
        T vn = l.Vector()*n;
        if(vn == T(0)) return false; // parallel
        T xI = ((t.A() - l.Point())*n)/vn;
        I = l.Point() + xI * l.Vector();
        TPoint3<T> nA = n^(t.B()-t.A());
        TPoint3<T> nB = n^(t.C()-t.B());
        TPoint3<T> nC = n^(t.A()-t.C());
        return (I - t.A())*nA>T(0) && (I - t.B())*nB>T(0) && (I - t.C())*nC>T(0);
    }
    /// Line3/(surfacic) Triangle3 intersection
    template<typename T> bool Intersects(const TLine3<T> & l, const TTriangle3<T> & t, TPoint3<T> & I)
    {
        return Intersects(t, l, I);
    }
    /// Line3/(surfacic) Triangle3 intersection
    template<typename T> bool Intersects(const TLine3<T> & l, const TTriangle3<T> & t)
    {
        TPoint3<T> I;
        return Intersects(t, l, I);
    }
    /// (surfacic) Triangle3/Line3 intersection
    template<typename T> bool Intersects(const TTriangle3<T> & t, const TLine3<T> & l)
    {
        TPoint3<T> I;
        return Intersects(t, l, I);
    }
#ifdef LgRay3_hpp
    /// (surfacic) Triangle3/Ray3 intersection
    template<typename T> bool Intersects(const TTriangle3<T> & t, const TRay3<T> & r, TPoint3<T> & I)
    {
        TPoint3<T> n = t.WeightedNormal();
        T vn = r.Vector()*n;
        if(vn == T(0)) return false; // parallel
        T xI = ((t.A() - r.Point())*n)/vn;
        if(xI < T(0)) return false; // wrong direction
        I = r.Point() + xI * r.Vector();
        TPoint3<T> nA = n^(t.B()-t.A());
        TPoint3<T> nB = n^(t.C()-t.B());
        TPoint3<T> nC = n^(t.A()-t.C());
        return (I - t.A())*nA>T(0) && (I - t.B())*nB>T(0) && (I - t.C())*nC>T(0);
    }
    /// Ray3/(surfacic) Triangle3 intersection
    template<typename T> bool Intersects(const TRay3<T> & r, const TTriangle3<T> & t, TPoint3<T> & I)
    {
        return Intersects(t, r, I);
    }
    /// (surfacic) Triangle3/Ray3 intersection
    template<typename T> bool Intersects(const TTriangle3<T> & t, const TRay3<T> & r)
    {
        TPoint3<T> I;
        return Intersects(t, r, I);
    }
    /// (surfacic) Triangle3/Ray3 intersection
    template<typename T> bool Intersects(const TRay3<T> & r, const TTriangle3<T> & t)
    {
        TPoint3<T> I;
        return Intersects(t, r, I);
    }
#endif // LgRay3_hpp
#endif // LgTriangle3_hpp
#endif // LgLine3_hpp

}; // namespace Lg

#endif // LgIntersection3_hpp
