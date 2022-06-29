/** @file
Light 3D bounding box class for use in MATIS: Lg::Bbox3(d, f).
Author: Bruno Vallet 09/2010
*/

#ifndef LgBbox3_hpp
#define LgBbox3_hpp

#include "LgPoint3.hpp"
#include <limits.h>

/// namespace for all LiteGeom objects
namespace Lg
{

    /// Light 3D bbox class for use in MATIS: Lg::Bbox3(d, f)
    /** Defined by a normal (first, N) and a signed distance to origin (second, d) */
    template <class T> class TBbox3 : public std::pair< TPoint3<T>, TPoint3<T> >
    {
        typedef std::pair< TPoint3<T>, TPoint3<T> > mother;
    public:
        typedef T coord_type;

        inline void Order(T & v1, T & v2) {if(v1>v2) {T tmp = v1; v1=v2; v2=tmp;}}
        inline void Orient()
        {
            Order(mother::first.x(), mother::second.x());
            Order(mother::first.y(), mother::second.y());
            Order(mother::first.z(), mother::second.z());
        }
        
        /// default constructor (empty bbox)
        TBbox3()
        {
            T max = std::numeric_limits<T>::max();
            T min = (std::numeric_limits<T>::is_integer ? std::numeric_limits<T>::min() : -std::numeric_limits<T>::max());
            mother::first = TPoint3<T>(max,max,max);
            mother::second = TPoint3<T>(min,min,min);
        }
        /// constructor from 1 value
        TBbox3(const T & val):
        mother(TPoint3<T>(val,val,val), TPoint3<T>(val,val,val)) {}

        /// constructor from 3 values
        TBbox3(const T & x, const T & y, const T & z):
        mother(TPoint3<T>(x,y,z), TPoint3<T>(x,y,z)) {}

        /// constructor from 6 values
        TBbox3(const T & xmin, const T & ymin, const T & zmin,
            const T & xmax, const T & ymax, const T & zmax):
        mother(TPoint3<T>(xmin,ymin,zmin), TPoint3<T>(xmax,ymax,zmax)) {Orient();}

        /// constructor from 1 point
        TBbox3(const TPoint3<T> & P):mother(P, P) {}

        /// constructor from 2 points
        TBbox3(const TPoint3<T> & Pmin, const TPoint3<T> & Pmax):mother(Pmin, Pmax) {Orient();}

        /* ----- access ----- */
        inline TPoint3<T> & Min() {return mother::first;}
        inline TPoint3<T> Min() const {return mother::first;}
        inline TPoint3<T> & Max() {return mother::second;}
        inline TPoint3<T> Max() const {return mother::second;}

        inline T & Xmin() {return mother::first.X();}
        inline T Xmin() const {return mother::first.X();}
        inline T & Xmax() {return mother::second.X();}
        inline T Xmax() const {return mother::second.X();}

        inline T & Ymin() {return mother::first.Y();}
        inline T Ymin() const {return mother::first.Y();}
        inline T & Ymax() {return mother::second.Y();}
        inline T Ymax() const {return mother::second.Y();}

        inline T & Zmin() {return mother::first.Z();}
        inline T Zmin() const {return mother::first.Z();}
        inline T & Zmax() {return mother::second.Z();}
        inline T Zmax() const {return mother::second.Z();}

        /* ----- modifiers ----- */
        inline void AddPoint(const T & x, const T & y, const T & z)
        {
            if(x < Xmin()) Xmin() = x;
            else if(x > Xmax()) Xmax() = x;
            if(y < Ymin()) Ymin() = y;
            else if(y > Ymax()) Ymax() = y;
            if(z < Zmin()) Zmin() = z;
            else if(z > Zmax()) Zmax() = z;
        }
        inline void AddPoint(const TPoint3<T> & P) {AddPoint(LGCOORD3(P));}
        inline void AddBbox(const TBbox3<T> & bbx)
        {
            if(bbx.Xmin() < Xmin()) Xmin() = bbx.Xmin();
            if(bbx.Xmax() > Xmax()) Xmax() = bbx.Xmax();
            if(bbx.Ymin() < Ymin()) Ymin() = bbx.Ymin();
            if(bbx.Ymax() > Ymax()) Ymax() = bbx.Ymax();
            if(bbx.Zmin() < Zmin()) Zmin() = bbx.Zmin();
            if(bbx.Zmax() > Zmax()) Zmax() = bbx.Zmax();
        }
        inline bool Contains(const T & x, const T & y, const T & z) const
        {
            return x > Xmin() && x < Xmax() && 
                y > Ymin() && y < Ymax() && 
                z > Zmin() && z < Zmax();
        }
        inline bool Contains(const TPoint3<T> & P) const {return Contains(LGCOORD3(P));}
        inline TPoint3<T> Center() const {return 0.5*(Min() + Max());}
        inline TPoint3<T> Size() const {return (Max() - Min());}
        inline void Translate(const TPoint3<T> & P) {Min() += P; Max()+= P;}
        inline void AddMargin(const TPoint3<T> & P) {Min() -= P; Max()+= P;}
        inline void AddMargin(const T & Margin) {AddMargin(TPoint3<T>(Margin, Margin, Margin));}

    };

    /* ----- algebraic operators ----- */
    template <class T> bool operator == ( const TBbox3<T> & bbx1, const TBbox3<T> & bbx2 )
    {
        return ( bbx1.Min() == bbx2.Min() && bbx1.Max() == bbx2.Max() );
    }
    template <class T> bool operator != ( const TBbox3<T> & bbx1, const TBbox3<T> & bbx2 )
    {
        return ( bbx1.Min() != bbx2.Min() || bbx1.Max() != bbx2.Max() );
    }
    template <class T> TBbox3<T> operator + ( const TBbox3<T> & bbx1, const TBbox3<T> & bbx2 )
    {
        TBbox3<T> & ret = bbx1;
        ret.AddBbox(bbx2);
        return ret;
    }
    template <class T> void operator += ( TBbox3<T> & bbx1, const TBbox3<T> & bbx2 )
    {
        bbx1.AddBbox(bbx2);
    }
    template <class T> TBbox3<T> operator + ( const TBbox3<T> & bbx, const TPoint3<T> & vect )
    {
        return TBbox3<T> (bbx.Min() + vect, bbx.Max() + vect);
    }
    template <class T> void operator += ( TBbox3<T> & bbx, const TPoint3<T> & vect )
    {
        bbx.Translate(vect);
    }
    template <class T> TBbox3<T> operator - ( const TBbox3<T> & bbx, const TPoint3<T> & vect )
    {
        return TBbox3<T> (bbx.Min() - vect, bbx.Max() - vect);
    }
    template <class T> void operator -= ( TBbox3<T> & bbx, const TPoint3<T> & vect )
    {
        bbx.Translate(-vect);
    }

    /* ----- stream operators ----- */
    template <typename T> std::ostream& operator << ( std::ostream &os, const TBbox3<T> & bbx )
    {
        return ( os << "[" << bbx.Min() << "," << bbx.Max() << "]" ) ;
    }
    template <typename T> std::istream& operator >> ( std::istream &is, TBbox3<T> & bbx )
    {
        char dummy;
        return ( is >> dummy >> bbx.Min() >> dummy >> bbx.Max() >> dummy ) ;
    }

    template <typename T, typename U> T * operator << ( T * data, const TBbox3<U> & bbx )
    {
        return ( data << bbx.Min() << bbx.Max() );
    }

    template <typename T, typename U> T * operator >> ( T * data, TBbox3<U> & bbx )
    {
        return ( data >> bbx.Min() >> bbx.Max() );
    }

    /* ----- typedefs ----- */
    typedef TBbox3<double> Bbox3d;
    typedef TBbox3<float> Bbox3f;
    typedef Bbox3d Bbox3;

}; // namespace Lg

#endif //LgBbox3_hpp

