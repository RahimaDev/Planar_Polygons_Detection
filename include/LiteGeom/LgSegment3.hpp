/** @file
 Light 3D segment class for use in MATIS: Lg::Segment3(d, f).
Author: Bruno Vallet 12/11/2009
*/

#ifndef LgSegment3_hpp
#define LgSegment3_hpp

#include "LgLine3.hpp"

/// namespace for all LiteGeom objects
namespace Lg {

    /// Light 3D segment class for use in MATIS: Lg::Segment3(d, f)
    /** Defined by their extremities Source (first, A) and Target (second, B) */
    template <class T> class TSegment3 : public std::pair< TPoint3<T>, TPoint3<T> >
    {
        typedef std::pair< TPoint3<T>, TPoint3<T> > mother;
    public:
        typedef T coord_type;

        /* ----- constructors ----- */
        TSegment3(const T & ax, const T & ay, const T & az,
                  const T & bx, const T & by, const T & bz):
        mother(TPoint3<T>(ax,ay,az), TPoint3<T>(bx,by,bz)){}
        TSegment3(TPoint3<T> A=TPoint3<T>(), TPoint3<T> B=TPoint3<T>()): mother(A, B){}
        TSegment3(T* data): mother(TPoint3<T>(data), TPoint3<T>(data+2), TPoint3<T>(data+4)){}
        TSegment3(T** data): mother(TPoint3<T>(data[0]), TPoint3<T>(data[1]), TPoint3<T>(data[2])){}

        /* ----- acces ----- */
        inline TPoint3<T> Source() const {return mother::first;}
        inline TPoint3<T> Target() const {return mother::second;}
        inline TPoint3<T> & Source() {return mother::first;}
        inline TPoint3<T> & Target() {return mother::second;}

        /// type conversion
        template <typename U> TSegment3<U> Convert() const
        {
            return TSegment3<U>(Source().Convert(), Target().Convert());
        }

        /* ----- modifiers ----- */
        inline void SetSource(TPoint3<T> src){mother::first=src;}
        inline void SetTarget(TPoint3<T> trgt){mother::second=trgt;}
        inline void SetSourceAndTarget(TPoint3<T> src, TPoint3<T> trgt){mother::first=src; mother::second=trgt;}

        /* ----- shortcuts ----- */
        inline TPoint3<T> & A() {return mother::first;}
        inline TPoint3<T> & B() {return mother::second;}
        inline TPoint3<T> A() const {return mother::first;}
        inline TPoint3<T> B() const {return mother::second;}
        inline void SetA(TPoint3<T> A){mother::first=A;}
        inline void SetB(TPoint3<T> B){mother::second=B;}
        inline void SetAB(TPoint3<T> A, TPoint3<T> B){mother::first=A; mother::second=B;}

        /* ----- convenience ----- */
        inline TPoint3<T> Vector() const {return B() - A();}
        inline TPoint3<T> V() const {return B() - A();}
        inline TLine3<T> Line() const {return TLine3<T>(A(), B() - A());}

        /* ----- geometry ----- */
        inline TPoint3<T> Middle() const {return 0.5*(A() + B());}
        inline T Length2() const {return (B() - A()).Norm2();}
        inline T Length() const {return (B() - A()).Norm();}

    };

    /* ----- algebraic operators ----- */
    template <class T> bool operator == ( const TSegment3<T> & s1, const TSegment3<T> & s2 )
    {
        return ( s1.A() == s2.A() && s1.B() == s2.B() );
    }
    template <class T> bool operator != ( const TSegment3<T> & s1, const TSegment3<T> & s2 )
    {
        return ( s1.A() != s2.A() || s1.B() != s2.B() );
    }
    template <class T> TSegment3<T> operator + ( const TSegment3<T> & s, const TPoint3<T> & vect )
    {
        return TSegment3<T> (s.A() + vect, s.B() + vect);
    }
    template <class T> void operator += ( const TSegment3<T> & s, const TPoint3<T> & vect )
                                        {
        s = s + vect;
    }
    template <class T> TSegment3<T> operator - ( const TSegment3<T> & s, const TPoint3<T> & vect )
    {
        return TSegment3<T> (s.A() - vect, s.B() - vect);
    }
    template <class T> void operator -= ( const TSegment3<T> & s, const TPoint3<T> & vect )
                                        {
        s = s - vect;
    }
    template <class T> TSegment3<T> operator * ( const T & t, const TSegment3<T> & s )
    {
        return TSegment3<T> (t * s.A(), t * s.B());
    }
    template <class T> TSegment3<T> operator / ( const TSegment3<T> & s, const T & t )
    {
        T t_inv = 1./t;
        return TSegment3<T> (t_inv * s.A(), t_inv * s.B());
    }
    template <class T> TSegment3<T> operator + ( const TSegment3<T> & s1, const TSegment3<T> & s2 )
    {
        return TSegment3<T> (s1.A() + s2.A(), s1.B() + s2.B());
    }
    template <class T> TSegment3<T> operator - ( const TSegment3<T> & s1, const TSegment3<T> & s2 )
    {
        return TSegment3<T> (s1.A() - s2.A(), s1.B() - s2.B());
    }

    /* ----- stream operators ----- */
    template <typename T> std::ostream& operator << ( std::ostream &os, const TSegment3<T> & s )
    {
        return ( os << "[" << s.A() << ";" << s.B() << "]" ) ;
    }
    template <typename T> std::istream& operator >> ( std::istream &is, TSegment3<T> & s )
    {
        char dummy;
        return ( is >> dummy >> s.A() >> dummy >> s.B() >> dummy ) ;
    }

    template <typename T, typename U> T * operator << ( T * data, const TSegment3<U> & s )
    {
        return ( data << s.A() << s.B() );
    }

    template <typename T, typename U> T * operator >> ( T * data, TSegment3<U> & s )
    {
        return ( data >> s.A() >> s.B() );
    }

    /* ----- typedefs ----- */
    typedef TSegment3<double> Segment3d;
    typedef TSegment3<float> Segment3f;
    typedef Segment3d Segment3;

}; // namespace Lg

#endif //LgSegment3_hpp
