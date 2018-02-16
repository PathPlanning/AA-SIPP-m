/**
* This file is part of HOG2.
*
* HOG2 is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* HOG2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with HOG2; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef VECTOR2D_H
#define VECTOR2D_H

#include <math.h>
#include <ostream>
static const double TOLERANCE = 0.000001;    // doubleing point tolerance

inline bool fless(double a, double b) { return (a < b - TOLERANCE); }
inline bool fgreater(double a, double b) { return (a > b + TOLERANCE); }
inline bool fequal(double a, double b)
{ return (a >= b - TOLERANCE) && (a <= b+TOLERANCE); }
inline bool fleq(double a, double b) { return fless(a,b)||fequal(a,b); }
inline bool fgeq(double a, double b) { return fgreater(a,b)||fequal(a,b); }

inline double min(double a, double b) { return fless(a, b)?a:b; }
inline double max(double a, double b) { return fless(a, b)?b:a; }

class Vector2D {
  public:
    Vector2D(double _x,double _y):x(_x),y(_y),updateTime(0),accessTime(0) {/*Normalize();*/}
    Vector2D():x(0),y(0),updateTime(0),accessTime(0) {}
    void Set(double _x, double _y) { x=_x; y=_y; /*Normalize();*/}

    void SetUpdateTime(double t) {updateTime=t;}
    double GetUpdateTime() {return updateTime;}

    void SetAccessTime(double t) {accessTime=t;}
    double GetAccessTime() {return accessTime;}

    bool operator==(const Vector2D &rhs)const{return (fequal(x,rhs.x)&&fequal(y,rhs.y));}
    bool operator<(const Vector2D &rhs)const{return fequal(x,rhs.x)?fless(y,rhs.y):fless(x,rhs.x);}

    // Dot product
    inline double operator *(Vector2D const& other){return x * other.x + y * other.y;}
    // Negation
    inline Vector2D operator -(Vector2D const& other)const{return Vector2D(x-other.x,y-other.y);}
    inline void operator -=(Vector2D const& other){x-=other.x;y-=other.y;}
    inline Vector2D operator -()const{return Vector2D(-x,-y);}
    // Slope angle of this vector
    inline double atan(){ return atan2(y,x); }
    // Square
    inline double sq(){ return (*this) * (*this); }
    inline double len(){ return sqrt(sq()); }

    friend Vector2D operator /(const Vector2D& vec, const double num)
    {
      return Vector2D(vec.x / num, vec.y / num);
    }

    friend Vector2D operator *(const Vector2D& vec, const double num)
    {
      return Vector2D(vec.x * num, vec.y * num);
    }

    friend Vector2D operator *(const double num, const Vector2D& vec)
    {
      return Vector2D(vec.x * num, vec.y * num);
    }

    friend Vector2D operator +(const Vector2D& v1, const Vector2D& v2)
    {
      return Vector2D(v1.x + v2.x, v1.y + v2.y);
    }

    inline void operator +=(const Vector2D& v2) { x +=v2.x; y +=v2.y; }

    //                                                //private:
    double x, y;
    double updateTime, accessTime;

    void Normalize()
    {
      if ((x==0)&&(y==0))
        return;
      double magnitude(len());
      x /= magnitude;
      y /= magnitude;
    }
};

// Calculate determinant of vector
inline double det(Vector2D const& v1, Vector2D const& v2){
  return v1.x*v2.y - v1.y*v2.x;
}

// Compute normal to line segment with specified endpoints
inline Vector2D normal(Vector2D const& v1, Vector2D const& v2){
  Vector2D tmp(v2.y-v1.y,v1.x-v2.x);
  tmp.Normalize();
  return tmp;
}

std::ostream& operator <<(std::ostream & out, Vector2D const& v);

#endif

