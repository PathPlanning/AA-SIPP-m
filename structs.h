#ifndef STRUCTS_H
#define STRUCTS_H
#include "gl_const.h"
#include <utility>
#include <vector>
#include <string>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/composite_key.hpp>
using namespace boost::multi_index;

struct conflict
{
    int agent1;
    int agent2;
    int sec1;
    int sec2;
    double i;
    double j;
    double g;
};


struct Agent
{
    std::string id;
    double start_i;
    double start_j;
    int start_id;
    double start_heading;
    double goal_i;
    double goal_j;
    int goal_id;
    double goal_heading;
    double size;
    double rspeed;
    double mspeed;
    Agent(){ start_i = -1; start_j = -1; start_id = -1; goal_i = -1; goal_j = -1; goal_id = -1;
             size = CN_DEFAULT_SIZE; mspeed = CN_DEFAULT_MSPEED; rspeed = CN_DEFAULT_RSPEED;
             start_heading = CN_DEFAULT_SHEADING; goal_heading = CN_DEFAULT_GHEADING; }
};

struct constraint
{
    double i;
    double j;
    double g;
    bool goal;
};

struct movement
{
    double g;
    int p_dir;
    int s_dir;
};

struct SafeInterval
{
    double begin;
    double end;
    int id;
    SafeInterval(double begin_=0, double end_=CN_INFINITY, int id_=0):begin(begin_), end(end_), id(id_) {}
};

struct gNode
{
    int id;
    double i;
    double j;
    std::vector<int> neighbors;
    gNode(double i_ = -1, double j_ = -1):i(i_), j(j_) {}
    ~gNode() { neighbors.clear(); }
};

struct Node
{
    Node(double _i=-1, double _j=-1, double _g=-1, double _F=-1):i(_i),j(_j),g(_g),F(_F),Parent(nullptr){}
    ~Node(){ Parent = nullptr; }
    int     id, interval_id;
    double  i, j, size, g, F, heading;
    Node*   Parent;
    SafeInterval interval;
    bool operator< (const Node& other) const
    {
        if(fabs(this->F - other.F) < CN_EPSILON) //breaking-ties
            return this->g > other.g; //g-max
        else
            return this->F < other.F;
    }
};

struct obstacle
{
    std::string id;
    double size;
    double mspeed;
    std::vector<Node> sections;
    obstacle(){ id = -1; size = CN_DEFAULT_SIZE; mspeed = CN_DEFAULT_MSPEED; }
};

struct section
{
    section(int _id1=-1, int _id2=-1, double _g1=-1, double _g2=-1)
        :id1(_id1), id2(_id2), g1(_g1), g2(_g2){}
    section(const Node &a, const Node &b):id1(a.id), id2(b.id), g1(a.g), g2(b.g), i1(a.i), j1(a.j), i2(b.i), j2(b.j) {}
    int id1;
    int id2;
    void set_ij(double _i1=-1, double _j1=-1, double _i2=-1, double _j2=-1) {i1=_i1; i2=_i2; j1=_j1; j2=_j2;}
    double i1, i2, j1, j2;
    double size;
    double g1;
    double g2;//is needed for goal and wait actions
    double mspeed;
    bool operator == (const section &comp) const {return (id1 == comp.id1 && id2 == comp.id2 && fabs(g1 - comp.g1) < CN_EPSILON);}

};

class Vector2D {
  public:
    Vector2D(double _i = 0.0, double _j = 0.0):i(_i),j(_j){}
    double i, j;

    inline Vector2D operator +(const Vector2D &vec) { return Vector2D(i + vec.i, j + vec.j); }
    inline Vector2D operator -(const Vector2D &vec) { return Vector2D(i - vec.i, j - vec.j); }
    inline Vector2D operator -() { return Vector2D(-i,-j); }
    inline Vector2D operator /(const double &num) { return Vector2D(i/num, j/num); }
    inline Vector2D operator *(const double &num) { return Vector2D(i*num, j*num); }
    inline double operator *(const Vector2D &vec){ return i*vec.i + j*vec.j; }
    inline void operator +=(const Vector2D &vec) { i += vec.i; j += vec.j; }
    inline void operator -=(const Vector2D &vec) { i -= vec.i; j -= vec.j; }
};

class Point {
public:
    double i;
    double j;

    Point(double _i = 0.0, double _j = 0.0):i (_i), j (_j){}
    Point operator-(Point &p){return Point(i - p.i, j - p.j);}
    int operator== (Point &p){return (i == p.i) && (j == p.j);}
    int classify(Point &pO, Point &p1)
    {
        Point p2 = *this;
        Point a = p1 - pO;
        Point b = p2 - pO;
        double sa = a.i * b.j - b.i * a.j;
        if (sa > 0.0)
            return 1;//LEFT;
        if (sa < 0.0)
            return 2;//RIGHT;
        if ((a.i * b.i < 0.0) || (a.j * b.j < 0.0))
            return 3;//BEHIND;
        if ((a.i*a.i + a.j*a.j) < (b.i*b.i + b.j*b.j))
            return 4;//BEYOND;
        if (pO == p2)
            return 5;//ORIGIN;
        if (p1 == p2)
            return 6;//DESTINATION;
        return 7;//BETWEEN;
    }
};


typedef multi_index_container<
        Node,
        indexed_by<
                    //ordered_non_unique<BOOST_MULTI_INDEX_MEMBER(Node, double, F)>,
                    ordered_non_unique<identity<Node>>,
                    hashed_non_unique<composite_key<Node, BOOST_MULTI_INDEX_MEMBER(Node, int, id),BOOST_MULTI_INDEX_MEMBER(Node, int, interval_id)>>
        >
> OPEN_container;
#endif
