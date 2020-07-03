#ifndef STRUCTS_H
#define STRUCTS_H
#include <utility>
#include <vector>
#include <string>
#include <math.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/composite_key.hpp>
#include "gl_const.h"
#include "primitive.h"
using boost::multi_index_container;
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
    int start_i;
    int start_j;
    double start_heading;
    int start_angle_id;
    int goal_i;
    int goal_j;
    double goal_heading;
    int goal_angle_id;
    double size;
    double rspeed;
    double mspeed;
    Agent(){ start_i = -1; start_j = -1; goal_i = -1; goal_j = -1;
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
struct TerminalPoint
{
    double i,j,t,angle;
    int primitive_id;
    TerminalPoint(double _i=-1, double _j=-1, double _t=-1, double _angle=-1, double _primitive_id=-1):i(_i),j(_j),t(_t),angle(_angle),primitive_id(_primitive_id){}
};


struct Node
{
    Node(int _i=-1, int _j=-1, double _g=-1, double _F=-1):i(_i),j(_j),g(_g),F(_F),Parent(nullptr){}
    ~Node(){ Parent = nullptr; }
    int     i, j, angle_id, interval_id, speed; //complex id
    double  g;
    double  F;
    double  heading;
    const   Node*   Parent;
    Primitive primitive;
    SafeInterval interval;
};

struct section
{
    section(int _i1=-1, int _j1=-1, int _i2=-1, int _j2=-1, double _g1=-1, double _g2=-1)
        :i1(_i1), j1(_j1), i2(_i2), j2(_j2), g1(_g1), g2(_g2){}
    section(const Node &a, const Node &b):i1(a.i), j1(a.j), i2(b.i), j2(b.j), g1(a.g), g2(b.g){}
    int i1;
    int j1;
    int i2;
    int j2;
    double size;
    double g1;
    double g2;//is needed for goals and wait actions
    double mspeed;
    bool operator == (const section &comp) const {return (i1 == comp.i1 && j1 == comp.j1 && g1 == comp.g1);}

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


struct cost{};
struct id{};

typedef multi_index_container<
        Primitive,
        indexed_by<
            hashed_unique<tag<id>, BOOST_MULTI_INDEX_MEMBER(Primitive, int, id)>
        >
> Constraints_container;

typedef multi_index_container<
    Node,
    indexed_by<
        hashed_unique<tag<id>, composite_key<Node, BOOST_MULTI_INDEX_MEMBER(Node, int, i), BOOST_MULTI_INDEX_MEMBER(Node, int, j), BOOST_MULTI_INDEX_MEMBER(Node, int, interval_id), BOOST_MULTI_INDEX_MEMBER(Node, int, angle_id), BOOST_MULTI_INDEX_MEMBER(Node, int, speed)>>,
        ordered_non_unique<tag<cost>, composite_key<Node, BOOST_MULTI_INDEX_MEMBER(Node, double, F), BOOST_MULTI_INDEX_MEMBER(Node, double, g)>>
    >
> OpenList;

class OpenContainer
{
    OpenList open;
public:
    void clear() {open.clear();}
    bool isEmpty() {return open.empty();}
    Node findMin()
    {
        Node min = *open.get<cost>().begin();
        open.get<cost>().erase(open.get<cost>().begin());
        return min;
    }

    void addOpen(Node &newNode)
    {
        auto key = open.get<id>().find(boost::make_tuple(newNode.i, newNode.j, newNode.interval_id, newNode.angle_id, newNode.speed));
        if(key == open.get<id>().end())
            open.insert(newNode);
        else if(key->g < newNode.g + CN_EPSILON)
            return;
        else
        {
            open.get<id>().erase(key);
            open.insert(newNode);
        }
    }

    Node findNode(Node n) const
    {
        auto it = open.get<id>().find(boost::make_tuple(n.i, n.j, n.interval_id, n.angle_id, n.speed));
        if(it != open.get<id>().end())
            return *it;
        else
            return Node(-1,-1, CN_INFINITY);
    }
};

typedef multi_index_container<
    Node,
    indexed_by<
        hashed_unique<tag<id>, composite_key<Node, BOOST_MULTI_INDEX_MEMBER(Node, int, i), BOOST_MULTI_INDEX_MEMBER(Node, int, j), BOOST_MULTI_INDEX_MEMBER(Node, int, interval_id), BOOST_MULTI_INDEX_MEMBER(Node, int, angle_id), BOOST_MULTI_INDEX_MEMBER(Node, int, speed)>>
    >
> ClosedList;

#endif
