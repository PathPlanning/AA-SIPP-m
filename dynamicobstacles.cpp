#include "dynamicobstacles.h"
using namespace tinyxml2;

DynamicObstacles::DynamicObstacles()
{
    obstacles.clear();
}

bool DynamicObstacles::getObstacles(const char *fileName)
{
    XMLDocument doc;
    if(doc.LoadFile(fileName) != XMLError::XML_SUCCESS)
    {
        std::cout << "Error openning input XML file."<<std::endl;
        return false;
    }
    XMLElement *root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "No '"<<CNS_TAG_ROOT<<"' element found in XML file."<<std::endl;
        return false;
    }
    root = root->FirstChildElement(CNS_TAG_DYNAMICOBSTACLES);
    if(!root)
    {
        std::cout << "No '"<<CNS_TAG_DYNAMICOBSTACLES<<"' element found in XML file."<<std::endl;
        return false;
    }
    double default_size(CN_DEFAULT_SIZE);
    if(root->FirstChildElement(CNS_TAG_DEF_PARAMS))
        if(root->FirstChildElement(CNS_TAG_DEF_PARAMS)->DoubleAttribute(CNS_TAG_ATTR_SIZE))
            default_size = root->FirstChildElement(CNS_TAG_DEF_PARAMS)->DoubleAttribute(CNS_TAG_ATTR_SIZE);
    if(!(default_size > 0 && default_size < 10.0 + CN_EPSILON))
    {
        std::cout<<"Warning! Default value of '"<<CNS_TAG_ATTR_SIZE<<"' atrribute is not correctly specified. It's set to '"<<CN_DEFAULT_SIZE<<"'.\n";
        default_size = CN_DEFAULT_SIZE;
    }
    obstacle obs;
    int prim_id(1);
    for(XMLElement *element = root->FirstChildElement(CNS_TAG_OBSTACLE); element; element = element->NextSiblingElement(CNS_TAG_OBSTACLE))
    {
        //obs.id = element->Attribute(CNS_TAG_ATTR_ID);
        if(element->DoubleAttribute(CNS_TAG_ATTR_SIZE))
            obs.size = element->DoubleAttribute(CNS_TAG_ATTR_SIZE);
        else
            obs.size = default_size;
        if(!(obs.size > 0 && obs.size < 10.0 + CN_EPSILON))
        {
            std::cout<<"Warning! '"<<CNS_TAG_ATTR_SIZE<<"' atrribute of obstacle '"<<obs.id<<"' is not correctly specified. Its value is set to the default '"<<default_size<<"'.\n";
            obs.size = default_size;
        }
        obs.sections.clear();
        obs.primitives.clear();
        Node node;
        Primitive prim;
        double curtime(0);
        for(XMLElement *sec = element->FirstChildElement("action"); sec; sec = sec->NextSiblingElement("action"))
        {
            prim.source.i = sec->IntAttribute("y0");
            prim.source.j = sec->IntAttribute("x0");
            prim.target.i = sec->IntAttribute("yf");
            prim.target.j = sec->IntAttribute("xf");

            prim.id = prim_id;
            prim_id++;
            prim.type = 1;
            prim.agentSize = obs.size;
            prim.begin = curtime;
            prim.duration = sec->DoubleAttribute("duration");
            curtime += prim.duration;

            prim.i_coefficients.resize(4);
            prim.j_coefficients.resize(4);
            prim.i_coefficients[0] = prim.source.i;
            prim.j_coefficients[0] = prim.source.j;
            prim.i_coefficients[1] = (prim.target.i - prim.source.i)/prim.duration;
            prim.j_coefficients[1] = (prim.target.j - prim.source.j)/prim.duration;
            prim.i_coefficients[2] = prim.j_coefficients[2] = prim.i_coefficients[3] = prim.j_coefficients[3] = 0;

            prim.countCells();
            prim.countIntervals(prim.agentSize);
            obs.primitives.push_back(prim);
        }
        Primitive last;
        last.type = -1;
        last.source = obs.primitives.back().target;
        last.target = obs.primitives.back().target;
        last.begin = obs.primitives.back().begin + obs.primitives.back().duration;
        last.cells = {Cell(last.source.i, last.source.j)};
        last.cells.back().interval = {last.begin, CN_INFINITY};
        last.duration = CN_INFINITY;
        last.setSize(obs.size);
        last.id = prim_id;
        prim_id++;
        obstacles.push_back(obs);
        //if(obstacles.size() == 200)
        //    break;
    }
    return true;
}

std::vector<Node> DynamicObstacles::getSections(int num) const
{
    if(num >= 0 && num < obstacles.size())
        return obstacles[num].sections;
    else
        return {};
}

std::vector<Primitive> DynamicObstacles::getPrimitives(int num) const
{
    if(num >= 0 && num < obstacles.size())
        return obstacles[num].primitives;
    else
        return {};
}

Primitive DynamicObstacles::getPrimitive(int id)
{
    for(int i = 0; i < obstacles.size(); i++)
        if(obstacles[i].primitives.begin()->id <= id && obstacles[i].primitives.begin()->id + obstacles[i].primitives.size() > id)
            return obstacles[i].primitives.at(id - obstacles[i].primitives.begin()->id);
    return Primitive();
}

double DynamicObstacles::getSize(int num) const
{
    if(num >= 0 && num < obstacles.size())
        return obstacles[num].size;
    else
        return -1;
}

double DynamicObstacles::getMSpeed(int num) const
{
    if(num >= 0 && num < obstacles.size())
        return obstacles[num].mspeed;
    else
        return -1;
}

std::string DynamicObstacles::getID(int num) const
{
    if(num >= 0 && num < obstacles.size())
        return obstacles[num].id;
    else
        return "";
}
