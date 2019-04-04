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
    for(XMLElement *element = root->FirstChildElement(CNS_TAG_OBSTACLE); element; element = element->NextSiblingElement(CNS_TAG_OBSTACLE))
    {
        obs.id = element->Attribute(CNS_TAG_ATTR_ID);
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
        Node node;
        for(XMLElement *sec = element->FirstChildElement(CNS_TAG_SECTION); sec; sec = sec->NextSiblingElement(CNS_TAG_SECTION))
        {
            if(node.i < 0)
            {
                node.i = sec->IntAttribute(CNS_TAG_ATTR_SY);
                node.j = sec->IntAttribute(CNS_TAG_ATTR_SX);
                node.g = 0;
                obs.sections.push_back(node);
            }
            node.i = sec->IntAttribute(CNS_TAG_ATTR_GY);
            node.j = sec->IntAttribute(CNS_TAG_ATTR_GX);
            node.g += sec->DoubleAttribute(CNS_TAG_ATTR_DURATION);
            obs.sections.push_back(node);
        }
        for(size_t i = 1; i < obs.sections.size(); i++)
            if(obs.sections[i-1].i != obs.sections[i].i || obs.sections[i-1].j != obs.sections[i].j)
            {
                double dist = sqrt(pow(obs.sections[i-1].i - obs.sections[i].i, 2) + pow(obs.sections[i-1].j - obs.sections[i].j, 2));
                obs.mspeed = dist/(obs.sections[i].g - obs.sections[i-1].g);
                break;
            }
        obstacles.push_back(obs);
    }
    return true;
}

std::vector<Node> DynamicObstacles::getSections(int num) const
{
    if(num >= 0 && num < obstacles.size())
        return obstacles[num].sections;
}

double DynamicObstacles::getSize(int num) const
{
    if(num >= 0 && num < obstacles.size())
        return obstacles[num].size;
}

double DynamicObstacles::getMSpeed(int num) const
{
    if(num >= 0 && num < obstacles.size())
        return obstacles[num].mspeed;
}

std::string DynamicObstacles::getID(int num) const
{
    if(num >= 0 && num < obstacles.size())
        return obstacles[num].id;
}
