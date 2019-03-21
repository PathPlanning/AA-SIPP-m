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
        std::cout << "No 'root' element found in XML file."<<std::endl;
        return false;
    }
    root = root->FirstChildElement("obstacles");
    if(!root)
    {
        std::cout << "No 'obstacles' element found in XML file."<<std::endl;
        return false;
    }
    obstacle obs;
    for(XMLElement *element = root->FirstChildElement("obstacle"); element; element = element->NextSiblingElement("obstacle"))
    {
        obs.id = element->IntAttribute("id");
        obs.size = element->DoubleAttribute("size");
        obs.sections.clear();
        Node node;
        for(XMLElement *sec = element->FirstChildElement("section"); sec; sec = sec->NextSiblingElement("section"))
        {
            if(node.i < 0)
            {
                node.i = sec->IntAttribute("start.y");
                node.j = sec->IntAttribute("start.x");
                node.g = 0;
                obs.sections.push_back(node);
            }
            node.i = sec->IntAttribute("goal.y");
            node.j = sec->IntAttribute("goal.x");
            node.g += sec->DoubleAttribute("duration");
            obs.sections.push_back(node);
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

int DynamicObstacles::getID(int num) const
{
    if(num >= 0 && num < obstacles.size())
        return obstacles[num].id;
}
