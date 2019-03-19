#include "task.h"
using namespace tinyxml2;

bool Task::getTask(const char *fileName)
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
    double defaultSize(CN_DEFAULT_SIZE), defaultRSpeed(CN_DEFAULT_RSPEED), defaultMSpeed(CN_DEFAULT_MSPEED);
    XMLElement *element = root->FirstChildElement("default_parameters");
    if(element)
    {
        defaultSize = element->DoubleAttribute("size");
        defaultRSpeed = element->DoubleAttribute("rotationspeed");
        defaultMSpeed = element->DoubleAttribute("movespeed");
        if(defaultSize <= 0 || defaultSize > 10.0)
        {
            std::cout<<"Incorrect value of \"default size\" parameter. The value is set to "<< CN_DEFAULT_SIZE<<".\n";
            defaultSize = CN_DEFAULT_SIZE;
        }
        if(defaultRSpeed <= 0 || defaultRSpeed > 10.0)
        {
            std::cout<<"Incorrect value of \"default rotation speed\" parameter. The value is set to "<< CN_DEFAULT_RSPEED<<".\n";
            defaultRSpeed = CN_DEFAULT_RSPEED;
        }
        if(defaultMSpeed <= 0 || defaultMSpeed > 10.0)
        {
            std::cout<<"Incorrect value of \"default movement speed\" parameter. The value is set to "<< CN_DEFAULT_MSPEED<<".\n";
            defaultMSpeed = CN_DEFAULT_MSPEED;
        }
    }
    root = root->FirstChildElement("agents");
    if (!root)
    {
        std::cout << "No 'agents' element found in XML file."<<std::endl;
        return false;
    }
    element = root->FirstChildElement("agent");
    if(!element)
    {
        std::cout << "No 'agent' element found in XML file."<<std::endl;
        return false;
    }
    int k(0);
    for(element; element; element = element->NextSiblingElement("agent"))
    {
        agent current;
        current.start_i = element->IntAttribute("start.y");
        current.start_j = element->IntAttribute("start.x");
        current.goal_i = element->IntAttribute("goal.y");
        current.goal_j = element->IntAttribute("goal.x");
        if(element->Attribute("size"))
            current.size = element->DoubleAttribute("size");
        else
            current.size = defaultSize;
        if(element->Attribute("rotationspeed"))
            current.rspeed = element->DoubleAttribute("rotationspeed");
        else
            current.rspeed = defaultRSpeed;
        if(element->Attribute("movespeed"))
            current.mspeed = element->DoubleAttribute("movespeed");
        else
            current.mspeed = defaultMSpeed;
        current.id = k;
        if(current.size <= 0 || current.size > 10.0)
        {
            std::cout<<"Incorrect size of agent "<<current.id<<". Its size is set to default value "<<defaultSize<<".\n";
            current.size = defaultSize;
        }
        if(current.rspeed <= 0 || current.rspeed > 10.0)
        {
            std::cout<<"Incorrect rotation speed of agent "<<current.id<<". Its rotation speed is set to default value "<<defaultRSpeed<<".\n";
            current.rspeed = defaultRSpeed;
        }
        if(current.mspeed <= 0 || current.mspeed > 10.0)
        {
            std::cout<<"Incorrect movement speed of agent "<<current.id<<". Its movement speed is set to default value "<<defaultMSpeed<<".\n";
            current.mspeed = defaultMSpeed;
        }
        k++;
        agents.push_back(current);
    }
    return true;
}

bool Task::validateTask(const Map &map)
{
    LineOfSight los;
    for(agent a:agents)
    {
        std::cout<<a.start_i<<" "<<a.start_j<<" "<<a.size<<" ";
        los.setSize(a.size);
        if(!los.checkTraversability(a.start_i, a.start_j, map))
        {
            std::cout<<"Error! Start position of agent "<<a.id<<" is invalid.\n";
            return false;
        }
        if(!los.checkTraversability(a.goal_i, a.goal_j, map))
        {
            std::cout<<"Error! Goal position of agent "<<a.id<<" is invalid.\n";
            return false;
        }
    }
    for(int i = 0; i < agents.size(); i++)
        for(int j = i + 1; j < agents.size(); j++)
        {
            agent a1 = agents[i], a2 = agents[j];
            if(sqrt(pow(a1.start_i - a2.start_i, 2) + pow(a1.start_j - a2.start_j, 2)) < (a1.size + a2.size))
            {
                std::cout<<"Error! Start positions of agents "<< a1.id <<" and "<< a2.id <<" are placed too close.\n";
                return false;
            }
            if(sqrt(pow(a1.goal_i - a2.goal_i, 2) + pow(a1.goal_j - a2.goal_j, 2)) < (a1.size + a2.size))
            {
                std::cout<<"Error! Goal positions of agents "<< a1.id <<" and "<< a2.id <<" are placed too close.\n";
                return false;
            }
        }
    return true;
}

agent Task::getAgent(unsigned int id) const
{
    if(id < agents.size())
        return agents[id];
    else
        return agent();
}

unsigned int Task::getNumberOfAgents() const
{
    return agents.size();
}
