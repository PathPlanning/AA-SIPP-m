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
    double defaultSize(CN_DEFAULT_SIZE), defaultRSpeed(CN_DEFAULT_RSPEED), defaultMSpeed(CN_DEFAULT_MSPEED),
           defaultSHeading(CN_DEFAULT_SHEADING), defaultGHeading(CN_DEFAULT_GHEADING);
    XMLElement *element = root->FirstChildElement("default_parameters");
    if(element)
    {
        defaultSize = element->DoubleAttribute("size");
        defaultRSpeed = element->DoubleAttribute("rotationspeed");
        defaultMSpeed = element->DoubleAttribute("movespeed");
        defaultSHeading = element->DoubleAttribute("start.heading");
        defaultGHeading = element->DoubleAttribute("goal.heading");
        if(element->Attribute("goal.heading") == "whatever")
            defaultGHeading = -1;
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
        if(defaultSHeading < 0 || defaultSHeading > 360)
        {
            std::cout<<"Incorrect value of \"default start heading\" parameter. The value is set to "<< CN_DEFAULT_SHEADING<<".\n";
            defaultSHeading = CN_DEFAULT_SHEADING;
        }
        if(defaultGHeading < -1 || defaultGHeading > 360)
        {
            std::cout<<"Incorrect value of \"default goal heading\" parameter. The value is set to "<< CN_DEFAULT_GHEADING<<".\n";
            defaultGHeading = CN_DEFAULT_GHEADING;
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
        Agent agent;
        agent.start_i = element->IntAttribute("start.y");
        agent.start_j = element->IntAttribute("start.x");
        agent.goal_i = element->IntAttribute("goal.y");
        agent.goal_j = element->IntAttribute("goal.x");

        if(element->Attribute("id"))
            agent.id = element->Attribute("id");
        else
            agent.id = std::to_string(k);
        if(element->Attribute("size"))
            agent.size = element->DoubleAttribute("size");
        else
            agent.size = defaultSize;
        if(element->Attribute("rotationspeed"))
            agent.rspeed = element->DoubleAttribute("rotationspeed");
        else
            agent.rspeed = defaultRSpeed;
        if(element->Attribute("movespeed"))
            agent.mspeed = element->DoubleAttribute("movespeed");
        else
            agent.mspeed = defaultMSpeed;
        if(element->Attribute("start.heading"))
            agent.start_heading = element->DoubleAttribute("start.heading");
        else
            agent.start_heading = defaultSHeading;
        if(element->Attribute("goal.heading"))
            agent.goal_heading = element->DoubleAttribute("goal.heading");
        else
            agent.goal_heading = defaultGHeading;
        if(agent.size <= 0 || agent.size > 10.0)
        {
            std::cout<<"Incorrect size of agent "<<agent.id<<". Its size is set to default value "<<defaultSize<<".\n";
            agent.size = defaultSize;
        }
        if(agent.rspeed <= 0 || agent.rspeed > 10.0)
        {
            std::cout<<"Incorrect rotation speed of agent "<<agent.id<<". Its rotation speed is set to default value "<<defaultRSpeed<<".\n";
            agent.rspeed = defaultRSpeed;
        }
        if(agent.mspeed <= 0 || agent.mspeed > 10.0)
        {
            std::cout<<"Incorrect movement speed of agent "<<agent.id<<". Its movement speed is set to default value "<<defaultMSpeed<<".\n";
            agent.mspeed = defaultMSpeed;
        }
        if(agent.start_heading < 0 || agent.start_heading > 360.0)
        {
            std::cout<<"Incorrect start heading of agent "<<agent.id<<". Its start heading is set to default value "<<defaultSHeading<<".\n";
            agent.start_heading = defaultSHeading;
        }
        if(agent.goal_heading < -1 || agent.goal_heading > 360.0)
        {
            std::cout<<"Incorrect goal heading of agent "<<agent.id<<". Its goal heading is set to default value "<<defaultSHeading<<".\n";
            agent.goal_heading = defaultGHeading;
        }
        k++;
        agents.push_back(agent);
    }
    return true;
}

bool Task::validateTask(const Map &map)
{
    LineOfSight los;
    for(Agent a:agents)
    {
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
            Agent a1 = agents[i], a2 = agents[j];
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

Agent Task::getAgent(unsigned int id) const
{
    if(id < agents.size())
        return agents[id];
    else
        return Agent();
}

unsigned int Task::getNumberOfAgents() const
{
    return agents.size();
}
