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
        std::cout << "No '"<<CNS_TAG_ROOT<<"' element found in XML task-file."<<std::endl;
        return false;
    }
    double defaultSize(CN_DEFAULT_SIZE), defaultRSpeed(CN_DEFAULT_RSPEED), defaultMSpeed(CN_DEFAULT_MSPEED),
           defaultSHeading(CN_DEFAULT_SHEADING), defaultGHeading(CN_DEFAULT_GHEADING);
    XMLElement *element = root->FirstChildElement(CNS_TAG_DEF_PARAMS);
    if(element)
    {
        defaultSize = element->DoubleAttribute(CNS_TAG_ATTR_SIZE);
        defaultRSpeed = element->DoubleAttribute(CNS_TAG_ATTR_RSPEED);
        defaultMSpeed = element->DoubleAttribute(CNS_TAG_ATTR_MSPEED);
        defaultSHeading = element->DoubleAttribute(CNS_TAG_ATTR_SHEADING);
        defaultGHeading = element->DoubleAttribute(CNS_TAG_ATTR_GHEADING);
        if(element->Attribute(CNS_TAG_ATTR_GHEADING) == CNS_HEADING_WHATEVER)
            defaultGHeading = CN_HEADING_WHATEVER;
        if(defaultSize <= 0 || defaultSize > 10.0)
        {
            std::cout<<"Incorrect value of default "<<CNS_TAG_ATTR_SIZE" parameter. The value is set to "<< CN_DEFAULT_SIZE<<".\n";
            defaultSize = CN_DEFAULT_SIZE;
        }
        if(defaultRSpeed <= 0 || defaultRSpeed > 10.0)
        {
            std::cout<<"Incorrect value of default "<<CNS_TAG_ATTR_ROTATIONSPEED<<" parameter. The value is set to "<< CN_DEFAULT_RSPEED<<".\n";
            defaultRSpeed = CN_DEFAULT_RSPEED;
        }
        if(defaultMSpeed <= 0 || defaultMSpeed > 10.0)
        {
            std::cout<<"Incorrect value of default "<<CNS_TAG_ATTR_MOVESPEED<<" parameter. The value is set to "<< CN_DEFAULT_MSPEED<<".\n";
            defaultMSpeed = CN_DEFAULT_MSPEED;
        }
        if(defaultSHeading < 0 || defaultSHeading > 360)
        {
            std::cout<<"Incorrect value of default "<<CNS_TAG_ATTR_SHEADING<<" parameter. The value is set to "<< CN_DEFAULT_SHEADING<<".\n";
            defaultSHeading = CN_DEFAULT_SHEADING;
        }
        if(defaultGHeading < -1 || defaultGHeading > 360)
        {
            std::cout<<"Incorrect value of default "<<CNS_TAG_ATTR_GHEADING<<" parameter. The value is set to "<< CN_DEFAULT_GHEADING<<".\n";
            defaultGHeading = CN_DEFAULT_GHEADING;
        }
    }
    root = root->FirstChildElement(CNS_TAG_AGENTS);
    if (!root)
    {
        std::cout << "No '"<<CNS_TAG_AGENTS<<"' element found in XML file."<<std::endl;
        return false;
    }
    element = root->FirstChildElement(CNS_TAG_AGENT);
    if(!element)
    {
        std::cout << "No '"<<CNS_TAG_AGENT<<"' element found in XML file."<<std::endl;
        return false;
    }
    int k(0);
    for(element; element; element = element->NextSiblingElement("agent"))
    {
        Agent agent;
        agent.start_i = element->IntAttribute(CNS_TAG_ATTR_SY);
        agent.start_j = element->IntAttribute(CNS_TAG_ATTR_SX);
        agent.goal_i = element->IntAttribute(CNS_TAG_ATTR_GY);
        agent.goal_j = element->IntAttribute(CNS_TAG_ATTR_GX);

        if(element->Attribute(CNS_TAG_ATTR_ID))
            agent.id = element->Attribute(CNS_TAG_ATTR_ID);
        else
            agent.id = std::to_string(k);
        if(element->Attribute(CNS_TAG_ATTR_SIZE))
            agent.size = element->DoubleAttribute(CNS_TAG_ATTR_SIZE);
        else
            agent.size = defaultSize;
        if(element->Attribute(CNS_TAG_ATTR_RSPEED))
            agent.rspeed = element->DoubleAttribute(CNS_TAG_ATTR_RSPEED);
        else
            agent.rspeed = defaultRSpeed;
        if(element->Attribute(CNS_TAG_ATTR_MSPEED))
            agent.mspeed = element->DoubleAttribute("CNS_TAG_ATTR_MSPEED");
        else
            agent.mspeed = defaultMSpeed;
        if(element->Attribute(CNS_TAG_ATTR_SHEADING))
            agent.start_heading = element->DoubleAttribute(CNS_TAG_ATTR_SHEADING);
        else
            agent.start_heading = defaultSHeading;
        if(element->Attribute(CNS_TAG_ATTR_GHEADING))
            agent.goal_heading = element->DoubleAttribute(CNS_TAG_ATTR_GHEADING);
        else
            agent.goal_heading = defaultGHeading;
        if(agent.size <= 0 || agent.size > 10.0)
        {
            std::cout<<"Incorrect value of "<<CNS_TAG_ATTR_SIZE<<" attribute of agent "<<agent.id<<". It's set to default value "<<defaultSize<<".\n";
            agent.size = defaultSize;
        }
        if(agent.rspeed <= 0 || agent.rspeed > 10.0)
        {
            std::cout<<"Incorrect value of "<<CNS_TAG_ATTR_RSPEED<<" of agent "<<agent.id<<". It's set to default value "<<defaultRSpeed<<".\n";
            agent.rspeed = defaultRSpeed;
        }
        if(agent.mspeed <= 0 || agent.mspeed > 10.0)
        {
            std::cout<<"Incorrect value of "<<CNS_TAG_ATTR_MSPEED<<" of agent "<<agent.id<<". It's set to default value "<<defaultMSpeed<<".\n";
            agent.mspeed = defaultMSpeed;
        }
        if(agent.start_heading < 0 || agent.start_heading > 360.0)
        {
            std::cout<<"Incorrect value of "<<CNS_TAG_ATTR_SH<<" of agent "<<agent.id<<". It's set to default value "<<defaultSHeading<<".\n";
            agent.start_heading = defaultSHeading;
        }
        if(agent.goal_heading < -1 || agent.goal_heading > 360.0)
        {
            std::cout<<"Incorrect value of "<<CNS_TAG_ATTR_GHEADING<<" of agent "<<agent.id<<". It's set to default value "<<defaultSHeading<<".\n";
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
