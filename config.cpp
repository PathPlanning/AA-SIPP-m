#include "config.h"
using namespace tinyxml2;

bool Config::getConfig(const char* fileName)
{
    std::string value;
    double weight;
    std::stringstream stream;

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

    XMLElement *algorithm = root->FirstChildElement(CNS_TAG_ALGORITHM);
    if (!algorithm)
    {
        std::cout << "No 'algorithm' element found in XML file."<<std::endl;
        return false;
    }

    XMLElement *element;

    element = algorithm->FirstChildElement(CNS_TAG_ALLOW_AA);
    if (!element)
    {
        std::cout << "Error! No '"<<CNS_TAG_ALLOW_AA<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to 'true'."<<std::endl;
        allowanyangle = true;
    }
    else
    {
        value = element->GetText();
        if(value == "true" || value == "1")
            allowanyangle = true;
        else if(value == "false" || value == "0")
            allowanyangle = false;
        else
        {
            std::cout << "Warning! Wrong '"<<CNS_TAG_ALLOW_AA<<"' value. It's compared to 'true'."<<std::endl;
            allowanyangle = true;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_METRICTYPE);
    if (!element)
    {
        if(allowanyangle == true)
        {
            std::cout << "Error! No '"<<CNS_TAG_METRICTYPE<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<< CNS_MT_EUCLID <<"'."<<std::endl;
            metrictype = CN_MT_EUCLID;
        }
        else
        {
            std::cout << "Error! No '"<<CNS_TAG_METRICTYPE<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<< CNS_MT_MANHATTAN <<"'."<<std::endl;
            metrictype = CN_MT_MANHATTAN;
        }
    }
    else
    {
        value = element->GetText();
        if(value == CNS_MT_EUCLID)
            metrictype = CN_MT_EUCLID;
        else if(value == CNS_MT_DIAGONAL)
            metrictype = CN_MT_DIAGONAL;
        else if(value == CNS_MT_MANHATTAN)
            metrictype = CN_MT_MANHATTAN;
        else
        {
            if(allowanyangle == true)
            {
                std::cout << "Warning! Wrong '"<<CNS_TAG_METRICTYPE<<"' value. It's compared to '"<< CNS_MT_EUCLID <<"'."<<std::endl;
                metrictype = CN_MT_EUCLID;
            }
            else
            {
                std::cout << "Warning! Wrong '"<<CNS_TAG_METRICTYPE<<"' value. It's compared to '"<< CNS_MT_MANHATTAN <<"'."<<std::endl;
                metrictype = CN_MT_MANHATTAN;
            }
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_STARTSAFEINTERVAL);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_STARTSAFEINTERVAL<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to 0."<<std::endl;
        startsafeinterval = 0;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>startsafeinterval;
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement(CNS_TAG_PRIORITIZATION);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_PRIORITIZATION<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to 'fifo'."<<std::endl;
        initialprioritization = CN_IP_FIFO;
    }
    else
    {
        value = element->GetText();
        if(value == CNS_IP_FIFO)
            initialprioritization = CN_IP_FIFO;
        else if(value == CNS_IP_LONGESTF)
            initialprioritization = CN_IP_LONGESTF;
        else if(value == CNS_IP_SHORTESTF)
            initialprioritization = CN_IP_SHORTESTF;
        else if(value == CNS_IP_RANDOM)
            initialprioritization = CN_IP_RANDOM;
        else
        {
            std::cout << "Warning! Wrong '"<<CNS_TAG_PRIORITIZATION<<"' value. It's compared to 'fifo'."<<std::endl;
            initialprioritization = CN_IP_FIFO;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_TIMELIMIT);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_TIMELIMIT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to infinity(no limit)."<<std::endl;
        timelimit = CN_INFINITY;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>timelimit;
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement(CNS_TAG_RESCHEDULING);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_RESCHEDULING<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to 'no'."<<std::endl;
        rescheduling = CN_RE_NO;
    }
    else
    {
        value = element->GetText();
        if(value == CNS_RE_NO)
            rescheduling = CN_RE_NO;
        else if(value == CNS_RE_RULED)
            rescheduling = CN_RE_RULED;
        else if(value == CNS_RE_RANDOM)
            rescheduling = CN_RE_RANDOM;
        else
        {
            std::cout << "Warning! Wrong '"<<CNS_TAG_RESCHEDULING<<"' value. It's compared to 'no'."<<std::endl;
            rescheduling = CN_RE_NO;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_HWEIGHT);
    if (!element)
    {
        element = algorithm->FirstChildElement(CNS_TAG_WEIGHT);
        if (!element)
        {
            std::cout << "Warning! No '"<<CNS_TAG_HWEIGHT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to "<< 1 <<"."<<std::endl;
            hweight = 1;
        }
        else
        {
            value = element->GetText();
            stream<<value;
            stream>>weight;
            stream.clear();
            stream.str("");
        }
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>weight;
        stream.clear();
        stream.str("");
    }

    if (weight == 0)
    {
        std::cout << "Warning! Wrong '"<<CNS_TAG_WEIGHT<<"' value. It's compared to " << 1 <<"."<<std::endl;
        weight = 1;
    }
    hweight = weight;

    element = algorithm->FirstChildElement(CNS_TAG_TURNINGWEIGHT);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_TURNINGWEIGHT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to 0."<<std::endl;
        tweight = 0;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>tweight;
        stream.clear();
        stream.str("");
    }

///TODO: ADD OPTIONS


    return true;
}
