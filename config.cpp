#include "config.h"
using namespace tinyxml2;

Config::Config()
{
    loglevel = CN_DEFAULT_LOGLVL;
    allowanyangle = CN_DEFAULT_ALLOWANYANGLE;
    hweight = CN_DEFAULT_HWEIGHT;
    metrictype = CN_DEFAULT_METRICTYPE;
    startsafeinterval = CN_DEFAULT_STARTSAFEINTERVAL;
    timelimit = CN_DEFAULT_TIMELIMIT;
    initialprioritization = CN_DEFAULT_INITIALPRIORITIZATION;
    rescheduling = CN_DEFAULT_RESCHEDULING;
    planforturns = CN_DEFAULT_PLANFORTURNS;
    additionalwait = CN_DEFAULT_ADDITIONALWAIT;
}

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
        std::cout << "Error! No '"<<CNS_TAG_ALLOW_AA<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set '"<<CNS_DEFAULT_ALLOWANYANGLE<<"'."<<std::endl;
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
            std::cout << "Warning! Wrong '"<<CNS_TAG_ALLOW_AA<<"' value. It's set to '"<<CNS_DEFAULT_ALLOWANYANGLE<<"'."<<std::endl;
            allowanyangle = CN_DEFAULT_ALLOWANYANGLE;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_METRICTYPE);
    if (!element)
    {
        if(allowanyangle == true)
        {
            std::cout << "Error! No '"<<CNS_TAG_METRICTYPE<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<< CNS_MT_EUCLID <<"'."<<std::endl;
            metrictype = CN_MT_EUCLID;
        }
        else
        {
            std::cout << "Error! No '"<<CNS_TAG_METRICTYPE<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<< CNS_MT_MANHATTAN <<"'."<<std::endl;
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
                std::cout << "Warning! Wrong '"<<CNS_TAG_METRICTYPE<<"' value. It's set to '"<< CNS_MT_EUCLID <<"'."<<std::endl;
                metrictype = CN_MT_EUCLID;
            }
            else
            {
                std::cout << "Warning! Wrong '"<<CNS_TAG_METRICTYPE<<"' value. It's set to '"<< CNS_MT_MANHATTAN <<"'."<<std::endl;
                metrictype = CN_MT_MANHATTAN;
            }
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_STARTSAFEINTERVAL);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_STARTSAFEINTERVAL<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CN_DEFAULT_STARTSAFEINTERVAL<<"'."<<std::endl;
        startsafeinterval = CN_DEFAULT_STARTSAFEINTERVAL;
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
        std::cout << "Warning! No '"<<CNS_TAG_PRIORITIZATION<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CNS_DEFAULT_INITIALPRIORITIZATION<<"'."<<std::endl;
        initialprioritization = CN_DEFAULT_INITIALPRIORITIZATION;
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
            std::cout << "Warning! Wrong '"<<CNS_TAG_PRIORITIZATION<<"' value. It's set to '"<<CNS_DEFAULT_INITIALPRIORITIZATION<<"'."<<std::endl;
            initialprioritization = CN_DEFAULT_INITIALPRIORITIZATION;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_TIMELIMIT);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_TIMELIMIT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to "<<CNS_DEFAULT_TIMELIMIT<<"."<<std::endl;
        timelimit = CN_DEFAULT_TIMELIMIT;
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
        std::cout << "Warning! No '"<<CNS_TAG_RESCHEDULING<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is to '"<<CNS_RE_NO<<"'."<<std::endl;
        rescheduling = CN_DEFAULT_RESCHEDULING;
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
            std::cout << "Warning! Wrong '"<<CNS_TAG_RESCHEDULING<<"' value. It's set to '"<<CNS_RE_NO<<"'."<<std::endl;
            rescheduling = CN_DEFAULT_RESCHEDULING;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_HWEIGHT);
    if (!element)
    {
        element = algorithm->FirstChildElement(CNS_TAG_WEIGHT);
        if (!element)
        {
            std::cout << "Warning! No '"<<CNS_TAG_HWEIGHT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is to "<< CN_DEFAULT_HWEIGHT <<"."<<std::endl;
            hweight = CN_DEFAULT_HWEIGHT;
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
        std::cout << "Warning! Wrong '"<<CNS_TAG_WEIGHT<<"' value. It's set to " << CN_DEFAULT_HWEIGHT <<"."<<std::endl;
        weight = CN_DEFAULT_HWEIGHT;
    }
    hweight = weight;

    element = algorithm->FirstChildElement(CNS_TAG_PLANFORTURNS);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_PLANFORTURNS<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CNS_DEFAULT_PLANFORTURNS<<"'."<<std::endl;
        planforturns = CN_DEFAULT_PLANFORTURNS;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>planforturns;
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement(CNS_TAG_ADDITIONALWAIT);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_ADDITIONALWAIT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. Its value is set to '"<<CN_DEFAULT_ADDITIONALWAIT<<"'."<<std::endl;
        additionalwait = CN_DEFAULT_ADDITIONALWAIT;
    }
    else
    {
        value = element->GetText();
        stream<<value;
        stream>>additionalwait;
        stream.clear();
        stream.str("");
        if(additionalwait < 0 || additionalwait > 100)
        {
            std::cout << "Warning! Wrong value of '"<<CNS_TAG_ADDITIONALWAIT<<"' element. It should belong to the interval [0,100]. Its value is set to '"<<CN_DEFAULT_ADDITIONALWAIT<<"'"<<std::endl;
            additionalwait = CN_DEFAULT_ADDITIONALWAIT;
        }
    }

    XMLElement *options = root->FirstChildElement(CNS_TAG_OPTIONS);
    if(!options)
    {
        std::cout << "Warning! No '"<<CNS_TAG_OPTIONS<<"' section found."<<std::endl;
        loglevel = CN_DEFAULT_LOGLVL;
    }
    else
    {
        element = options->FirstChildElement(CNS_TAG_LOGLVL);
        if (!element)
        {
            std::cout << "Warning! No '"<<CNS_TAG_LOGLVL<<"' element found inside '"<<CNS_TAG_OPTIONS<<"' section. Its value is set to '"<<CNS_DEFAULT_LOGLVL<<"'."<<std::endl;
            loglevel = CN_DEFAULT_LOGLVL;
        }
        else
        {
            value = element->GetText();
            stream<<value;
            stream>>loglevel;
            stream.clear();
            stream.str("");
        }
        if(loglevel != CN_LOGLVL_NO && loglevel != CN_LOGLVL_NORM && loglevel != CN_LOGLVL_ALL)
        {
            std::cout << "Warning! Wrong value of '"<<CNS_TAG_LOGLVL<<"' element found inside '"<<CNS_TAG_OPTIONS<<"' section. Its value is set to '"<<CNS_DEFAULT_LOGLVL<<"'."<<std::endl;
            loglevel = CN_DEFAULT_LOGLVL;
        }
        element = options->FirstChildElement(CNS_TAG_LOGPATH);
        if(element->GetText() != nullptr)
            logpath = element->GetText();
        element = options->FirstChildElement(CNS_TAG_LOGFILENAME);
        if(element->GetText() != nullptr)
            logfilename = element->GetText();
    }
    return true;
}
