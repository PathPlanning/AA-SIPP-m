#include "cConfig.h"

cConfig::cConfig()
{
    N = -1;
    searchParams = 0;
}

cConfig::~cConfig()
{
    if (searchParams)
    {
        delete[] searchParams;
    }
}

bool cConfig::getConfig(const char* FileName)
{
    std::string value;
    float weight;
    float loglevel;
    std::stringstream stream;

    TiXmlDocument doc(FileName);
    if(!doc.LoadFile())
    {
        std::cout << "Error openning input XML file."<<std::endl;
        return false;
    }

    TiXmlElement *root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "No 'root' element found in XML file."<<std::endl;
        return false;
    }

    TiXmlElement *algorithm = root->FirstChildElement(CNS_TAG_ALGORITHM);
    if (!algorithm)
    {
        std::cout << "No 'algorithm' element found in XML file."<<std::endl;
        return false;
    }

    TiXmlElement *element;

    N = CN_PT_NUM;
    searchParams = new float[N];

    element = algorithm->FirstChildElement(CNS_TAG_ALLOW_AA);
    if (!element)
    {
        std::cout << "Error! No '"<<CNS_TAG_ALLOW_AA<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to 'true'."<<std::endl;
        searchParams[CN_PT_AA] = 1;
    }
    else
    {
        value = element->GetText();
        if(value == "true" || value == "1")
            searchParams[CN_PT_AA] = 1;
        else if(value == "false" || value == "0")
            searchParams[CN_PT_AA] = 0;
        else
        {
            std::cout << "Warning! Wrong '"<<CNS_TAG_ALLOW_AA<<"' value. It's compared to 'true'."<<std::endl;
            searchParams[CN_PT_AA] = 1;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_BT);
    if (!element)
    {
        std::cout << "Error! No '"<<CNS_TAG_BT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to 'g-max'."<<std::endl;
        searchParams[CN_PT_BT] = CN_BT_G_MAX;
    }
    else
    {
        value = element->GetText();
        if(value == "g-max" || value == "gmax")
            searchParams[CN_PT_BT] = CN_BT_G_MAX;
        else if(value == "g-min" || value == "gmin")
            searchParams[CN_PT_BT] = CN_BT_G_MIN;
        else
        {
            std::cout << "Warning! Wrong '"<<CNS_TAG_BT<<"' value. It's compared to 'g-max'."<<std::endl;
            searchParams[CN_PT_BT] = CN_BT_G_MAX;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_METRICTYPE);
    if (!element)
    {
        if(searchParams[CN_PT_AA] == 1.0)
        {
            std::cout << "Error! No '"<<CNS_TAG_METRICTYPE<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<< CNS_MT_EUCLID <<"'."<<std::endl;
            searchParams[CN_PT_MT]=CN_MT_EUCLID;
        }
        else
        {
            std::cout << "Error! No '"<<CNS_TAG_METRICTYPE<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<< CNS_MT_MANHATTAN <<"'."<<std::endl;
            searchParams[CN_PT_MT]=CN_MT_MANHATTAN;
        }
    }
    else
    {
        value = element->GetText();
        if(value == CNS_MT_EUCLID)
            searchParams[CN_PT_MT] = CN_MT_EUCLID;
        else if(value == CNS_MT_DIAGONAL)
            searchParams[CN_PT_MT] = CN_MT_DIAGONAL;
        else if(value == CNS_MT_MANHATTAN)
            searchParams[CN_PT_MT] = CN_MT_MANHATTAN;
        else
        {
            if(searchParams[CN_PT_AA] == 1.0)
            {
                std::cout << "Warning! Wrong '"<<CNS_TAG_METRICTYPE<<"' value. It's compared to '"<< CNS_MT_EUCLID <<"'."<<std::endl;
                searchParams[CN_PT_MT] = CN_MT_EUCLID;
            }
            else
            {
                std::cout << "Warning! Wrong '"<<CNS_TAG_METRICTYPE<<"' value. It's compared to '"<< CNS_MT_MANHATTAN <<"'."<<std::endl;
                searchParams[CN_PT_MT] = CN_MT_MANHATTAN;
            }
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_CONSTRAINTSTYPE);
    if (!element)
    {
        std::cout << "Warning! No '"<<CNS_TAG_CONSTRAINTSTYPE<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to 'section'."<<std::endl;
    }
    else
    {
        value = element->GetText();
        if(value == CNS_CT_POINT)
            searchParams[CN_PT_CT] = CN_CT_POINT;
        else if(value == CNS_CT_VELOCITY)
            searchParams[CN_PT_CT] = CN_CT_VELOCITY;
        else if(value == CNS_CT_SECTION)
            searchParams[CN_PT_CT] = CN_CT_SECTION;
        else
        {
            std::cout << "Warning! Wrong '"<<CNS_TAG_CONSTRAINTSTYPE<<"' value. It's compared to 'section'."<<std::endl;
            searchParams[CN_PT_CT] = CN_CT_SECTION;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_WEIGHT);
    if (!element)
    {
        std::cout << "Error! No '"<<CNS_TAG_WEIGHT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to "<< 1 <<"."<<std::endl;
        weight = 1;
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
    searchParams[CN_PT_WEIGHT] = weight;

    TiXmlElement *options = root->FirstChildElement(CNS_TAG_OPTIONS);
    if(!options)
    {
        std::cout << "No '"<<CNS_TAG_OPTIONS<<"' element found in XML file."<<std::endl;
        return false;
    }

    element = options->FirstChildElement(CNS_TAG_LOGLVL);
    if(!element)
    {
        std::cout << "No '"<<CNS_TAG_LOGLVL<<"' element found in XML file."<<std::endl;
        return false;
    }

    value = element->GetText();
    stream<<value;
    stream>>loglevel;
    stream.clear();
    stream.str("");

    searchParams[CN_PT_LOGLVL] = loglevel;

    return true;
}
