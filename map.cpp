#include "map.h"
using namespace tinyxml2;

Map::Map()
{
    height = 0;
    width = 0;
}
Map::~Map()
{	
    Grid.clear();
}

bool Map::get_map(const char* FileName)
{
    tinyxml2::XMLElement *root = nullptr;
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (root)
    {
        map_is_roadmap = false;
        return get_grid(FileName);
    }
    else
    {
        map_is_roadmap = true;
        return get_roadmap(FileName);
    }
}

bool Map::get_grid(const char* FileName)
{
    XMLDocument doc;
    if(doc.LoadFile(FileName) != XMLError::XML_SUCCESS)
    {
        std::cout << "Error openning input XML file."<<std::endl;
        return false;
    }

    XMLElement *root = nullptr;
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "No '"<<CNS_TAG_ROOT<<"' element found in XML map-file."<<std::endl;
        return false;
    }

    XMLElement *map = root->FirstChildElement(CNS_TAG_MAP);
    if (!map)
    {
        std::cout << "No '"<<CNS_TAG_MAP<<"' element found in XML file."<<std::endl;
        return false;
    }

    XMLElement *grid = map->FirstChildElement(CNS_TAG_GRID);
    if (!grid)
    {
        std::cout << "No '"<<CNS_TAG_GRID<<"' element found in XML file."<<std::endl;
        return false;
    }
    height = grid->IntAttribute(CNS_TAG_ATTR_HEIGHT);
    if(height <= 0)
    {
        std::cout<<"Wrong value of "<<CNS_TAG_ATTR_HEIGHT<<" attribute. It should be >0.\n";
        return false;
    }
    width = grid->IntAttribute(CNS_TAG_ATTR_WIDTH);
    if(width <= 0)
    {
        std::cout<<"Wrong value of "<<CNS_TAG_ATTR_WIDTH<<" attribute. It should be >0.\n";
        return false;
    }
    XMLElement *row = grid->FirstChildElement(CNS_TAG_ROW);
    Grid.resize(height);
    for(int i = 0; i < height; i++)
        Grid[i].resize(width, 0);

    std::string value;
    const char* rowtext;
    std::stringstream stream;
    for(int i = 0; i < height; i++)
    {
        if (!row)
        {
            std::cout << "Not enough '" << CNS_TAG_ROW << "' in '" << CNS_TAG_GRID << "' given." << std::endl;
            return false;
        }

        rowtext = row->GetText();
        unsigned int k = 0;
        value = "";
        int j = 0;

        for(k = 0; k < strlen(rowtext); k++)
        {
            if (rowtext[k] == ' ')
            {
                stream << value;
                stream >> Grid[i][j];
                stream.clear();
                stream.str("");
                value = "";
                j++;
            }
            else
            {
                value += rowtext[k];
            }
        }
        stream << value;
        stream >> Grid[i][j];
        stream.clear();
        stream.str("");

        if (j < width-1)
        {
            std::cout << "Not enough cells in '" << CNS_TAG_ROW << "' " << i << " given." << std::endl;
            return false;
        }
        row = row->NextSiblingElement(CNS_TAG_ROW);
    }
    return true;
}


bool Map::CellIsTraversable(int i, int j) const
{
    return (Grid[i][j] == 0);
}

bool Map::CellIsObstacle(int i, int j) const
{
    return (Grid[i][j] != 0);
}

bool Map::CellOnGrid(int i, int j) const
{
    return (i < height && i >= 0 && j < width && j >= 0);
}

int Map::getValue(int i, int j) const
{
    if(i < 0 || i >= height)
        return -1;
    if(j < 0 || j >= width)
        return -1;

    return Grid[i][j];
}

std::vector<Node> Map::getValidMoves(Node curNode, int k, double size) const
{
    std::vector<Node> v_moves = {};
    if(!map_is_roadmap)
    {
        LineOfSight los;
        los.setSize(size);
        std::vector<Node> moves;
        if(k == 2)
            moves = {Node(0,1,1.0),   Node(1,0,1.0),         Node(-1,0,1.0), Node(0,-1,1.0)};
        else if(k == 3)
            moves = {Node(0,1,1.0),   Node(1,1,sqrt(2.0)),   Node(1,0,1.0),  Node(1,-1,sqrt(2.0)),
                     Node(0,-1,1.0),  Node(-1,-1,sqrt(2.0)), Node(-1,0,1.0), Node(-1,1,sqrt(2.0))};
        else if(k == 4)
            moves = {Node(0,1,1.0),          Node(1,1,sqrt(2.0)),    Node(1,0,1.0),          Node(1,-1,sqrt(2.0)),
                     Node(0,-1,1.0),         Node(-1,-1,sqrt(2.0)),  Node(-1,0,1.0),         Node(-1,1,sqrt(2.0)),
                     Node(1,2,sqrt(5.0)),    Node(2,1,sqrt(5.0)),    Node(2,-1,sqrt(5.0)),   Node(1,-2,sqrt(5.0)),
                     Node(-1,-2,sqrt(5.0)),  Node(-2,-1,sqrt(5.0)),  Node(-2,1,sqrt(5.0)),   Node(-1,2,sqrt(5.0))};
        else
            moves = {Node(0,1,1.0),          Node(1,1,sqrt(2.0)),    Node(1,0,1.0),          Node(1,-1,sqrt(2.0)),
                     Node(0,-1,1.0),         Node(-1,-1,sqrt(2.0)),  Node(-1,0,1.0),         Node(-1,1,sqrt(2.0)),
                     Node(1,2,sqrt(5.0)),    Node(2,1,sqrt(5.0)),    Node(2,-1,sqrt(5.0)),   Node(1,-2,sqrt(5.0)),
                     Node(-1,-2,sqrt(5.0)),  Node(-2,-1,sqrt(5.0)),  Node(-2,1,sqrt(5.0)),   Node(-1,2,sqrt(5.0)),
                     Node(1,3,sqrt(10.0)),   Node(2,3,sqrt(13.0)),   Node(3,2,sqrt(13.0)),   Node(3,1,sqrt(10.0)),
                     Node(3,-1,sqrt(10.0)),  Node(3,-2,sqrt(13.0)),  Node(2,-3,sqrt(13.0)),  Node(1,-3,sqrt(10.0)),
                     Node(-1,-3,sqrt(10.0)), Node(-2,-3,sqrt(13.0)), Node(-3,-2,sqrt(13.0)), Node(-3,-1,sqrt(10.0)),
                     Node(-3,1,sqrt(10.0)),  Node(-3,2,sqrt(13.0)),  Node(-2,3,sqrt(13.0)),  Node(-1,3,sqrt(10.0))};
        std::vector<bool> valid(moves.size(), true);
        for(int k = 0; k < moves.size(); k++)
            if(!CellOnGrid(curNode.i + moves[k].i, curNode.j + moves[k].j) || CellIsObstacle(curNode.i + moves[k].i, curNode.j + moves[k].j)
                    || !los.checkLine(curNode.i, curNode.j, curNode.i + moves[k].i, curNode.j + moves[k].j, *this))
                valid[k] = false;
        for(int k = 0; k < valid.size(); k++)
            if(valid[k])
                v_moves.push_back(moves[k]);
    }
    else
        v_moves = valid_moves[curNode.id];
    return v_moves;
}

bool Map::get_roadmap(const char *FileName)
{
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }
    tinyxml2::XMLElement *root = 0, *element = 0, *data;
    std::string value;
    std::stringstream stream;
    root = doc.FirstChildElement("graphml")->FirstChildElement("graph");
    for(element = root->FirstChildElement("node"); element; element = element->NextSiblingElement("node"))
    {
        data = element->FirstChildElement();

        stream.str("");
        stream.clear();
        stream << data->GetText();
        stream >> value;
        auto it = value.find_first_of(",");
        stream.str("");
        stream.clear();
        stream << value.substr(0, it);
        double i;
        stream >> i;
        stream.str("");
        stream.clear();
        value.erase(0, ++it);
        stream << value;
        double j;
        stream >> j;
        gNode node;
        node.i = i;
        node.j = j;
        nodes.push_back(node);
    }
    for(element = root->FirstChildElement("edge"); element; element = element->NextSiblingElement("edge"))
    {
        std::string source = std::string(element->Attribute("source"));
        std::string target = std::string(element->Attribute("target"));
        source.erase(source.begin(),++source.begin());
        target.erase(target.begin(),++target.begin());
        int id1, id2;
        stream.str("");
        stream.clear();
        stream << source;
        stream >> id1;
        stream.str("");
        stream.clear();
        stream << target;
        stream >> id2;
        nodes[id1].neighbors.push_back(id2);
    }
    for(gNode cur:nodes)
    {
        Node node;
        std::vector<Node> neighbors;
        neighbors.clear();
        for(int i = 0; i < cur.neighbors.size(); i++)
        {
            node.i = nodes[cur.neighbors[i]].i;
            node.j = nodes[cur.neighbors[i]].j;
            node.id = cur.neighbors[i];
            neighbors.push_back(node);
        }
        valid_moves.push_back(neighbors);
    }
    size = nodes.size();
    return true;
}
