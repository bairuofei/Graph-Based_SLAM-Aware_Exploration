#include "graph.h"
#include <iostream>

Graph::Vertex::Vertex(int id)
{
    _id = id;
    _position = Vector2f(0.0, 0.0);
}


Graph::Vertex::Vertex(int id, Vector2f position)
{
    _id = id;
    _position = position;
}

Graph::Vertex::~Vertex()
{

}

Graph::EdgeSet Graph::Vertex::enteringEdges()
{
    Graph::EdgeSet eset;
    for (Graph::Edge* e : edges()){
        
		if ((e->to() == this) || (e->undirected()))
			eset.insert(e);
	}

	return eset;
}

Graph::EdgeSet Graph::Vertex::exitingEdges()
{
    Graph::EdgeSet eset;
    for (Graph::Edge* e : edges()){
        
        if (e->from() == this || e->undirected())
			eset.insert(e);
	}

	return eset;
}

std::set<Graph::PathElement> Graph::Vertex::reachable(){
    std::set<Graph::PathElement> result;

    Graph::EdgeSet eset = edges();

    for (Graph::Edge* e : eset){
        Graph::Vertex* from = e->from();
        Graph::Vertex* to = e->to();
        bool undirected = e->undirected();

        if (from == this){
            result.insert(std::make_pair(to, e));
        }

        if (undirected){
            Graph::Vertex* to = (e->from() == this) ? e->to() : e->from();
            result.insert(std::make_pair(to, e));
        }
    }

    return result;
}




Graph::Vertex* Graph::addVertex(const int &id, Vector2f position)
{
    Vertex* v = new Graph::Vertex(id, position);

    bool result = Graph::addVertex(v);

    if (result){
        return v;
    }
    else {
        delete v;
        return 0;
    }
}

bool Graph::addVertex(Vertex* v)
{
    auto result = _vertices.insert(std::make_pair(v->id(), v));

    if (!result.second){
        return false;
    }
    
    _verticesIdCount++;

    return true;
}


Graph::Edge::Edge(Vertex* from, Vertex* to, bool undirected, int id, float cost, int capacity)
{   
	_from = from;
	_to = to;
    _id = id;


	_undirected = undirected;
	_cost = cost;
	_capacity = capacity;
    _parentId = id;

    _vertices.insert(from);
    _vertices.insert(to);
    
}

Graph::Edge::~Edge()
{

}

Graph::Edge::Edge(const Graph::Edge & e)
{
    _from = new Graph::Vertex(e.from()->id(), e.from()->position());
    _to = new Graph::Vertex(e.to()->id(), e.to()->position());
    _id = e.id();

    _undirected = e.undirected();
    _cost = e.cost();
	_capacity = e.capacity();
    _parentId = e.id();

}


Graph::Edge & Graph::Edge::operator=(const Graph::Edge & e)
{
    _from = new Graph::Vertex(e.from()->id(), e.from()->position());
    _to = new Graph::Vertex(e.to()->id(), e.to()->position());
    _id = e.id();

    _undirected = e.undirected();
    _cost = e.cost();
	_capacity = e.capacity();
    _parentId = e.id();

}


bool Graph::Edge::operator<(const Edge& other) const
{
    return _id < other.id();
}


Graph::Edge* Graph::addEdge(Vertex* from, Vertex* to, bool undirected, float cost, int capacity, int edgeId)
{   
    if (cost < 0){
        Vector2f vertexDifference = to->position() - from->position();
        cost = vertexDifference.norm();
    }

    if (edgeId == UnassignedId){
        edgeId = _edgesIdCount + 1;
    }

    Edge* e = new Graph::Edge(from, to, undirected, edgeId, cost, capacity);
    
    bool result = Graph::addEdge(e);

    if (result){
        return e;
    }
    else{
        delete e;
        return 0;
    }


}

Graph::Edge* Graph::addEdge(int fromId, int toId, bool undirected, float cost, int capacity, int edgeId)
{   
    Vertex* from = vertex(fromId);
    Vertex* to = vertex(toId);

    if (from == 0 || to == 0){
        return 0;
    }

    if (cost < 0){
        Vector2f vertexDifference = to->position() - from->position();
        cost = vertexDifference.norm();
    }


    if (edgeId == UnassignedId){
        edgeId = _edgesIdCount + 1;
    }



    Edge* e = new Graph::Edge(from, to, undirected, edgeId, cost, capacity);
    
    bool result = Graph::addEdge(e);

    if (result){
        return e;
    }
    else{
        delete e;
        return 0;
    }

}




bool Graph::addEdge(Edge* e)
{
    auto result = _edges.insert(std::make_pair(e->id(), e));

    if (!result.second){
        return false;
    }

    _edgesIdCount++;

    for (Vertex* v : e->vertices()){
        if (v){
            v->edges().insert(e);
        }
    }

    return true;

}



Graph::Graph()
{
    _verticesIdCount = 0;
    _edgesIdCount = 0;
}


Graph::Vertex* Graph::vertex(int id)
{
    VertexIDMap::iterator it =_vertices.find(id);
    if (it == _vertices.end()){
        return nullptr;
    }
    return it->second;
}

const Graph::Vertex* Graph::vertex(int id) const
{
    VertexIDMap::const_iterator it =_vertices.find(id);
    if (it == _vertices.end()){
        return nullptr;
    }
    return it->second;
}

Graph::Edge* Graph::edge(int id)
{
    EdgeIDMap::iterator it =_edges.find(id);
    if (it == _edges.end()){
        return nullptr;
    }
    return it->second;
}

const Graph::Edge* Graph::edge(int id) const
{
    EdgeIDMap::const_iterator it =_edges.find(id);
    if (it == _edges.end()){
        return nullptr;
    }
    return it->second;
}


bool Graph::removeVertex(Graph::Vertex* v)
{

    Graph::VertexIDMap::iterator it=_vertices.find(v->id());

    if (it == _vertices.end())
      return false;

    assert(it->second == v);

    Graph::EdgeSet edges = v->edges();
    for (Graph::EdgeSet::iterator it = edges.begin(); it != edges.end(); it ++){
        removeEdge(*it);
    }

    _vertices.erase(it);
    delete v;
    return true;

}


bool Graph::removeEdge(Graph::Edge* e)
{

    if (e == nullptr){
        return false;
    }

    Graph::EdgeIDMap::iterator it = _edges.find(e->id());

    if (it == _edges.end())
        return false;
    
    _edges.erase(it);
    
    Graph::EdgeSet::iterator its;

    its = e->from()->edges().find(e);
    e->from()->edges().erase(its);

    its = e->to()->edges().find(e);
    e->to()->edges().erase(its);

    delete e;
    return true;

}



std::set<int> Graph::getEdgesBetweenVertices(int v1Id, int v2Id)
{

    std::set<int> eset;

    Graph::Vertex* v1 = vertex(v1Id);

    if (v1 == nullptr){
        return eset;
    }

    Graph::Vertex* v2 = vertex(v2Id);

    if (v2 == nullptr){
        return eset;
    }

    Graph::EdgeSet v1Edges = v1->edges();

    for (Graph::EdgeSet::iterator it = v1Edges.begin(); it != v1Edges.end(); it ++){

        Graph::Edge* e = *it;

        if (e->from() == v2 || e->to() == v2){
            eset.insert(e->id());
        }
    }


    return eset;
}



Graph::~Graph()
{
    clear();
}

void Graph::clear()
{
    for (VertexIDMap::iterator it=_vertices.begin(); it!=_vertices.end(); ++it)
        delete (it->second);
    for (EdgeIDMap::iterator it=_edges.begin(); it!=_edges.end(); ++it)
        delete (it->second);

    _vertices.clear();
    _edges.clear();

    _verticesIdCount = 0;
    _edgesIdCount = 0;
}

Graph::Graph( const Graph& graph)
{

    _edges = EdgeIDMap();
    _vertices = VertexIDMap();

    Graph::VertexIDMap vertices = graph.vertices();
    for (Graph::VertexIDMap::iterator itV  = vertices.begin(); itV != vertices.end(); itV ++){
        Graph::Vertex* v = itV->second;
        int id = v->id();
        Vector2f position = v->position();
        addVertex(id, position);
    }

    Graph::EdgeIDMap edges = graph.edges();
    for (Graph::EdgeIDMap::iterator itE  = edges.begin(); itE != edges.end(); itE ++){
        Graph::Edge* e = itE->second;
        int id = e->id();
        int fromId = e->from()->id();
        int toId = e->to()->id();
        bool undirected = e->undirected();
        float cost = e->cost();
        int capacity = e->capacity();

        Graph::Edge* _e = addEdge(fromId, toId, undirected, cost, capacity, e->id());
        _e->setParentId(e->parentId());

    }

    _verticesIdCount = graph._verticesIdCount;
    _edgesIdCount = graph._edgesIdCount;

}
Graph & Graph::operator=(const Graph & graph)
{

    if (this == &graph){
        return *this;
    }

    clear();

    Graph::VertexIDMap vertices = graph.vertices();
    for (Graph::VertexIDMap::iterator itV  = vertices.begin(); itV != vertices.end(); itV ++){
        Graph::Vertex* v = itV->second;
        int id = v->id();
        Vector2f position = v->position();
        addVertex(id, position);
    }

    Graph::EdgeIDMap edges = graph.edges();
    for (Graph::EdgeIDMap::iterator itE  = edges.begin(); itE != edges.end(); itE ++){
        Graph::Edge* e = itE->second;
        int id = e->id();
        int fromId = e->from()->id();
        int toId = e->to()->id();
        bool undirected = e->undirected();
        float cost = e->cost();
        int capacity = e->capacity();

        Graph::Edge* _e = addEdge(fromId, toId, undirected, cost, capacity, e->id());
        _e->setParentId(e->parentId());

    }

    _verticesIdCount = graph._verticesIdCount;
    _edgesIdCount = graph._edgesIdCount;

    return *this;


}

