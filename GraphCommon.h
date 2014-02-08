/********************************************************************
 * File   : GraphCommon.h
 * Project: Multiple
 *
 ********************************************************************
 * Created on 2/8/14 By Nonlinear Ideas Inc.
 * Copyright (c) 2014 Nonlinear Ideas Inc. All rights reserved.
 ********************************************************************
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any 
 * damages arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any 
 * purpose, including commercial applications, and to alter it and 
 * redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must 
 *    not claim that you wrote the original software. If you use this 
 *    software in a product, an acknowledgment in the product 
 *    documentation would be appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and 
 *    must not be misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source 
 *    distribution. 
 */

/* This set of code is derived largely from the work by Matt Buckland in 
 * the book "Programming Game AI By Example".  
 * ISBN-10: 1-55622-078-2
 * ISBN-13: 978-1-55622-078-4
 *
 * "If I have seen farther than others, it is because of I have stood
 *  on the shoulders of giants."
 *                                     - Albert Einstein
 *
 */

#ifndef __GraphCommon__
#define __GraphCommon__

#include "CommonProject.h"
#include "CommonSTL.h"

const uint32 INVALID_NODE_INDEX = (uint32)-1;

class HasFlags
{
private:
   uint32 _flags;
public:
   HasFlags(uint32 flags) : _flags(flags)
   {
      
   }
   
   HasFlags() : _flags(0)
   {
      
   }
   
   inline void SetFlags(uint32 value)
   {
      _flags = value;
   }
   
   inline uint32 GetFlags() const
   {
      return _flags;
   }
   
   inline void SetFlag(uint32 flag)
   {
      _flags |= flag;
   }
   
   inline void ClearFlag(uint32 flag)
   {
      _flags &= ~flag;
   }
   
   inline bool IsFlagSet(uint32 flag) const
   {
      return (_flags & flag) > 0;
   }
   
   inline bool IsFlagClear(uint32 flag) const
   {
      return (_flags & flag) == 0;
   }
};

/* This base class is used for ALL derived NODE types.
 * It contains the most basic information a node needs in
 * order to be used in a graph.
 */
class GraphNode : public HasFlags
{
private:
   uint32 _index;
public:
   // Flags used for the edge.  These
   // can be state flags or may indicate
   // the edge type (fly, swim, etc.), etc.
   enum
   {
      GNF_IS_CONNECTED = 0x00000001,
   };

   
   GraphNode() :
      HasFlags(GNF_IS_CONNECTED),
      _index(INVALID_NODE_INDEX)
   {
      
   }
   
   GraphNode(uint32 flags) :
      HasFlags(flags),
      _index(INVALID_NODE_INDEX)
   {
      
   }
   
   inline void SetIndex(uint32 index) { _index = index; }
   inline uint32 GetIndex() const { return _index; }
   
   void Dump()
   {
      cout << "Node(" << _index+1 << ")" << endl;
   }
};


/* This base class is used for ALL derived EDGE types.
 * It contains the most basic information that needs to 
 * be used by all search algorithms to search a graph.
 */
class GraphEdge : public HasFlags
{
private:
   uint32 _src;
   uint32 _des;
   double _cost;
   
public:
   // Flags used for the edge.  These
   // can be state flags or may indicate
   // the edge type (fly, swim, etc.), etc.
   enum
   {
      GEF_IS_CONNECTED = 0x00000001,
   };

   
   GraphEdge() :
      HasFlags(GEF_IS_CONNECTED),
      _src(INVALID_NODE_INDEX),
      _des(INVALID_NODE_INDEX),
      _cost(1.0)
   {
      
   }
   
   GraphEdge(uint32 src, uint32 des) :
      HasFlags(GEF_IS_CONNECTED),
      _src(src),
      _des(des),
      _cost(1.0)
   {
      
   }
   
   inline double GetCost() const { return _cost; }
   inline uint32 GetSrc() const { return _src; }
   inline uint32 GetDes() const { return _des; }
   
   inline void SetCost(double cost) { _cost = cost; }
   inline void SetSrc(uint32 src) { _src = src; }
   inline void SetDes(uint32 des) { _des = des; }
   
   inline bool operator==(const GraphEdge& other) const
   {
      if(_src == other._src &&
         _des == other._des &&
         _cost == other._cost
         )
         return true;
      return false;
   }
   
   inline bool operator!=(const GraphEdge& other) const
   {
      if(_src != other._src ||
         _des != other._des ||
         _cost != other._cost
         )
         return true;
      return false;
   }
   
   inline void Dump()
   {
      cout << "Edge: " << _src+1 << "-->" << _des+1 << endl;
   }
};
/* This class manages a sparse DIRECTED ACYCLIC GRAPH (DAG).
 * It will NOT automatically add connections for both directions
 * for an edge from a node.
 */
class Graph
{
public:
   typedef GraphNode Node;
   typedef GraphEdge Edge;
   
   typedef vector<GraphNode>   NODES_T;
   typedef vector<GraphEdge>     EDGES_T;
   typedef vector< vector<GraphEdge> >    NODE_EDGES_T;
   
private:
   // A vector of the nodes in this graph.  Keeping them as a
   // vector should provide O(1) access.
   NODES_T _nodes;
   
   // A vector of the container of edges associated with each
   // node.
   NODE_EDGES_T _edges;
   
   Node* FindNode(uint32 src)
   {
      assert(src < _nodes.size());
      return &_nodes[src];
   }
   
   const Node* FindNode(uint32 src) const
   {
      return &_nodes[src];
   }
   
   Edge* FindEdge(uint32 src, uint32 des)
   {
      assert(src < _edges.size());
      EDGES_T& edges = _edges[src];
      for(EDGES_T::iterator iter = edges.begin(); iter != edges.end(); ++iter)
      {
         if(iter->GetDes() == des)
         {
            return &(*iter);
         }
      }
      return NULL;
   }
   
   const Edge* FindEdge(uint32 src, uint32 des) const
   {
      return FindEdge(src,des);
   }
   
   
public:
   void Reset()
   {
      _nodes.clear();
      _edges.clear();
   }
   
   
   Graph(uint32 minNodes = 100)
   {
      Reset();
      _nodes.reserve(minNodes);
      _edges.reserve(minNodes);
   }
   
   uint32 GetNodeCount() const { return _nodes.size(); }
   
   virtual ~Graph()
   {
      Reset();
   }
   
   const EDGES_T& GetEdges(uint32 idx) const
   {
      assert(idx < _edges.size());
      return _edges[idx];
   }
  
   const Node& GetNode(uint32 idx) const
   {
      assert(idx < _nodes.size());
      return _nodes[idx];
   }
   
   Node& GetNode(uint32 idx)
   {
      assert(idx < _nodes.size());
      return _nodes[idx];
   }
   
   const Edge& GetEdge(uint32 from, uint32 to) const
   {
      return *FindEdge(from,to);
   }
   
   Edge& GetEdge(uint32 from, uint32 to)
   {
      return *FindEdge(from,to);
   }
   
   uint32 AddNode(const Node& node)
   {
      assert(node.GetIndex() == INVALID_NODE_INDEX);
      uint32 index = _nodes.size();
      _nodes.push_back(node);
      _nodes[index].SetIndex(index);
      _edges.push_back(EDGES_T());
      return index;
   }
   
   /* This method enables/disables a node by 
    * marking the GNF_IS_CONNECTED flag for it.
    *
    * Search Algorithms use this for handling
    * dynamic nodes without adding/removing them
    * from the graph.
    */
   void EnableNode(uint32 src, bool enable)
   {
      assert(src < _nodes.size());
      if(src < _nodes.size())
      {
         Node* node = FindNode(src);
         if(enable)
         {
            node->SetFlag(Node::GNF_IS_CONNECTED);
         }
         else
         {
            node->ClearFlag(Node::GNF_IS_CONNECTED);
         }
      }
   }
   
   void AddEdge(const Edge& edge)
   {
      uint32 src = edge.GetSrc();
      uint32 des = edge.GetDes();
      
      assert(edge.GetSrc() < _edges.size());
      assert(edge.GetDes() < _edges.size());
      if(src < _edges.size() && des < _edges.size())
      {
         Edge* other;
         // Verify it does not exist first.
         other = FindEdge(src,des);
         assert(other == NULL);
         if(other == NULL)
         {  // We can add it.
            _edges[src].push_back(edge);
         }
      }
   }
   
   void Dump()
   {
      cout << "Graph: (Nodes = " << _nodes.size() << endl;
      for(uint32 idx = 0; idx < _nodes.size(); ++idx)
      {
         cout << "----------------------------" << endl;
         _nodes[idx].Dump();
         EDGES_T edges = _edges[idx];
         for(int idx2 = 0; idx2 < edges.size(); ++idx2)
         {
            edges[idx2].Dump();
         }
      }
   }
};

/* This class is a base class for all the 
 * different search algorithms.  A derived class
 * overloads the methods as needed to implement
 * the graph search.
 */
class GraphSearchAlgorithm
{
protected:
   typedef enum
   {
      NS_VISITED,
      NS_NOT_VISITED,
      NS_NO_PARENT
   } NODE_STATE_T;
   
public:
   typedef enum
   {
      SS_FOUND,
      SS_NOT_FOUND,
      SS_STILL_WORKING,
   } SEARCH_STATE_T;
   
private:
   const Graph& _graph;
   vector<NODE_STATE_T> _visited;
   vector<uint32> _route;
   uint32 _startNode;
   uint32 _targetNode;
   SEARCH_STATE_T _searchState;
   
protected:
   inline SEARCH_STATE_T& SearchState() { return _searchState; }
   inline vector<NODE_STATE_T>& Visited() { return _visited; }
   inline vector<uint32>& Route() { return _route; }
   inline const uint32& StartNode() const { return _startNode; }
   inline const uint32& TargetNode() const { return _targetNode; }
   inline const Graph& Graph() { return _graph; }
   virtual SEARCH_STATE_T SearchCycle() = 0;
   
   typedef Graph::Edge Edge;
   typedef Graph::EDGES_T EDGES_T;
   
public:
   SEARCH_STATE_T GetSearchState() { return _searchState; }
   
   GraphSearchAlgorithm(const class Graph& graph,
                        uint32 start,
                        uint32 target) :
      _graph(graph),
      _startNode(start),
      _targetNode(target),
      _searchState(SS_STILL_WORKING),
      _visited(_graph.GetNodeCount(),NS_NOT_VISITED),
      _route(_graph.GetNodeCount(),NS_NO_PARENT)
   {
      
   }
   
   
   SEARCH_STATE_T SearchGraph()
   {
      while(SearchState() == SS_STILL_WORKING)
      {
         SearchState() = SearchCycle();
      }
      return SearchState();
   }

   SEARCH_STATE_T SearchGraph(uint32 cycles)
   {
      while(SearchState() == SS_STILL_WORKING && cycles > 0)
      {
         SearchState() = SearchCycle();
         --cycles;
      }
      return SearchState();
   }
   
   
   list<uint32> GetPathNodes()
   {
      list<uint32> path;
      
      if(SearchState() != SS_FOUND)
      {
         return path;
      }
      
      uint32 nodeIdx = TargetNode();
      path.push_front(nodeIdx);
      while(nodeIdx != StartNode())
      {
         nodeIdx = Route()[nodeIdx];
         path.push_front(nodeIdx);
      }
      return path;
   }
   
   list<Edge> GetPathEdges()
   {
      list<Edge> path;
      if(SearchState() != SS_FOUND)
      {
         return path;
      }
      
      uint32 desNode = TargetNode();
      uint32 srcNode = Route()[desNode];
      
      path.push_front(Edge(srcNode,desNode));
      while(srcNode != StartNode())
      {
         desNode = srcNode;
         srcNode = Route()[desNode];
         path.push_front(Edge(srcNode,desNode));
      }
   
      return path;
   }
};

class GraphSearchDFS : public GraphSearchAlgorithm
{
private:
   stack<const Edge*> _stack;
   Edge _firstEdge;
protected:
   virtual SEARCH_STATE_T SearchCycle()
   {
      if(_stack.empty())
      {
         return SS_NOT_FOUND;
      }
      // Start with the edge at the top of the stack.
      const Edge * edge = _stack.top();
      // Remove it, we are processing it now.
      _stack.pop();
      
      // For convience
      uint32 src = edge->GetSrc();
      uint32 des = edge->GetDes();
      // Update the route for the path we are following.
      Route()[des] = src;
      // Mark that we have visited this node.
      Visited()[des] = NS_VISITED;
      // Is this the node we are looking for?
      if(des == TargetNode())
      {  // Yes.  We are done.
         return SS_FOUND;
      }
      // No.  Push all the edges that lead from the
      // node onto the stack IFF we have not already
      // visited that destination.
      const EDGES_T& edges = Graph().GetEdges(des);
      for(EDGES_T::const_iterator iter = edges.begin();
          iter != edges.end();
          ++iter)
      {  // If the destination node has not been visited,
         // then add it.
         if(Visited()[iter->GetDes()] == NS_NOT_VISITED)
         {
            _stack.push(&(*iter));
         }
      }
      
      return SS_STILL_WORKING;
   }

public:
   GraphSearchDFS(const class Graph& graph,
                  uint32 start,
                  uint32 target) :
   GraphSearchAlgorithm(graph,start,target)
   {
      _firstEdge.SetDes(start);
      _firstEdge.SetSrc(start);
      _stack.push(&_firstEdge);
   }
};


class GraphSearchBFS : public GraphSearchAlgorithm
{
private:
   queue<const Edge*> _queue;
   Edge _firstEdge;
protected:
   virtual SEARCH_STATE_T SearchCycle()
   {
      if(_queue.empty())
      {
         return SS_NOT_FOUND;
      }
      // Start with the edge at the top of the stack.
      const Edge * edge = _queue.front();
      // Remove it, we are processing it now.
      _queue.pop();
      
      // For convience
      uint32 src = edge->GetSrc();
      uint32 des = edge->GetDes();
      // Update the route for the path we are following.
      Route()[des] = src;
      // Mark that we have visited this node.
      Visited()[des] = NS_VISITED;
      // Is this the node we are looking for?
      if(des == TargetNode())
      {  // Yes.  We are done.
         return SS_FOUND;
      }
      // No.  Push all the edges that lead from the
      // node onto the stack IFF we have not already
      // visited that destination.
      const EDGES_T& edges = Graph().GetEdges(des);
      for(EDGES_T::const_iterator iter = edges.begin();
          iter != edges.end();
          ++iter)
      {  // If the destination node has not been visited,
         // then add it.
         if(Visited()[iter->GetDes()] == NS_NOT_VISITED)
         {
            _queue.push(&(*iter));
         }
      }
      
      return SS_STILL_WORKING;
   }
   
public:
   GraphSearchBFS(const class Graph& graph,
                  uint32 start,
                  uint32 target) :
   GraphSearchAlgorithm(graph,start,target)
   {
      _firstEdge.SetDes(start);
      _firstEdge.SetSrc(start);
      _queue.push(&_firstEdge);
   }
};


// A small Test Program for the DFS search
void TestDFS();
void TestBFS();
void Dump(list<uint32>& nodeList);
void Dump(list<Graph::Edge>& edgeList);


#endif /* defined(__GraphCommon__) */
