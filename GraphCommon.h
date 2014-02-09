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
   uint32 _graphIndex;
   uint32 _ID;
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
      _graphIndex(INVALID_NODE_INDEX),
      _ID(0)
   {
      
   }
   
   GraphNode(uint32 ID,uint32 flags) :
      HasFlags(flags),
      _graphIndex(INVALID_NODE_INDEX),
      _ID(ID)
   {
      
   }
   
   GraphNode(uint32 ID) :
      HasFlags(GNF_IS_CONNECTED),
      _graphIndex(INVALID_NODE_INDEX),
      _ID(ID)
   {
      
   }
   
   inline void SetGraphIndex(uint32 index) { _graphIndex = index; }
   inline uint32 GetGraphIndex() const { return _graphIndex; }
   inline void SetID(uint32 ID) { _ID = ID; }
   inline uint32 GetID() const { return _ID; }
   
   void Dump()
   {
      cout << "Node(" << _ID << ")" << endl;
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
      cout << "Edge: " << _src << "-->" << _des << endl;
   }
};
/* This class manages a sparse DIRECTED ACYCLIC GRAPH (DAG).
 * It will NOT automatically add connections for both directions
 * for an edge from a node.
 */
template <class NODE_TYPE, class EDGE_TYPE>
class Graph
{
public:
   typedef NODE_TYPE Node;
   typedef EDGE_TYPE Edge;
   
   typedef vector<NODE_TYPE>   NODES_T;
   typedef vector<EDGE_TYPE>     EDGES_T;
   typedef vector< vector<EDGE_TYPE> >    NODE_EDGES_T;
   
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
      for(int idx = 0; idx < edges.size(); ++idx)
      {
         if(edges[idx].GetDes() == des)
         {
            return &(edges[idx]);
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
      assert(node.GetGraphIndex() == INVALID_NODE_INDEX);
      uint32 index = _nodes.size();
      _nodes.push_back(node);
      _nodes[index].SetGraphIndex(index);
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
         if(node != NULL)
         {
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
   }
   
   void EnableEdge(uint32 src, uint32 des, bool enable)
   {
      assert(src < _edges.size());
      assert(des < _edges.size());
      Edge* edge = FindEdge(src,des);
      if(edge != NULL)
      {
         if(enable)
         {
            edge->SetFlag(Edge::GEF_IS_CONNECTED);
         }
         else
         {
            edge->ClearFlag(Edge::GEF_IS_CONNECTED);
         }
      }
   }

   void EnableEdges(uint32 src, uint32 des, bool enable)
   {
      EnableEdge(src,des,enable);
      EnableEdge(des,src,enable);
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

typedef enum
{
   NS_VISITED,
   NS_NOT_VISITED,
   NS_NO_PARENT
} NODE_STATE_T;

typedef enum
{
   SS_FOUND,
   SS_NOT_FOUND,
   SS_STILL_WORKING,
} SEARCH_STATE_T;

template <class NODE_TYPE, class EDGE_TYPE>
class GraphSearchAlgorithm
{
protected:
   
   typedef Graph<NODE_TYPE,EDGE_TYPE> GRAPH_TYPE;
   
public:
   
private:
   const GRAPH_TYPE& _graph;
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
   inline const Graph<NODE_TYPE,EDGE_TYPE>& Graph() { return _graph; }
   virtual SEARCH_STATE_T SearchCycle() = 0;
   
   typedef typename GRAPH_TYPE::Node Node;
   typedef typename GRAPH_TYPE::Edge Edge;
   typedef typename GRAPH_TYPE::EDGES_T EDGES_T;
   
public:
   SEARCH_STATE_T GetSearchState() { return _searchState; }
   
   GraphSearchAlgorithm(const GRAPH_TYPE& graph,
                        uint32 start,
                        uint32 target) :
      _graph(graph),
      _startNode(start),
      _targetNode(target),
      _searchState(SS_STILL_WORKING),
      _visited(_graph.GetNodeCount(),NS_NOT_VISITED),
      _route(_graph.GetNodeCount(),NS_NO_PARENT)
   {
      // Special case...if the start/target node has already
      // been disconnected, then fail immediately.
      if(graph.GetNode(_startNode).IsFlagClear(GraphNode::GNF_IS_CONNECTED))
      {
         _searchState = SS_NOT_FOUND;
      }
      if(graph.GetNode(_targetNode).IsFlagClear(GraphNode::GNF_IS_CONNECTED))
      {
         _searchState = SS_NOT_FOUND;
      }
   }
   
   
   SEARCH_STATE_T SearchGraph()
   {
      while(_searchState == SS_STILL_WORKING)
      {
         // Special case...if the start/target node has been disabled
         if(_graph.GetNode(_startNode).IsFlagClear(GraphNode::GNF_IS_CONNECTED))
         {
            _searchState = SS_NOT_FOUND;
         }
         else if(_graph.GetNode(_targetNode).IsFlagClear(GraphNode::GNF_IS_CONNECTED))
         {
            _searchState = SS_NOT_FOUND;
         }
         else
         {
            _searchState = SearchCycle();
         }
      }
      return _searchState;
   }

   SEARCH_STATE_T SearchGraph(uint32 cycles)
   {
      while(_searchState == SS_STILL_WORKING && cycles > 0)
      {
         // Special case...if the start/target node has been disabled
         if(_graph.GetNode(_startNode).IsFlagClear(GraphNode::GNF_IS_CONNECTED))
         {
            _searchState = SS_NOT_FOUND;
         }
         else if(_graph.GetNode(_targetNode).IsFlagClear(GraphNode::GNF_IS_CONNECTED))
         {
            _searchState = SS_NOT_FOUND;
         }
         else
         {
            _searchState = SearchCycle();
         }
         --cycles;
      }
      return _searchState;
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
   
   void Dump()
   {
      switch(_searchState)
      {
         case SS_STILL_WORKING:
            cout << "YIKES...STILL WORKING" << endl;
            break;
         case SS_NOT_FOUND:
            cout << "YIKES...NO RESULT FOUND" << endl;
            break;
         case SS_FOUND:
         {
            list<Edge> edgeList = GetPathEdges();
            while(edgeList.size() > 0)
            {
               edgeList.begin()->Dump();
               edgeList.erase(edgeList.begin());
            }
         }
            break;
      }
      
   }
   
};

template <class NODE_TYPE, class EDGE_TYPE>
class GraphSearchDFS : public GraphSearchAlgorithm<NODE_TYPE,EDGE_TYPE>
{
private:
   typedef Graph<NODE_TYPE,EDGE_TYPE> GRAPH_TYPE;
   typedef typename GRAPH_TYPE::Node Node;
   typedef typename GRAPH_TYPE::Edge Edge;
   typedef typename GRAPH_TYPE::EDGES_T EDGES_T;
   
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
      this->Route()[des] = src;
      // Mark that we have visited this node.
      this->Visited()[des] = NS_VISITED;
      // Is this the node we are looking for?
      if(des == this->TargetNode())
      {  // Yes.  We are done.
         return SS_FOUND;
      }
      // No.  Push all the edges that lead to the des
      // node onto the stack IFF we have not already
      // visited that destination and if it is available.
      const Node& node = this->Graph().GetNode(des);
      if(node.IsFlagSet(GraphNode::GNF_IS_CONNECTED))
      {
         const EDGES_T& edges = this->Graph().GetEdges(des);
         for(int idx = 0; idx < edges.size(); ++idx)
         {  // If the destination node has not been visited,
            // then add it.
            if(edges[idx].IsFlagSet(GraphEdge::GEF_IS_CONNECTED))
            {
               if(this->Visited()[edges[idx].GetDes()] == NS_NOT_VISITED)
               {
                  _stack.push(&(edges[idx]));
               }
            }
         }
      }
      return SS_STILL_WORKING;
   }

public:
   GraphSearchDFS(const GRAPH_TYPE& graph,
                  uint32 start,
                  uint32 target) :
   GraphSearchAlgorithm<NODE_TYPE,EDGE_TYPE>(graph,start,target)
   {
      _firstEdge.SetDes(start);
      _firstEdge.SetSrc(start);
      _stack.push(&_firstEdge);
   }
};


template <class NODE_TYPE, class EDGE_TYPE>
class GraphSearchBFS : public GraphSearchAlgorithm<NODE_TYPE,EDGE_TYPE>
{
private:
   typedef Graph<NODE_TYPE,EDGE_TYPE> GRAPH_TYPE;
   typedef typename GRAPH_TYPE::Node Node;
   typedef typename GRAPH_TYPE::Edge Edge;
   typedef typename GRAPH_TYPE::EDGES_T EDGES_T;
   
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
      this->Route()[des] = src;
      // Mark that we have visited this node.
      this->Visited()[des] = NS_VISITED;
      // Is this the node we are looking for?
      if(des == this->TargetNode())
      {  // Yes.  We are done.
         return SS_FOUND;
      }
      // No.  Push all the edges that lead to the des
      // node onto the stack IFF we have not already
      // visited that destination and if it is available.
      const Node& node = this->Graph().GetNode(des);
      if(node.IsFlagSet(GraphNode::GNF_IS_CONNECTED))
      {
         const EDGES_T& edges = this->Graph().GetEdges(des);
         for(int idx = 0; idx < edges.size(); ++idx)
         {  // If the destination node has not been visited,
            // then add it.
            if(edges[idx].IsFlagSet(GraphEdge::GEF_IS_CONNECTED))
            {
               if(this->Visited()[edges[idx].GetDes()] == NS_NOT_VISITED)
               {
                  _queue.push(&(edges[idx]));
               }
            }
         }
      }
      return SS_STILL_WORKING;
   }
   
public:
   GraphSearchBFS(const GRAPH_TYPE& graph,
                  uint32 start,
                  uint32 target) :
   GraphSearchAlgorithm<NODE_TYPE,EDGE_TYPE>(graph,start,target)
   {
      _firstEdge.SetDes(start);
      _firstEdge.SetSrc(start);
      _queue.push(&_firstEdge);
   }
};

// A small Test Program for the DFS search
void TestDFS();
void TestBFS();


#endif /* defined(__GraphCommon__) */
