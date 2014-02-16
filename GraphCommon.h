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

/* This set of code is derived largely from the work by Mat Buckland in 
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
#include "HasFlags.h"

#define DEBUG_FLOODING

const uint32 INVALID_NODE_INDEX = (uint32)-1;

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
   GraphNode() :
      HasFlags(HF_IS_CONNECTED),
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
      HasFlags(HF_IS_CONNECTED),
      _graphIndex(INVALID_NODE_INDEX),
      _ID(ID)
   {
      
   }
   
   inline void SetGraphIndex(uint32 index) { assert(index >= 0); _graphIndex = index; }
   inline uint32 GetGraphIndex() const { return _graphIndex; }
   inline void SetID(uint32 ID) { _ID = ID; }
   inline uint32 GetID() const { return _ID; }
   
   virtual void Dump() const
   {
      cout << "GraphNode(" << _ID << ")" << endl;
   }
};


class NavGraphNode : public GraphNode
{
private:
   Vec2 _pos;
   
public:
   const Vec2& GetPos() const { return _pos; }
   void SetPos(const Vec2& pos) { _pos = pos; }
   
   NavGraphNode(uint32 ID) :
   GraphNode(ID)
   {
      _pos.x = ID;
      _pos.y = 2*ID;
   }
   
   virtual void Dump() const
   {
      GraphNode::Dump();
      cout << "- NavGraphNode(" << _pos.x << "," << _pos.y << ")" << endl;
   }
};

/* This base class is used for ALL derived EDGE types.
 * It contains the most basic information that needs to 
 * be used by all search algorithms to search a graph.
 */
class GraphEdge : public HasFlags
{
private:
   int32 _src;
   int32 _des;
   double _cost;
   
public:

   
   GraphEdge() :
      HasFlags(HF_IS_CONNECTED),
      _src(INVALID_NODE_INDEX),
      _des(INVALID_NODE_INDEX),
      _cost(1.0)
   {
      
   }
   
   GraphEdge(uint32 src, uint32 des) :
      HasFlags(HF_IS_CONNECTED),
      _src(src),
      _des(des),
      _cost(1.0)
   {
      assert(src >= 0);
      assert(des >= 0);
   }
   
   inline double GetCost() const { return _cost; }
   inline uint32 GetSrc() const { return _src; }
   inline uint32 GetDes() const { return _des; }
   
   inline void SetCost(double cost) { assert(cost >= 0.0); _cost = cost; }
   inline void SetSrc(uint32 src) { assert(src >= 0); _src = src; }
   inline void SetDes(uint32 des) { assert(des >= 0); _des = des; }
   
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
   
   virtual void Dump() const
   {
      cout << "GraphEdge: " << _src << "-->" << _des << endl;
   }
};

class NavGraphEdge : public GraphEdge
{
private:
   Vec2 _srcPos;
   Vec2 _desPos;
   
public:
   const Vec2& GetSrcPos() { return _srcPos; }
   void SetSrcPos(const Vec2& pos) { _srcPos = pos; }
   const Vec2& GetDesPos() { return _desPos; }
   void SetDesPos(const Vec2& pos) { _desPos = pos; }
   
   virtual void Dump() const
   {
      GraphEdge::Dump();
      cout << " - NavGraphEdge: (" << _srcPos.x << "," << _srcPos.y << ")"
      << "-->(" << _desPos.x << "," << _desPos.y << ")" << endl;
   }
   
   NavGraphEdge(uint32 src, uint32 des, const Vec2& srcPos, const Vec2& desPos) :
      GraphEdge(src,des),
      _srcPos(srcPos),
      _desPos(desPos)
   {
      
   }
};

/* This class manages a sparse DIRECTED ACYCLIC GRAPH (DAG).
 * It will NOT automatically add connections for both directions
 * for an edge from a node.
 *
 * Objects added to the Graph are managed by the graph.
 */

typedef vector<GraphNode*>   GraphNodes;
typedef vector<GraphEdge*>     GraphEdges;
typedef vector< vector<GraphEdge*> >    GraphNodeEdges;


class Graph
{
public:
   
   
private:
   // A vector of the nodes in this graph.  Keeping them as a
   // vector should provide O(1) access.
   GraphNodes _nodes;
   
   // A vector of the container of edges associated with each
   // node.
   GraphNodeEdges _edges;
   
   GraphNode* FindNode(uint32 src)
   {
      assert(src < _nodes.size());
      return _nodes[src];
   }
   
   const GraphNode* FindNode(uint32 src) const
   {
      assert(src < _nodes.size());
      return _nodes[src];
   }
   
   GraphEdge* FindEdge(uint32 src, uint32 des)
   {
      assert(src < _edges.size());
      assert(des < _edges.size());
      GraphEdges& edges = _edges[src];
      for(int idx = 0; idx < edges.size(); ++idx)
      {
         if(edges[idx]->GetDes() == des)
         {
            return (edges[idx]);
         }
      }
      return NULL;
   }
   
   const GraphEdge* FindEdge(uint32 src, uint32 des) const
   {
      assert(src < _edges.size());
      assert(des < _edges.size());
      const GraphEdges& edges = _edges[src];
      for(int idx = 0; idx < edges.size(); ++idx)
      {
         if(edges[idx]->GetDes() == des)
         {
            return (edges[idx]);
         }
      }
      return NULL;
   }
   
   
public:
   void Reset()
   {
      for(int idx = 0; idx < _nodes.size(); idx++)
      {
         delete _nodes[idx];
         for(int edIdx = 0; edIdx < _edges[idx].size(); edIdx++)
         {
            delete _edges[idx][edIdx];
         }
      }
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
   
   const GraphEdges& GetEdges(uint32 idx) const
   {
      assert(idx < _edges.size());
      return _edges[idx];
   }
  
   const GraphNode* GetNode(uint32 idx) const
   {
      assert(idx < _nodes.size());
      return _nodes[idx];
   }
   
   GraphNode* GetNode(uint32 idx)
   {
      assert(idx < _nodes.size());
      return _nodes[idx];
   }
   
   const GraphEdge* GetEdge(uint32 from, uint32 to) const
   {
      return FindEdge(from,to);
   }
   
   GraphEdge* GetEdge(uint32 from, uint32 to)
   {
      return FindEdge(from,to);
   }
   
   uint32 AddNode(GraphNode* node)
   {
      assert(node != NULL);
      assert(node->GetGraphIndex() == INVALID_NODE_INDEX);
      uint32 index = _nodes.size();
      _nodes.push_back(node);
      _nodes[index]->SetGraphIndex(index);
      _edges.push_back(GraphEdges());
      return index;
   }
   
   /* This method enables/disables a node by 
    * marking the HF_IS_CONNECTED flag for it.
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
         GraphNode* node = FindNode(src);
         if(node != NULL)
         {
            if(enable)
            {
               node->SetFlag(GraphNode::HF_IS_CONNECTED);
            }
            else
            {
               node->ClearFlag(GraphNode::HF_IS_CONNECTED);
            }
         }
      }
   }
   
   void EnableEdge(uint32 src, uint32 des, bool enable)
   {
      assert(src < _edges.size());
      assert(des < _edges.size());
      GraphEdge* edge = FindEdge(src,des);
      if(edge != NULL)
      {
         if(enable)
         {
            edge->SetFlag(HasFlags::HF_IS_CONNECTED);
         }
         else
         {
            edge->ClearFlag(HasFlags::HF_IS_CONNECTED);
         }
      }
   }

   void EnableEdges(uint32 src, uint32 des, bool enable)
   {
      EnableEdge(src,des,enable);
      EnableEdge(des,src,enable);
   }   
   
   void AddEdge(GraphEdge* edge)
   {
      assert(edge != NULL);
      assert(edge->GetSrc() < _edges.size());
      assert(edge->GetDes() < _edges.size());
      
      uint32 src = edge->GetSrc();
      uint32 des = edge->GetDes();
      
      if(src < _edges.size() && des < _edges.size())
      {
         GraphEdge* other;
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
         _nodes[idx]->Dump();
         GraphEdges& edges = _edges[idx];
         for(int idx2 = 0; idx2 < edges.size(); ++idx2)
         {
            edges[idx2]->Dump();
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
public:
   typedef enum
   {
      SS_FOUND,
      SS_NOT_FOUND,
      SS_STILL_WORKING,
      SS_NOT_STARTED,
   } SEARCH_STATE_T;
   
protected:
   typedef enum
   {
      NS_VISITED,
      NS_NOT_VISITED,
      NS_NO_PARENT
   } NODE_STATE_T;
   
public:
   
private:
   const Graph& _graph;
   vector<NODE_STATE_T> _visited;
   vector<uint32> _flood;
   vector<uint32> _route;
   uint32 _startNode;
   uint32 _targetNode;
   SEARCH_STATE_T _searchState;
   GraphEdge _firstEdge;
   bool _allowDisconnectedStartNode;

   bool IsPathPossible()
   {
      if(_graph.GetNode(_startNode)->IsFlagClear(GraphNode::HF_IS_CONNECTED) &&
         !_allowDisconnectedStartNode)
      {
         CCLOG("Start Node %d NOT CONNECTED",_startNode);
         return false;
      }
      if(_graph.GetNode(_targetNode)->IsFlagClear(GraphNode::HF_IS_CONNECTED))
      {
         CCLOG("Target Node %d NOT CONNECTED",_targetNode);
         return false;
      }
      return true;
   }
   
   
protected:
   inline vector<NODE_STATE_T>& GetVisited() { return _visited; }
   inline vector<uint32>& GetRoute() { return _route; }
   inline const uint32& GetStartNode() const { return _startNode; }
   inline const uint32& GetTargetNode() const { return _targetNode; }
   inline const Graph& GetGraph() { return _graph; }
   inline const GraphEdge* GetFirstEdge() { return &_firstEdge; }
   inline void AddToFlood(uint32 nodeID) { _flood.push_back(nodeID); }
   
   /* These two virtual functions are overloaded by 
    * derived classes.
    *
    * The first function is called whenever a search is 
    * started to load the first node and do any special
    * operations (such as checking if the path is even 
    * possible.
    *
    */
   virtual SEARCH_STATE_T SearchCycleStart() = 0;
   virtual SEARCH_STATE_T SearchCycle() = 0;
   
public:
   SEARCH_STATE_T GetSearchState() { return _searchState; }
   
   GraphSearchAlgorithm(const class Graph& graph,
                        uint32 start,
                        uint32 target) :
      _graph(graph),
      _startNode(start),
      _targetNode(target),
      _searchState(SS_NOT_STARTED),
      _visited(_graph.GetNodeCount(),NS_NOT_VISITED),
      _route(_graph.GetNodeCount(),NS_NO_PARENT),
      _allowDisconnectedStartNode(true)
   {
      _firstEdge.SetSrc(start);
      _firstEdge.SetDes(start);
   }
   
   void SetAllowDisconnectedStartNode(bool allow) { _allowDisconnectedStartNode = allow; }
   bool GetAllowDisconnectedStartNode() { return _allowDisconnectedStartNode; }
   
   /* This will allow reuse of this class after a search has
    * been run (so it can be used mulitple times).
    */

   void Init(uint32 start, uint32 target)
   {
      _visited.assign(_visited.size(),NS_NOT_VISITED);
      _route.assign(_route.size(),NS_NO_PARENT);
      _searchState = SS_NOT_STARTED;
      _firstEdge.SetSrc(start);
      _firstEdge.SetDes(start);
      _flood.clear();
   }

   SEARCH_STATE_T SearchGraph()
   {
      if(_searchState == SS_NOT_STARTED)
      {
         _searchState = SearchCycleStart();
      }
      while(_searchState == SS_STILL_WORKING)
      {
         // Special case...if the start/target node has been disabled
         if(!IsPathPossible())
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
      if(_searchState == SS_NOT_STARTED)
      {
         _searchState = SearchCycleStart();
      }
      while(_searchState == SS_STILL_WORKING && cycles > 0)
      {
         // Special case...if the start/target node has been disabled
         if(!IsPathPossible())
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
   
   
   list<const GraphNode*> GetPathNodes()
   {
      list<const GraphNode*> path;
      
      if(GetSearchState() != SS_FOUND)
      {
         return path;
      }
      
      uint32 nodeIdx = GetTargetNode();
      path.push_front(_graph.GetNode(nodeIdx));
      while(nodeIdx != GetStartNode())
      {
         nodeIdx = GetRoute()[nodeIdx];
         path.push_front(_graph.GetNode(nodeIdx));
      }
      return path;
   }

   list<const GraphNode*> GetFloodNodes()
   {
      list<const GraphNode*> path;
      for(uint32 idx = 0; idx < _flood.size(); ++idx)
      {
         const GraphNode* node = _graph.GetNode(_flood[idx]);
         path.push_back(node);
      }
      return path;
   }
   
   
   list<const GraphEdge*> GetPathEdges()
   {
      list<const GraphEdge*> path;
      if(GetSearchState() != SS_FOUND)
      {
         return path;
      }
      
      uint32 desNode = GetTargetNode();
      uint32 srcNode = GetRoute()[desNode];
      
      path.push_front(_graph.GetEdge(srcNode, desNode));
      while(srcNode != GetStartNode())
      {
         desNode = srcNode;
         srcNode = GetRoute()[desNode];
         path.push_front(_graph.GetEdge(srcNode, desNode));
      }
   
      return path;
   }
   
   void Dump()
   {
      switch(_searchState)
      {
         case SS_NOT_STARTED:
            cout << "YIKES...NOT EVEN STARTED YET!!" << endl;
            break;
         case SS_STILL_WORKING:
            cout << "YIKES...STILL WORKING" << endl;
            break;
         case SS_NOT_FOUND:
            cout << "YIKES...NO RESULT FOUND" << endl;
            break;
         case SS_FOUND:
         {
            list<const GraphNode*> nodeList = GetPathNodes();
            for(list<const GraphNode*>::iterator iter = nodeList.begin();
                iter != nodeList.end();
                ++iter)
            {
               (*iter)->Dump();
            }
         }
            break;
      }
   }
   
};

class GraphSearchDFS : public GraphSearchAlgorithm
{
private:
   
   stack<const GraphEdge*> _stack;
protected:
   virtual SEARCH_STATE_T SearchCycleStart()
   {
      _stack.push(GetFirstEdge());
      return SS_STILL_WORKING;
   }
   
   virtual SEARCH_STATE_T SearchCycle()
   {
      if(_stack.empty())
      {
         return SS_NOT_FOUND;
      }
      // Start with the edge at the top of the stack.
      const GraphEdge * edge = _stack.top();
      // Remove it, we are processing it now.
      _stack.pop();
      
      // Update the route for the path we are following.
      GetRoute()[edge->GetDes()] = edge->GetSrc();
      // Mark that we have visited this node.
      GetVisited()[edge->GetDes()] = NS_VISITED;
      // Is this the node we are looking for?
      if(edge->GetDes() == GetTargetNode())
      {  // Yes.  We are done.
         return SS_FOUND;
      }
      // No.  Push all the edges that lead to the des
      // node onto the stack IFF we have not already
      // visited that destination and if it is available.
      const GraphNode* node = GetGraph().GetNode(edge->GetDes());
      if(node->IsFlagSet(HasFlags::HF_IS_CONNECTED))
      {
         const GraphEdges& edges = GetGraph().GetEdges(edge->GetDes());
         for(int idx = 0; idx < edges.size(); ++idx)
         {  // If the destination node has not been visited,
            // then add it.
            if(edges[idx]->IsFlagSet(HasFlags::HF_IS_CONNECTED))
            {
               if(GetVisited()[edges[idx]->GetDes()] == NS_NOT_VISITED)
               {
                  _stack.push(edges[idx]);
               }
            }
         }
      }
      return SS_STILL_WORKING;
   }

public:
   GraphSearchDFS(const Graph& graph,
                  uint32 start,
                  uint32 target) :
   GraphSearchAlgorithm(graph,start,target)
   {
   }
};


class GraphSearchBFS : public GraphSearchAlgorithm
{
private:
   
   list<const GraphEdge*> _queue;
   void DumpQueue()
   {
      cout << "Queue Size: " << _queue.size() << endl;
      cout << "-------------------------------" << endl;
      cout << "- Elements" << endl;
      for(list<const GraphEdge*>::const_iterator iter = _queue.begin();
          iter != _queue.end();
          ++iter)
      {
         (*iter)->Dump();
      }
      cout << endl;
   }
   
protected:
   virtual SEARCH_STATE_T SearchCycleStart()
   {
      _queue.push_back(GetFirstEdge());
      GetVisited()[GetStartNode()] = NS_VISITED;
      return SS_STILL_WORKING;
   }
   
   
   virtual SEARCH_STATE_T SearchCycle()
   {
      //      DumpQueue();
      if(_queue.empty())
      {
         return SS_NOT_FOUND;
      }
      // Start with the edge at the top of the stack.
      const GraphEdge * edge = _queue.front();
      // Remove it, we are processing it now.
      _queue.erase(_queue.begin());

#ifdef DEBUG_FLOODING
      // Add this for flood debugging.
      AddToFlood(edge->GetSrc());
#endif
      // Update the route for the path we are following.
      GetRoute()[edge->GetDes()] = edge->GetSrc();
      // Is this the node we are looking for?
      if(edge->GetDes() == GetTargetNode())
      {  // Yes.  We are done.
         return SS_FOUND;
      }
      // No.  Push all the edges that lead to the des
      // node into the queue IFF we have not already
      // visited that destination and if it is available.
      if(GetGraph().GetNode(edge->GetDes())->IsFlagSet(HasFlags::HF_IS_CONNECTED) ||
         (edge->GetDes() == GetStartNode() && GetAllowDisconnectedStartNode()))
      {
         const GraphEdges& edges = GetGraph().GetEdges(edge->GetDes());
         for(int idx = 0; idx < edges.size(); ++idx)
         {  // If the destination node has not been visited,
            // then add it.
            if(GetVisited()[edges[idx]->GetDes()] == NS_NOT_VISITED)
            {
               if(edges[idx]->IsFlagSet(HasFlags::HF_IS_CONNECTED))
               {
                  _queue.push_back(edges[idx]);
                  GetVisited()[edges[idx]->GetDes()] = NS_VISITED;
               }
            }
         }
      }
      return SS_STILL_WORKING;
   }
   
public:
   GraphSearchBFS(const Graph& graph,
                  uint32 start,
                  uint32 target) :
   GraphSearchAlgorithm(graph,start,target)
   {
   }
};

void TestDFS();
void TestBFS();

#endif /* defined(__GraphCommon__) */
