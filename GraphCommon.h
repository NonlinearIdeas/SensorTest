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
   
   inline bool IsNodeEnabled(uint32 nodeIdx)
   {
      assert(nodeIdx < _nodes.size());
      return _nodes[nodeIdx]->IsFlagSet(HasFlags::HF_IS_CONNECTED);
   }
   
   inline bool IsEdgeEnabled(uint32 src, uint32 des)
   {
      assert(src < _edges.size());
      assert(des < _edges.size());
      GraphEdge* edge = FindEdge(src,des);
      if(edge != NULL)
      {
         return edge->IsFlagSet(HasFlags::HF_IS_CONNECTED);
      }
      return false;
   }

   void EnableEdges(uint32 src, uint32 des, bool enable)
   {
      EnableEdge(src,des,enable);
      EnableEdge(des,src,enable);
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
   
   
   virtual ~GraphSearchAlgorithm() { }
   
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
   
   
   virtual list<const GraphNode*> GetPathNodes()
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


/* This class is a base class for a GoalEvaluator.  This
 * is used by the search algorithm to determine if it has
 * reached its goal node.  Sometimes this may just be the 
 * node has been reached, but other times it may be that the 
 * search is looking for a specific "something" about the 
 * node it is exploring.
 *
 * This evaluator expects to KNOW what type of graph it 
 * is looking at when casting nodes, etc.  Yes, this is
 * potentially a violation of type safety.  Some 
 * improvement will have to be made to address this...
 */
class GoalEvaluator
{
private:
   const Graph& _graph;
protected:
   const Graph& GetGraph() const { return _graph; }
public:
   GoalEvaluator(const Graph& graph) :
   _graph(graph)
   {
      
   }
   
   virtual bool IsGoalMet(uint32 nodeIdx) const
   {
      return false;
   }
};

class NodeIndexGoal : GoalEvaluator
{
   uint32 _nodeIdx;
public:
   NodeIndexGoal(const Graph& graph, uint32 nodeIdx) :
      GoalEvaluator(graph),
      _nodeIdx(nodeIdx)
   {
      
   }

   virtual bool IsGoalMet(uint32 nodeIdx) const
   {
      return nodeIdx == _nodeIdx;
   }
};



// --------- THIS PART WAS LIFTED DIRECTLY FROM MAT BUCLAND'S BOOK --------
//----------------------- Swap -------------------------------------------
//
//  used to swap two values
//------------------------------------------------------------------------
template<class T>
void Swap(T &a, T &b)
{
   T temp = a;
   a = b;
   b = temp;
}

//-------------------- ReorderUpwards ------------------------------------
//
//  given a heap and a node in the heap, this function moves upwards
//  through the heap swapping elements until the heap is ordered
//------------------------------------------------------------------------
template<class T>
void ReorderUpwards(std::vector<T>& heap, int nd)
{
   //move up the heap swapping the elements until the heap is ordered
   while ( (nd>1) && (heap[nd/2] < heap[nd]))
   {
      Swap(heap[nd/2], heap[nd]);
      
      nd /= 2;
   }
}

//--------------------- ReorderDownwards ---------------------------------
//
//  given a heap, the heapsize and a node in the heap, this function
//  reorders the elements in a top down fashion by moving down the heap
//  and swapping the current node with the greater of its two children
//  (provided a child is larger than the current node)
//------------------------------------------------------------------------
template<class T>
void ReorderDownwards(std::vector<T>& heap, int nd, int HeapSize)
{
   //move down the heap from node nd swapping the elements until
   //the heap is reordered
   while (2*nd <= HeapSize)
   {
      int child = 2 * nd;
      
      //set child to largest of nd's two children
      if ( (child < HeapSize) && (heap[child] < heap[child+1]) )
      {
         ++child;
      }
      
      //if this nd is smaller than its child, swap
      if (heap[nd] < heap[child])
      {
         Swap(heap[child], heap[nd]);
         
         //move the current node down the tree
         nd = child;
      }
      
      else
      {
         break;
      }
   }
}



//--------------------- PriorityQ ----------------------------------------
//
//  basic heap based priority queue implementation
//------------------------------------------------------------------------
template<class T>
class PriorityQ
{
private:
   
   std::vector<T>  m_Heap;
   
   int             m_iSize;
   
   int             m_iMaxSize;
   
   //given a heap and a node in the heap, this function moves upwards
   //through the heap swapping elements until the heap is ordered
   void ReorderUpwards(std::vector<T>& heap, int nd)
   {
      //move up the heap swapping the elements until the heap is ordered
      while ( (nd>1) && (heap[nd/2] < heap[nd]))
      {
         Swap(heap[nd/2], heap[nd]);
         
         nd /= 2;
      }
   }
   
   //given a heap, the heapsize and a node in the heap, this function
   //reorders the elements in a top down fashion by moving down the heap
   //and swapping the current node with the greater of its two children
   //(provided a child is larger than the current node)
   void ReorderDownwards(std::vector<T>& heap, int nd, int HeapSize)
   {
      //move down the heap from node nd swapping the elements until
      //the heap is reordered
      while (2*nd <= HeapSize)
      {
         int child = 2 * nd;
         
         //set child to largest of nd's two children
         if ( (child < HeapSize) && (heap[child] < heap[child+1]) )
         {
            ++child;
         }
         
         //if this nd is smaller than its child, swap
         if (heap[nd] < heap[child])
         {
            Swap(heap[child], heap[nd]);
            
            //move the current node down the tree
            nd = child;
         }
         
         else
         {
            break;
         }
      }
   }
   
public:
   
   PriorityQ(int MaxSize):m_iMaxSize(MaxSize), m_iSize(0)
   {
      m_Heap.assign(MaxSize+1, T());
   }
   
   bool empty()const{return (m_iSize==0);}
   
   //to insert an item into the queue it gets added to the end of the heap
   //and then the heap is reordered
   void insert(const T item)
   {
      
      assert (m_iSize+1 <= m_iMaxSize);
      
      ++m_iSize;
      
      m_Heap[m_iSize] = item;
      
      ReorderUpwards(m_Heap, m_iSize);
   }
   
   //to get the max item the first element is exchanged with the lowest
   //in the heap and then the heap is reordered from the top down.
   T pop()
   {
      Swap(m_Heap[1], m_Heap[m_iSize]);
      
      ReorderDownwards(m_Heap, 1, m_iSize-1);
      
      return m_Heap[m_iSize--];
   }
   
   //so we can take a peek at the first in line
   const T& Peek()const{return m_Heap[1];}
};

//--------------------- PriorityQLow -------------------------------------
//
//  basic 2-way heap based priority queue implementation. This time the priority
//  is given to the lowest valued key
//------------------------------------------------------------------------
template<class T>
class PriorityQLow
{
private:
   
   std::vector<T>  m_Heap;
   
   int             m_iSize;
   
   int             m_iMaxSize;
   
   //given a heap and a node in the heap, this function moves upwards
   //through the heap swapping elements until the heap is ordered
   void ReorderUpwards(std::vector<T>& heap, int nd)
   {
      //move up the heap swapping the elements until the heap is ordered
      while ( (nd>1) && (heap[nd/2] > heap[nd]))
      {
         Swap(heap[nd/2], heap[nd]);
         
         nd /= 2;
      }
   }
   
   //given a heap, the heapsize and a node in the heap, this function
   //reorders the elements in a top down fashion by moving down the heap
   //and swapping the current node with the smaller of its two children
   //(provided a child is larger than the current node)
   void ReorderDownwards(std::vector<T>& heap, int nd, int HeapSize)
   {
      //move down the heap from node nd swapping the elements until
      //the heap is reordered
      while (2*nd <= HeapSize)
      {
         int child = 2 * nd;
         
         //set child to largest of nd's two children
         if ( (child < HeapSize) && (heap[child] > heap[child+1]) )
         {
            ++child;
         }
         
         //if this nd is smaller than its child, swap
         if (heap[nd] > heap[child])
         {
            Swap(heap[child], heap[nd]);
            
            //move the current node down the tree
            nd = child;
         }
         
         else
         {
            break;
         }
      }
   }
   
public:
   
   PriorityQLow(int MaxSize):m_iMaxSize(MaxSize), m_iSize(0)
   {
      m_Heap.assign(MaxSize+1, T());
   }
   
   bool empty()const{return (m_iSize==0);}
   
   //to insert an item into the queue it gets added to the end of the heap
   //and then the heap is reordered
   void insert(const T item)
   {
      assert (m_iSize+1 <= m_iMaxSize);
      
      ++m_iSize;
      
      m_Heap[m_iSize] = item;
      
      ReorderUpwards(m_Heap, m_iSize);
   }
   
   //to get the max item the first element is exchanged with the lowest
   //in the heap and then the heap is reordered from the top down.
   T pop()
   {
      Swap(m_Heap[1], m_Heap[m_iSize]);
      
      ReorderDownwards(m_Heap, 1, m_iSize-1);
      
      return m_Heap[m_iSize--];
   }
   
   //so we can take a peek at the first in line
   const T& peek()const{return m_Heap[1];}
};

//----------------------- IndexedPriorityQLow ---------------------------
//
//  Priority queue based on an index into a set of keys. The queue is
//  maintained as a 2-way heap.
//
//  The priority in this implementation is the lowest valued key
//------------------------------------------------------------------------
template<class KeyType>
class IndexedPriorityQLow
{
private:
   
   std::vector<KeyType>&  m_vecKeys;
   
   std::vector<int>       m_Heap;
   
   std::vector<int>       m_invHeap;
   
   int                    m_iSize,
   m_iMaxSize;
   
   void Swap(int a, int b)
   {
      int temp = m_Heap[a]; m_Heap[a] = m_Heap[b]; m_Heap[b] = temp;
      
      //change the handles too
      m_invHeap[m_Heap[a]] = a; m_invHeap[m_Heap[b]] = b;
   }
   
   void ReorderUpwards(int nd)
   {
      //move up the heap swapping the elements until the heap is ordered
      while ( (nd>1) && (m_vecKeys[m_Heap[nd/2]] > m_vecKeys[m_Heap[nd]]) )
      {
         Swap(nd/2, nd);
         
         nd /= 2;
      }
   }
   
   void ReorderDownwards(int nd, int HeapSize)
   {
      //move down the heap from node nd swapping the elements until
      //the heap is reordered
      while (2*nd <= HeapSize)
      {
         int child = 2 * nd;
         
         //set child to smaller of nd's two children
         if ((child < HeapSize) && (m_vecKeys[m_Heap[child]] > m_vecKeys[m_Heap[child+1]]))
         {
            ++child;
         }
         
         //if this nd is larger than its child, swap
         if (m_vecKeys[m_Heap[nd]] > m_vecKeys[m_Heap[child]])
         {
            Swap(child, nd);
            
            //move the current node down the tree
            nd = child;
         }
         
         else
         {
            break;
         }
      }
   }
   
   
public:
   
   //you must pass the constructor a reference to the std::vector the PQ
   //will be indexing into and the maximum size of the queue.
   IndexedPriorityQLow(std::vector<KeyType>& keys,
                       int              MaxSize):m_vecKeys(keys),
   m_iMaxSize(MaxSize),
   m_iSize(0)
   {
      m_Heap.assign(MaxSize+1, 0);
      m_invHeap.assign(MaxSize+1, 0);
   }
   
   bool empty()const{return (m_iSize==0);}
   
   //to insert an item into the queue it gets added to the end of the heap
   //and then the heap is reordered from the bottom up.
   void insert(const int idx)
   {
      assert (m_iSize+1 <= m_iMaxSize);
      
      ++m_iSize;
      
      m_Heap[m_iSize] = idx;
      
      m_invHeap[idx] = m_iSize;
      
      ReorderUpwards(m_iSize);
   }
   
   //to get the min item the first element is exchanged with the lowest
   //in the heap and then the heap is reordered from the top down. 
   int Pop()
   {
      Swap(1, m_iSize);
      
      ReorderDownwards(1, m_iSize-1);
      
      return m_Heap[m_iSize--];
   }
   
   //if the value of one of the client key's changes then call this with 
   //the key's index to adjust the queue accordingly
   void ChangePriority(const int idx)
   {
      ReorderUpwards(m_invHeap[idx]);
   }

};


/* To implement A* and Dijkstra, we will need a class
 * to manage the "Open List", which is a bottleneck
 * in the processing.
 *
 * Many implementations use an indexed priority queue.
 *
 * For now, the class below will wrap the open list
 * so that we can explore different algorithms without
 * changing the interface.
 */

class OpenList
{
private:
   IndexedPriorityQLow<double>* _pq;
   vector<double>* _costToNode;
   
public:
   OpenList() :
      _pq(NULL),
      _costToNode(NULL)
   {
      
   }
   
   ~OpenList()
   {
      if(_pq != NULL)
      {
         delete _pq;
         _pq = NULL;
      }
   }
   
   void Init(vector<double>& costToNode)
   {
      if(_pq != NULL)
      {
         delete _pq;
         _pq = NULL;
      }
      _costToNode = &costToNode;
      _pq = new IndexedPriorityQLow<double>(costToNode,costToNode.size());
   }
   
   void Insert(int32 index)
   {
      assert(_pq != NULL);
      _pq->insert(index);
   }
   
   void NodeCostChanged(int32 index)
   {
      assert(_pq != NULL);
      assert(_costToNode != NULL);
      assert(index < _costToNode->size());
      _pq->ChangePriority(index);
   }
   
   int32 GetLowestCostNodeIndex()
   {
      assert(_pq != NULL);
      return _pq->Pop();
   }
   
   bool IsEmpty()
   {
      assert(_pq != NULL);
      return _pq->empty();
   }
   
};

class GraphSearchDijkstra : public GraphSearchAlgorithm
{
private:
   vector<const GraphEdge*> _shortestPathTree;
   vector<const GraphEdge*> _searchFrontier;
   vector<double> _costToNode;
   OpenList _openList;
   
protected:
   virtual SEARCH_STATE_T SearchCycleStart()
   {
      GetVisited()[GetStartNode()] = NS_VISITED;
      _costToNode.resize(GetGraph().GetNodeCount());
      _costToNode.assign(_costToNode.size(), 0.0);
      _shortestPathTree.resize(GetGraph().GetNodeCount());
      _shortestPathTree.assign(_shortestPathTree.size(),NULL);
      _searchFrontier.resize(GetGraph().GetNodeCount());
      _searchFrontier.assign(_searchFrontier.size(),NULL);
      
      _openList.Init(_costToNode);
      _openList.Insert(GetStartNode());
      return SS_STILL_WORKING;
   }
   
   
   virtual SEARCH_STATE_T SearchCycle()
   {
      if(_openList.IsEmpty())
      {
         return SS_NOT_FOUND;
      }
      int32 nextNode = _openList.GetLowestCostNodeIndex();
      _shortestPathTree[nextNode] = _searchFrontier[nextNode];
      if(nextNode == GetTargetNode())
      {
         return SS_FOUND;
      }
#ifdef DEBUG_FLOODING
      // Add this for flood debugging.
      AddToFlood(nextNode);
#endif
      // No.  Push all the edges that lead to the des
      // node into the queue IFF we have not already
      // visited that destination and if it is available.
      if(GetGraph().GetNode(nextNode)->IsFlagSet(HasFlags::HF_IS_CONNECTED) ||
         (nextNode == GetStartNode() && GetAllowDisconnectedStartNode()))
      {
         const GraphEdges& edges = GetGraph().GetEdges(nextNode);
         for(int idx = 0; idx < edges.size(); ++idx)
         {
            GraphEdge* edge = edges[idx];
            double cost = _costToNode[nextNode] + edge->GetCost();
            if(_searchFrontier[edge->GetDes()] == NULL)
            {  // Never been visited
               _costToNode[edge->GetDes()] = cost;
               _openList.Insert(edge->GetDes());
               _searchFrontier[edge->GetDes()] = edge;
            }
            else if((cost < _costToNode[edge->GetDes()]) &&
                    (_shortestPathTree[edge->GetDes()] == NULL)
                    )
               
            {
               _costToNode[edge->GetDes()] = cost;
               _openList.NodeCostChanged(edge->GetDes());
               _searchFrontier[edge->GetDes()] = edge;
            }
         }
      }
      return SS_STILL_WORKING;
   }
   
public:
   GraphSearchDijkstra(const Graph& graph,
                  uint32 start,
                  uint32 target) :
   GraphSearchAlgorithm(graph,start,target)
   {
   }
   
   virtual ~GraphSearchDijkstra()
   {
   }
   
   virtual list<const GraphNode*> GetPathNodes()
   {
      list<const GraphNode*> path;
      
      if(GetSearchState() != SS_FOUND)
      {
         return path;
      }
      
      uint32 nodeIdx = GetTargetNode();
      path.push_front(GetGraph().GetNode(nodeIdx));
      while(nodeIdx != GetStartNode() && _shortestPathTree[nodeIdx] != NULL)
      {
         nodeIdx = _shortestPathTree[nodeIdx]->GetSrc();
         path.push_front(GetGraph().GetNode(nodeIdx));
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
      uint32 srcNode = _shortestPathTree[desNode]->GetSrc();
      
      path.push_front(GetGraph().GetEdge(srcNode, desNode));
      while(srcNode != GetStartNode())
      {
         desNode = srcNode;
         srcNode = _shortestPathTree[desNode]->GetSrc();
         path.push_front(GetGraph().GetEdge(srcNode, desNode));
      }
      
      return path;
   }
   
};

class AStarHeuristic
{
public:
   virtual double CalculateCost(const Graph& graph, uint32 srcNode, uint32 desNode) const
   {
      return 0.0;
   }
};

class AStarHeuristic_Distance : public AStarHeuristic
{
   virtual double CalculateCost(const Graph& graph, uint32 srcNode, uint32 desNode) const
   {
      NavGraphNode* src = (NavGraphNode*)graph.GetNode(srcNode);
      NavGraphNode* des = (NavGraphNode*)graph.GetNode(desNode);
      Vec2 dir = src->GetPos() - des->GetPos();
      return dir.Length();
   }
};

class AStarHeuristic_DistanceSquared : public AStarHeuristic
{
   virtual double CalculateCost(const Graph& graph, uint32 srcNode, uint32 desNode) const
   {
      NavGraphNode* src = (NavGraphNode*)graph.GetNode(srcNode);
      NavGraphNode* des = (NavGraphNode*)graph.GetNode(desNode);
      Vec2 dir = src->GetPos() - des->GetPos();
      return dir.LengthSquared();
   }
};

class GraphSearchAStar : public GraphSearchAlgorithm
{
private:
   vector<const GraphEdge*> _shortestPathTree;
   vector<const GraphEdge*> _searchFrontier;
   vector<double> _FCostToNode;
   vector<double> _GCostToNode;
   OpenList _openList;
   const AStarHeuristic* _heuristic;
   
protected:
   virtual SEARCH_STATE_T SearchCycleStart()
   {
      _FCostToNode.resize(GetGraph().GetNodeCount());
      _FCostToNode.assign(_FCostToNode.size(), 0.0);
      _GCostToNode.resize(GetGraph().GetNodeCount());
      _GCostToNode.assign(_GCostToNode.size(), 0.0);
      _shortestPathTree.resize(GetGraph().GetNodeCount());
      _shortestPathTree.assign(_shortestPathTree.size(),NULL);
      _searchFrontier.resize(GetGraph().GetNodeCount());
      _searchFrontier.assign(_searchFrontier.size(),NULL);
      
      _openList.Init(_FCostToNode);
      _openList.Insert(GetStartNode());
      return SS_STILL_WORKING;
   }
   
   
   virtual SEARCH_STATE_T SearchCycle()
   {
      if(_openList.IsEmpty())
      {
         return SS_NOT_FOUND;
      }
      int32 nextNode = _openList.GetLowestCostNodeIndex();
      _shortestPathTree[nextNode] = _searchFrontier[nextNode];
      if(nextNode == GetTargetNode())
      {
         return SS_FOUND;
      }
#ifdef DEBUG_FLOODING
      // Add this for flood debugging.
      AddToFlood(nextNode);
#endif
      // No.  Push all the edges that lead to the des
      // node into the queue IFF we have not already
      // visited that destination and if it is available.
      if(GetGraph().GetNode(nextNode)->IsFlagSet(HasFlags::HF_IS_CONNECTED) ||
         (nextNode == GetStartNode() && GetAllowDisconnectedStartNode()))
      {
         const GraphEdges& edges = GetGraph().GetEdges(nextNode);
         for(int idx = 0; idx < edges.size(); ++idx)
         {
            GraphEdge* edge = edges[idx];
            
            double hCost = _heuristic->CalculateCost(GetGraph(), GetTargetNode(), edge->GetDes());
            double gCost = _GCostToNode[nextNode] + edge->GetCost();
            
            if(_searchFrontier[edge->GetDes()] == NULL)
            {  // Never been visited
               // F = G + H
               _FCostToNode[edge->GetDes()] = gCost + hCost;
               _GCostToNode[edge->GetDes()] = gCost;
               _openList.Insert(edge->GetDes());
               _searchFrontier[edge->GetDes()] = edge;
            }
            else if((gCost < _GCostToNode[edge->GetDes()]) &&
                    (_shortestPathTree[edge->GetDes()] == NULL)
                    )
               
            {
               _FCostToNode[edge->GetDes()] = gCost + hCost;
               _GCostToNode[edge->GetDes()] = gCost;
               _openList.NodeCostChanged(edge->GetDes());
               _searchFrontier[edge->GetDes()] = edge;
            }
         }
      }
      return SS_STILL_WORKING;
   }
   
public:
   GraphSearchAStar(const Graph& graph,
                       uint32 start,
                       uint32 target,
                       const AStarHeuristic* heuristic) :
   GraphSearchAlgorithm(graph,start,target),
   _heuristic(heuristic)
   {
   }
   
   virtual ~GraphSearchAStar()
   {
   }
   
   virtual list<const GraphNode*> GetPathNodes()
   {
      list<const GraphNode*> path;
      
      if(GetSearchState() != SS_FOUND)
      {
         return path;
      }
      
      uint32 nodeIdx = GetTargetNode();
      path.push_front(GetGraph().GetNode(nodeIdx));
      while(nodeIdx != GetStartNode() && _shortestPathTree[nodeIdx] != NULL)
      {
         nodeIdx = _shortestPathTree[nodeIdx]->GetSrc();
         path.push_front(GetGraph().GetNode(nodeIdx));
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
      uint32 srcNode = _shortestPathTree[desNode]->GetSrc();
      
      path.push_front(GetGraph().GetEdge(srcNode, desNode));
      while(srcNode != GetStartNode())
      {
         desNode = srcNode;
         srcNode = _shortestPathTree[desNode]->GetSrc();
         path.push_front(GetGraph().GetEdge(srcNode, desNode));
      }
      
      return path;
   }
   
};



void TestDFS();
void TestBFS();

#endif /* defined(__GraphCommon__) */
