/********************************************************************
 * File   : GraphCommon.cpp
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


#include "GraphCommon.h"

class NavGraphNode : public GraphNode
{
private:
   Vec2 _pos;
   
public:
   const Vec2& GetPos() { return _pos; }
   void SetPos(const Vec2& pos) { _pos = pos; }
   
   NavGraphNode(uint32 ID) :
      GraphNode(ID)
   {
      
   }
};

void TestDFS()
{
   /* A Simple Graph with 6 nodes:
    *
    * 0 ==> 1, 2
    * 1 ==> 0, 4
    * 2 ==> 0, 3
    * 3 ==> 2, 4, 5
    * 4 ==> 1, 3, 5
    * 5 ==> 3, 4
    */
   
   Graph<NavGraphNode,GraphEdge> graph;
   // Add Nodes
   graph.AddNode(NavGraphNode(1));
   graph.AddNode(NavGraphNode(2));
   graph.AddNode(NavGraphNode(3));
   graph.AddNode(NavGraphNode(4));
   graph.AddNode(NavGraphNode(5));
   graph.AddNode(NavGraphNode(6));

   // Add Edges
   graph.AddEdge(GraphEdge(0,1));
   graph.AddEdge(GraphEdge(0,2));
   graph.AddEdge(GraphEdge(1,0));
   graph.AddEdge(GraphEdge(1,4));
   graph.AddEdge(GraphEdge(2,0));
   graph.AddEdge(GraphEdge(2,3));
   graph.AddEdge(GraphEdge(3,2));
   graph.AddEdge(GraphEdge(3,4));
   graph.AddEdge(GraphEdge(3,5));
   graph.AddEdge(GraphEdge(4,5));
   graph.AddEdge(GraphEdge(4,3));
   graph.AddEdge(GraphEdge(4,1));
   graph.AddEdge(GraphEdge(5,3));
   graph.AddEdge(GraphEdge(5,4));
   
   // Disable a node
   //graph.EnableNode(4, false);
   //graph.EnableEdge(1, 0, false);
   graph.EnableEdges(4, 1, false);
   graph.EnableEdges(4, 3, false);
   // Perform the search.
   GraphSearchDFS<NavGraphNode,GraphEdge> search(graph,4,2);
   
   search.SearchGraph();
   search.Dump();
}

void TestBFS()
{
   /* A Simple Graph with 6 nodes:
    *
    * 0 ==> 1, 2
    * 1 ==> 0, 4
    * 2 ==> 0, 3
    * 3 ==> 2, 4. 5
    * 4 ==> 1, 3, 5
    * 5 ==> 3, 4
    */
   
   Graph<NavGraphNode,GraphEdge> graph;
   // Add Nodes
   graph.AddNode(NavGraphNode(1));
   graph.AddNode(NavGraphNode(2));
   graph.AddNode(NavGraphNode(3));
   graph.AddNode(NavGraphNode(4));
   graph.AddNode(NavGraphNode(5));
   graph.AddNode(NavGraphNode(6));
   
   // Add Edges
   graph.AddEdge(GraphEdge(0,1));
   graph.AddEdge(GraphEdge(0,2));
   graph.AddEdge(GraphEdge(1,0));
   graph.AddEdge(GraphEdge(1,4));
   graph.AddEdge(GraphEdge(2,0));
   graph.AddEdge(GraphEdge(2,3));
   graph.AddEdge(GraphEdge(3,2));
   graph.AddEdge(GraphEdge(3,4));
   graph.AddEdge(GraphEdge(3,5));
   graph.AddEdge(GraphEdge(4,5));
   graph.AddEdge(GraphEdge(4,3));
   graph.AddEdge(GraphEdge(4,1));
   graph.AddEdge(GraphEdge(5,3));
   graph.AddEdge(GraphEdge(5,4));
   
   // Perform the search.
   GraphSearchBFS<NavGraphNode,GraphEdge> search(graph,0,5);
   
   //   graph.EnableNode(5, false);
   graph.EnableEdge(0, 2, false);
   graph.EnableEdge(4, 5, false);
   
   search.SearchGraph();
   search.Dump();
}


void Dump(list<uint32>& nodeList)
{
   cout << "Node List:" << endl;
   for(list<uint32>::iterator iter = nodeList.begin();
       iter != nodeList.end();
       ++iter)
   {
      cout << "Node(" << *iter+1 << ")" << endl;
   }
}


