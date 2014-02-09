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
   
   Graph graph;
   // Add Nodes
   graph.AddNode(GraphNode());
   graph.AddNode(GraphNode());
   graph.AddNode(GraphNode());
   graph.AddNode(GraphNode());
   graph.AddNode(GraphNode());
   graph.AddNode(GraphNode());

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
   graph.EnableEdge(4, 1, false);
   graph.EnableEdge(4, 3, false);
   // Perform the search.
   GraphSearchDFS search(graph,4,2);
   
   
   
   GraphSearchAlgorithm::SEARCH_STATE_T sstate = search.SearchGraph();
   
   switch(sstate)
   {
      case GraphSearchAlgorithm::SS_STILL_WORKING:
         cout << "YIKES...STILL WORKING" << endl;
         break;
      case GraphSearchAlgorithm::SS_NOT_FOUND:
         cout << "YIKES...NO RESULT FOUND" << endl;
         break;
      case GraphSearchAlgorithm::SS_FOUND:
      {
         list<uint32> nodeList = search.GetPathNodes();
         list<Graph::Edge> edgeList = search.GetPathEdges();
         Dump(edgeList);
      }
         break;
   }
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
   
   Graph graph;
   // Add Nodes
   graph.AddNode(GraphNode());
   graph.AddNode(GraphNode());
   graph.AddNode(GraphNode());
   graph.AddNode(GraphNode());
   graph.AddNode(GraphNode());
   graph.AddNode(GraphNode());
   
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
   GraphSearchBFS search(graph,0,5);
   
   //   graph.EnableNode(5, false);
   graph.EnableEdge(0, 2, false);
   graph.EnableEdge(4, 5, false);
   
   GraphSearchAlgorithm::SEARCH_STATE_T sstate = search.SearchGraph();
   
   switch(sstate)
   {
      case GraphSearchAlgorithm::SS_STILL_WORKING:
         cout << "YIKES...STILL WORKING" << endl;
         break;
      case GraphSearchAlgorithm::SS_NOT_FOUND:
         cout << "YIKES...NO RESULT FOUND" << endl;
         break;
      case GraphSearchAlgorithm::SS_FOUND:
      {
         list<uint32> nodeList = search.GetPathNodes();
         list<Graph::Edge> edgeList = search.GetPathEdges();
         Dump(edgeList);
      }
         break;
   }
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

void Dump(list<Graph::Edge>& edgeList)
{
   cout << "Edge List:" << endl;
   for(list<Graph::Edge>::iterator iter = edgeList.begin();
       iter != edgeList.end();
       ++iter)
   {
      iter->Dump();
   }
}
