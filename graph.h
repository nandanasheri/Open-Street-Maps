// graph.h
/* Name : Nandana Sheri
   Course : CS 251 Spring 2022
   Date : 14/04/2022
   Project Description : Implementation of graph.h using adjacency list for
   openstreets map implementation
*/
//
// Project #7 - Openstreet Maps
//

#pragma once

#include <algorithm>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <vector>

using namespace std;
template <typename VertexT, typename WeightT>
class graph {
 private:
  typedef unordered_map<VertexT, WeightT>
      vwMap;  // Map to account for to vertices and wights
  unordered_map<VertexT, vwMap> adjList;  // Adjacency list to keep track of
                                          // each vertex and its neighbours
  int edges = 0;

 public:
  //
  // constructor:
  //
  // Constructs an empty graph
  //
  graph() {}

  // clear
  // clears the graph and deallocates all memory associated with it

  // void clear () {
  //   adjList.clear();
  // }

  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const { return adjList.size(); }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const { return edges; }

  //
  // addVertex
  //
  // Adds the vertex v to the graph with no edges and if so returns true. If
  // vertex already exists in graph return false.
  //
  bool addVertex(VertexT v) {
    if (adjList.count(v)) {  // Vertex already exists in the graph
      return false;
    }
    adjList[v];  // Initialize the map's value to be an empty map since it has
                 // no edges.
    return true;
  }

  //
  // addEdge
  //
  // Adds the edge (from, to, weight) to the graph, and returns
  // true.
  //
  // NOTE: if the edge already exists, the existing edge weight
  // is overwritten with the new edge weight.
  //
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    if (adjList.count(from) == 0) {
      return false;
    }
    if (adjList.count(to) == 0) {
      return false;
    }
    if (!adjList.at(from).count(to)) {  // An edge already exists
      edges++;
    }
    adjList[from][to] =
        weight;  // If does not exist, the map creates an edge of this weight
    // If edge exists, it overwrites the value. Both are taken care of by [] box
    // operator
    return true;
  }

  //
  // getWeight
  //
  // Returns the weight associated with a given edge.  If
  // the edge exists, the weight is returned via the reference
  // parameter and true is returned.  If the edge does not
  // exist, the weight parameter is unchanged and false is
  // returned.
  //
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    if (!adjList.count(from)) {
      return false;
    }
    if (!adjList.count(to)) {
      return false;
    }
    vwMap edges = adjList.at(from);
    if (edges.count(to)) {  // If edge exists, update weight
      weight = edges.at(to);
      return true;
    } else {  // Edge does not exist
      return false;
    }
  }

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order; use foreach to iterate through the set.
  //
  set<VertexT> neighbors(VertexT v) const {
    if (!adjList.count(v)) {
      return {};
    } else {
      set<VertexT> S;
      vwMap edges = adjList.at(v);
      for (auto e :
           edges) {  // Loop through edges of the vertex and insert to the set
        VertexT to = e.first;
        S.insert(to);
      }
      return S;
    }
  }

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
    vector<VertexT> vertices;
    for (auto e : adjList) {
      vertices.push_back(e.first);
    }
    return vertices;
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //
  void dump(ostream& output) const {
    //   output << "***************************************************" <<
    //   endl; output << "********************* GRAPH ***********************"
    //   << endl;

    //   output << "**Num vertices: " << this->NumVertices() << endl;
    //   output << "**Num edges: " << this->NumEdges() << endl;

    //   output << endl;
    //   output << "**Vertices:" << endl;
    //   for (int i = 0; i < this->NumVertices(); ++i) {
    //     output << " " << i << ". " << this->vertices[i] << endl;
    //   }

    //   output << endl;
    //   output << "**Edges:" << endl;
    //   for (auto &row : adjList) {
    //     output << " row " << row.first << ": ";
    //     vwMap edges = row.second;
    //     for (auto &col : edges) {
    //       VertexT to = col.first;
    //       output << "(" << to << ", "
    //       << col.second
    //       << ") ";
    //     }
    //     output << endl;
    //   }
    //   output << "**************************************************" << endl;
  }
};
