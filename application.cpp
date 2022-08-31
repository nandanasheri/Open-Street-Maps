// application.cpp
// Name : Nandana Sheri
// University of Illinois at Chicago
// CS 251: Spring 2022
// Project #7 - Openstreet Maps
// application.cpp : Program main file which builds the application of a graph
// which creates bidirectional edges from each node of a footway to each
// building and performs Dijkstra's Algorithm and checks whether each person is
// able to reach a certain destination building which is the center between the
// two persons. If it is not reachable it resets and checks for next destination
/* CREATIVE COMPONENT :
   Asks user for input for number of people that is planning to meet at a common
   point. Asks for each person's building and asks for a common destination as
   well. It further performs Dikstra's Algorithm on every single person's
   building and then checks whether each person can make it to the destination
   building. If all persons can meet at the destination, then it is a valid
   destination point. If even one person cannot reach there, the program ouputs
   which person cannot reach that common destination.
*/
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iomanip> /*setprecision*/
#include <iostream>
#include <map>
#include <queue>
#include <stack>
#include <string>
#include <vector>

#include "dist.h"
#include "graph.h"
#include "osm.h"
#include "tinyxml2.h"
using namespace std;
using namespace tinyxml2;
const double INF = numeric_limits<double>::max();

class prioritize  // For the priority queue
{
 public:
  bool operator()(const pair<long long, double> &p1,
                  const pair<long long, double> &p2) const {
    return p1.second > p2.second;
  }
};

// Function which takes in a value of a string and returns the Building Info for
// that certain building
BuildingInfo searchBuilding(string query,
                            const vector<BuildingInfo> &buildings) {
  for (auto &e : buildings) {
    if (query ==
        e.Abbrev) {  // Checks whether string is the abbreviation of a building
      return e;
    }
  }
  for (auto &f : buildings) {
    if (f.Fullname.find(query) != string::npos) {
      return f;
    }
  }
  BuildingInfo nullB;
  nullB.Fullname = "";
  return nullB;
}

// Function which returns the closest building to the coordinate given
BuildingInfo nearestBuilding(Coordinates midpoint,
                             const vector<BuildingInfo> &buildings,
                             const set<string> &unreachableBuildings) {
  double min = INF;
  BuildingInfo nearest;
  for (auto &e : buildings) {
    if (unreachableBuildings.count(
            e.Fullname)) {  // If building is in unreachable building, it cannot
                            // be the destination
      continue;
    }
    Coordinates cord = e.Coords;
    double distance =
        distBetween2Points(cord.Lat, cord.Lon, midpoint.Lat, midpoint.Lon);
    if (distance < min) {
      min = distance;
      nearest = e;
    }
  }
  return nearest;
}

// Function which returns the nearest node to a certain building passed in as
// parameter
long long nearestNode(BuildingInfo b, const map<long long, Coordinates> &Nodes,
                      const vector<FootwayInfo> &Footways) {
  double min = INF;
  long long nearestNode;
  Coordinates bcord = b.Coords;
  for (auto &f : Footways) {
    vector<long long> nodes =
        f.Nodes;  // Stores a series of nodes which make up a footway
    for (long unsigned int i = 0; i < nodes.size(); i++) {
      Coordinates c = Nodes.at(nodes[i]);
      double distance = distBetween2Points(c.Lat, c.Lon, bcord.Lat, bcord.Lon);
      if (distance < min) {
        min = distance;
        nearestNode = nodes[i];
      }
    }
  }
  return nearestNode;
}

// Dijkstra's Algorithm to find shortest weighted path between two nodes in a
// graph
void DijkstraShortestPath(graph<long long, double> G, long long startV,
                          map<long long, double> &distances,
                          map<long long, long long> &predecessors) {
  priority_queue<pair<long long, double>, vector<pair<long long, double>>,
                 prioritize>
      unvisitedQ;
  vector<long long> visited;
  vector<long long> vertices =
      G.getVertices();  // Returns a vector of all vertices in the graph
  for (auto &e : vertices) {
    distances[e] = INF;
    predecessors[e] = 0;
    unvisitedQ.push(
        make_pair(e, INF));  // Each vertex is initially pushed into the queue
                             // with a distance of infinity
  }
  distances[startV] = 0;
  unvisitedQ.push(
      make_pair(startV, 0));  // The building has a distance of zero from itself
  while (!unvisitedQ.empty()) {  // When a node existed in the queue
    long long currentNode = unvisitedQ.top().first;
    unvisitedQ.pop();
    if (distances[currentNode] ==
        INF) {  // If distance is inifnity, break out of the loop
      break;
    } else if (count(visited.begin(), visited.end(),
                     currentNode)) {  // If currentNode has been visited
      continue;
    } else {
      visited.push_back(currentNode);
    }
    set<long long> neighbors =
        G.neighbors(currentNode);  // Returns neighbours of the current node
    for (auto &adjV : neighbors) {
      double edgeWeight;
      G.getWeight(currentNode, adjV,
                  edgeWeight);  // Returns edgeweight of each adjacent node
      double alternativePathDist = edgeWeight + distances[currentNode];
      if (alternativePathDist <
          distances[adjV]) {  // If alternative path distance is the minimum
        distances[adjV] =
            alternativePathDist;  // The distance to adjacent vertex is now the
                                  // alternate distance
        predecessors[adjV] =
            currentNode;  // Predecessor of adjacent node is current node
        unvisitedQ.push(make_pair(adjV, alternativePathDist));
      }
    }
  }
}
// Function which implements a stack to get a path using the predecessors array
// of the path that Dijkstra's ALgorithm does
void getPath(long long endV, map<long long, long long> predecessors,
             vector<long long> &path) {
  stack<long long> pathStack;
  long long currV = endV;
  while (currV != 0) {
    pathStack.push(currV);
    currV = predecessors[currV];
  }
  while (!pathStack.empty()) {
    currV = pathStack.top();
    pathStack.pop();
    path.push_back(currV);
  }
}

// Creative Component :
void creative(map<long long, Coordinates> &Nodes, vector<FootwayInfo> &Footways,
              vector<BuildingInfo> &Buildings, graph<long long, double> G) {
  vector<BuildingInfo> listOfBuildings;
  int numOfPeople = 0;
  cout
      << "Enter the number of people who need to meet at a common destination ";
  cin >> numOfPeople;
  string empty;
  getline(cin, empty);
  for (int i = 0; i < numOfPeople;
       i++) {  // Populates the vector with all buildings of people
    string personBuilding;
    cout << "Enter building (partial name or abbreviation) for Person No. "
         << i + 1 << " ";
    getline(cin, personBuilding);
    BuildingInfo building = searchBuilding(personBuilding, Buildings);
    if (building.Fullname == "") {
      cout << "Building not found" << endl;
      continue;
    }
    listOfBuildings.push_back(building);
  }
  string center;
  BuildingInfo centerBuilding;
  while (centerBuilding.Fullname == "") {
    cout << "Enter the common destination for all people meeting ";
    getline(cin, center);
    centerBuilding = searchBuilding(center, Buildings);
    if (centerBuilding.Fullname == "") {
      cout << "Center Building not found. Retry " << endl;
    }
  }
  for (long unsigned int i = 0; i < listOfBuildings.size(); i++) {
    BuildingInfo building = listOfBuildings.at(i);
    cout << "Person " << i + 1 << "'s point:" << endl;
    cout << " " << building.Fullname << endl;
    cout << " (" << building.Coords.Lat << ", " << building.Coords.Lon << ")"
         << endl;
  }
  cout << "Destination Building:" << endl;
  cout << " " << centerBuilding.Fullname << endl;
  cout << " (" << centerBuilding.Coords.Lat << ", " << centerBuilding.Coords.Lon
       << ")" << endl;
  cout << endl;
  for (long unsigned int i = 0; i < listOfBuildings.size(); i++) {
    BuildingInfo building = listOfBuildings.at(i);
    long long nearestNodeB = nearestNode(building, Nodes, Footways);
    cout << "Nearest node for P" << i + 1 << ": " << endl;
    cout << " " << nearestNodeB << endl;
    cout << " (" << Nodes[nearestNodeB].Lat << ", " << Nodes[nearestNodeB].Lon
         << ")" << endl;
  }
  long long nearestNodeMidB = nearestNode(centerBuilding, Nodes, Footways);
  for (long unsigned int i = 0; i < listOfBuildings.size(); i++) {
    BuildingInfo building = listOfBuildings.at(i);
    long long nearestNodeB = nearestNode(building, Nodes, Footways);
    map<long long, double>
        distancesB;  // Declaring distances array for building 1
    map<long long, long long>
        predecessorsB;  // Declaring predecessors array for building 1
    DijkstraShortestPath(
        G, nearestNodeB, distancesB,
        predecessorsB);  // Returns distances and predecessors from start vertex
    if (distancesB[nearestNodeMidB] >= INF) {  // Cannot reach destination
      cout << "Person No. " << i + 1 << " cannot reach destination." << endl;
      return;
    }
  }
  cout << "All people can meet at the common destination. " << endl;
}

//
// Standard Application
//
void application(map<long long, Coordinates> &Nodes,
                 vector<FootwayInfo> &Footways, vector<BuildingInfo> &Buildings,
                 graph<long long, double> G) {
  string person1Building, person2Building;
  set<string> unreachableBuildings;
  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);
    bool retry = false;
    BuildingInfo building1 = searchBuilding(
        person1Building, Buildings);  // Returns the building of first person
    BuildingInfo building2 =
        searchBuilding(person2Building,
                       Buildings);  // Returns the building of the second person
    if (building1.Fullname == "") {
      cout << "Person 1's building not found" << endl;
      cout
          << "Enter person 1's building (partial name or abbreviation), or #> ";
      getline(cin, person1Building);
      continue;
    } else if (building2.Fullname == "") {
      cout << "Person 2's building not found" << endl;
      cout
          << "Enter person 1's building (partial name or abbreviation), or #> ";
      getline(cin, person1Building);
      continue;
    }
    while (true) {
      Coordinates c1 = building1.Coords;
      Coordinates c2 = building2.Coords;
      Coordinates midpoint = centerBetween2Points(
          c1.Lat, c1.Lon, c2.Lat,
          c2.Lon);  // Returns a midpoint coordinate for both persons
      BuildingInfo centerBuilding = nearestBuilding(
          midpoint, Buildings,
          unreachableBuildings);  // Returns building at the center
      if (centerBuilding.Fullname == building1.Fullname ||
          centerBuilding.Fullname == building2.Fullname) {
        centerBuilding = building2;
      }
      long long nearestNodeB1 = nearestNode(
          building1, Nodes,
          Footways);  // Returns nearest nodes to all three buildings
      long long nearestNodeB2 = nearestNode(building2, Nodes, Footways);
      long long nearestNodeMidB = nearestNode(centerBuilding, Nodes, Footways);
      if (!retry) {
        cout << "Person 1's point:" << endl;
        cout << " " << building1.Fullname << endl;
        cout << " (" << c1.Lat << ", " << c1.Lon << ")" << endl;
        cout << "Person 2's point:" << endl;
        cout << " " << building2.Fullname << endl;
        cout << " (" << c2.Lat << ", " << c2.Lon << ")" << endl;
        cout << "Destination Building:" << endl;
        cout << " " << centerBuilding.Fullname << endl;
        cout << " (" << centerBuilding.Coords.Lat << ", "
             << centerBuilding.Coords.Lon << ")" << endl;
        cout << endl;
        cout << "Nearest P1 node:" << endl;
        cout << " " << nearestNodeB1 << endl;
        cout << " (" << Nodes[nearestNodeB1].Lat << ", "
             << Nodes[nearestNodeB1].Lon << ")" << endl;
        cout << "Nearest P2 node:" << endl;
        cout << " " << nearestNodeB2 << endl;
        cout << " (" << Nodes[nearestNodeB2].Lat << ", "
             << Nodes[nearestNodeB2].Lon << ")" << endl;
        cout << "Nearest destination node:" << endl;
        cout << " " << nearestNodeMidB << endl;
        cout << " (" << Nodes[nearestNodeMidB].Lat << ", "
             << Nodes[nearestNodeMidB].Lon << ")" << endl;
        cout << endl;
      } else {
        cout << "New destination building:" << endl;
        cout << " " << centerBuilding.Fullname << endl;
        cout << " (" << centerBuilding.Coords.Lat << ", "
             << centerBuilding.Coords.Lon << ")" << endl;
        cout << endl;
        cout << "Nearest destination node:" << endl;
        cout << " " << nearestNodeMidB << endl;
        cout << " (" << Nodes[nearestNodeMidB].Lat << ", "
             << Nodes[nearestNodeMidB].Lon << ")" << endl;
        cout << endl;
      }
      map<long long, double>
          distancesB1;  // Declaring distances array for building 1
      map<long long, long long>
          predecessorsB1;  // Declaring predecessors array for building 1
      map<long long, double> distancesB2;
      map<long long, long long> predecessorsB2;
      DijkstraShortestPath(G, nearestNodeB1, distancesB1,
                           predecessorsB1);  // Returns distances and
                                             // predecessors from start vertex
      DijkstraShortestPath(G, nearestNodeB2, distancesB2, predecessorsB2);
      if (distancesB1[nearestNodeB2] >=
          INF) {  // Building 2 is not reachable from Building 1
        cout << "Sorry, destination unreachable." << endl;
        cout << endl;
        cout << "Enter person 1's building (partial name or abbreviation), or "
                "#> ";
        getline(cin, person1Building);
        cout << "Enter person 2's building (partial name or abbreviation)> ";
        getline(cin, person2Building);
        building1 =
            searchBuilding(person1Building,
                           Buildings);  // Returns the building of first person
        building2 = searchBuilding(
            person2Building,
            Buildings);  // Returns the building of the second person
        if (building1.Fullname == "") {
          cout << "Person 1's building not found" << endl;
          continue;
        } else if (building2.Fullname == "") {
          cout << "Person 2's building not found" << endl;
        }
      } else if (distancesB1[nearestNodeMidB] >= INF ||
                 distancesB2[nearestNodeMidB] >=
                     INF) {  // Either cannot reach destination building
        unreachableBuildings.insert(
            centerBuilding
                .Fullname);  // Adding Building to unreachable buildings
        retry = true;
        cout << "At least one person was unable to reach the destination "
                "building. Finding next closest building..."
             << endl;
        cout << endl;
        continue;
      } else {  // Reachable path
        vector<long long> pathVectorB1;
        vector<long long> pathVectorB2;
        getPath(nearestNodeMidB, predecessorsB2,
                pathVectorB2);  // Return path from person 1 to destination
        getPath(nearestNodeMidB, predecessorsB1,
                pathVectorB1);  // Return path from person 1 to destination
        string pathB1 = "";
        string pathB2 = "";
        long unsigned int i, j;
        cout << "Person 1's distance to dest: " << distancesB1[nearestNodeMidB]
             << " miles" << endl;
        cout << "Path: ";
        for (i = 0; i < pathVectorB1.size() - 1; i++) {
          cout << pathVectorB1.at(i) << "->";
        }
        cout << pathVectorB1.at(i)
             << endl;  // To not add a "->" at the end node
        cout << endl;
        cout << "Person 2's distance to dest: " << distancesB2[nearestNodeMidB]
             << " miles" << endl;
        cout << "Path: ";
        for (j = 0; j < pathVectorB2.size() - 1; j++) {
          cout << pathVectorB2.at(j) << "->";
        }
        cout << pathVectorB2.at(j) << endl;
        break;  // Path found! break out of the loop
      }
    }
    cout << endl;
    unreachableBuildings.clear();
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}

int main() {
  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates> Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo> Footways;
  // info about each building, in no particular order
  vector<BuildingInfo> Buildings;
  XMLDocument xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;
  graph<long long, double> G;
  for (auto &e : Nodes) {  // Add each NODE ID as a vertex to the graph
    G.addVertex(e.first);
  }
  for (auto &f : Footways) {
    vector<long long> nodes =
        f.Nodes;  // Stores a series of nodes which make up a footway
    for (long unsigned int i = 0; i < nodes.size() - 1; i++) {
      Coordinates c1 = Nodes.at(nodes[i]);
      Coordinates c2 = Nodes.at(nodes[i + 1]);
      double distance = distBetween2Points(c1.Lat, c1.Lon, c2.Lat, c2.Lon);
      G.addEdge(
          nodes[i], nodes[i + 1],
          distance);  // Adding bidirectional weighted edge inbetween two nodes
      G.addEdge(nodes[i + 1], nodes[i], distance);
    }
  }
  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  //
  // Menu
  //
  string userInput;
  cout << "Enter \"a\" for the standard application or "
       << "\"c\" for the creative component application> ";
  getline(cin, userInput);
  if (userInput == "a") {
    application(Nodes, Footways, Buildings, G);
  } else if (userInput == "c") {
    creative(Nodes, Footways, Buildings, G);
  }
  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}
