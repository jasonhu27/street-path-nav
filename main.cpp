/*main.cpp*/

//
// Program to input Nodes (positions), Buildings and Footways
// from an Open Street Map file.
// 
// Prof. Joe Hummel
// Northwestern University
// CS 211: Winter 2023
// 

#include <iostream>
#include <string>
#include <iomanip>
#include <limits>
#include <queue>
#include <stack>

#include "building.h"
#include "buildings.h"
#include "footway.h"
#include "footways.h"
#include "node.h"
#include "nodes.h"
#include "osm.h"
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

//
// footwaysGraph
//
// adds the edges between nodes into the graph
//
void footwaysGraph(graph& G, Footways& footways, Nodes& nodes){
  double lat1, lat2;
  double lon1, lon2;
  bool ent1, ent2;
  
  // loops through all of the footway in the MapFootways vector
  for(Footway f : footways.MapFootways){

    // loops through all of the nodes except for the last in the footway NodeID vector
    for(unsigned int i = 0; i < f.NodeIDs.size() - 1; i++){
      // finds the latitude and longitude of that node and the next
      nodes.find(f.NodeIDs[i], lat1, lon1, ent1);
      nodes.find(f.NodeIDs[i + 1], lat2, lon2, ent2);
      // finds distance between the two and pushes it as an edge in the forward direction
      // i.e 1->2, 2->3, 3->4 etc.
      double dist = distBetween2Points(lat1, lon1, lat2, lon2);
      G.addEdge(f.NodeIDs[i], f.NodeIDs[i + 1], dist);
    }

    // loops through all the nodes in the footway NodeID vector except for the first in reverse order
    for(unsigned int j = f.NodeIDs.size() - 1; j > 0; j--){
      // finds the latitude and longitude of that node and the previous
      nodes.find(f.NodeIDs[j], lat1, lon1, ent1);
      nodes.find(f.NodeIDs[j - 1], lat2, lon2, ent2);
      double dist = distBetween2Points(lat1, lon1, lat2, lon2);
      // finds distance between the two and pushes it as an edge in the reverse direction
      // i.e 4->3, 3->2, 2->1 etc.
      G.addEdge(f.NodeIDs[j], f.NodeIDs[j - 1], dist);
    }
  }
}

//
// graphCheck
//
// implements the ! command, which outputs the edges provided in the handout
// to ensure weights are correct
//
void graphCheck(graph& G){
  cout << "Graph Check of Footway ID 986532630" << endl;
  // Node IDs of all the nodes used in the ! check
  long long e1 = 9119071425;
  long long e2 = 533996671;
  long long e3 = 533996671;
  long long e4 = 9119071425;
  long long e5 = 533996672;
  long long e6 = 2240260064;
  double weight;
  // Get the weight of each Edge and output it
  G.getWeight(e1, e2, weight);
  cout << "  Edge: (" << e1 << ", " << e2 << ", " << weight << ")" << endl;
  G.getWeight(e3, e4, weight);
  cout << "  Edge: (" << e3 << ", " << e4 << ", " << weight << ")" << endl;
  G.getWeight(e3, e5, weight);
  cout << "  Edge: (" << e3 << ", " << e5 << ", " << weight << ")" << endl;
  G.getWeight(e5, e3, weight);
  cout << "  Edge: (" << e5 << ", " << e3 << ", " << weight << ")" << endl;
  G.getWeight(e5, e6, weight);
  cout << "  Edge: (" << e5 << ", " << e6 << ", " << weight << ")" << endl;
  G.getWeight(e6, e5, weight);
  cout << "  Edge: (" << e6 << ", " << e5 << ", " << weight << ")" << endl;
}

//
// navStart
//
// takes in a lat and lon from a building, and returns the node ID that has the smallest distance
// from it, the footway that the node is from, and the minimum distance
//
void navStart(Footways& footways, double buildLat, double buildLon, Nodes& nodes, long long& startFoot){
  // initializing variables
  constexpr double INF = numeric_limits<double>::max();
  double minDist = INF;
  double dist = 0.0;
  double lat = 0.0;
  double lon = 0.0;
  bool isEntrance; 
  long long minID, footID;

  // loops through each Footway in MapFootways
  for(Footway& F : footways.MapFootways){

    // loops through each node ID for the footway
    for(long long d : F.NodeIDs){
      // finds the distance from the node to the given node
      nodes.find(d, lat, lon, isEntrance);
      dist = distBetween2Points(buildLat, buildLon, lat, lon);
      // if the distance is less than the current minDist, set minDist as the distance,
      // and update the minID and footID variables 
      if(dist < minDist){
        minDist = dist;
        minID = d;
        footID = F.ID;
      }
    }
  }
  startFoot = minID;
  cout << "  Closest footway ID " << footID << ", node ID " << minID << ", distance " << minDist << endl;
}

//
// prioritize class needed for the priority_queue
//
class prioritize{
  public:
    bool operator()(const pair<long long, double>& p1, const pair<long long, double>& p2) const{
      if(p1.second > p2.second){
        return true;
      }
      else if (p1.second < p2.second){
        return false;
      }
      else{ // values the same, look at key
        return p1.first > p2.first; // smaller keys first
      }
    }
};

//
// search
//
// searches a vector, and returns true if the cVec is found in the vector,
// false otherwise
//
bool search(vector<long long> visited, long long cVec){
  
  // loops through each long long in the visited vector
  for(unsigned int i = 0; i < visited.size(); i++){
    // if the element matches cVec, return true
    if(visited[i] == cVec){
      return true;
    }
  }
  return false;
}

//
// dijkstra
//
// performs Dijkstra's shorted weighted path algorithm from the given start vertex,
// and returns a distances and predecessor map by reference, as well as a visited
// vector with all of the vertexes that the algorithm visited
//
vector<long long> dijkstra(graph& G, long long startV, map<long long, double>& distances, map<long long, long long>& predecessor)
{
  // creates variables
  vector<long long>  visited;
  constexpr double INF = numeric_limits<double>::max();
  double edgeWeight;  
  double altPath;
  // priority_queue
  priority_queue<
    pair<long long, double>, // key value pair
    vector<pair<long long, double>>, // store pairs in a vector
    prioritize> unvisited;
    int count = 0;

  // loops through each of the vertices
  for(long long currentV : G.getVertices()){
    // if the vertex isn't the start, set the distance to INF
    if(currentV != startV){
      unvisited.push(make_pair(currentV, INF));  
      distances[currentV] = INF;
    }
    // if the vertex is the start, set the distance to 0
    else{
      unvisited.push(make_pair(currentV, 0));
      distances[currentV] = 0;
    }
    // initialzie predecessor
    predecessor[currentV] = 0;
  }
  
  // while the unvisited priority_queue isn't empty
  while(unvisited.size() != 0){
    // take the top and pop
    auto currentVec = unvisited.top();
    unvisited.pop();

    // if the currentVec distance is INF, break
    if(currentVec.second == INF){
      break;  
    }
    // else if currentVect has already been visited, continue
    else if(search(visited, currentVec.first)){
      continue;
    }
    else{
      // loops through each neighbor vertex
      for(long long adjVertex : G.neighbors(currentVec.first)){
        // determines the edgeWeight
        G.getWeight(currentVec.first, adjVertex, edgeWeight);
        // finds the distance of the alternate path
        altPath = currentVec.second + edgeWeight;
        // if a shorter path from currentVec to the adjVertex is found
        if(altPath < distances.at(adjVertex)){
          // sets the distance to the altPath distance, pushes the pair into unvisited,
          // and sets the predecessor
          distances.at(adjVertex) = altPath;
          unvisited.push(make_pair(adjVertex, altPath));
          predecessor.at(adjVertex) = currentVec.first;
        }
      }
      // pushes back the vertex into the visited vector
      visited.push_back(currentVec.first);  
    }
  } 
  return visited;
}

//
// path
//
// takes in the ID of the start building and the end building, builds a stack representing
// the path in reverse order through the predecessor map, and then prints the path in the correct
// order
//
void path(long long& startFoot, long long& endFoot, map<long long, long long>& predecessor){
  stack<long long> prePath;
  long long check = endFoot;
  // push the end node into the stack
  prePath.push(endFoot);

  // while the predecessor isn't the starting node
  while(predecessor[check] != startFoot){
    // push the node into the stack, and update check to be the predecessor
    prePath.push(predecessor[check]);
    check = predecessor[check];
  }
  // push the start node into the stack
  prePath.push(startFoot);
  cout << "  Path: ";
  // while the stack isn't empty
  while(!prePath.empty()){
    // print out the top of the stack and pop
    if(prePath.size() != 1){
      cout << prePath.top() << "->";
    }
    else{
      cout << prePath.top();
    }
    prePath.pop();
  }
}


//
// navigate
//
// implements what happens when the user inputs @, by taking in a start and a destination building, 
// and returning their names, approximate location, and nearest footway ID, node ID, and the distance 
// between them
//
void navigate(Buildings& buildings, Nodes& nodes, Footways& footways, graph& G){
  cout << "Enter start building name (partial or complete)>" << endl;
  //variable initializations
  string build, build1;
  double startLat, startLon, startLat2, startLon2;
  Building startB(0, "", "");
  Building endB(0, "", "");
  getline(cin, build);
  long long startFoot = 0;
  long long endFoot = 0;
  constexpr double INF = numeric_limits<double>::max();

  // loops through each building in the MapBuildings vector
  for(Building& B : buildings.MapBuildings){

    // if the start building is found
    if(B.Name.find(build) != string::npos){
      // initialize empty object so we know a match has been found
      startB = B;
      // outputs the name and approximate location
      cout << "  Name: " << B.Name << endl;
      B.getLocation(nodes, startLat, startLon);
      cout << "  Approximate location: (" << startLat << ", " << startLon << ")" << endl;
      navStart(footways, startLat, startLon, nodes, startFoot);
      break;
    }
  }

  // if match hasn't been found
  if(startB.ID == 0){
    cout << "**Start building not found" << endl;
  }
  else{
    cout << "Enter destination building name (partial or complete)>" << endl;
    getline(cin, build1);

    // loops through each building in the MapBuildings vector
    for(Building& C : buildings.MapBuildings){
      // if the destination building is found
      if(C.Name.find(build1) != string::npos){
        //initialize empty object so we know it has been found
        endB = C;
        //output name and approximate location
        cout << "  Name: " << C.Name << endl;
        C.getLocation(nodes, startLat2, startLon2);
        cout << "  Approximate location: (" << startLat2 << ", " << startLon2 << ")" << endl;
        navStart(footways, startLat2, startLon2, nodes, endFoot);

        // initializes variables necessary for dijkstra function and calls it
        map<long long, double> distances;
        vector<long long> dist;
        map<long long, long long> predecessor;
        dist = dijkstra(G, startFoot, distances, predecessor);

        // finds the shortest distance to the destination
        double smallDist = distances[endFoot];
        cout << "Shortest weighted path: " << endl;
        // if smallDist is infinity, that means that there is no path to the destination building
        if(smallDist == INF){
          cout << "**Sorry, destination unreachable" << endl;
        }
        else{
          cout << "  # nodes visited: " << dist.size() << endl;
          cout << "  Distance: " << smallDist << " miles" << endl;

          path(startFoot, endFoot, predecessor);

        }
        break;
      }
    }
    // if the Destination building isn't found
    if(endB.ID == 0){
      cout << "**Destination building not found" << endl;
    }
  }
}

//
// main
//
int main()
{
  XMLDocument xmldoc;
  Nodes nodes;
  Buildings buildings;
  Footways footways;
  graph G;

  cout << setprecision(12);
  
  cout << "** NU open street map **" << endl;

  string filename;

  cout << endl;
  cout << "Enter map filename> " << endl;
  getline(cin, filename);

  //
  // 1. load XML-based map file 
  //
  if (!osmLoadMapFile(filename, xmldoc))
  {
    // failed, error message already output
    return 0;
  }
  
  //
  // 2. read the nodes, which are the various known positions on the map and adds them to the graph:
  //
  nodes.readMapNodes(xmldoc);

  //
  // NOTE: let's sort so we can use binary search when we need 
  // to lookup nodes.
  //
  nodes.sortByID();

  // adds nodes to the graph as vertexes
  nodes.graphVert(G);

  //
  // 3. read the university buildings:
  //
  buildings.readMapBuildings(xmldoc);

  //
  // 4. read the footways, which are the walking paths:
  //
  footways.readMapFootways(xmldoc);

  // adds the edges between nodes to the graph
  footwaysGraph(G, footways, nodes);

  //
  // 5. stats
  //
  cout << "# of nodes: " << nodes.getNumMapNodes() << endl;
  cout << "# of buildings: " << buildings.getNumMapBuildings() << endl;
  cout << "# of footways: " << footways.getNumMapFootways() << endl;
  cout << "# of graph vertices: " << G.NumVertices() << endl;
  cout << "# of graph edges: " << G.NumEdges() << endl;

  //
  // now let the user for search for 1 or more buildings:
  //
  while (true)
  {
    string name;

    cout << endl;
    cout << "Enter building name, * to list, @ to navigate, or $ to end> " << endl;

    getline(cin, name);

    if (name == "$") {
      break;
    }
    else if (name == "*") {
      buildings.print();
    }
    else if (name == "@"){
      navigate(buildings, nodes, footways, G);
    }
    else if (name == "!"){
      graphCheck(G);
    }
    else {
      buildings.findAndPrint(name, nodes, footways);
    }

  }//while

  //
  // done:
  //
  cout << endl;
  cout << "** Done  **" << endl;
  cout << "# of calls to getID(): " << Node::getCallsToGetID() << endl;
  cout << "# of Nodes created: " << Node::getCreated() << endl;
  cout << "# of Nodes copied: " << Node::getCopied() << endl;
  cout << endl;

  return 0;
}
