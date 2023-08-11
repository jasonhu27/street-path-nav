/*building.cpp*/

//
// A building in the Open Street Map.
// 
// Prof. Joe Hummel
// Northwestern University
// CS 211: Winter 2023
// 

#include <iostream>

#include "building.h"

using namespace std;


//
// constructor
//
Building::Building(long long id, string name, string streetAddr)
  : ID(id), Name(name), StreetAddress(streetAddr)
{
  //
  // the proper technique is to use member initialization list above,
  // in the same order as the data members are declared:
  //
  //this->ID = id;
  //this->Name = name;
  //this->StreetAddress = streetAddr;

  // vector is default initialized by its constructor
}

//
// containsThisNode
//
// Returns true if the building's nodes contains the given node id,
// false if not.
//
bool Building::containsThisNode(long long nodeid)
{
  for (long long id : this->NodeIDs)
  {
    if (nodeid == id)
      return true;
  }

  // 
  // if get here, not found:
  //
  return false;
}

//
// print
// 
// prints information about a building --- id, name, etc. -- to
// the console. The function is passed the Nodes for searching 
// purposes.
//
void Building::print(Nodes& nodes)
{
  cout << this->Name << endl;
  cout << "Address: " << this->StreetAddress << endl;
  cout << "Building ID: " << this->ID << endl;
  // adds the approximate location to the output
  double avgLat;
  double avgLon;
  getLocation(nodes, avgLat, avgLon);
  cout << "Approximate Location: (" << avgLat << ", " << avgLon << ")" << endl;
  cout << "Nodes:" << endl;

  for (long long nodeid : this->NodeIDs)
  {
    cout << "  " << nodeid << ": ";

    double lat = 0.0;
    double lon = 0.0;
    bool entrance = false;

    bool found = nodes.find(nodeid, lat, lon, entrance);

    if (found) {
      cout << "(" << lat << ", " << lon << ")";

      if (entrance)
        cout << ", is entrance";

      cout << endl;
    }
    else {
      cout << "**NOT FOUND**" << endl;
    }
  }//for
}

//
// adds the given nodeid to the end of the vector.
//
void Building::add(long long nodeid)
{
  this->NodeIDs.push_back(nodeid);
}

//
// getLocation
// 
// returns the average latitude and longitude by reference of all the 
// nodes in the building
//
void Building::getLocation(Nodes& nodes, double& avgLat, double& avgLon){
  double lat, lon;
  double latSum = 0.0;
  double lonSum = 0.0;
  bool entrance, found;

  // loops through each Node ID in the NodeIDs vector of the building
  for(long long i : this->NodeIDs){
    // if the node is found, the latitude and longtidue is added to the 
    // latSUm and lonSum variables
    found = nodes.find(i, lat, lon, entrance);
    if(found){
      latSum += lat;
      lonSum += lon;
    }
  }
  
  // returns the average latitude and average longitude by reference
  avgLat = latSum / this->NodeIDs.size();
  avgLon = lonSum / this->NodeIDs.size();
}
