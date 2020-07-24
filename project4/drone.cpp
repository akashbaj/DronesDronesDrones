//PROJECT IDENTIFIER  = 1761414855B69983BD8035097EFBD312EB0527F0
//  drone.cpp
//  project4
//
//  Created by Akash on 4/7/20.
//  Copyright © 2020 Akash Bajpai. All rights reserved.
//

#include <stdio.h>
#include "xcode_redirect.hpp"
#include <getopt.h>
#include <iostream>
#include <vector>
#include <iomanip>
#include <math.h>
#include <limits>

using namespace std;


struct drones {
    size_t parent = 0;
    double distance = numeric_limits<double>::infinity();
    int xCor;
    int yCor;
    bool visited = false;
    char area = 'n';
    drones(int x_in, int y_in) : xCor(x_in), yCor(y_in) {}
    
};

struct mstNode {
    size_t index;
    double distance = numeric_limits<double>::infinity();
    bool visited = false;
};

struct Node {
    int xCor;
    int yCor;
    Node(int x_in, int y_in) : xCor(x_in), yCor(y_in) {};
};//end of Node struct

void getHelp() {
    cerr << "Nigthmare Nightmare Nightmare!!!\n" << flush;
    exit(0);
} //end of help

string readCommands(int argc, char *argv[]) {
    int choice = 0;
    int option_index = 0;
    string command = "";
    string outSpec;
    
    option long_options[] = {
        {"mode",             required_argument, nullptr,  'm'},
        {"help",                   no_argument, nullptr,  'h'},
        {nullptr,                            0, nullptr, '\0'}
    }; //end of options
    
    while((choice = getopt_long(argc, argv, "hm:", long_options, &option_index)) != -1) {
        switch(choice) {
            case 'h':
                getHelp();
                exit(1);
            case 'm':
                command = optarg;
                if(command != "MST" && command != "OPTTSP" && command != "FASTTSP") {
                    cerr << "Error: Invalid mode. Mode must be one of MST, OPTTSP, FASTTSP" << flush;
                    exit(1);
                }
                break;
            default:
                cerr << "Error: invalid option" << flush;
                exit(1);
        } //end of switch
    } //end of while
    return command;
} //end of readCommands

bool isMed(vector<drones> &myDrones, size_t &index) {
    return (myDrones[index].xCor < 0 && myDrones[index].yCor < 0);
}

bool isBorder(vector<drones> &myDrones, size_t &index) {
    return (myDrones[index].xCor <= 0 && myDrones[index].yCor == 0) ||
    (myDrones[index].xCor == 0 && myDrones[index].yCor <= 0);
}

bool isNorm(vector<drones> &myDrones, size_t & index) {
    return (!isMed(myDrones, index) && !isBorder(myDrones, index));
}

char determineArea(drones &temp) {
    if(temp.xCor < 0 && temp.yCor < 0) {
        return 'm';
    }
    else if((temp.xCor <= 0 && temp.yCor == 0) && (temp.xCor == 0 && temp.yCor <= 0)) {
        return 'b';
    }
    else {
        return 'n';
    }
}

double distanceMST(vector<drones> &myDrones, size_t &index1, size_t &index2) {
    if( (isMed(myDrones, index1) && isNorm(myDrones, index2)) || (isNorm(myDrones, index1) && isMed(myDrones, index2)) ) {
        return numeric_limits<double>::infinity();
    }
    double sq1 = static_cast<double>(myDrones[index1].xCor - myDrones[index2].xCor) * (myDrones[index1].xCor - myDrones[index2].xCor);
    double sq2 = static_cast<double>(myDrones[index1].yCor - myDrones[index2].yCor) * (myDrones[index1].yCor - myDrones[index2].yCor);
    return sqrt(sq1 + sq2);
}

double droneDistance(size_t index1, size_t index2, vector<drones> &myDrones) {
    double sq1 = static_cast<double>(myDrones[index1].xCor - myDrones[index2].xCor) * (myDrones[index1].xCor - myDrones[index2].xCor);
    double sq2 = static_cast<double>(myDrones[index1].yCor - myDrones[index2].yCor) * (myDrones[index1].yCor - myDrones[index2].yCor);
    return sqrt(sq1 + sq2);
}

double nodeDistance(size_t index1, size_t index2, vector<Node> &currNodes) {
    double sq1 = static_cast<double>(currNodes[index1].xCor - currNodes[index2].xCor) * (currNodes[index1].xCor - currNodes[index2].xCor);
    double sq2 = static_cast<double>(currNodes[index1].yCor - currNodes[index2].yCor) * (currNodes[index1].yCor - currNodes[index2].yCor);
    return sqrt(sq1 + sq2);
}


double calcWeight(vector<drones> &myDrones) {
    double weight = 0;
    for(size_t i = 0; i < myDrones.size(); ++i) {
        weight += myDrones[i].distance;
    }
    return weight;
}

void print(vector<drones> &myDrones) {
    double weight = calcWeight(myDrones);
    cout << weight << "\n";
    cout << "0 ";
    size_t index = 0;
    while(myDrones[index].parent != 0) {
        index = myDrones[index].parent;
        cout << index << " ";
    }
}

void findMinDistance(vector<drones> &myDrones, size_t &minIndex, size_t &startIndex, double &minDistance) {
    for(size_t i = 1;  i < myDrones.size(); ++i) {
        if(droneDistance(startIndex, i, myDrones) < minDistance) {
            minDistance = droneDistance(startIndex, i, myDrones);
            minIndex = i;
        }//end of if
    }
}
void findCost(vector<mstNode> &mstNodes, vector<size_t> &tempPath, vector<vector<double>> &distances, size_t &currIndex, size_t &minIndex, size_t permLength, double &weight) {
    currIndex = 0;
    minIndex = 0;
    double tempCost1 = numeric_limits<double>::infinity();
    double tempCost2 = numeric_limits<double>::infinity();
    for(auto i : mstNodes) {
        if(distances[tempPath[permLength - 1]][i.index] < tempCost2) {
            tempCost2 = distances[tempPath[permLength - 1]][i.index];
            minIndex = i.index;
        }
        if(distances[tempPath[0]][i.index] < tempCost1) {
            tempCost1 = distances[tempPath[0]][i.index];
            currIndex = i.index;
        }
        weight += i.distance;
    }//end of for loop
    weight += distances[tempPath[0]][currIndex] + distances[tempPath[permLength-1]][minIndex];
}

double mstDistance(size_t permLength, vector<size_t> &tempPath, vector<vector<double>> &distances) {
    vector<mstNode> mstNodes;
    double weight = 0;
    mstNodes.resize(tempPath.size() - permLength);
    for(size_t i = 0; i < mstNodes.size(); ++i) {
        mstNodes[i].index = tempPath[i + permLength];
    }
    size_t currIndex = 0;
    size_t minIndex = 0;
    mstNodes[currIndex].distance = 0;
    mstNodes[currIndex].visited = true;
    size_t nodesVisited = 1;
    while(nodesVisited != mstNodes.size()) {
        double minDistance = numeric_limits<double>::infinity();
        for(size_t i = 0; i < mstNodes.size(); ++i) {
            if(!mstNodes[i].visited) {
                double tempDistance = distances[mstNodes[i].index][mstNodes[currIndex].index];
                if(tempDistance < mstNodes[i].distance) {
                    mstNodes[i].distance = tempDistance;
                }//end of 1st inner if
                if(mstNodes[i].distance < minDistance) {
                    minDistance = mstNodes[i].distance;
                    minIndex = i;
                }//end of 2nd inner if
            }//end of big if
        }//end of for loop
        mstNodes[minIndex].visited = true;
        currIndex = minIndex;
        ++nodesVisited;
    }//end of while
    findCost(mstNodes, tempPath, distances, currIndex, minIndex, permLength, weight);
    return weight;
}

bool promising(size_t permLength, vector<size_t> &tempPath, double &optimalWeight, double &currBest, vector<vector<double>> &distances) {
    if(tempPath.size() - permLength < 5) {
        return true;
    }
    double mstDist = mstDistance(permLength, tempPath, distances);
    return optimalWeight + mstDist < currBest;
}

void genPerms(size_t permLength, double &optimalWeight, double &currBest, vector<size_t> &tempPath, vector<vector<double>> &distances, vector<size_t> &path) {
    if(permLength == tempPath.size()) {
        double tempDist = distances[tempPath[0]][tempPath[permLength-1]];
        if(optimalWeight + tempDist < currBest) {
            currBest = optimalWeight + tempDist;
            path = tempPath;
        }
        return;
    }
    if(!promising(permLength, tempPath, optimalWeight, currBest, distances)) {
        return;
    }
    for(size_t i = permLength; i < tempPath.size(); ++i) {
        swap(tempPath[permLength], tempPath[i]);
        optimalWeight += distances[tempPath[permLength-1]][tempPath[permLength]];
        genPerms(permLength + 1, optimalWeight, currBest, tempPath, distances, path);
        optimalWeight -= distances[tempPath[permLength - 1]][tempPath[permLength]];
        swap(tempPath[permLength], tempPath[i]);
    }
}


int main(int argc, char *argv[]) {
    ios_base::sync_with_stdio(false);
    xcode_redirect(argc, argv);
    string command = readCommands(argc, argv);
    
    cout << setprecision(2);
    cout << fixed;
    
    vector<drones> myDrones;
    vector<Node> currNodes;
    unsigned int numDrones;
    cin >> numDrones;
    myDrones.reserve((size_t)numDrones);
    if(command == "MST" || command == "FASTTSP") {
        int x, y;
        while(cin >> x >> y) {
            drones temp(x, y);
            temp.area = determineArea(temp);
            myDrones.push_back(temp);
        }//reads input for MST and FASTTSP mode
        currNodes.clear();
    }
    else if(command == "OPTTSP") {
        int x, y;
        while(cin >> x >> y) {
            drones temp(x, y);
            Node tempNode(x, y);
            myDrones.push_back(temp);
            currNodes.push_back(tempNode);
        }
    }//reads input for OPTTSP mode
    
    if(command == "MST") {
        double weight = 0;
        size_t currIndex = 0;
        myDrones[0].visited = true;
        double currMinDistance;
        double tempDistance;
        size_t minIndex = 0;
        size_t nodesVisited = 1;
        while(nodesVisited != myDrones.size()) {
            currMinDistance = numeric_limits<double>::infinity();
            for(size_t i = 0; i < myDrones.size(); ++i) {
                if(!myDrones[i].visited) {
                    tempDistance = distanceMST(myDrones, currIndex, i);
                    if(tempDistance < myDrones[i].distance) {
                        myDrones[i].distance = tempDistance;
                        myDrones[i].parent = currIndex;
                    }
                    if(myDrones[i].distance < currMinDistance) {
                        currMinDistance = myDrones[i].distance;
                        minIndex = i;
                    }
                }//end of checking if visited
            }//end of for loop
            myDrones[minIndex].visited = true;
            weight += myDrones[minIndex].distance;
            currIndex = minIndex;
            ++nodesVisited;
        }//end of while loop
        cout << weight << "\n";
        for(size_t i = 1; i < myDrones.size(); ++i) {
            if(i < myDrones[i].parent){
                cout << i << " ";
            }
            else {
                cout << myDrones[i].parent << " ";
            }
            if(i < myDrones[i].parent) {
                cout << myDrones[i].parent << "\n";
            }
            else {
                cout << i << "\n";
            }
        }
    }//end of MST mode
    
    else if(command == "FASTTSP" || command == "OPTTSP") {
        size_t minIndex = 0;
        size_t startIndex = 0;
        double minDistance = numeric_limits<double>::infinity();
        findMinDistance(myDrones, minIndex, startIndex, minDistance);
        
        myDrones[startIndex].visited = true;
        myDrones[minIndex].visited = true;
        myDrones[minIndex].distance = minDistance;
        myDrones[startIndex].distance = minDistance;
        myDrones[startIndex].parent = minIndex;

        size_t nodesVisited = 2;
        startIndex = 1;
        
        while(nodesVisited != myDrones.size()) {
            if(!myDrones[startIndex].visited) {
                minDistance = numeric_limits<double>::infinity();
                for(size_t j = 0; j < myDrones.size(); ++j) {
                    if(myDrones[j].visited) {
                        double tempDistance = droneDistance(startIndex, j, myDrones) +
                        droneDistance(startIndex, myDrones[j].parent, myDrones) -
                        droneDistance(j, myDrones[j].parent, myDrones);
                        if(tempDistance < minDistance && tempDistance != minDistance) {
                            minDistance = tempDistance;
                            minIndex = j;
                        }//inner if
                    }//end of outter if
                }//end of for loop
                myDrones[startIndex].parent = myDrones[minIndex].parent;
                myDrones[startIndex].distance = droneDistance(startIndex, myDrones[minIndex].parent, myDrones);
                myDrones[startIndex].visited = true;
                
                myDrones[minIndex].parent = startIndex;
                myDrones[minIndex].distance = droneDistance(startIndex, minIndex, myDrones);
                ++nodesVisited;
            }
            ++startIndex;
        }//end of while loop
        if(command == "FASTTSP") {
            print(myDrones);
            return 0;
        }//program stops here if FASTTSP mode
        else if (command == "OPTTSP"){
                vector<size_t> path;
                size_t startIndex = 0;
                path.push_back(startIndex);
                while(myDrones[startIndex].parent != 0) {
                    startIndex = myDrones[startIndex].parent;
                    path.push_back(startIndex);
                }
                vector<size_t> tempPath(path);
                vector<vector<double>> distances(currNodes.size(), vector<double>(currNodes.size()));
                for(size_t i = 0; i < distances.size(); ++i) {
                    for(size_t j = 0; j < distances[i].size(); ++j) {
                        if(i != j) {
                            distances[i][j] = nodeDistance(i, j, currNodes);
                        }
                    }
                }//fill 2D vector of distances
                double currBest = numeric_limits<double>::infinity();
                double optimalWeight = 0;
                genPerms(1, optimalWeight, currBest, tempPath, distances, path);
                
                cout << currBest << "\n";
                for(auto i : path) {
                    cout << i << " ";
                }
            }//end of OPTTSP mode
        }//continuation for OPTTSP mode

    
  
    return 0;
}//end of main

