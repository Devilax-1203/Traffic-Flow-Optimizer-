#include <bits/stdc++.h>

using namespace std;
double minTime=0;

class DisjointSet{
vector<int> parent,size;
public:
    DisjointSet(int n){
    parent=vector<int>(n);
    size=vector<int>(n,1);
    for(int i=0;i<n;i++) parent[i]=i;
    }

    int findPar(int node){
    if(node==parent[node]) return node;
    return parent[node]=findPar(parent[node]);
    }

    void UnionBySize(int u,int v){
    int upar_u=findPar(u);
    int upar_v=findPar(v);
    if(upar_v==upar_u) return;
    if(size[upar_v]>size[upar_u]){
        parent[upar_u]=upar_v;
        size[upar_v]+=size[upar_u];
    }else{
        parent[upar_v]=upar_u;
        size[upar_u]+=size[upar_v];
    }
    }
};

class Graph {
public:
    Graph(int vertices) : adjList(vertices) {}

    void addEdge(int u, int v, int weight) {
        adjList[u].push_back({v, weight,1});
        adjList[v].push_back({u, weight,1});
    }

   void updateCongestion(int u, int v, int cf) {
    for (auto& edge : adjList[u]) {
        if (edge[0] == v) { // Check if the destination is v
            edge[2] = cf;   // Update the congestion factor
            break;
        }
    }


    for (auto& edge : adjList[v]) {
        if (edge[0] == u) { // Check if the destination is u
            edge[2] = cf;   // Update the congestion factor
            break;
        }
    }
}
    void shortestRoute() {
    vector<vector<int>> edges;
    vector<vector<int>> mst;
    int totalWeight = 0;

    for (int u = 0; u < adjList.size(); ++u) {
        for (const auto& edge : adjList[u]) {
            int v = edge[0];
            int weight = edge[1];
            int congestion = edge[2];
            int effectiveWeight = weight * congestion;
            if (u < v) // Avoid duplicate edges
                edges.push_back({effectiveWeight, u, v});
        }
    }

    sort(edges.begin(), edges.end());

    DisjointSet dsu(adjList.size());
    vector<vector<int>> mstAdjList(adjList.size());
    for (const auto& edge : edges) {
        if (dsu.findPar(edge[1]) != dsu.findPar(edge[2])) {
            dsu.UnionBySize(edge[1], edge[2]);
            mst.push_back(edge);
            mstAdjList[edge[1]].push_back(edge[2]);
            mstAdjList[edge[2]].push_back(edge[1]);
            totalWeight += edge[0];
        }
    }

    vector<bool> visited(adjList.size(), false);
    vector<int> contiguousPath;

    function<void(int)> dfs = [&](int node) {
        visited[node] = true;
        contiguousPath.push_back(node);
        for (int neighbor : mstAdjList[node]) {
            if (!visited[neighbor]) {
                dfs(neighbor);
            }
        }
    };

    dfs(0);

    cout << "\nShortest Route (Contiguous Path):\nStart::";
    for (const auto& edge : contiguousPath) {
        cout << " -> "<<edge;
    }

    cout << "\nTotal distance of route: " << totalWeight << "\n";
}






    void displayNetwork() {
        cout << "Road Network Layout (Nodes and Connections):\n";
        cout << "       (0)---10---(1)\n";
        cout << "        |         |\n";
        cout << "       15       5 |\n";
        cout << "        |         |\n";
        cout << "       (2)---20---(3)---10---(4)---15---(5)\n";
        cout << "        |         |                      |\n";
        cout << "        15        15                     5\n";
        cout << "        |         |                      |\n";
        cout << "       (6)--- 5 --(7)---------30--------(8)---25---(9)\n";
        cout << endl;
    }

    vector<int> dijkstra(int start, int target) {
        minTime=0;
        vector<int> dist(adjList.size(), numeric_limits<int>::max());
        vector<int> prev(adjList.size(), -1);
        dist[start] = 0;
        priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> pq;
        pq.push({0,start});

        while (!pq.empty()) {
            pair<int, int> top = pq.top();
            int currentDist = top.first;
            int u = top.second;
            pq.pop();

            if (u == target) break;

            for (const auto& edge : adjList[u]) {
                int v = edge[0];
                int weight = edge[1];
                int cf=edge[2];
                int newDist = currentDist + (weight*cf);
                if (newDist < dist[v]) {
                    dist[v] = newDist;
                    prev[v] = u;
                    pq.push({newDist, v});
                }
            }
        }

        // Reconstruct the shortest path
        minTime=dist[target];
        minTime=minTime/45.0;
        vector<int> path;
        for (int at = target; at != -1; at = prev[at]) {
            path.push_back(at);
        }
        reverse(path.begin(), path.end());
        return path;
    }

    int getNumConnections(int node) const {
        return adjList[node].size();
    }

private:
    vector<vector<vector<int>>> adjList;
    // v,wt,congestionfactor
    //adjlist of vector of size n
};

class TrafficLight {
public:
    TrafficLight(int baseGreenTime, int numConnections, int nodeId)
        : baseGreenTime(baseGreenTime), greenTime(baseGreenTime), numConnections(numConnections), nodeId(nodeId) {}

    void setTrafficInDirections(const vector<int>& trafficData) {
        for (int i = 0; i < numConnections; ++i) {
            traffic[i] = trafficData[i];
        }
        adjustTiming();
    }

    int getGreenTime() const { return greenTime; }

    int getNumConnections() const { return numConnections; }

    void displayTraffic() const {
        cout << "\nTraffic for Node " << nodeId << ":\n";
        cout << "Traffic - Direction 1: " << traffic[0] << ", Direction 2: " << traffic[1]
             << ", Direction 3: " << (traffic.size() > 2 ? traffic[2] : 0) << ", Direction 4: " << (traffic.size() > 3 ? traffic[3] : 0) << "\n";
        cout << "Adjusted Green Times for Node " << nodeId << ":\n";
        for (int i = 0; i < numConnections; ++i) {
            cout << " Direction " << (i + 1) << " green time: " << adjustedGreenTimes[i] << " seconds\n";
        }
    }

    void updateTrafficAfterGreenLight() {
        for (int i = 0; i < numConnections; ++i) {
            // Each car takes 1.5 seconds to pass
            int carsPassed = adjustedGreenTimes[i] / 1.5;
            if (traffic[i] > 0) {
                traffic[i] = max(0, traffic[i] - carsPassed);
            }
        }
    }

    void addMoreTraffic(const vector<int>& additionalTraffic) {
        for (int i = 0; i < numConnections; ++i) {
            traffic[i] += additionalTraffic[i];
        }
        adjustTiming();
    }

    int baseGreenTime;
    int greenTime;
    int numConnections;
    int nodeId;
    vector<int> traffic = {0, 0, 0, 0}; // For up to 4 directions
    vector<int> adjustedGreenTimes = {0, 0, 0, 0}; // Adjusted green times for each direction

    void adjustTiming() {
        int totalTraffic = 0;
        for (int i = 0; i < numConnections; ++i) {
            totalTraffic += traffic[i];
        }

        // Adjust green time proportionally to the traffic in each direction
        for (int i = 0; i < numConnections; ++i) {
            if (totalTraffic > 0) {
                adjustedGreenTimes[i] = (traffic[i] * baseGreenTime) / totalTraffic;
            } else {
                adjustedGreenTimes[i] = 0;
            }
        }
    }
};

class TrafficFlow {
public:
    void updateTrafficData(vector<TrafficLight>& lights, const Graph& graph) {
        cout << "\nUpdating traffic data after each green light cycle...\n";
        for (int i = 0; i < lights.size(); ++i) {
            lights[i].updateTrafficAfterGreenLight();
            lights[i].displayTraffic();  // Display updated traffic after each cycle
        }
    }
};

class UI {
public:
    void simulateTraffic() {
        Graph graph(10);
        graph.addEdge(0, 1, 10);
        graph.addEdge(0, 2, 15);
        graph.addEdge(1, 3, 5);
        graph.addEdge(2, 3, 20);
        graph.addEdge(3, 4, 10);
        graph.addEdge(4, 5, 15);
        graph.addEdge(5, 8, 5);
        graph.addEdge(6, 7, 5);
        graph.addEdge(7, 8, 30);
        graph.addEdge(8, 9, 25);
        graph.addEdge(2, 6, 15);
        graph.addEdge(3, 7, 15);

        graph.displayNetwork();

        vector<TrafficLight> lights;
        for (int i = 0; i < 10; ++i) {
            int baseGreenTime;
            cout << "Enter base green time for traffic light at node " << i << " (-1 for no traffic light): ";
            cin >> baseGreenTime;

            if (baseGreenTime<0) {
                continue;  // No traffic light at this node
            }

            int numConnections = graph.getNumConnections(i); // Use this from the graph
            TrafficLight light(baseGreenTime, numConnections, i);

            vector<int> trafficData(numConnections);

            cout << "Enter number of vehicles in each direction for traffic light at node " << i << ":\n";
            for (int j = 0; j < numConnections; ++j) {
                cout << "  Direction " << (j + 1) << " traffic: ";
                cin >> trafficData[j];
            }

            light.setTrafficInDirections(trafficData);
            lights.push_back(light);
        }

        // Menu Loop
        int choice = 0;
        while (choice != 7) {
            cout << "\nMenu:\n";
            cout << "1. Simulate Traffic Flow\n";
            cout << "2. Add More Traffic\n";
            cout << "3. Show Road Network\n";
            cout << "4. Find Emergency Path (Dijkstra)\n";
            cout << "5. Add Congestion\n";
            cout << "6. Shortest Path to Travel the route\n";
            cout << "7. Exit\n";
            cout << "Enter your choice: ";
            cin >> choice;

            switch (choice) {
                case 1: {
                    TrafficFlow flow;
                    flow.updateTrafficData(lights, graph);
                    break;
                }
                case 2: {
                    for (int i = 0; i < lights.size(); ++i) {
                        vector<int> additionalTraffic(lights[i].getNumConnections());

                        cout << "\nEnter additional traffic for node " << lights[i].nodeId << " (total " << lights[i].getNumConnections() << " directions):\n";
                        for (int j = 0; j < lights[i].getNumConnections(); ++j) {
                            cout << "  Direction " << (j + 1) << " additional traffic: ";
                            cin >> additionalTraffic[j];
                        }
                        lights[i].addMoreTraffic(additionalTraffic);

//                        cout << "Traffic updated for Node " << i << "!\n";
//                        lights[i].displayTraffic();
                    }
                    break;
                }
                case 3:
                    graph.displayNetwork();
                    break;
                case 4: {
                    int start, target;
                    cout << "Enter the start node for emergency vehicle (0-9): ";
                    cin >> start;
                    cout << "Enter the target node for emergency vehicle (0-9): ";
                    cin >> target;

                    vector<int> path = graph.dijkstra(start, target);

                    cout << "\nEmergency Vehicle Path: ";
                    for (int node : path) {
                        cout << node << " ";
                    }
                    double mins=(minTime-int(minTime))*60;
                    cout << endl<<"Time taken by emergency vehicle(assuming constant speed of 45KMPH):"<<int(minTime)<<"Hr "<<mins<<"mins"<<endl;
                    break;
                }
                case 5: {
                int u,v;
                cout<<"Enter the nodes between which congestion to be added:";
                cin>>u>>v;
                cout<<"Enter congestion factor::\n";
                cout<<"-1 for Rain(C.F.=1.5)\n";
                cout<<"-2 for Accident(C.F.=2)\n";
                cout<<"Anything else for manual addition:\n";
                int CF;
                cin>>CF;
                if(CF==-1) graph.updateCongestion(u,v,1.5);
                if(CF==-2) graph.updateCongestion(u,v,2);
                else graph.updateCongestion(u,v,CF);
                cout<<"Congestion Added\n";
                break;
                }
                case 6:{
                graph.shortestRoute();
                }
                case 7:
                    cout << "Exiting Program...\n";
                    break;
                default:
                    cout << "Invalid choice, try again!\n";
            }
        }
    }
};

int main() {
    UI ui;
    ui.simulateTraffic();
    return 0;
}
