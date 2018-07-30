#pragma once



#include<iterator>
#include<fstream>
#include<map>
#include<string>
#include<cmath>
#include<vector>
#include<queue>
#include<list>
#include<stack>
#include<iostream>

template<class T, class Units> class Graph {

public:


	void setGraph(std::ifstream& inputDevice) {
		int edgesCount = 0;
		inputDevice >> edgesCount;
		while (edgesCount--) {
			T from, to;
			Units len;
			inputDevice >> from >> to >> len;
			graph[from][to] = len;
			graph[to];//////important
		}
	}

	void showGraphEdges(std::ofstream& outputDevice) {

		for (auto from : graph) {
			if (from.second.empty()) {
				outputDevice << from.first << std::endl;
			}
			for (auto to : from.second) {
				outputDevice << from.first << " " << to.first << " " << to.second << std::endl;
			}
		}

	}

	void addVertex(T vertex) {
		graph[vertex];
	}

	void addEdge(T from, T to, Units length) {
		graph[from][to] = length;
		addVertex(to);
	}

	void eraseEdge(T from, T to) {
		graph[from].erase(graph[from].find(to));
	}

	void eraseVertex(T vertex) {
		auto found = this->graph.find(vertex);
		if (found != this->graph.end())
			this->graph.erase(this->graph.find(vertex));
		else return;

		for (auto v : this->graph) {
			auto found = v.second.find(vertex);
			if (found != v.second.end()) {
				eraseEdge(v.first, vertex);
			}
		}

	}

	Graph<T, Units>   operator+(const Graph<T, Units>& g) {

		std::map<T, std::map<T, Units>> new_graph_st;

		for (auto v : this->graph) {
			new_graph_st[v.first] = v.second;
		}
		for (auto v : g) {
			new_graph_st[v.first] = v.second;
		}

		Graph<T, Units>  new_graph(new_graph_st);

		return new_graph;
	}


	Graph<T, Units>&  operator+=(const Graph<T, Units>& g) {
		for (auto v : g) {
			this->graph[v.first] = v.second;
		}
		return *this;
	}


	bool ifNegativeCycle(const Units InfiniteForEdgesWeightType) {
		std::map<T, bool> used;
		std::map<T, Units> costs;
		int edges_count = 0;
		const Units max_cost = InfiniteForEdgesWeightType;
		for (auto v : graph) {
			edges_count += v.second.size();
			used[v.first] = 0;
			costs[v.first] = max_cost;
		}


		T vertex;
		for (int i = 0; i < edges_count; ++i) {
			vertex = T();
			for (auto from : graph) {
				for (auto to : from.second) {
					Units new_cost = costs[from.first] + to.second;
					if (new_cost < costs[to.first]) {
						costs[to.first] = new_cost;
						vertex = to.first;
					}
				}
			}
		}
		if (vertex == T()) {
			return false;
		}
		else {
			return true;
		}
	}

	bool ifDirectedGraph() {
		for (auto from : graph) {
			for (auto to : from.second) {
				auto vertex = graph.find(to.first);
				if (vertex->second.size() != 0 && vertex->second.find(from.first) != vertex->second.end()) {
					return false;
				}
			}
		}
		return true;
	}

	bool ifCycleInGraph() {
		std::map<T, int> color;

		for (auto v : graph) {
			color[v.first] = 0;
		}
		for (auto v : graph) {
			if (color[v.first] == 0) {
				std::pair<T, std::map<T, Units>> p = v;
				if (dfsCycleChecker(p, color)) {
					return true;
				}
			}
		}
		return false;
	}

	bool ifNegativeEdge() {
		for (auto from : graph) {
			for (auto to : from.second) {
				if (to.second < 0) {
					return true;
				}
			}
		}
		return false;
	}

	std::map<T, std::map<T, Units>>const&  getGraphEdges() const {
		return graph;
	}

private:

	std::map<T, std::map<T, Units>> graph;
	//map::size()


	bool dfsCycleChecker(std::pair<T, std::map<T, Units>> v, std::map<T, int>& color) {
		color[v.first] = 1;
		for (auto to : v.second) {
			auto g = graph.find(to.first);
			if (g->second.size() == 0) return false;
			std::pair<T, std::map<T, Units>> p = (*g);
			if (color[to.first] == 0) {
				if (dfsCycleChecker(p, color)) return true;
			}
			else if (color[to.first] == 1) {
				return true;
			}
		}
		color[v.first] = 2;
		return false;
	}
};