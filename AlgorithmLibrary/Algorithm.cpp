#pragma once

#include"graph.h"

namespace AlgorithmsOnGraphs {

	//=========================================================================================
	namespace set {

		template<class T> void make_set(T a, std::map<T, int>& rank, std::map<T, T>& parents) {
			parents[a] = a;
			rank[a] = 0;
		}

		template<class T> T find_set(T a, std::map<T, T>& parents) {
			if (a == parents[a]) {
				return a;
			}
			return parents[a] = find_set<T>(parents[a], parents);
		}

		template<class T> void union_sets(T a, T b, std::map<T, int>& rank, std::map<T, T>& parents) {
			a = find_set<T>(a, parents);
			b = find_set<T>(b, parents);

			if (a != b) {
				if (rank[a] < rank[b]) {
					swap(a, b);
				}

				parents[b] = a;
				if (rank[a] == rank[b]) {
					++rank[a];
				}
			}
		}
	}
	//=================================================================================================================


	//==================================================================================================================



	template<class T, class Units> void dfs1(T vertex, Graph<T, Units> const& graph, std::map<T, bool>& used, std::vector<T>& time_line) {

		used[vertex] = true;

		for (auto v : graph.getGraphEdges().at(vertex)) {
			if (!used[v.first]) {
				dfs1(v.first, graph, used, time_line);
			}
		}

		time_line.push_back(vertex);
	}


	template<class T, class Units> void dfs2(T vertex, Graph<T, Units> const& graph, std::map<T, bool>& used, std::list<T>& component) {

		used[vertex] = true;
		component.push_back(vertex);

		for (auto v : graph.getGraphEdges().at(vertex)) {
			if (!used[v.first]) {
				dfs2(v.first, graph, used, component);
			}
		}

	}

	template<class T, class Units>  void findStrongComponents(Graph<T, Units> const& graph, std::list<std::list<T>>& components) {

		std::map<T, bool> used;
		Graph<T, Units> t_graph;
		for (auto from : graph.getGraphEdges()) {
			for (auto to : from.second) {
				t_graph.addEdge(to.first, from.first, to.second);
			}
			used[from.first] = false;
		}

		std::vector<T> time_line;

		for (auto v : graph.getGraphEdges()) {
			if (!used[v.first]) {
				dfs1(v.first, graph, used, time_line);
			}
		}

		for (auto v : graph.getGraphEdges()) {
			used[v.first] = false;
		}

		for (int i = time_line.size() - 1; i >= 0; --i) {
			if (!used[time_line[i]]) {
				components.resize(components.size() + 1);
				dfs2(time_line[i], t_graph, used, components.back());
			}
		}



	}







	//==================================================================Find Bridges==============================================================================
	template<class T, class Units>  void bridgesDfs(int timer, T vertex, T parent, std::map<T, int>& tin, std::map<T, int>& fup, std::map<T, bool>& used, Graph<T, Units>const & graph, std::vector< std::pair<T, T> >& bridges) {
		used[vertex] = true;

		tin[vertex] = fup[vertex] = timer++;


		for (auto to : graph.getGraphEdges().at(vertex)) {

			if (to.first == parent) continue;


			if (used[to.first]) {

				fup[vertex] = std::min(fup[vertex], tin[to.first]);
			}
			else {

				bridgesDfs(timer, to.first, vertex, tin, fup, used, graph, bridges);

				fup[vertex] = std::min(fup[vertex], fup[to.first]);

				if (fup[to.first] > tin[vertex]) {
					bridges.push_back(std::make_pair(vertex, to.first));
				}

			}


		}
	}
	template<class T, class Units>  void findBridges(Graph<T, Units> const& graph, std::vector< std::pair<T, T> >& bridges) {
		std::map<T, bool> used;
		std::map<T, int> tin;
		std::map<T, int> fup;
		for (auto v : graph.getGraphEdges()) {
			used[v.first] = false;
		}
		int timer = 0;

		for (auto v : graph.getGraphEdges()) {
			if (!used[v.first]) {
				bridgesDfs(timer, v.first, v.first, tin, fup, used, graph, bridges);
			}
			timer = 0;
		}

	}
	//======================================================================================


	//use deep_search_class
	template<class T, class Units> void getConnectivity—omponents(Graph<T, Units> const& graph, std::list<std::list<T>>& components) {

		std::map<T, bool> used;

		for (auto v : graph.getGraphEdges()) {
			used[v.first] = false;
		}

		DeepSearch<T, Units> dp(graph.getGraphEdges().begin()->first);

		for (auto v : used) {
			if (!used[v.first]) {
				dp.initSearch(v.first);
				components.resize(components.size() + 1);
				while (!dp.end()) {
					T vertex = dp.nextStep(graph, used);
					components.back().push_back(vertex);
					used[v.first] = true;
				}
			}
		}

	}
	//
	//not directed graph
	template<class T, class Units> void kruskalAlgorithm(Graph<T, Units> const& graph, std::vector< std::pair<T, T> >& bestEdges, const Units InfiniteForEdgesWeightType) {

		std::multimap<Units, std::pair<T, T>> edges;
		int edges_count = graph.getGraphEdges().size();
		std::map<T, T> parents;
		std::map<T, int> rank;



		for (auto v : graph.getGraphEdges()) {
			set::make_set<T>(v.first, rank, parents);
		}

		for (auto from : graph.getGraphEdges()) {
			for (auto to : from.second) {
				edges.emplace(to.second, std::make_pair(from.first, to.first));
			}
		}

		for (auto e : edges) {
			T v1 = e.second.second;
			T v2 = e.second.first;
			if (set::find_set<T>(v1, parents) != set::find_set<T>(v2, parents)) {
				bestEdges.push_back(std::make_pair(v1, v2));
				set::union_sets<T>(v1, v2, rank, parents);
			}
		}

	}
	//
	//
	//not directed graph , can work with negative edges , atantion - negative cycle
	template<class T, class Units> void belmanaFordaAlgorithm(T start_vertex, Graph<T, Units> const& graph, std::map<T, T>& parents, std::map<T, Units>& costs, const Units InfiniteForEdgesWeightType) {
		const Units max_cost = InfiniteForEdgesWeightType;
		for (auto v : graph.getGraphEdges()) {
			used[v.first] = 0;
			parents[v.first] = v.first;
			costs[v.first] = max_cost;
		}
		costs[start_vertex] = Units();
		for (;;) {
			bool any = false;
			for (auto from : graph.getGraphEdges()) {
				for (auto to : from.second) {
					if (costs[from.first] < max_cost) {
						Units new_cost = costs[from.first] + to.second;
						if (new_cost < costs[to.first]) {
							costs[to.first] = new_cost;
							parents[to.first] = from.first;
							any = true;
						}
					}
				}
			}
			if (!any) break;
		}
	}
	//

	//not directed graph
	template<class T, class Units> bool primaAlgorithm(Graph<T, Units> const& graph, std::vector< std::pair<T, T> >& bestEdges, const Units InfiniteForEdgesWeightType) {
		int vertex_count = graph.getGraphEdges().size();
		std::map<T, bool> used;
		std::map<T, Units> min_e;
		for (auto v : graph.getGraphEdges()) {
			used[v.first] = false;
			min_e[v.first] = InfiniteForEdgesWeightType;
		}
		min_e[graph.getGraphEdges().begin()->first] = 0;
		for (int i = 0; i < vertex_count; ++i) {
			bool found = false;
			T best_vertex;

			for (auto v : used) {
				if (!v.second && (!found || min_e[v.first] < min_e[best_vertex])) {
					best_vertex = v.first;
					found = true;
				}
			}
			// if graph is not connected - no mst
			if (min_e[best_vertex] == InfiniteForEdgesWeightType) {
				return false;
			}

			used[best_vertex] = true;
			for (auto to : graph.getGraphEdges().at(best_vertex)) {
				if (to.second < min_e[to.first]) {
					min_e[to.first] = to.second;
					bestEdges.push_back(std::make_pair(to.first, best_vertex));
				}
			}
		}
	}
	//
	//

	//find best way from x to others ,  works without negative edges
	template<class T, class Units> void dejkstraAlgorithm(T start_vertex, Graph<T, Units> const& graph, std::map<T, T>& parents, std::map<T, Units>& costs, const Units InfiniteForEdgesWeightType) {

		const Units max_cost = InfiniteForEdgesWeightType;
		std::map<T, short> used;
		for (auto v : graph.getGraphEdges()) {
			used[v.first] = 0;
			parents[v.first] = v.first;
			costs[v.first] = max_cost;
		}
		used[start_vertex] = 1;
		costs[start_vertex] = Units();
		T closer_vertex = T();

		//fill costs max
		auto getMinCost = [](std::map<T, Units>& costs, std::map<T, short>& used, T& closer_vertex, const Units InfiniteForEdgesWeightType) {
			closer_vertex = T();
			Units min_cost = InfiniteForEdgesWeightType;
			Units curr_cost;
			for (auto v : costs) {
				curr_cost = v.second;
				if (curr_cost < min_cost && used[v.first] == 1) {
					min_cost = curr_cost;
					closer_vertex = v.first;
				}
			}
		};

		getMinCost(costs, used, closer_vertex, InfiniteForEdgesWeightType);

		while (closer_vertex != T()) {
			Units cost = costs[closer_vertex];

			for (auto to : graph.getGraphEdges().at(closer_vertex)) {
				Units new_cost = to.second + cost;
				if (used[to.first] == 0)//‰ÂÈÍÒÚ‡ ‚Â‰¸ ·ÂÁ ˆËÍÎÓ‚ ÚÓÎ¸ÍÓ ‡·ÓÚ‡ÂÚ ?ÎÓÎ
					used[to.first] = 1;
				if (new_cost < costs[to.first]) {
					costs[to.first] = new_cost;
					parents[to.first] = closer_vertex;
				}
			}
			used[closer_vertex] = 2;
			getMinCost(costs, used, closer_vertex, InfiniteForEdgesWeightType);
		}

	}


}

//==============================DeepSearch=================================
//=========================================================================

template<class T, class Units> class DeepSearch {


public:

	DeepSearch(T start_vertex) {
		init(start_vertex);
	}

	void initSearch(T start_vertex) {
		dp_queue.clear();
		init(start_vertex);
	}

	bool end() {
		if (dp_queue.empty()) {
			return true;
		}
		return false;
	}

	T nextStep(Graph<T, Units> const& graph, std::map<T, bool>& used) {
		T vertex = dp_queue.back();
		dp_queue.pop_back();
		used[vertex] = true;
		for (auto v : graph.getGraphEdges().at(vertex)) {
			if (!used[v.first]) {
				dp_queue.push_back(v.first);
				used[v.first] = true;
			}
		}
		return vertex;
	}

	~DeepSearch() {
		dp_queue.clear();
	}

private:

	void init(T start_vertex) {
		dp_queue.push_back(start_vertex);
	}
	std::list<T> dp_queue;

};

//==============================BreadtdSearch======================================
template<class T, class Units> class Breadth_search {

public:

	Breadth_search(T start_vertex) {
		init(start_vertex);
	}

	bool end() {
		if (bf_queue.empty()) {
			return true;
		}
		return false;
	}

	void initSearch(T start_vertex) {
		bf_queue.clear();
		init(start_vertex);
	}

	T nextStep(Graph<T, Units> const& graph, std::map<T, bool>& used) {
		T vertex = bf_queue.front();
		bf_queue.pop_front();
		used[vertex] = true;
		for (auto v : graph.getGraphEdges().at(vertex)) {
			if (!used[v.first])
				bf_queue.push_back(v.first);
			used[vertex] = true;
		}

		return vertex;
	}

	~Breadth_search() {
		bf_queue.clear();
	}


private:
	void init(T start_vertex) {
		bf_queue.push_back(start_vertex);
	}
	std::list<T> bf_queue;
};