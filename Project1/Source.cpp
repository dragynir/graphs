
#include"Graph.h"


int main() {

	std::ifstream fin("input.txt");
	std::ofstream fout("output.txt");

	Graph<int, int> g;

	g.setGraph(fin);

	g.showGraphEdges(fout);

	return 0;
}