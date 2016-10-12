#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <iterator>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <fstream>

class Graph {
  public:
    using Vertex = size_t;
    using VertexSet = std::unordered_set<Vertex>;
    using AdjacencyList = std::unordered_map<Vertex, VertexSet>;

    void AddVertex(Vertex v) {
        adjacency_list_[v];
    }

    void AddEdge(Vertex u, Vertex v) {
        adjacency_list_[u].insert(v);
        adjacency_list_[v].insert(u);
    }

    const VertexSet& AdjacentVertices(Vertex v) const {
        const auto it = adjacency_list_.find(v);
        if (it != adjacency_list_.end()) {
            return it->second;
        } else {
            return empty_set_;
        }
    }

    VertexSet AllVertices() const {
        VertexSet vs;
        vs.reserve(adjacency_list_.size());
        for (const auto& pair : adjacency_list_) {
            const auto& vertex = pair.first;
            vs.insert(vertex);
        }
        return vs;
    }

    const AdjacencyList& AsAdjacencyList() const {
        return adjacency_list_;
    }

  private:
    AdjacencyList adjacency_list_;
    static const VertexSet empty_set_;
};

const Graph::VertexSet Graph::empty_set_;

class LongestSimplePath {
  public:
    explicit LongestSimplePath(const Graph& graph)
        : graph_(graph), set_complement_(graph.AllVertices()) {
            auto candidates = graph.AllVertices();
            auto random_vertex = candidates.begin();
            std::advance(random_vertex, rand() % candidates.size());
            start_ = *random_vertex;
            finish_ = *random_vertex;
            set_.insert(start_);
        }

    Graph::VertexSet CandidatesToAdd() const {
        Graph::VertexSet candidates;
        for (const auto v : graph_.AdjacentVertices(start_)){
            if (find(v) == end()) {
                candidates.insert(v);
            }
        }
        for (const auto v : graph_.AdjacentVertices(finish_)) {
            if (find(v) == end()) {
                candidates.insert(v);
            }
        }
        return candidates;
    }

    const Graph::VertexSet CandidatesToRemove() const {
        Graph::VertexSet candidates;
        if (start_ != finish_) {
            candidates.insert(start_);
            candidates.insert(finish_);
        }
        return candidates;
    }

    void Add(Graph::Vertex v) {
        if (graph_.AdjacentVertices(start_).find(v) != graph_.AdjacentVertices(start_).end()) {
            start_ = v;

        } else {
            if (graph_.AdjacentVertices(finish_).find(v) != graph_.AdjacentVertices(finish_).end()) {
                finish_ = v;
            }
        }
        set_.insert(v);
        set_complement_.erase(v);

    }
    void Remove(Graph::Vertex v) {
        if (v == start_) {
            for (const auto u: graph_.AdjacentVertices(start_)) {
                if (find(u) != end()) {
                    start_ = u;
                }
            }
        } else if (v == finish_) {
            for (const auto u: graph_.AdjacentVertices(finish_)) {
                if (find(u) != end()) {
                    finish_ = u;
                }
            }
        }
        set_.erase(v);
        set_complement_.insert(v);
    }

    size_t Cost() const {
        return set_.size();
    }

    Graph::VertexSet::const_iterator find(Graph::Vertex v) const {
        return set_.find(v);
    }

    Graph::VertexSet::const_iterator begin() const {
        return set_.begin();
    }

    Graph::VertexSet::const_iterator end() const {
        return set_.end();
    }

    const Graph& GetGraph() const {
        return graph_;
    }
    Graph::Vertex StartVertex() const{
        return start_;
    }

    Graph::Vertex FinishVertex() const{
        return finish_;
    }

  private:
    const Graph& graph_;
    Graph::VertexSet set_;
    Graph::VertexSet set_complement_;
    Graph::Vertex start_;
    Graph::Vertex finish_;
};

void GraphEdges(std::ostream& out, const Graph::AdjacencyList& adjacency_list) {
    for (const auto& pair : adjacency_list) {
        const auto& vertex = pair.first;
        const auto& neighbours = pair.second;
        for (const auto adj_vertex : neighbours) {
            out << "\t" << vertex << " -- " << adj_vertex << "\n";
        }
    }
}

// Use http://www.webgraphviz.com to take a look at the graph
void GraphViz(std::ostream& out, const Graph& graph) {
    out << "strict graph {\n";
    for (const auto& pair : graph.AsAdjacencyList()) {
        const auto& vertex = pair.first;
        out << "\t" << vertex << "\n";
    }
    GraphEdges(out, graph.AsAdjacencyList());
    out << "}\n";
}

void GraphViz(std::ostream& out, const LongestSimplePath& simple_path) {
    out << "strict graph {\n";
    for (const auto& pair : simple_path.GetGraph().AsAdjacencyList()) {
        const auto& vertex = pair.first;
        if (simple_path.find(vertex) != simple_path.end()) {
            out << "\t" <<  vertex << " [shape=doublecircle]\n";
        } else {
            out << "\t" << vertex << "\n";
        }
    }
    GraphEdges(out, simple_path.GetGraph().AsAdjacencyList());
    out << "}\n";
}

struct DebugInfo {
    std::vector<size_t> costs;
};

// Use http://gnuplot.respawned.com/ to plot costs
std::ostream& operator<<(std::ostream& out, const DebugInfo& debug_info) {
    for (size_t i = 0; i < debug_info.costs.size(); ++i) {
        out << i << " " << debug_info.costs[i] << "\n";
    }
    return out;
}


class LongestSimplePathSolver {
  public:
    virtual LongestSimplePath Solve(const Graph& graph,
                              DebugInfo& debug_info) const = 0;
    virtual ~LongestSimplePathSolver() = default;
};

class GradientDescent final: public LongestSimplePathSolver {
    LongestSimplePath Solve(const Graph& graph, DebugInfo& debug_info) const {
        LongestSimplePath simple_path(graph);
        debug_info.costs.emplace_back(simple_path.Cost());
        while (1) {
            const auto candidates = simple_path.CandidatesToAdd();
            if (candidates.empty()) {
                return simple_path;
            }
            auto random_candidate = candidates.begin();
            std::advance(random_candidate, rand() % candidates.size());
            simple_path.Add(*random_candidate);
            debug_info.costs.emplace_back(simple_path.Cost());
        }
    }
};

class Metropolis final: public LongestSimplePathSolver {
  public:
    Metropolis(double k, double t, bool annealing=false, size_t iterations=100)
        : k_(k), t_(t), annealing_(annealing), iterations_(iterations) {
    }

    LongestSimplePath Solve(const Graph& graph, DebugInfo& debug_info) const {
        double t = t_;
        LongestSimplePath simple_path(graph);
        debug_info.costs.emplace_back(simple_path.Cost());
        for (size_t i = 0; i < iterations_; ++i) {
            const auto add_candidates = simple_path.CandidatesToAdd();
            const auto remove_candidates = simple_path.CandidatesToRemove();

            if (add_candidates.size() != 0 &&
                rand() % (remove_candidates.size() + add_candidates.size()) <
                add_candidates.size()) {
                auto random_candidate = add_candidates.begin();
                std::advance(random_candidate, rand() % add_candidates.size());
                simple_path.Add(*random_candidate);
            } else if (remove_candidates.size() != 0 && double(rand()) / RAND_MAX <= exp(-1. / k_ / t)) {
                auto random_candidate = remove_candidates.begin();
                std::advance(random_candidate, rand() % remove_candidates.size());
                simple_path.Remove(*random_candidate);
            }
            debug_info.costs.emplace_back(simple_path.Cost());
            if (annealing_) {
                t /= 2;
            }
        }
        return simple_path;
    }

  private:
    double k_;
    double t_;
    bool annealing_;
    size_t iterations_;
};



Graph RandomGraph(size_t size, double edge_probability) {
    Graph graph;
    for (Graph::Vertex v = 1; v <= size; ++v) {
        graph.AddVertex(v);
    }
    for (Graph::Vertex v = 1; v <= size; ++v) {
        for (Graph::Vertex u = v + 1; u <= size; ++u) {
            if (double(rand()) / RAND_MAX <= edge_probability) {
                graph.AddEdge(v, u);
            }
        }
    }
    return graph;
}

Graph StarGraph(size_t size) {
    Graph graph;
    for (Graph::Vertex v = 2; v <= size; ++v) {
        graph.AddEdge(1, v);
    }
    return graph;
}

int InitRandSeed(int argc, const char* argv[]) {
    int rand_seed;
    if (argc >= 2) {
        rand_seed = atoi(argv[1]);
    } else {
        rand_seed = time(nullptr);
    }
    srand(rand_seed);
    return rand_seed;
}

void TrySolver(const LongestSimplePathSolver& solver, const Graph& graph) {
    std::ofstream fout("final.txt");
    //GraphViz(std::cout, graph);
    auto best_cost = 0;
    size_t results = 0;
    for (int attempt = 1; attempt < 100; ++attempt) {
        DebugInfo debug_info;
        const auto simple_path = solver.Solve(graph, debug_info);
        auto cost = simple_path.Cost();
        if (cost > best_cost) {
            best_cost = cost;
            GraphViz(std::cout, simple_path);
            fout << "Trace info:\n" << debug_info << "\n";
            ++results;
        }
    }
    std::cout << "Results: " << results << std::endl;
    std::cout << "Costs: " << best_cost << std::endl;
}

int main(int argc, const char* argv[]) {
    std::cout << "Using rand seed: " << InitRandSeed(argc, argv) << "\n";

    const auto graph = RandomGraph(70, 0.04);
    //const auto graph = StarGraph(7);
    GradientDescent gradient_descent;
    Metropolis metropolis(1, 100, false);
    Metropolis metropolis_annealing(1, 100, true);
    TrySolver(gradient_descent, graph);
    TrySolver(metropolis, graph);
    TrySolver(metropolis_annealing, graph);


    return 0;
}
