#include <HyperGraphSclamOptimizer.hpp>

int main(int argc, char **argv) {

    if (argc != 3) {

        // error
        std::cerr << "Usage: hypergraphslam <sync_file> <output_file>\n";

        return -1;

    }

    // create the HypergraphSclam optimizer
    hyper::HyperGraphSclamOptimizer hypergraph_optimizer(argc, argv);

    if (hypergraph_optimizer.Good()) {

        // run the optimization process
        hypergraph_optimizer.Run();

    }

    std::cout << "Optimization done!" << std::endl;

    return 0;

}
