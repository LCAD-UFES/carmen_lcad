#include <HyperGraphSclamOptimizer.hpp>


int main(int argc, char **argv)
{
    if (3 > argc || 5 < argc)
    {
        std::cerr << "Usage: hypergraphsclam <sync_file> <output_file_base_name> [optimization_config_filename]\n";
        return -1;
    }

    // create the HypergraphSclam optimizer
    hyper::HyperGraphSclamOptimizer hypergraph_optimizer(argc, argv);

    if (hypergraph_optimizer.Good())
    {
        // run the optimization process
        hypergraph_optimizer.Run();
    }

    std::cout << "Optimization done =D !" << std::endl;

    return 0;

}
