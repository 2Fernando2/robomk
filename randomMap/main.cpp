#include <iostream>
#include <vector>
#include <map>
#include <random>

//RANDOM
/*
1. Create a random device (seed source) std::random_device rd;
2. Create a random number generator std::mt19937 gen(rd());
3. Create distributions std::uniform_int_distribution<int> dice(1, 6);
Dice roll 1-6 std::uniform_real_distribution<double> percent(0.0, 100.0);
Percentage 0-100 std::normal_distribution<double> height(170.0, 10.0);
Height: mean=170cm, std=10cm
std::cout << "=== Random Examples ===\n";
Generate some dice rolls
std::cout << "Dice rolls: ";
for (int i = 0; i < 5; ++i)
{ std::cout <<  dice(gen) << " ";
    std::cout << percentage(gen);
    std::cout << height(gen);
}
std::cout << "\n";
*/

int main()
{
    struct Node{int id; std::vector<int> links;};
    std::map<int, Node> graph;

    for(int i=0; i<100; i++)
        graph.emplace(std::make_pair(i, Node{i, std::vector<int>()}));

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dice6(0, 5);
    std::uniform_int_distribution<int> dice100(0, 99);

    for(auto &[key, value] : graph)
    {
        int vecinos = dice6(gen);
        for(int j = 0 ; j < vecinos; j++)
            value.links.emplace_back(dice100(gen));
    }

    return 0;
}