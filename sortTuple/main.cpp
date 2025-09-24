#include <iostream>
#include <execution>
#include <algorithm>
#include <chrono>

int main()
{
    std::vector<std::tuple<int, float, double>> vector(10000000);

    for(auto &e : vector)
    {
        e = std::make_tuple(std::rand(), std::rand(), std::rand());
    }

    const auto start = std::chrono::high_resolution_clock::now();
    std::sort( std::execution::par, vector.begin(), vector.end(),
                [](auto e1, auto e2) {return std::get<0>(e1) < std::get<0>(e2);});
    const auto end = std::chrono::high_resolution_clock::now();

    const std::chrono::duration<float, std::milli> duration = end - start;
    std::cout << duration.count();

    return 0;
}