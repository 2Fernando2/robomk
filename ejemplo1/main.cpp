#include <QtWidgets>
#include "ejemplo1.h"
#include <chrono>
#include <iostream>

int main(int argc, char** argv)
{
    std::vector<std::tuple<int, float, double>> vector(10000000);


    for(auto &e : vector)
    {
        e = std::make_tuple(std::rand(), std::rand(), std::rand());
    }

    const auto start = std::chrono::high_resolution_clock::now();
    std::sort(vector.begin(), vector.end(),
                [](auto e1, auto e2) {return std::get<0>(e1) < std::get<0>(e2);});
    const auto end = std::chrono::high_resolution_clock::now();

    const std::chrono::duration<float, std::milli> duration = end - start;
    qDebug() << duration.count();

    //const std::chrono::duration<double> elapsed_seconds{end - start};
    //std::cout << elapsed_seconds;

    return 0;

    //QApplication app(argc, argv);
    //ejemplo1 foo;
    //foo.show();
    //return app.exec();
}
