#include <iostream>
#include "MyClass.h"
#include "ImageProcessor.h"

int main() {
    MyClass myObject;
    myObject.myMethod();

    std::cout << "Hello, C++!" << std::endl;

    std::string imageFilePath = "../data/sequence_01/imgs/00000.jpg";
    double threshold = 50.0;

    ImageProcessor imageProcessor(imageFilePath, threshold);
    return 0;
}

