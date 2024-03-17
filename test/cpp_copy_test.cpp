#include <gtest/gtest.h>

#include <iostream>
#include <vector>

class TestClass {
private:
    std::vector<int> data;
    // class name
    std::string name;

public:
    // Constructor to initialize TestClass with data
    TestClass(const std::vector<int>& initData,
              const std::string class_name) : data(initData), name(class_name) {}    

    ~TestClass() {
        std::cout << "Destructor called for " << name << std::endl;
    }

    // Function to combine data from another TestClass instance
    void combine(TestClass& other) {
        std::vector<int> otherData = other.getData();
        for (auto& item : otherData) {
            data.push_back(item);
        }
    }

    // Function to display data
    void display() const {
        for (const auto& item : data) {
            std::cout << item << " ";
        }
        std::cout << std::endl;
    }

    std::vector<int> getData() {
        std::vector<int> copy;
        for (const auto& item : data) {
            copy.push_back(item);
        }
        return copy;
    }
};

TEST(TestClassTest, TestCombine) {
    // Create TestClass instances a and b
    TestClass a({1, 2, 3}, "a");
    TestClass* b = new TestClass({4, 5, 6}, "b");

    // Display data before combining
    std::cout << "Data in a before combining: ";
    a.display();
    std::cout << "Data in b before combining: ";
    b->display();

    // Combine data from b into a
    std::vector<int> data = b->getData();
    a.combine(*b);

    // Display data after combining
    std::cout << "Data in a after combining: ";
    a.display();
    std::cout << "Data in b after combining: ";
    b->display(); // b has been cleared, it will display nothing

    delete b;
}

