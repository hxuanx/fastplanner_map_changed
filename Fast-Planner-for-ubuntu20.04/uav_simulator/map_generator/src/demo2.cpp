#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

// 定义全局变量
const int arraySize = 50;
double num[arraySize];

int main() {
    // 文件名
    std::string filename = "random_numbers.txt";

    // 打开文件
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file " << filename << std::endl;
        return 1;
    }

    // 初始化索引变量
    int index = 0;

    // 读取数据
    std::string line;
    while (std::getline(file, line) && index < arraySize) {
        std::istringstream iss(line);
        double number;
        if (!(iss >> number)) { // 检查是否成功读取数字
            std::cerr << "Error reading a number from file." << std::endl;
            file.close();
            return 1;
        }
        num[index++] = number; // 将读取的数字存入全局数组
    }
    file.close();

    // 检查是否成功读取了预期数量的数据
    if (index != arraySize) {
        std::cerr << "Warning: The file does not contain exactly " << arraySize << " numbers." << std::endl;
    }

    // 使用全局数组
    std::cout << "Numbers from file:" << std::endl;
    for (int i = 0; i < index; ++i) {
        std::cout << num[i] << std::endl; // 输出全局数组中的每个数字
    }

    std::cout << num[49]<<std::endl;
    return 0;
}