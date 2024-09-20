#include <iostream>
#include <random>
#include <vector>
#include <fstream>
#include <iomanip> // 用于控制输出格式

int main() {
    // 随机数生成器
    std::random_device rd;  // 用于获取随机种子
    std::mt19937 gen(rd()); // 以随机设备初始化 Mersenne Twister 生成器

    // 定义随机数分布范围
    std::uniform_real_distribution<> dis(0.3,1.2); // 定义一个范围在0.3到0.8的均匀分布

    // 要生成的随机数数量
    int num_count = 105;

    // 存储随机数的容器
    std::vector<double> random_numbers;

    // 生成随机数
    for (int i = 0; i < num_count; ++i) {
        double random_number = dis(gen); // 生成一个随机数
        random_numbers.push_back(random_number); // 将随机数添加到容器中
    }

    // 保存随机数到文件
    std::ofstream file("random_numbers.txt");
    if (file.is_open()) {
        // 设置精度，例如保留4位小数
        file << std::fixed << std::setprecision(3);
        for (double num : random_numbers) {
            file << num << ","<<std::endl;
        }
        file.close();
        std::cout << "Random numbers have been saved to random_numbers.txt" << std::endl;
    } else {
        std::cerr << "Unable to open file for writing." << std::endl;
    }

    // 读取文件中的随机数
    std::ifstream infile("random_numbers.txt");
    if (infile.is_open()) {
        std::cout << "Random numbers from file:" << std::endl;
        double num;
        while (infile >> num) {
            std::cout << std::fixed << std::setprecision(4) << num << std::endl;
        }
        infile.close();
    } else {
        std::cerr << "Unable to open file for reading." << std::endl;
    }

    return 0;
}