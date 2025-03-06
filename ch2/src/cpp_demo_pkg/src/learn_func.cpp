#include <iostream>
#include <functional>

void save_with_free_func(const std::string& file_name) {
    std::cout << "called free function, save: " << file_name << std::endl;
}

class FileSaver {
public:
    void save_with_member_func(const std::string& file_name) {
        std::cout << "called member func, save: " << file_name << std::endl;
    }
};

int main(int argc, char** argv) {
    FileSaver filesaver;
    auto save_with_lambda_func = [](const std::string& file_name) {
        std::cout << "called lambda func, save: " << file_name << std::endl;
    };

    std::function<void(const std::string&)> save1 = save_with_free_func;
    std::function<void(const std::string&)> save2 = save_with_lambda_func;
    std::function<void(const std::string&)> save3 = std::bind(&FileSaver::save_with_member_func, &filesaver, std::placeholders::_1);

    return 0;
}