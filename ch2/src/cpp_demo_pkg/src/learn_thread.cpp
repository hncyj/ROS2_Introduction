#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <cpp-httplib/httplib.h>

class Download {
public:
    Download() = default;
    ~Download() = default;

    void download(
        const std::string& host, 
        const std::string& path, 
        const std::function<void(const std::string&, const std::string&)>& callback 
    ) {
        std::cout << "thread id: " << std::this_thread::get_id() << std::endl;
        httplib::Client client(host);
        auto response = client.Get(path);
        if (response && response->status == 200) {
            callback(path, response->body);
        }
    }

    void start_download(
        const std::string& host, 
        const std::string& path, 
        const std::function<void(const std::string&, const std::string&)>& callback
    ) {
        auto download_func = std::bind(
            &Download::download, 
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3
        );

        std::thread download_thread(download_func, host, path, callback);
        download_thread.detach();
    }
};

int main() {
    Download download;
    auto download_finished_callback = [](const std::string& path, const std::string& result) {
        std::cout << "Download finished! " << path << ", total: " << result.length() << std::endl;
    };

    download.start_download("http://localhost:8000", "/novel1.txt", download_finished_callback);
    download.start_download("http://localhost:8000", "/novel2.txt", download_finished_callback);
    download.start_download("http://localhost:8000", "/novel3.txt", download_finished_callback);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000 * 10));

    return 0;
}