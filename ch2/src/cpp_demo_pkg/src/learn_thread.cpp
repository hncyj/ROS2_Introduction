#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <cpp-httplib/httplib.h>

class Download {
public:
    Download() = default;
    ~Download() = default;

    template <typename CallbackT>
    void download(const std::string& host, const std::string& path, CallbackT&& callback) {
        std::cout << "thread id: " << std::this_thread::get_id() << std::endl;
        httplib::Client client(host);
        auto response = client.Get(path);
        if (response && response->status == 200) { callback(path, response->body); }
    }

    template <typename CallbackT>
    void start_download(const std::string& host, const std::string& path, CallbackT&& callback) {
        auto thread_func = [this, host, path, callback = std::forward<CallbackT>(callback)]() {
            this->download(host, path, callback);
        };
        std::thread download_thread(std::move(thread_func));
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
    // download.start_download("http://localhost:8000", "/novel3.txt", download_finished_callback);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000 * 5));

    return 0;
}