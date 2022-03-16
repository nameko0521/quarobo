#include <future>
#include <thread>
#include <vector>
#include <iostream>
#include <mutex>
using namespace std;

// 標準出力のmutex
mutex mtx_;

int long_calc(int i) {

    this_thread::sleep_for(chrono::milliseconds(1000));
    mtx_.lock();
    cout << "a" << i << endl;
    mtx_.unlock();

    this_thread::sleep_for(chrono::milliseconds(1000));
    mtx_.lock();
    cout << "b" << i << endl;
    mtx_.unlock();

    this_thread::sleep_for(chrono::milliseconds(1000));
    mtx_.lock();
    cout << "c" << i << endl;
    mtx_.unlock();
    return 0;
}
class MyClass {
    // 最大スレッド数
    const int max_num_threads = 4;

    vector<future<int>> v_status;

    // max_num_threads未満になるまで待つ関数
    void wait_threads() {
        if (v_status.size() < max_num_threads)return;
        else {
            while (true) {
                for (int i = 0; i < v_status.size(); i++) {
                    auto status = v_status[i].wait_for(chrono::milliseconds(100));
                    if (status != future_status::timeout) {
                        try {
                            // timeout以外はスレッドが終わったことを意味するのでgetして戻り値と例外を取得
                            int j = v_status[i].get();
                        }
                        catch (exception & ex) {
                            cout << ex.what() << endl;
                        }
                        catch (...) {
                            cout << "Unknown exception" << endl;
                        }
                        // 終わったスレッドを削除
                        v_status.erase(v_status.begin() + i, v_status.begin() + i + 1);
                    }
                }
                if (v_status.size() < max_num_threads)return;
            }
        }
    }
public:
    // マルチスレッド可能な関数
    void my_func(int i) {
        wait_threads();
        v_status.emplace_back(async(launch::async, long_calc, i));
    }
    // 最後に全てのスレッドを同期する関数
    void wait_for_completion() {
        for (int i = 0; i < v_status.size(); i++) {
            v_status[i].wait();
            try {
                // getして戻り値と例外を取得
                int j = v_status[i].get();
            }
            catch (exception & ex) {
                cout << ex.what() << endl;
            }
            catch (...) {
                cout << "Unknown exception" << endl;
            }
        }
    }
};

int main() {
    cout << "concurrency = " << thread::hardware_concurrency() << endl;
    MyClass a;
    for (int i = 0; i < 20; i++) {
        a.my_func(i);
        this_thread::sleep_for(chrono::milliseconds(200));
    }
    a.wait_for_completion();
    return 0;
}
