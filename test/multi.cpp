#include <thread>
#include <cstdio>
#include <cstdint>
#include <mutex>

std::mutex mtx_;
uint32_t count_;

void add_count()
{
    // count_を加算する前にミューテックスを取得する
    std::lock_guard<std::mutex> lock(mtx_);
    ++count_;
}

void ThreadA()
{
    for(int i=0; i<100000; ++i){
        add_count();
    }
}

void ThreadB()
{
    for (int i = 0; i<100000; ++i) {
        add_count();
    }
}
void ThreadC()
{
    for(int i=0; i<100000; ++i){
        add_count();
    }
}

void ThreadD()
{
    for (int i = 0; i<100000; ++i) {
        add_count();
    }
}

int main()
{
    count_ = 0;
    int test = 0;

    std::thread th_a(ThreadA);
    std::thread th_b(ThreadB);
    std::thread th_c(ThreadC);
    std::thread th_d(ThreadD);

    scanf("%d", &test);

    th_a.join();
    th_b.join();
    th_c.join();
    th_d.join();

    printf("count_ : %d\n", count_ );


    return 0;
}
