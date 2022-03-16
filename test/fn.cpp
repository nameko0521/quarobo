#include <iostream>
using namespace std;

int MyFunc(int a, int b){
    return a+b;
}

int OutPutFunc(int (*ptn)(int,int)){
    cout << ptn(10,10) << endl;
}

int main(){
    //戻り値の型 関数ポインタ Myfuncの引数の型  

    OutPutFunc(MyFunc);

    // 関数ポインタを使って関数を実行も可能
    //cout << ptn(10,10) << endl;
}
