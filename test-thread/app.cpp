#include <thread>
#include <vector>
#include <chrono>
#include <iostream>
#include <signal.h>
#include "app.h"

#define THREAD_NUM 30
bool ready = false;

class thd {
public:
  void operator()(int thdNo) {
    while(!ready) {
      std::this_thread::yield();
    }
    std::cout << thdNo << ":";
  }
};

void main_task(intptr_t unused) {
  sigset_t ss;  
  sigemptyset(&ss);
  sigaddset(&ss, SIGUSR2);
  sigaddset(&ss, SIGALRM);
  sigaddset(&ss, SIGPOLL);

  std::cout << "creating thread(s)..." << std::endl;
  std::vector<std::thread> thds;

  pthread_sigmask(SIG_BLOCK, &ss, 0); // prevent threads from interfaring with ASP TCB
  for (int id = 1; id <= THREAD_NUM; id++) {
    thds.emplace_back(thd(), id);
  }
  pthread_sigmask(SIG_UNBLOCK, &ss, 0); // let ASP manage the main thread

  std::cout << "finished creating thread(s)!" << std::endl;

  std::this_thread::sleep_for( std::chrono::seconds( 1 ) );

  std::cout << "joining thread(s)..." << std::endl;
  ready = true; // tell the child thread(s) that they can finish
  for (auto& t : thds) {
    t.join();
  }
  std::cout << std::endl;

  std::cout << "exiting..." << std::endl;
  ext_tsk();
}
