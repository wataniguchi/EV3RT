#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <iostream>
#include <signal.h>
#include "app.h"

#if defined(WITH_OPENCV)
#include <opencv2/opencv.hpp>
using namespace cv;
#else
#define Mat void*
#endif


#define THREAD_NUM 30
#define BUF_SIZE 128
#define LOOP_NUM 100
bool ready = false;

int s1 = 0;
int s2 = 0;
std::mutex m1, m2;

class vcap_thd {
public:
  void operator()(int unused) {
    while(!ready) {
      // critical section 1
      if ( m1.try_lock() ) {
	s1++;
	m1.unlock();
      }
      std::this_thread::sleep_for( std::chrono::milliseconds( 11 ) );
      //std::this_thread::yield();
    }
  }
};

class video_thd {
public:
  void operator()(int unused) {
    while(!ready) {
      // critical section 1
      m1.lock();
      int i = s1;
      m1.unlock();
      i--;
      // critical section 2
      m2.lock();
      s2 = i;
      m2.unlock();
      std::this_thread::sleep_for( std::chrono::milliseconds( 20 ) );
      //std::this_thread::yield();
    }
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

  int u = 0;
  pthread_sigmask(SIG_BLOCK, &ss, 0); // prevent threads from interfaring with ASP TCB
  thds.emplace_back(vcap_thd(), u);
  thds.emplace_back(video_thd(), u);
  pthread_sigmask(SIG_UNBLOCK, &ss, 0); // let ASP manage the main thread

  std::cout << "finished creating thread(s)!" << std::endl;

  for (int i = 0; i < LOOP_NUM; i++) {
    // critical section 2
    if ( m2.try_lock() ) {
      int j = s2;
      m2.unlock();
      std::cout << j << "(" << i << "):";
    }
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
  }
  
  std::cout << "joining thread(s)..." << std::endl;
  ready = true; // tell the child thread(s) that they can finish
  for (auto& t : thds) {
    t.join();
  }
  std::cout << std::endl;

  std::cout << "exiting..." << std::endl;
  ext_tsk();
}
