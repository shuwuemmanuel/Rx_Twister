// Parallel helpers: TBB when available, otherwise a plain std::thread fan-out.
#pragma once
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <thread>
#include <vector>
#if defined(RX_HAVE_TBB)
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/global_control.h>
#include <tbb/parallel_sort.h>
#endif

namespace rx {

int hardwareThreads();
void setThreadLimit(int n);   // 0 = all
int threadLimit();

// f(begin, end) is called on disjoint ranges covering [0, n).
template <class F>
void parallelRanges(size_t n, size_t grain, F&& f) {
  if (n == 0) return;
  if (grain == 0) grain = 1;
  if (n <= grain || threadLimit() <= 1) { f(size_t(0), n); return; }
#if defined(RX_HAVE_TBB)
  tbb::parallel_for(tbb::blocked_range<size_t>(0, n, grain),
                    [&](const tbb::blocked_range<size_t>& r) { f(r.begin(), r.end()); });
#else
  size_t workers = std::min<size_t>(threadLimit(), (n + grain - 1) / grain);
  std::atomic<size_t> next{0};
  auto body = [&]() {
    for (;;) {
      size_t b = next.fetch_add(grain);
      if (b >= n) break;
      f(b, std::min(n, b + grain));
    }
  };
  std::vector<std::thread> ts;
  for (size_t i = 1; i < workers; ++i) ts.emplace_back(body);
  body();
  for (auto& t : ts) t.join();
#endif
}

// f(i) for every i in [0, n)
template <class F>
void parallelFor(size_t n, F&& f, size_t grain = 256) {
  parallelRanges(n, grain, [&](size_t b, size_t e) { for (size_t i = b; i < e; ++i) f(i); });
}

template <class It, class Cmp>
void parallelSort(It b, It e, Cmp cmp) {
#if defined(RX_HAVE_TBB)
  tbb::parallel_sort(b, e, cmp);
#else
  std::sort(b, e, cmp);
#endif
}

}  // namespace rx
