// Runs the same wk_selftest() the car runs at boot, on the PC.
// python -m ziglang c++ -std=c++17 -I firmware/libraries/WkLink firmware/test/host_selftest.cpp -o build/selftest.exe
#include <cstdio>
#include "WkLink.h"

int main() {
  int bad = wk_selftest(WK_DEFAULTS);
  WkCfg odd = WK_DEFAULTS;                 // asymmetric endpoints + reversed steering + trimmed neutral
  odd.steerLeft = 1100; odd.steerRight = 1880; odd.steerCenter = 1490; odd.steerReverse = 1;
  odd.escNeutral = 1520; odd.escFwdMax = 1950; odd.escRevMax = 1300;
  bad += wk_selftest(odd);
  bad += wk_selftest(WkCfg{}) == 0;        // an all-zero config must fail (version 0)
  std::printf("host selftest: %s (%d failed)\n", bad ? "FAIL" : "pass", bad);
  return bad != 0;
}
