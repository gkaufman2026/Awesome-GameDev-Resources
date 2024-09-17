#include <iostream>
#include <istream>
// REMOVED DEFINES FOR READABILITY
#define n 624

// NOT USING AS MERSENNE TWISTER IS NOT RELEVANT TO TESTS
const std::string TEST_FOLDER = "\\tests\\";

struct MersenneTwister {
  uint32_t stateArray[n];
  int stateIndex;
};

void initializeState(MersenneTwister* state, uint32_t seed) {
  int i;
  uint32_t* stateArray = &state->stateArray[0];
  stateArray[0] = seed;

  for (i = 1; i < n; i++) {
    seed = 1812433253UL * (seed ^ seed >> 30) + i;
    stateArray[i] = seed;
  }

  state->stateIndex = 0;
}

uint32_t permute(MersenneTwister* state) {
  uint32_t* stateArray = &state->stateArray[0];
  int k = state->stateIndex;

  int j = k - (n - 1);
  if (j < 0) {
    j += n;
  }

  uint32_t x = stateArray[k] & (0xffffffffUL << 31) | stateArray[j] & (0xffffffffUL >> 1);
  uint32_t xA = x >> 1;

  if (x & 0x00000001UL) {
    xA ^= 0x9908b0dfUL;
  }

  j = k - 227;
  if (j < 0) {
    j += n;
  }

  x = stateArray[j] ^ xA;
  stateArray[k++] = x;
  if (k >= n) {
    k = 0;
  }

  state->stateIndex = k;

  uint32_t y = x ^ (x >> 11);
  y = y ^ (y << 7) & 0x9d2c5680UL;
  y = y ^ (y << 15) & 0xefc60000UL;
  const uint32_t z = y ^ y >> 1;
  return z;
}

int main() {
  unsigned int seed, N, min, max, i;
  std::cin >> seed >> N >> min >> max;
  MersenneTwister state{};
  initializeState(&state, seed);

  const auto result = permute(&state);
  std::cout << "value = min + (result % (max - min + 1))" << std::endl;
  std::cout << "value = " << min << " + (" << result << " % (" << max << " - " << min << " + " << 1 << "))" << std::endl;
  std::cout << "value = " << min << " + (" << result << " % " << max - min + 1 << ")" << std::endl;
  std::cout << "value = " << min << " + " << result % (max - min + 1) << std::endl;
  std::cout << "value = " << min + (result % (max - min + 1)) << std::endl;
}

// XOR
// T | T = F
// F | F = T
// T | F = F

// 0 0 1 0 1
// 1 0 0 1 1
// ---------
// 1 0 1 1 0
