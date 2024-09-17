// add your imports here
#include <cstdint>
#include <fstream>
#include <iostream>
#include <istream>
#define n 624
#define m 397
#define w 32
#define r 31
#define UMASK (0xffffffffUL << r)
#define LMASK (0xffffffffUL >> (w - r))
#define a 0x9908b0dfUL
#define u 11
#define s 7
#define t 15
#define l 18
#define b 0x9d2c5680UL
#define c 0xefc60000UL
#define f 1812433253UL

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
    seed = f * (seed ^ seed >> w - 2) + i;
    stateArray[i] = seed;
  }

  state->stateIndex = 0;
}

unsigned __int32 permute(MersenneTwister* state) {
  uint32_t* stateArray = &state->stateArray[0];
  int k = state->stateIndex;

  int j = k - (n - 1);
  if (j < 0) {
    j += n;
  }

  uint32_t x = stateArray[k] & UMASK | stateArray[j] & LMASK;
  uint32_t xA = x >> 1;

  if (x & 0x00000001UL) {
    xA ^= a;
  }

  j = k - (n - m);
  if (j < 0) {
    j += n;
  }

  x = stateArray[j] ^ xA;
  stateArray[k++] = x;
  if (k >= n) {
    k = 0;
  }

  state->stateIndex = k;

  uint32_t y = x ^ (x >> u);
  y = y ^ (y << s) & b;
  y = y ^ (y << t) & c;
  const uint32_t z = y ^ y >> 1;
  return z;
}

int main() {
  unsigned int seed, N, min, max;
  std::cin >> seed >> N >> min >> max;
  MersenneTwister state{};
  initializeState(&state, seed);
  const auto res = permute(&state);
  std::cout << min + res % (max - min + 1);
}

// XOR
// T | T = F
// F | F = T
// T | F = F

// 0 0 1 0 1
// 1 0 0 1 1
// ---------
// 1 0 1 1 0
