// gtest
#include <gtest/gtest.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  srand((int) time(nullptr));
  return RUN_ALL_TESTS();
}