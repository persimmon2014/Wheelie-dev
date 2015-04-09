// Unit tests for Wheelie.
#include "wheelie.hpp"

#include <gtest/gtest.h>


namespace wheelie {

TEST(WheelieUnitTests, TestCreateNetwork) {
   wheelie::Simulator sim;

   EXPECT_TRUE(sim.sim_ == NULL);
}

}

using namespace wheelie;

int main(int argc, char** argv) {
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}


