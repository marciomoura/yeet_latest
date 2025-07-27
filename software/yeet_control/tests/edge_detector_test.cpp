#include <gtest/gtest.h>

#include "yeet_control/edge_detector.hpp"

using namespace yeet::control;

TEST(EdgeDetectorTest, InitialStateIsFalse) {
    edge_detector detector;
    EXPECT_FALSE(detector.is_rising_edge());
    EXPECT_FALSE(static_cast<bool>(detector));
}

TEST(EdgeDetectorTest, DetectsRisingEdge) {
    edge_detector detector;
    detector.update(false);
    EXPECT_TRUE(detector.update(true));
    EXPECT_TRUE(detector.is_rising_edge());
    EXPECT_TRUE(static_cast<bool>(detector));
}

TEST(EdgeDetectorTest, NoEdgeOnStableHigh) {
    edge_detector detector;
    detector.update(true);  // Set previous state to true
    EXPECT_FALSE(detector.update(true));
    EXPECT_FALSE(detector.is_rising_edge());
}

TEST(EdgeDetectorTest, NoEdgeOnStableLow) {
    edge_detector detector;
    detector.update(false);  // Set previous state to false
    EXPECT_FALSE(detector.update(false));
    EXPECT_FALSE(detector.is_rising_edge());
}

TEST(EdgeDetectorTest, NoEdgeOnFallingEdge) {
    edge_detector detector;
    detector.update(true);  // Set previous state to true
    EXPECT_FALSE(detector.update(false));
    EXPECT_FALSE(detector.is_rising_edge());
}

TEST(EdgeDetectorTest, StateIsCorrectAfterRisingEdge) {
    edge_detector detector;
    detector.update(false);
    detector.update(true);  // Rising edge detected here
    EXPECT_TRUE(detector.is_rising_edge());

    // The next update should clear the flag
    EXPECT_FALSE(detector.update(true));
    EXPECT_FALSE(detector.is_rising_edge());
}

TEST(EdgeDetectorTest, FullSequence) {
    edge_detector detector;
    // false -> false
    EXPECT_FALSE(detector.update(false));
    EXPECT_FALSE(detector.is_rising_edge());
    // false -> true (Rising)
    EXPECT_TRUE(detector.update(true));
    EXPECT_TRUE(detector.is_rising_edge());
    // true -> true
    EXPECT_FALSE(detector.update(true));
    EXPECT_FALSE(detector.is_rising_edge());
    // true -> false (Falling)
    EXPECT_FALSE(detector.update(false));
    EXPECT_FALSE(detector.is_rising_edge());
    // false -> true (Rising)
    EXPECT_TRUE(detector.update(true));
    EXPECT_TRUE(detector.is_rising_edge());
}