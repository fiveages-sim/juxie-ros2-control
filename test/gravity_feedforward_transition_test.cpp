#include <gtest/gtest.h>
#include "juxie_ros2_control/gravity_feedforward_transition.h"

using juxie_ros2_control::GravityFeedforwardTransition;

TEST(GravityTransition, StartupKeepsFullTableCompensation) {
    GravityFeedforwardTransition blend;
    EXPECT_DOUBLE_EQ(blend.topicWeight(false, 0.0, 1.0), 0.0);
    const double weight = blend.topicWeight(true, 10.0, 1.0);
    EXPECT_DOUBLE_EQ(weight, 0.0);
    EXPECT_DOUBLE_EQ((1.0 - weight) * 6000.0 + weight * 6500.0, 6000.0);
}

TEST(GravityTransition, AcquiresTopicOnceWithoutRestartingOnEveryMessage) {
    GravityFeedforwardTransition blend;
    blend.topicWeight(true, 10.0, 1.0);
    EXPECT_NEAR(blend.topicWeight(true, 10.25, 1.0), 0.15625, 1e-12);
    EXPECT_NEAR(blend.topicWeight(true, 10.5, 1.0), 0.5, 1e-12);
    EXPECT_DOUBLE_EQ(blend.topicWeight(true, 11.0, 1.0), 1.0);
    EXPECT_DOUBLE_EQ(blend.topicWeight(true, 12.0, 1.0), 1.0);
}

TEST(GravityTransition, TimeoutImmediatelyRestoresTableAndReacquisitionRamps) {
    GravityFeedforwardTransition blend;
    blend.topicWeight(true, 0.0, 1.0);
    EXPECT_DOUBLE_EQ(blend.topicWeight(true, 1.0, 1.0), 1.0);
    EXPECT_DOUBLE_EQ(blend.topicWeight(false, 2.0, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(blend.topicWeight(true, 3.0, 1.0), 0.0);
    EXPECT_NEAR(blend.topicWeight(true, 3.5, 1.0), 0.5, 1e-12);
}

TEST(GravityTransition, DisabledTransitionUsesTopicImmediately) {
    GravityFeedforwardTransition blend;
    EXPECT_DOUBLE_EQ(blend.topicWeight(true, 0.0, 0.0), 1.0);
    EXPECT_DOUBLE_EQ(blend.topicWeight(false, 1.0, 0.0), 0.0);
}

TEST(GravityTransition, ReactivationStartsANewTransition) {
    GravityFeedforwardTransition blend;
    blend.topicWeight(true, 0.0, 1.0);
    EXPECT_DOUBLE_EQ(blend.topicWeight(true, 1.0, 1.0), 1.0);
    blend.reset();
    EXPECT_DOUBLE_EQ(blend.topicWeight(true, 2.0, 1.0), 0.0);
}
