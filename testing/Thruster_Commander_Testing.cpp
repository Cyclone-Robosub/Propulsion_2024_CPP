#include "Thruster_Commander.h"
#include <gtest/gtest.h>

TEST(ThrusterCommanderTest, SimpleVertical) {
    testing::internal::CaptureStdout();
    Thruster_Commander thruster_commander = Thruster_Commander();
    thruster_commander.print_info();
    thruster_commander.thrust_compute_fz(5);

    std::string output = testing::internal::GetCapturedStdout(); // TODO: check that output matches expected output
}

TEST(ThrusterCommanderTest, GetPwmNoInterpolation) {
    auto thrusterCommander = new Thruster_Commander();
    double result = thrusterCommander->get_pwm(1, -3.52);
    ASSERT_DOUBLE_EQ(result, 1100);

    result = thrusterCommander->get_pwm(1, -3);
    ASSERT_DOUBLE_EQ(result, 1148);

    result = thrusterCommander->get_pwm(1, -0.03);
    ASSERT_DOUBLE_EQ(result, 1464);

    result = thrusterCommander->get_pwm(1, 0);
    ASSERT_DOUBLE_EQ(result, 1468);

    result = thrusterCommander->get_pwm(1, 0.05);
    ASSERT_DOUBLE_EQ(result, 1536);

    result = thrusterCommander->get_pwm(1, 1);
    ASSERT_DOUBLE_EQ(result, 1652);

    result = thrusterCommander->get_pwm(1, 4.52);
    ASSERT_DOUBLE_EQ(result, 1900);

    delete thrusterCommander;
}

TEST(ThrusterCommanderTest, GetPwmWithInterpolation) {
    auto thrusterCommander = new Thruster_Commander();
    double result = thrusterCommander->get_pwm(1, -3.51);
    ASSERT_NEAR(result, 1102.0000, 0.1);

    result = thrusterCommander->get_pwm(1, -3.46);
    ASSERT_NEAR(result, 1111.0000, 0.1);

    result = thrusterCommander->get_pwm(1, -2.99);
    ASSERT_NEAR(result, 1148.6667, 0.1);

    result = thrusterCommander->get_pwm(1, -0.264);
    ASSERT_NEAR(result, 1419.4667, 0.1);

    result = thrusterCommander->get_pwm(1, -0.029);
    ASSERT_NEAR(result, 1464.1333, 0.1);

    result = thrusterCommander->get_pwm(1, -0.01);
    ASSERT_NEAR(result, 1466.6667, 0.1);

    result = thrusterCommander->get_pwm(1, 0.02);
    ASSERT_NEAR(result, 1533.6000, 0.1);

    result = thrusterCommander->get_pwm(1, 0.051);
    ASSERT_NEAR(result, 1536.4000, 0.1);

    result = thrusterCommander->get_pwm(1, 1.1);
    ASSERT_NEAR(result, 1660.8000, 0.1);

    result = thrusterCommander->get_pwm(1, 4.5);
    ASSERT_NEAR(result, 1891.5556, 0.1);

    result = thrusterCommander->get_pwm(1, 4.525);
    ASSERT_NEAR(result, 1898.0000, 0.1);

    delete thrusterCommander;
}

TEST(ThrusterCommanderTest, GetPwmOutOfBounds) {
    testing::internal::CaptureStderr();
    auto thrusterCommander = new Thruster_Commander();
    double result = thrusterCommander->get_pwm(1, -3.521);
    ASSERT_DOUBLE_EQ(result, 1100);

    result = thrusterCommander->get_pwm(1, -500);
    ASSERT_DOUBLE_EQ(result, 1100);

    result = thrusterCommander->get_pwm(1, 4.531);
    ASSERT_DOUBLE_EQ(result, 1896);

    result = thrusterCommander->get_pwm(1, 500);
    ASSERT_DOUBLE_EQ(result, 1896);

    delete thrusterCommander;
}

TEST(ThrusterCommanderTest, CompareConstructors) {
    ThrusterCommander commander = new ThrusterCommander();
    ThrusterCommander commander2 = new ThrusterCommander(/data/config1.yaml);
    commander.print_info();
    commander2.print_info();
    ASSERT_TRUE(commander.print_info() == commander2.print_info());
}
