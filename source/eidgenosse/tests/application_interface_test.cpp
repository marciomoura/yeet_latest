#ifndef EIDGENOSSE_TEST_APPLICATION_INTERFACE_HPP
#define EIDGENOSSE_TEST_APPLICATION_INTERFACE_HPP

#include "eidgenosse/application_interface.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "gmock/gmock.h"

class mock_application_interface : public application_interface {
public:
    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, run, (), (override));
    MOCK_METHOD(void, stop, (), (override));
    MOCK_METHOD(signal_registry, get_signal_registry, (), (override));
    MOCK_METHOD(parameter_registry, get_parameter_registry, (), (override));
};

TEST(application_interface_test, initialize) {
    mock_application_interface mock_app;
    EXPECT_CALL(mock_app, initialize()).Times(1);
    mock_app.initialize();
}

TEST(application_interface_test, run) {
    mock_application_interface mock_app;
    EXPECT_CALL(mock_app, run()).Times(1);
    mock_app.run();
}

TEST(application_interface_test, stop) {
    mock_application_interface mock_app;
    EXPECT_CALL(mock_app, stop()).Times(1);
    mock_app.stop();
}

TEST(application_interface_test, return_empty_registries) {
    mock_application_interface mock_app;

    signal_registry return_signal_registry;
    parameter_registry return_parameter_registry;

    ON_CALL(mock_app, get_signal_registry()).WillByDefault(::testing::Return(return_signal_registry));
    ON_CALL(mock_app, get_parameter_registry()).WillByDefault(::testing::Return(return_parameter_registry));

    EXPECT_CALL(mock_app, get_signal_registry()).Times(1);
    EXPECT_CALL(mock_app, get_parameter_registry()).Times(1);

    signal_registry default_signal_registry = mock_app.get_signal_registry();
    parameter_registry default_parameter_registry = mock_app.get_parameter_registry();

    EXPECT_TRUE(default_signal_registry.empty());
    EXPECT_TRUE(default_parameter_registry.empty());
}

#endif  // EIDGENOSSE_TEST_APPLICATION_INTERFACE_HPP