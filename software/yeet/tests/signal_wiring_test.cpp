#include "yeet/signal_wiring.hpp"

#include <gtest/gtest.h>

namespace {

struct mock_inputs {
    yeet::signal_input<float> input_a;
    yeet::signal_input<float> input_b;
    yeet::signal_input<bool> input_c;
};

struct mock_outputs {
    yeet::signal_output<float> output_x{1.0f};
    yeet::signal_output<float> output_y{2.0f};
    yeet::signal_output<bool> output_z{true};
};

class mock_component {
public:
    mock_inputs& get_inputs() { return _inputs; }
    mock_outputs& get_outputs() { return _outputs; }

private:
    mock_inputs _inputs;
    mock_outputs _outputs;
};

}  // namespace

TEST(signal_wiring_test, wire_inputs_basic)
{
    mock_inputs inputs;
    mock_outputs outputs;

    yeet::wire_inputs(inputs)
        .from(outputs.output_x)
        .to(&mock_inputs::input_a)
        .from(outputs.output_y)
        .to(&mock_inputs::input_b)
        .from(outputs.output_z)
        .to(&mock_inputs::input_c);

    EXPECT_FALSE(inputs.input_a.empty());
    EXPECT_FALSE(inputs.input_b.empty());
    EXPECT_FALSE(inputs.input_c.empty());

    EXPECT_FLOAT_EQ(inputs.input_a.read(), 1.0f);
    EXPECT_FLOAT_EQ(inputs.input_b.read(), 2.0f);
    EXPECT_TRUE(inputs.input_c.read());

    outputs.output_x.write(42.0f);
    EXPECT_FLOAT_EQ(inputs.input_a.read(), 42.0f);
}

TEST(signal_wiring_test, wire_component)
{
    mock_component source;
    mock_component target;

    yeet::wire(target)
        .from(source.get_outputs().output_x)
        .to(&mock_inputs::input_a)
        .from(source.get_outputs().output_y)
        .to(&mock_inputs::input_b)
        .from(source.get_outputs().output_z)
        .to(&mock_inputs::input_c);

    EXPECT_FALSE(target.get_inputs().input_a.empty());
    EXPECT_FLOAT_EQ(target.get_inputs().input_a.read(), 1.0f);

    source.get_outputs().output_x.write(99.0f);
    EXPECT_FLOAT_EQ(target.get_inputs().input_a.read(), 99.0f);
}

TEST(signal_wiring_test, wire_multiple_sources)
{
    mock_component source_a;
    mock_component source_b;
    mock_component target;

    source_a.get_outputs().output_x.write(10.0f);
    source_b.get_outputs().output_y.write(20.0f);

    yeet::wire(target)
        .from(source_a.get_outputs().output_x)
        .to(&mock_inputs::input_a)
        .from(source_b.get_outputs().output_y)
        .to(&mock_inputs::input_b);

    EXPECT_FLOAT_EQ(target.get_inputs().input_a.read(), 10.0f);
    EXPECT_FLOAT_EQ(target.get_inputs().input_b.read(), 20.0f);
}

TEST(signal_wiring_test, wire_mocked_values)
{
    mock_component target;

    yeet::wire(target)
        .mocked(42.0f)
        .to(&mock_inputs::input_a)
        .mocked(99.0f)
        .to(&mock_inputs::input_b)
        .mocked(true)
        .to(&mock_inputs::input_c);

    EXPECT_FLOAT_EQ(target.get_inputs().input_a.read(), 42.0f);
    EXPECT_FLOAT_EQ(target.get_inputs().input_b.read(), 99.0f);
    EXPECT_TRUE(target.get_inputs().input_c.read());
}

TEST(signal_wiring_test, wire_mixed_connections_and_mocks)
{
    mock_component source;
    mock_component target;

    source.get_outputs().output_x.write(10.0f);

    yeet::wire(target)
        .from(source.get_outputs().output_x)
        .to(&mock_inputs::input_a)
        .mocked(77.0f)
        .to(&mock_inputs::input_b)
        .mocked(false)
        .to(&mock_inputs::input_c);

    EXPECT_FLOAT_EQ(target.get_inputs().input_a.read(), 10.0f);
    EXPECT_FLOAT_EQ(target.get_inputs().input_b.read(), 77.0f);
    EXPECT_FALSE(target.get_inputs().input_c.read());

    // Verify connected signal updates while mocked stays constant
    source.get_outputs().output_x.write(25.0f);
    EXPECT_FLOAT_EQ(target.get_inputs().input_a.read(), 25.0f);
    EXPECT_FLOAT_EQ(target.get_inputs().input_b.read(), 77.0f);
}
