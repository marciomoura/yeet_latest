#include <gtest/gtest.h>

#include "yeet/application_datamodel.hpp"
#include "yeet/data_model_connector.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"

namespace yeet {

struct example_a_inputs {
    signal_input<double> ex_a_input_1;
    signal_input<double> ex_a_input_2;
};

struct example_a_outputs {
    signal_output<double> ex_a_output_1;
    signal_output<double> ex_a_output_2;
};

using example_a_datamodel = signal_flow_datamodel<example_a_inputs, example_a_outputs>;

struct example_b_inputs {
    signal_input<double> ex_b_input_1;
    signal_input<double> ex_b_input_2;
};

struct example_b_outputs {
    signal_output<double> ex_b_output_1;
    signal_output<double> ex_b_output_2;
};

using example_b_datamodel = signal_flow_datamodel<example_b_inputs, example_b_outputs>;

// class provides an a to b connection
class connect_a_to_b : public data_model_connector<example_b_datamodel, example_a_datamodel> {
public:
    connect_a_to_b(example_b_datamodel& b, example_a_datamodel& a)
        : data_model_connector(b.get_inputs(), a.get_outputs()) {}

    void connect() override {
        signals_connector::connect_output_to_input(get_outputs().ex_a_output_1, get_inputs().ex_b_input_1);
        signals_connector::connect_output_to_input(get_outputs().ex_a_output_2, get_inputs().ex_b_input_2);
    }
};

class connect_b_to_a : public data_model_connector<example_a_datamodel, example_b_datamodel> {
public:
    connect_b_to_a(example_a_datamodel& a, example_b_datamodel& b)
        : data_model_connector(a.get_inputs(), b.get_outputs()) {}

    void connect() override {
        signals_connector::connect_output_to_input(get_outputs().ex_b_output_1, get_inputs().ex_a_input_1);
        signals_connector::connect_output_to_input(get_outputs().ex_b_output_2, get_inputs().ex_a_input_2);
    }
};

struct example_c_inputs {
    signal_input<double> ex_c_input_1;
    signal_input<double> ex_c_input_2;
};

struct example_c_outputs {
    signal_output<double> ex_c_output_1;
    signal_output<double> ex_c_output_2;
};

using example_c_datamodel = signal_flow_datamodel<example_c_inputs, example_c_outputs>;

struct example_d_inputs {
    signal_input<double> ex_d_input_1;
    signal_input<double> ex_d_input_2;
    signal_input<double> ex_d_input_3;
    signal_input<double> ex_d_input_4;
};

struct example_d_outputs {
    signal_output<double> ex_d_output_1;
    signal_output<double> ex_d_output_2;
};

using example_d_datamodel = signal_flow_datamodel<example_d_inputs, example_d_outputs>;

class connect_b_and_c_to_d
    : public data_model_connector<example_d_datamodel, example_b_datamodel, example_c_datamodel> {
public:
    connect_b_and_c_to_d(example_d_datamodel& d, example_b_datamodel& b, example_c_datamodel& c)
        : data_model_connector(d.get_inputs(), b.get_outputs(), c.get_outputs()) {}

    void connect() override {
        signals_connector::connect_output_to_input(get_outputs<example_b_datamodel>().ex_b_output_1,
                                                   get_inputs().ex_d_input_1);
        signals_connector::connect_output_to_input(get_outputs<example_b_datamodel>().ex_b_output_2,
                                                   get_inputs().ex_d_input_2);

        signals_connector::connect_output_to_input(get_outputs<example_c_datamodel>().ex_c_output_1,
                                                   get_inputs().ex_d_input_3);
        signals_connector::connect_output_to_input(get_outputs<example_c_datamodel>().ex_c_output_2,
                                                   get_inputs().ex_d_input_4);
    }
};

}  // namespace yeet

TEST(datamodel_test, connect_datamodel) {
    // Construct the A model
    yeet::example_a_inputs ex_a_inputs;
    yeet::example_a_outputs ex_a_outputs;

    yeet::signal_flow_datamodel ex_a_datamodel(ex_a_inputs, ex_a_outputs);

    // Construct the B model
    yeet::example_b_inputs ex_b_inputs;
    yeet::example_b_outputs ex_b_outputs;

    yeet::signal_flow_datamodel ex_b_datamodel(ex_b_inputs, ex_b_outputs);

    // Provides an A -> B connection flow
    yeet::connect_a_to_b a_to_b(ex_b_datamodel, ex_a_datamodel);
    // Provides a B -> A connection flow
    yeet::connect_b_to_a b_to_a(ex_a_datamodel, ex_b_datamodel);

    // Expect that B is empty
    EXPECT_TRUE(ex_b_inputs.ex_b_input_1.empty());
    EXPECT_TRUE(ex_b_inputs.ex_b_input_2.empty());
    // Perform connection of A -> B signals
    a_to_b.connect();
    // Expect that B is not empty
    EXPECT_FALSE(ex_b_inputs.ex_b_input_1.empty());
    EXPECT_FALSE(ex_b_inputs.ex_b_input_2.empty());
    // Evaluate that writing to A affects B
    ex_a_outputs.ex_a_output_1.write(1.0);
    ex_a_outputs.ex_a_output_2.write(2.0);
    EXPECT_FLOAT_EQ(ex_b_inputs.ex_b_input_1.read(), 1.0);
    EXPECT_FLOAT_EQ(ex_b_inputs.ex_b_input_2.read(), 2.0);

    // Expect that A inputs are still empty
    EXPECT_TRUE(ex_a_inputs.ex_a_input_1.empty());
    EXPECT_TRUE(ex_a_inputs.ex_a_input_2.empty());
    // Perform connection of B -> A signals
    b_to_a.connect();
    // Expect that A is not empty
    EXPECT_FALSE(ex_a_inputs.ex_a_input_1.empty());
    EXPECT_FALSE(ex_a_inputs.ex_a_input_2.empty());
    // Evaluate that writing to B affects A
    ex_b_outputs.ex_b_output_1.write(3.0);
    ex_b_outputs.ex_b_output_2.write(4.0);
    EXPECT_FLOAT_EQ(ex_a_inputs.ex_a_input_1.read(), 3.0);
    EXPECT_FLOAT_EQ(ex_a_inputs.ex_a_input_2.read(), 4.0);
}

TEST(datamodel_test, connect_b_and_c_to_a) {
    // Construct the B model
    yeet::example_b_inputs ex_b_inputs;
    yeet::example_b_outputs ex_b_outputs;

    yeet::example_b_datamodel ex_b_datamodel(ex_b_inputs, ex_b_outputs);

    // Construct the C model
    yeet::example_c_inputs ex_c_inputs;
    yeet::example_c_outputs ex_c_outputs;

    yeet::example_c_datamodel ex_c_datamodel(ex_c_inputs, ex_c_outputs);

    // Construct the D model
    yeet::example_d_inputs ex_d_inputs;
    yeet::example_d_outputs ex_d_outputs;

    yeet::example_d_datamodel ex_d_datamodel(ex_d_inputs, ex_d_outputs);

    // Provides a B and C -> D connection flow
    yeet::connect_b_and_c_to_d b_and_c_to_d(ex_d_datamodel, ex_b_datamodel, ex_c_datamodel);

    // Expect that D is empty
    EXPECT_TRUE(ex_d_inputs.ex_d_input_1.empty());
    EXPECT_TRUE(ex_d_inputs.ex_d_input_2.empty());
    EXPECT_TRUE(ex_d_inputs.ex_d_input_3.empty());
    EXPECT_TRUE(ex_d_inputs.ex_d_input_4.empty());
    // Perform connection of B and C -> D signals
    b_and_c_to_d.connect();
    // Expect that D is not empty
    EXPECT_FALSE(ex_d_inputs.ex_d_input_1.empty());
    EXPECT_FALSE(ex_d_inputs.ex_d_input_2.empty());
    EXPECT_FALSE(ex_d_inputs.ex_d_input_3.empty());
    EXPECT_FALSE(ex_d_inputs.ex_d_input_4.empty());
    // Evaluate that writing to B and C affects D
    ex_b_outputs.ex_b_output_1.write(1.0);
    ex_b_outputs.ex_b_output_2.write(2.0);
    ex_c_outputs.ex_c_output_1.write(3.0);
    ex_c_outputs.ex_c_output_2.write(4.0);
    EXPECT_FLOAT_EQ(ex_d_inputs.ex_d_input_1.read(), 1.0);
    EXPECT_FLOAT_EQ(ex_d_inputs.ex_d_input_2.read(), 2.0);
    EXPECT_FLOAT_EQ(ex_d_inputs.ex_d_input_3.read(), 3.0);
    EXPECT_FLOAT_EQ(ex_d_inputs.ex_d_input_4.read(), 4.0);
}