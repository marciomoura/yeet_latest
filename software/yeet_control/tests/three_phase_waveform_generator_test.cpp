#include "yeet_control/three_phase_waveform_generator.hpp"

#include <gtest/gtest.h>

#include "yeet_testing/data_sample_json_post_processing.hpp"
#include "yeet_testing/data_sampler.hpp"

namespace {

using namespace yeet::control;

TEST(ThreePhaseWaveformGeneratorTest, BasicInitialization) {
    Three_Phase_Waveform_Generator generator(0.0001);

    abc<double> signal{};

    data_sampler sampler{};

    float time = 0;
    sampler.register_entry(data_sampler_entry{"time", [&]() { return time; }});
    sampler.register_entry(data_sampler_entry{"a", [&]() { return signal[0]; }});
    sampler.register_entry(data_sampler_entry{"b", [&]() { return signal[1]; }});
    sampler.register_entry(data_sampler_entry{"c", [&]() { return signal[2]; }});

    sampler.allocate(1000);

    for (std::size_t i = 0; i < 1000; ++i) {
        time = i * 0.01;
        generator.update();

        signal = generator.get_signal_abc();
        sampler.sample();
    }

    data_sample_json_post_processing post_processor{};
    sampler.dump("ThreePhaseWaveformGeneratorTest", "BasicInitialization", post_processor);
}

}  // namespace