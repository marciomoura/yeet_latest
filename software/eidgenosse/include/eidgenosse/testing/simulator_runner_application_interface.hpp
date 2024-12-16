#ifndef EIDGENOSSE_TESTING_SIMULATOR_RUNNER_APPLICATION_INTERFACE
#define EIDGENOSSE_TESTING_SIMULATOR_RUNNER_APPLICATION_INTERFACE

class simulator_runner_application_interface {
public:
    virtual double get_sampling_time() = 0;

    virtual void run() = 0;

    virtual void initialize() = 0;

    virtual void stop() = 0;

    // virtual void register_data_sampler_entries(data_sampler& data_sampler) = 0;
};

#endif  // EIDGENOSSE_TESTING_SIMULATOR_RUNNER_APPLICATION_INTERFACE