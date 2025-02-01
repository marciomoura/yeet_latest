#pragma once

#include <memory>
#include <tuple>
#include <type_traits>

#include "yeet/model_registry.hpp"
#include "yeet/testing/simulator.hpp"

namespace yeet {

template <typename T>
class parameter_set {
public:
    explicit parameter_set(T& params) : _params(params) {}

    T& get() { return _params; }
    const T& get() const { return _params; }

    // Allow direct access to members through ->
    T* operator->() { return &_params; }
    const T* operator->() const { return &_params; }

private:
    T& _params;
};

// Base concept for closed loop configurations
template <typename ParameterTypes>
class closed_loop_configuration {
public:
    virtual ~closed_loop_configuration() = default;
    virtual void register_applications(simulator& sim) = 0;
    virtual void register_datamodel(model_registry& registry) = 0;
    virtual ParameterTypes create_parameter_collection() = 0;
};

// Generic parameter management
template <typename... Params>
class parameter_collection {
public:
    parameter_collection(Params&... args) : _parameters(parameter_set<Params>(args)...) {}

    template <typename T>
    parameter_set<T>& get() {
        static_assert((std::is_same_v<T, Params> || ...), "Requested type not found in parameter_collection");
        return std::get<parameter_set<T>>(_parameters);
    }

private:
    std::tuple<parameter_set<Params>...> _parameters;
};

// Generic simulation runner
template <typename ParametersTypes>
class simulation_runner {
public:
    explicit simulation_runner(simulator& sim, closed_loop_configuration<ParametersTypes>& config)
        : _simulator(sim), _configuration(config), _parameters(config.create_parameter_collection()) {}

    void initialize() { _simulator.initialize(); }

    void run_for(double time) { _simulator.simulate_for(time); }

    void dump_results(testing_info_interface* info) { _simulator.dump_json(info); }

    template <typename T>
    parameter_set<T>& get_parameters() {
        return _parameters.template get<T>();
    }

private:
    simulator& _simulator;
    closed_loop_configuration<ParametersTypes>& _configuration;
    ParametersTypes _parameters;
};

// Generic simulation builder
template <typename ParametersTypes>
class simulation_builder {
public:
    static std::unique_ptr<simulation_runner<ParametersTypes>> create(
        simulator& sim, closed_loop_configuration<ParametersTypes>& config) {
        // Register all available applications
        // and data models into the simulator
        configure_simulator(sim, config);

        return std::make_unique<simulation_runner<ParametersTypes>>(sim, config);
    }

private:
    static void configure_simulator(simulator& sim, closed_loop_configuration<ParametersTypes>& config) {
        // Register all available applications
        config.register_applications(sim);

        // Register all available data models into the simulator
        // sampler for data analysis
        model_registry model;
        config.register_datamodel(model);
        sim.register_data_sampler_entries(model);
    }
};

}  // namespace yeet