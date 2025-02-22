#pragma once

#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <vector>

#include "simulator_runner_application.hpp"
#include "simulator_runner_application_interface.hpp"

namespace yeet::testing {

class simulator_runner {
public:
    void register_lambda_runner(double sampling_time, std::function<void()> lambda_runner) {
        assert(lambda_runner != nullptr);
        register_application(std::make_shared<simulator_lambda_runner_application>(sampling_time, lambda_runner));
        update_applications_list();
    }

    void initialize() {
        for (auto& [normalized_ticks, apps] : _applications) {
            for (auto& app : apps) {
                app->initialize();
            }
        }
    }

    void run(std::size_t current_tick) {
        for (auto& [normalized_ticks, apps] : _applications) {
            if (current_tick % normalized_ticks == 0) {
                for (auto& app : apps) {
                    app->run();
                }
            }
        }
    }

    double get_fastest_sampling_rate() const { return _fastest_sampling_rate; }

private:
    void register_application(std::shared_ptr<simulator_runner_application_interface> app) {
        _fastest_sampling_rate = std::min(_fastest_sampling_rate, app->get_sampling_time());
        _applications_vector.push_back(app);
    }

    void update_applications_list() {
        _applications.clear();
        for (auto& app : _applications_vector) {
            const std::size_t normalized_ticks = app->get_sampling_time() / _fastest_sampling_rate;
            _applications[normalized_ticks].push_back(app);
        }
    }

    double _fastest_sampling_rate{std::numeric_limits<double>::max()};
    std::vector<std::shared_ptr<simulator_runner_application_interface>> _applications_vector;
    std::map<std::size_t, std::vector<std::shared_ptr<simulator_runner_application_interface>>> _applications;
};

}  // namespace yeet::testing