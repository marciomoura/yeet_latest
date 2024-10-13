/**
 * @copyright Techperts AG.
 * @date 10-12-2024
 * @brief
 */

#ifndef EIDGENOSSE_TESTING_TEST
#define EIDGENOSSE_TESTING_TEST

#include <cstdint>
#include <map>
#include <memory>

class data_sampler {
public:
private:
};

class cyclic_application {
public:
    virtual void run() = 0;

private:
    data_sampler data_sampler_;
};

class simulator {
public:
    // register new cyclic_application
    void register_cyclic_application(std::uint32_t ticks_per_second,
                                     std::shared_ptr<cyclic_application> cyclic_application) {
        cyclic_applications_[ticks_per_second] = cyclic_application;
    }

    void simulate_for(float time_seconds) {
        for (float time = 0.0f; time < time_seconds; time += 0.1f) {
            run_once();
        }
    }

private:
    void run_once() {
        for (auto& [id, cyclic_application] : cyclic_applications_) {
            cyclic_application->run();
        }
    }

    std::map<std::uint32_t, std::shared_ptr<cyclic_application>> cyclic_applications_;
};

#endif  // EIDGENOSSE_TESTING_TEST