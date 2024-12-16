#ifndef EIDGENOSSE_MODEL_REGISTRY_HPP
#define EIDGENOSSE_MODEL_REGISTRY_HPP

#include <functional>
#include <map>
#include <string>

#include "parameter.hpp"
#include "signal.hpp"
#include "utils/low_level_access.hpp"

struct model_registry_entry {
    signal_registry signal_registry{};
    parameter_registry parameter_registry{};
};

class model_registry {
public:
    model_registry() = default;
    ~model_registry() = default;
    model_registry(const model_registry&) = delete;
    model_registry& operator=(const model_registry&) = delete;
    model_registry(model_registry&&) = default;
    model_registry& operator=(model_registry&&) = default;

    void insert(const std::string& name, model_registry_entry&& entry) {
        if (_registry_map.find(name) != _registry_map.end()) {
            assert(false && "Entry already exists in registry");
            return;
        }

        _registry_map[name] = std::move(entry);
    }

    std::optional<low_level_access> get_signal(const std::string& name) const {
        auto it = _registry_map.find(name);

        if (it == _registry_map.end()) {
            assert(false && "Entry not found");
            return {};
        }

        return it->second.signal_registry.get(name);
    }

    std::optional<low_level_access> get_parameter(const std::string& name) const {
        auto it = _registry_map.find(name);

        if (it == _registry_map.end()) {
            assert(false && "Entry not found");
            return {};
        }

        return it->second.parameter_registry.get(name);
    }

    void iterate_entries(
        std::function<void(const std::string& group, const std::string& name, const low_level_access& llaccess)> func)
        const {
        for (auto& [group, entry] : _registry_map) {
            for (auto it = entry.signal_registry.cbegin(); it != entry.signal_registry.cend(); ++it) {
                func(group, it->first, it->second);
            }

            for (auto it = entry.parameter_registry.cbegin(); it != entry.parameter_registry.cend(); ++it) {
                func(group, it->first, it->second);
            }
        }
    }

private:
    std::map<std::string, model_registry_entry> _registry_map{};
};

#endif  // EIDGENOSSE_MODEL_REGISTRY_HP
