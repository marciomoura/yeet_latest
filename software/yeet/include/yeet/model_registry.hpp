#ifndef YEET_MODEL_REGISTRY_HPP
#define YEET_MODEL_REGISTRY_HPP

#include <map>
#include <string_view>

#include "parameter.hpp"
#include "signal.hpp"
#include "utils/low_level_access.hpp"

namespace yeet {

struct model_registry_entry {
    signal_registry signals{};
    parameter_registry parameters{};
};

class model_registry {
public:
    model_registry() = default;
    ~model_registry() = default;
    model_registry(const model_registry&) = delete;
    model_registry& operator=(const model_registry&) = delete;
    model_registry(model_registry&&) = default;
    model_registry& operator=(model_registry&&) = default;

    void insert(std::string_view name, model_registry_entry&& entry)
    {
        if (_registry_map.find(name) != _registry_map.end()) {
            assert(false && "Entry already exists in registry");
            return;
        }

        _registry_map[name] = std::move(entry);
    }

    std::optional<low_level_access> get_signal(std::string_view name) const
    {
        auto it = _registry_map.find(name);

        if (it == _registry_map.end()) {
            return {};
        }

        return it->second.signals.get(name);
    }

    std::optional<low_level_access> get_parameter(std::string_view name) const
    {
        auto it = _registry_map.find(name);

        if (it == _registry_map.end()) {
            return {};
        }

        return it->second.parameters.get(name);
    }

    void iterate_entries(
        std::function<void(std::string_view group, std::string_view name, const low_level_access& llaccess)> func) const
    {
        for (auto& [group, entry] : _registry_map) {
            for (auto it = entry.signals.cbegin(); it != entry.signals.cend(); ++it) {
                func(group, it->first, it->second);
            }

            for (auto it = entry.parameters.cbegin(); it != entry.parameters.cend(); ++it) {
                func(group, it->first, it->second);
            }
        }
    }

private:
    std::map<std::string_view, model_registry_entry> _registry_map{};
};

}  // namespace yeet

#endif  // YEET_MODEL_REGISTRY_HP
