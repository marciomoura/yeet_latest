#ifndef YEET_UTILS_LOW_LEVEL_ACCESS_REGISTRY_HPP
#define YEET_UTILS_LOW_LEVEL_ACCESS_REGISTRY_HPP

#include <map>
#include <optional>
#include <string_view>

#include "low_level_access.hpp"

class low_level_access_registry {
public:
    low_level_access_registry() = default;
    ~low_level_access_registry() = default;

    bool empty() const { return _registry_map.empty(); }

    void clear() { _registry_map.clear(); }

    void insert(std::string_view name, low_level_access llaccess)
    {
        if (_registry_map.find(name) != _registry_map.end()) {
            assert(false && "Entry already exists in registry");
            return;
        }

        _registry_map[name] = llaccess;
    }

    std::optional<low_level_access> get(std::string_view name) const
    {
        auto it = _registry_map.find(name);

        if (it == _registry_map.end()) {
            return {};
        }

        return it->second;
    }

    auto begin() { return _registry_map.begin(); }
    auto end() { return _registry_map.end(); }

    auto cbegin() const { return _registry_map.cbegin(); }
    auto cend() const { return _registry_map.cend(); }

private:
    std::map<std::string_view, low_level_access> _registry_map{};
};

#endif  // YEET_UTILS_LOW_LEVEL_ACCESS_REGISTRY_HPP