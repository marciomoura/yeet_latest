#pragma once

#include <string>
#include <vector>

namespace yeet {

struct checker_fail_info {
    std::string name{};
    std::string message{};
    double time{};
};

class checker_failures_post_processing_interface {
public:
    virtual void dump(std::vector<checker_fail_info> const& failures) = 0;
};

}  // namespace yeet