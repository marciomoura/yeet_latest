#ifndef YEET_DATA_MODEL_CONNECTOR_HPP
#define YEET_DATA_MODEL_CONNECTOR_HPP

#include <tuple>
#include <type_traits>

namespace yeet {

class data_model_connector_interface {
public:
    virtual void connect() = 0;
};

namespace utils {
template <typename First, typename... Rest>
struct fetch_parameter_pack_first_type {
    using type = First;
};

template <typename First>
struct fetch_parameter_pack_first_type<First> {
    using type = First;
};
}  // namespace utils

template <typename InputsModel, typename... OutputsModel>
class data_model_connector : public data_model_connector_interface {
public:
    data_model_connector(typename InputsModel::inputs_type& inputs, typename OutputsModel::outputs_type&... outputs)
        : _inputs(inputs), _outputs(outputs...) {}

    virtual void connect() = 0;

protected:
    using first_output_type = typename utils::fetch_parameter_pack_first_type<OutputsModel...>::type;

    typename InputsModel::inputs_type& get_inputs() { return _inputs; }

    template <typename T = first_output_type>
    typename T::outputs_type& get_outputs() {
        static_assert((std::is_same_v<T, OutputsModel> || ...), "Requested type not found in OutputsModel");
        return std::get<typename T::outputs_type&>(_outputs);
    }

private:
    typename InputsModel::inputs_type& _inputs;
    std::tuple<typename OutputsModel::outputs_type&...> _outputs;
};

}  // namespace yeet

#endif  // YEET_DATA_MODEL_CONNECTOR_HPP
