#ifndef YEET_DATA_MODEL_CONNECTOR_HPP
#define YEET_DATA_MODEL_CONNECTOR_HPP

#include <tuple>
#include <type_traits>

namespace yeet {

class data_model_connector_interface {
public:
    virtual void connect() = 0;
};

/**
 * @brief Wrapper struct to identify input datamodel types
 */
template <typename T>
struct inputs {
    using type = T;
    using inputs_type = typename T::inputs_type;
};

/**
 * @brief Wrapper struct to identify output datamodel types
 */
template <typename T>
struct outputs {
    using type = T;
    using outputs_type = typename T::outputs_type;
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

// Helper to extract the actual datamodel type from inputs<T>
template <typename T>
struct extract_inputs_type;

template <typename T>
struct extract_inputs_type<inputs<T>> {
    using type = T;
    using inputs_type = typename T::inputs_type;
};

// Helper to extract the actual datamodel type from outputs<T>
template <typename T>
struct extract_outputs_type;

template <typename T>
struct extract_outputs_type<outputs<T>> {
    using type = T;
    using outputs_type = typename T::outputs_type;
};

}  // namespace utils

/**
 * @brief Generic data model connector with cleaner template interface
 *
 * Usage: data_model_connector<inputs<ModelA>, outputs<ModelB>, outputs<ModelC>>
 *
 * This creates a connector that takes inputs from ModelA and connects them to
 * outputs from ModelB and ModelC.
 */
template <typename InputsWrapper, typename... OutputsWrappers>
class data_model_connector : public data_model_connector_interface {
    static_assert(std::is_same_v<InputsWrapper, inputs<typename InputsWrapper::type>>,
        "First template parameter must be inputs<SomeDataModel>");
    static_assert((std::is_same_v<OutputsWrappers, outputs<typename OutputsWrappers::type>> && ...),
        "All subsequent template parameters must be outputs<SomeDataModel>");

public:
    using inputs_model_type = typename utils::extract_inputs_type<InputsWrapper>::type;
    using inputs_type = typename utils::extract_inputs_type<InputsWrapper>::inputs_type;

    data_model_connector(
        inputs_type& inputs, typename utils::extract_outputs_type<OutputsWrappers>::outputs_type&... outputs)
        : _inputs(inputs), _outputs(outputs...)
    {
    }

    virtual void connect() = 0;

protected:
    using first_output_wrapper = typename utils::fetch_parameter_pack_first_type<OutputsWrappers...>::type;
    using first_output_type = typename utils::extract_outputs_type<first_output_wrapper>::outputs_type;

    /**
     * @brief Get reference to the inputs structure
     */
    inputs_type& get_inputs() noexcept { return _inputs; }

    /**
     * @brief Get reference to outputs from a specific datamodel
     *
     * @tparam T The outputs wrapper type (e.g., outputs<SomeDataModel>)
     * @return Reference to the outputs structure of the specified datamodel
     */
    template <typename T = first_output_wrapper>
    typename utils::extract_outputs_type<outputs<T>>::outputs_type& get_outputs() noexcept
    {
        // static_assert(
        // (std::is_same_v<T, outputs<OutputsWrappers>> || ...), "Requested output type not found in OutputsWrappers");
        return std::get<typename utils::extract_outputs_type<outputs<T>>::outputs_type&>(_outputs);
    }

private:
    inputs_type& _inputs;
    std::tuple<typename utils::extract_outputs_type<OutputsWrappers>::outputs_type&...> _outputs;
};

}  // namespace yeet

#endif  // YEET_DATA_MODEL_CONNECTOR_HPP
