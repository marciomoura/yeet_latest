
#include "yeet/utils/low_level_access.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>

TEST(low_level_access, empty)
{
    low_level_access llaccess;
    EXPECT_TRUE(llaccess.empty());
}

TEST(low_level_access, set_storage_type)
{
    std::int32_t data = 0;
    storage_type<std::int32_t> storage = storage_type<std::int32_t>(&data);
    std::int32_t* internal_storage = storage.get_internal_storage();

    low_level_access llaccess{storage};

    EXPECT_FALSE(llaccess.empty());

    EXPECT_EQ(llaccess.read<std::int32_t>(), *internal_storage);
    EXPECT_EQ(llaccess.read<std::int32_t>(), 0);

    llaccess.write(10);
    EXPECT_EQ(llaccess.read<std::int32_t>(), *internal_storage);
    EXPECT_EQ(llaccess.read<std::int32_t>(), 10);

    *internal_storage = 20;
    EXPECT_EQ(llaccess.read<std::int32_t>(), *internal_storage);
    EXPECT_EQ(llaccess.read<std::int32_t>(), 20);

    // test other data types
    EXPECT_EQ(llaccess.read<std::uint32_t>(), 20);
    EXPECT_EQ(llaccess.read<std::uint16_t>(), 20);
    EXPECT_EQ(llaccess.read<std::uint8_t>(), 20);
    EXPECT_EQ(llaccess.read<std::int32_t>(), 20);
    EXPECT_EQ(llaccess.read<std::int16_t>(), 20);
    EXPECT_EQ(llaccess.read<std::int8_t>(), 20);
}

// test data type combinations to see the limitations of this approach.
