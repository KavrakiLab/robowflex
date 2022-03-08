/* Author: Zachary Kingston */

#include <gtest/gtest.h>

#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/yaml.h>

using namespace robowflex;

TEST(YAML, isNode)
{
    YAML::Node node = YAML::Load("key1: null\nkey2: \"test\"");
    ASSERT_EQ(false, IO::isNode(node["key1"]));
    ASSERT_EQ(true, IO::isNode(node["key2"]));
    ASSERT_EQ(false, IO::isNode(node["key3"]));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
