/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vector>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "Policy.hpp"
#include "Exception.hpp"
#include "geopm_test.hpp"

using geopm::Policy;
using geopm::Exception;

template <typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &vec)
{
    for (int ii = 0; ii < vec.size(); ++ii) {
        if (ii != 0) {
            os << ", ";
        }
        os << vec.at(ii);
    }
    return os;
}

::testing::AssertionResult
PoliciesAreSame(const Policy &p1, const Policy &p2)
{
    if (p1 != p2) {
        std::ostringstream msg;
        msg << "{" << p1.policy_names() << "}; {" << p1.to_string(",")
            << "} is not equivalent to {" << p2.policy_names() << "}: {"
            << p2.to_string(",") << "}";
        return ::testing::AssertionFailure() << msg.str();
    }
    return ::testing::AssertionSuccess();
}

::testing::AssertionResult
PoliciesAreNotSame(const Policy &p1, const Policy &p2)
{
    if (p1 == p2) {
        std::ostringstream msg;
        msg << "{" << p1.policy_names() << "}: {" << p1.to_string(",")
            << "} is equivalent to {" << p2.policy_names() << "}: {"
            << p2.to_string(",") << "}";
        return ::testing::AssertionFailure() << msg.str();
    }
    return ::testing::AssertionSuccess();
}

TEST(PolicyTest, construct)
{
    Policy pol1{{}, {}};
    EXPECT_EQ(0, pol1.size());

    std::vector<std::string> names{"one", "two"};
    std::vector<double> values{4.4, 5.5};
    Policy pol2{names, values};
    EXPECT_EQ(2, pol2.size());
    EXPECT_EQ(5.5, pol2[1]);
    EXPECT_EQ(5.5, pol2["two"]);
    Policy pol2a {{"a", "b"}, {4.4, 5.5}};
    EXPECT_TRUE(PoliciesAreNotSame(pol2, pol2a));
    Policy pol2b {{"one", "two"}, {4.4, 5.5}};
    EXPECT_TRUE(PoliciesAreSame(pol2, pol2b));

    Policy pol3 {pol2};
    EXPECT_EQ(2, pol3.size());

    EXPECT_EQ(4.4, pol3[0]);

    EXPECT_EQ(4.4, pol3["one"]);
    EXPECT_TRUE(PoliciesAreSame(pol2, pol3));

    Policy pol4 = pol3;
    EXPECT_TRUE(PoliciesAreSame(pol3, pol4));
    EXPECT_TRUE(PoliciesAreNotSame(pol1, pol4));

    // more names than values is allowed
    Policy pol5 {{"a", "c"}, {7.7}};
    EXPECT_EQ(1, pol5.size());
    EXPECT_EQ(std::vector<std::string>({"a", "c"}), pol5.policy_names());

    // not enough policy names
    GEOPM_EXPECT_THROW_MESSAGE(Policy({"one", "two"}, {5, 6, 7}),
                               GEOPM_ERROR_INVALID,
                               "incorrect number of policy names");
}

TEST(PolicyTest, access_values)
{
    Policy pol{{"uno", "dos", "tres", "cuatro", "seis", "siete"}, {1.5, 2.0}};
    EXPECT_EQ(1.5, pol[0]);
    EXPECT_EQ(1.5, pol["uno"]);
    EXPECT_EQ(2.0, pol[1]);
    EXPECT_EQ(2.0, pol["dos"]);

    GEOPM_EXPECT_THROW_MESSAGE(pol[7], GEOPM_ERROR_INVALID,
                               "invalid index for policy");
    GEOPM_EXPECT_THROW_MESSAGE(pol.at(7), GEOPM_ERROR_INVALID,
                               "invalid index for policy");

    GEOPM_EXPECT_THROW_MESSAGE(pol["cinco"], GEOPM_ERROR_INVALID,
                               "invalid policy name");
    GEOPM_EXPECT_THROW_MESSAGE(pol.at("cinco"), GEOPM_ERROR_INVALID,
                               "invalid policy name");

    GEOPM_EXPECT_THROW_MESSAGE(pol.at("tres"), GEOPM_ERROR_RUNTIME,
                               "no value for policy with name");

    GEOPM_EXPECT_THROW_MESSAGE(pol.at(3), GEOPM_ERROR_RUNTIME,
                               "no value for policy at index");

    // update values
    pol["tres"] = 3.0;
    pol[3] = 4.0;

    EXPECT_EQ(3.0, pol.at("tres"));
    EXPECT_EQ(4.0, pol.at(3));
    EXPECT_EQ(4, pol.size());

    // default inserted by [] operators is NaN
    EXPECT_TRUE(std::isnan(pol[4]));
    EXPECT_TRUE(std::isnan(pol["siete"]));
    EXPECT_EQ(6, pol.size());
}

TEST(PolicyTest, to_vector)
{
    std::vector<double> v1 {7.4, 6.33, 5.2};
    Policy p1 {{"a", "b", "c"}, v1};
    EXPECT_EQ(v1, p1.to_vector());
    std::vector<double> v2;
    Policy p2 {{}, {}};
    EXPECT_EQ(v2, p2.to_vector());
}

TEST(PolicyTest, equality)
{
    EXPECT_TRUE(PoliciesAreSame({{"A", "B"}, {4, 5}},
                                {{"A", "B"}, {4, 5}}));

    // trailing NaNs are considered implicit if missing
    EXPECT_TRUE(PoliciesAreSame({{"a", "b", "c"}, {6, 5}},
                                {{"a", "b", "c"}, {6, 5, NAN}}));
    EXPECT_TRUE(PoliciesAreSame({{"o", "p"}, {}},
                                {{"o", "p"}, {NAN, NAN}}));
    EXPECT_TRUE(PoliciesAreSame({{"b", "c", "d"}, {8.8, NAN, NAN}},
                                {{"b", "c", "d"}, {8.8, NAN}}));
    EXPECT_TRUE(PoliciesAreSame({{"b", "c", "d"}, {8.8, NAN}},
                                {{"b", "c", "d"}, {8.8, NAN, NAN}}));

    // policy name lists must match
    EXPECT_TRUE(PoliciesAreNotSame({{"d", "e", "f"}, {}},
                                   {{"d", "f", "e"}, {}}));
    EXPECT_TRUE(PoliciesAreNotSame({{"i", "j"}, {}},
                                   {{"i", "j", "k"}, {}}));
    EXPECT_TRUE(PoliciesAreNotSame({{"i", "j", "k"}, {}},
                                   {{"i", "j"}, {}}));

    // all values must match
    EXPECT_TRUE(PoliciesAreNotSame({{"r", "s", "t"}, {5, 4, 3}},
                                   {{"r", "s", "t"}, {NAN, 4, 3}}));
    EXPECT_TRUE(PoliciesAreNotSame({{"J", "K", "L"}, {8, 9, NAN}},
                                   {{"J", "K", "L"}, {8, 9, 7}}));
    EXPECT_TRUE(PoliciesAreNotSame({{"D", "E"}, {2, 7}},
                                   {{"D", "E"}, {2}}));
}

TEST(PolicyTest, to_string)
{
    EXPECT_EQ("6.6", Policy({"A"}, {6.6}).to_string(", "));
    EXPECT_EQ("4,5,6", Policy({"A", "B", "C"}, {4, 5, 6}).to_string(","));
    EXPECT_EQ("NAN, NAN, 0", Policy({"A", "B", "C"}, {NAN, NAN, 0.0}).to_string(", "));
    EXPECT_EQ("8.5|7.7|6.6", Policy({"A", "B", "C"}, {8.5, 7.7, 6.6}).to_string("|"));
}

TEST(PolicyTest, to_json_string)
{
    std::string json1 = "{}";
    EXPECT_EQ(json1, Policy({}, {}).to_json());

    std::string json2 = "{\"val\": 5.5}";
    EXPECT_EQ(json2, Policy({"val"}, {5.5}).to_json());

    std::string json3 = "{\"banana\": 44.44, \"apple\": \"NAN\", "
                        "\"coconut\": 0, \"durian\": 8.76}";
    EXPECT_EQ(json3, Policy({"banana", "apple", "coconut", "durian"},
                            {44.44, NAN, 0, 8.76}).to_json());

    // errors:
    GEOPM_EXPECT_THROW_MESSAGE(Policy({"one", "two"}, {1, 2, 3}).to_json(),
                               GEOPM_ERROR_INVALID, "number of policy names");
}

TEST(PolicyTest, fill_missing_with_nans)
{
    Policy p1 {{"un", "deux", "troi", "quatre", "cinq"}, {4.4, NAN, 7.7}};
    EXPECT_EQ(3, p1.size());

    // same size is not an error
    p1.pad_nan_to(3);
    EXPECT_EQ(3, p1.size());
    GEOPM_EXPECT_THROW_MESSAGE(p1.at("cinq"), GEOPM_ERROR_RUNTIME,
                               "no value for policy with name");

    Policy before = p1;
    p1.pad_nan_to(5);
    EXPECT_EQ(5, p1.size());
    EXPECT_TRUE(PoliciesAreSame(before, p1));
    EXPECT_TRUE(std::isnan(p1.at(3)));
    EXPECT_TRUE(std::isnan(p1.at("cinq")));
    EXPECT_TRUE(std::isnan(p1.at(4)));

    // disallow shrinking
    GEOPM_EXPECT_THROW_MESSAGE(p1.pad_nan_to(3),
                               GEOPM_ERROR_INVALID,
                               "size of policy cannot be reduced");
}
