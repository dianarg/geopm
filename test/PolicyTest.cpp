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
    for (size_t ii = 0; ii < vec.size(); ++ii) {
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
        msg << "{" << p1.policy_names() << "}: {" << p1.to_vector()
            << "} is not equivalent to {" << p2.policy_names() << "}: {"
            << p2.to_vector() << "}";
        return ::testing::AssertionFailure() << msg.str();
    }
    return ::testing::AssertionSuccess();
}

::testing::AssertionResult
PoliciesAreNotSame(const Policy &p1, const Policy &p2)
{
    if (p1 == p2) {
        std::ostringstream msg;
        msg << "{" << p1.policy_names() << "}: {" << p1.to_vector()
            << "} is equivalent to {" << p2.policy_names() << "}: {"
            << p2.to_vector() << "}";
        return ::testing::AssertionFailure() << msg.str();
    }
    return ::testing::AssertionSuccess();
}

TEST(PolicyTest, construct)
{
    Policy pol1{{}, std::vector<double>{}};
    EXPECT_EQ(0u, pol1.size());

    std::vector<std::string> names{"one", "two"};
    std::vector<double> values{4.4, 5.5};
    Policy pol2{names, values};
    EXPECT_EQ(2u, pol2.size());
    EXPECT_EQ(5.5, pol2[1]);
    EXPECT_EQ(5.5, pol2["two"]);
    Policy pol2a {{"a", "b"}, values};
    EXPECT_TRUE(PoliciesAreNotSame(pol2, pol2a));
    Policy pol2b {{"one", "two"}, values};
    EXPECT_TRUE(PoliciesAreSame(pol2, pol2b));

    Policy pol3 {pol2};
    EXPECT_EQ(2u, pol3.size());

    EXPECT_EQ(4.4, pol3[0]);

    EXPECT_EQ(4.4, pol3["one"]);
    EXPECT_TRUE(PoliciesAreSame(pol2, pol3));

    Policy pol4 = pol3;
    EXPECT_TRUE(PoliciesAreSame(pol3, pol4));
    EXPECT_TRUE(PoliciesAreNotSame(pol1, pol4));

    // more names than values is allowed
    std::vector<double> v1 {7.7};
    Policy pol5 {{"a", "c"}, v1};
    EXPECT_EQ(1u, pol5.size());
    EXPECT_EQ(std::vector<std::string>({"a", "c"}), pol5.policy_names());

    // not enough policy names
    std::vector<double> v2 {5, 6, 7};
    GEOPM_EXPECT_THROW_MESSAGE(Policy({"one", "two"}, v2),
                               GEOPM_ERROR_INVALID,
                               "incorrect number of policy names");
}

TEST(PolicyTest, access_values)
{
    std::vector<double> values {1.5, 2.0};
    Policy pol{{"uno", "dos", "tres", "cuatro", "seis", "siete"}, values};
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
    EXPECT_EQ(4u, pol.size());

    pol.at("tres") = 3.3;
    pol.at(3) = 4.4;
    EXPECT_EQ(3.3, pol.at("tres"));
    EXPECT_EQ(4.4, pol.at(3));

    // default inserted by [] operators is NaN
    EXPECT_TRUE(std::isnan(pol[4]));
    EXPECT_TRUE(std::isnan(pol["siete"]));
    EXPECT_EQ(6u, pol.size());
}

TEST(PolicyTest, to_vector)
{
    std::vector<double> v1 {7.4, 6.33, 5.2};
    Policy p1 {{"a", "b", "c"}, v1};
    EXPECT_EQ(v1, p1.to_vector());
    std::vector<double> v2;
    Policy p2 {{}, std::vector<double>{}};
    EXPECT_EQ(v2, p2.to_vector());
}

TEST(PolicyTest, equality)
{
    std::vector<double> v1 {4, 5};
    EXPECT_TRUE(PoliciesAreSame({{"A", "B"}, v1},
                                {{"A", "B"}, v1}));

    std::vector<double> empty {};
    // trailing NaNs are considered implicit if missing
    EXPECT_TRUE(PoliciesAreSame({{"a", "b", "c"}, std::vector<double>{6, 5}},
                                {{"a", "b", "c"}, std::vector<double>{6, 5, NAN}}));
    EXPECT_TRUE(PoliciesAreSame({{"o", "p"}, empty},
                                {{"o", "p"}, std::vector<double>{NAN, NAN}}));
    EXPECT_TRUE(PoliciesAreSame({{"b", "c", "d"}, std::vector<double>{8.8, NAN, NAN}},
                                {{"b", "c", "d"}, std::vector<double>{8.8, NAN}}));
    EXPECT_TRUE(PoliciesAreSame({{"b", "c", "d"}, std::vector<double>{8.8, NAN}},
                                {{"b", "c", "d"}, std::vector<double>{8.8, NAN, NAN}}));

    // policy name lists must match
    EXPECT_TRUE(PoliciesAreNotSame({{"d", "e", "f"}, empty},
                                   {{"d", "f", "e"}, empty}));
    EXPECT_TRUE(PoliciesAreNotSame({{"i", "j"}, empty},
                                   {{"i", "j", "k"}, empty}));
    EXPECT_TRUE(PoliciesAreNotSame({{"i", "j", "k"}, empty},
                                   {{"i", "j"}, empty}));

    // all values must match
    EXPECT_TRUE(PoliciesAreNotSame({{"r", "s", "t"}, std::vector<double>{5, 4, 3}},
                                   {{"r", "s", "t"}, std::vector<double>{NAN, 4, 3}}));
    EXPECT_TRUE(PoliciesAreNotSame({{"J", "K", "L"}, std::vector<double>{8, 9, NAN}},
                                   {{"J", "K", "L"}, std::vector<double>{8, 9, 7}}));
    EXPECT_TRUE(PoliciesAreNotSame({{"D", "E"}, std::vector<double>{2, 7}},
                                   {{"D", "E"}, std::vector<double>{2}}));
}

TEST(PolicyTest, to_json_string)
{
    std::string json1 = "{}";
    EXPECT_EQ(json1, Policy({{}, std::vector<double>{}}).to_json());

    std::string json2 = "{\"val\": 5.5}";
    EXPECT_EQ(json2, Policy({"val"}, std::vector<double>{5.5}).to_json());

    std::string json3 = "{\"banana\": 44.44, \"apple\": \"NAN\", "
                        "\"coconut\": 0, \"durian\": 8.76}";
    EXPECT_EQ(json3, Policy({"banana", "apple", "coconut", "durian"},
                            std::vector<double>{44.44, NAN, 0, 8.76}).to_json());

    // errors:
    GEOPM_EXPECT_THROW_MESSAGE(Policy({"one", "two"}, std::vector<double>{1, 2, 3}).to_json(),
                               GEOPM_ERROR_INVALID, "number of policy names");
}

TEST(PolicyTest, fill_missing_with_nans)
{
    std::vector<std::string> names {"un", "deux", "troi", "quatre", "cinq"};
    Policy p1 {names, std::vector<double>{4.4, NAN, 7.7}};
    EXPECT_EQ(3u, p1.size());

    // same size is not an error
    EXPECT_EQ(3u, p1.to_vector(3u).size());

    Policy before = p1;
    auto temp2 = p1.to_vector(5u);
    Policy temp = Policy(names, temp2);//p1.to_vector(5u));
    p1 = temp; //Policy(names, p1.to_vector(5u));
    EXPECT_EQ(5u, p1.size());
    EXPECT_TRUE(PoliciesAreSame(before, p1));
    EXPECT_EQ(4.4, p1.at("un"));
    EXPECT_EQ(7.7, p1.at("troi"));
    EXPECT_TRUE(std::isnan(p1.at(3)));
    EXPECT_TRUE(std::isnan(p1.at("cinq")));
    EXPECT_TRUE(std::isnan(p1.at(4)));

    // disallow shrinking
    GEOPM_EXPECT_THROW_MESSAGE(p1.to_vector(3u),
                               GEOPM_ERROR_INVALID,
                               "size of policy cannot be reduced");

    // cannot pad beyond number of policy names
    GEOPM_EXPECT_THROW_MESSAGE(p1.to_vector(20), GEOPM_ERROR_INVALID,
                               "cannot pad more than maximum policy size");

    // fill without size
    Policy p2 {names, std::map<std::string, double>{{"deux", 2.2}, {"quatre", 4.3}}};
    EXPECT_EQ(2u, p2.size());
    EXPECT_EQ(2.2, p2.at("deux"));
    EXPECT_EQ(4.3, p2.at("quatre"));
    EXPECT_THROW(p2.at("troi"), geopm::Exception);
    auto result = p2.to_vector();
    EXPECT_EQ(4u, result.size());
    std::vector<double> expected {NAN, 2.2, NAN, 4.3};
    EXPECT_EQ(expected, result);

}

TEST(PolicyTest, from_json_string)
{
    std::vector<std::string> names1 = {};
    Policy p1(names1, "{}");
    EXPECT_TRUE(PoliciesAreSame({names1, std::vector<double>{}}, p1));

    std::vector<std::string> names2 = {"a", "b"};
    Policy p2(names2, "{}");
    EXPECT_TRUE(PoliciesAreSame({names2, std::vector<double>{}}, p2));

    std::vector<std::string> names3 = {"x", "y", "z"};
    Policy p3(names3, "{\"x\": 7.7, \"y\": 4.5}");
    EXPECT_TRUE(PoliciesAreSame({names3, std::vector<double>{7.7, 4.5}}, p3));

    std::vector<std::string> names4 = {"f", "g", "h"};
    Policy p4(names4, "{\"g\": \"NAN\", \"f\": 3.4, \"h\": 6e9 }");
    EXPECT_TRUE(PoliciesAreSame({names4, std::vector<double>{3.4, NAN, 6e9}}, p4));

    // bad json errors
    GEOPM_EXPECT_THROW_MESSAGE(Policy(names1, ""),
                               GEOPM_ERROR_INVALID, "malformed json");

    GEOPM_EXPECT_THROW_MESSAGE(Policy(names1, "{{ }"),
                               GEOPM_ERROR_INVALID, "malformed json");

    GEOPM_EXPECT_THROW_MESSAGE(Policy(names1, "{\"a\": 6, }"),
                               GEOPM_ERROR_INVALID, "malformed json");

    // invalid policy names
    GEOPM_EXPECT_THROW_MESSAGE(Policy(names2, "{\"a\": 1, \"c\": 2}"),
                               GEOPM_ERROR_INVALID, "invalid policy name");

    // invalid value
    GEOPM_EXPECT_THROW_MESSAGE(Policy(names2, "{\"a\": \"invalid\"}"),
                               GEOPM_ERROR_INVALID, "invalid value for policy");
    GEOPM_EXPECT_THROW_MESSAGE(Policy(names2, "{\"a\": {} }"),
                               GEOPM_ERROR_INVALID, "invalid value for policy");
    GEOPM_EXPECT_THROW_MESSAGE(Policy(names2, "{\"a\": true }"),
                               GEOPM_ERROR_INVALID, "invalid value for policy");
    GEOPM_EXPECT_THROW_MESSAGE(Policy(names2, "{\"a\": null }"),
                               GEOPM_ERROR_INVALID, "invalid value for policy");
}
