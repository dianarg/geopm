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

#ifndef POLICY_HPP_INCLUDE
#define POLICY_HPP_INCLUDE

#include <vector>
#include <string>
#include <map>

namespace geopm
{
    /// @brief The Policy class is used to handle operations on a
    ///        policy as a vector of doubles, such as comparison and
    ///        formatting.
    class Policy
    {
        public:
            Policy(const std::vector<std::string> &names,
                   const std::vector<double> &values);
            virtual ~Policy() = default;

            /// @brief Returns the number of values in the policy.
            size_t size(void) const;

            /// @brief Returns the vector of policy names.
            std::vector<std::string> policy_names(void) const;

            /// @brief Accesses a value of the policy by index or adds if
            ///        missing and in bounds, or throws if out-of-bounds.
            double &operator[](size_t index);

            /// @brief Accesses a value of the policy by name or adds if
            ///        missing and name is valid, or throws if name is invalid.
            double &operator[](const std::string &name);

            /// @brief Accesses a value of the policy by index and throws if
            ///        missing.
            double at(size_t index) const;
            /// @brief Accesses a reference to the policy value by index and
            ///        throws if missing.
            double &at(size_t index);

            /// @brief Accesses a value of the policy by name and throws if
            ///        missing.
            double at(const std::string &name) const;
            /// @brief Accesses a reference to the policy value by name and
            ///        throws if missing.
            double &at(const std::string &name);

            /// @brief Equality comparison operator.  Trailing NANs are
            ///        not considered when checking for equality.
            bool operator==(const Policy &other) const;
            bool operator!=(const Policy &other) const;

            /// @brief Format the Policy vector as a character-delimited
            ///        list.
            std::string to_string(const std::string &delimiter) const;

            /// @brief Format the Policy values as a JSON string.
            std::string to_json(void) const;

            /// @brief Convert the policy values to a std::vector
            std::vector<double> to_vector(void) const;

            /// @brief Fill in NAN for missing values up until Policy
            ///        reaches the given size.  The new size must be
            ///        greater than or equal to the current size.
            void pad_nan_to(size_t size);

            /// @brief Create a new Policy object from a string of
            ///        comma-separated values and a vector of policy
            ///        names for the target agent.  The string representing
            ///        "NAN" is not case-sensitive.
            static Policy from_string(const std::vector<std::string> &names,
                                      const std::string &values);

            /// @brief Create a new Policy object from a JSON-formatted
            ///        string containing the values mapping to each policy
            ///        name and the vector of all names for the target agent.
            ///        Any missing non-trailing values will be filled in
            ///        with NAN, but trailing values will be left empty.
            ///        The string representing "NAN" is not case-sensitive.
            static Policy from_json(const std::vector<std::string> &names,
                                    const std::string &json);

        private:
            void check_index(size_t index) const;
            void check_name(const std::string &name) const;

            std::vector<std::string> m_names;
            std::map<std::string, double> m_values;
    };
}

std::ostream& operator<<(std::ostream &os, const geopm::Policy &policy);

#endif
