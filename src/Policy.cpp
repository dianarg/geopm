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

#include "Policy.hpp"

#include <cmath>

#include <sstream>
#include <algorithm>

#include "Exception.hpp"
#include "Helper.hpp"

namespace geopm
{
    Policy::Policy(const std::vector<std::string> &names,
                   const std::vector<double> &values)
        : m_names(names)
    {
        if (names.size() < values.size()) {
            throw Exception("Policy(): incorrect number of policy names.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        for (size_t ii = 0; ii < values.size(); ++ii) {
            m_values[m_names.at(ii)] = values[ii];
        }
    }

    size_t Policy::size(void) const
    {
        return m_values.size();
    }

    std::vector<std::string> Policy::policy_names(void) const
    {
        return m_names;
    }

    double &Policy::operator[](size_t index)
    {
        if (index >= m_names.size()) {
            throw Exception("Policy: invalid index for policy",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        if (m_values.find(m_names[index]) == m_values.end()) {
            m_values[m_names[index]] = NAN;
        }
        return m_values[m_names[index]];
    }

    double &Policy::operator[](const std::string &name)
    {
        if (std::find(m_names.begin(), m_names.end(), name) == m_names.end()) {
            throw Exception("Policy: invalid policy name: " + name,
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (m_values.find(name) == m_values.end()) {
            m_values[name] = NAN;
        }
        return m_values[name];
    }

    void Policy::check_index(size_t index) const
    {
        if (index >= m_names.size()) {
            throw Exception("Policy: invalid index for policy",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (m_values.find(m_names[index]) == m_values.end()) {
            throw Exception("Policy: no value for policy at index " + std::to_string(index),
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
    }

    double Policy::at(size_t index) const
    {
        check_index(index);
        return m_values.at(m_names[index]);
    }

    double &Policy::at(size_t index)
    {
        check_index(index);
        return m_values.at(m_names[index]);
    }

    void Policy::check_name(const std::string &name) const
    {
        if (std::find(m_names.begin(), m_names.end(), name) == m_names.end()) {
            throw Exception("Policy: invalid policy name: " + name,
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (m_values.find(name) == m_values.end()) {
            throw Exception("Policy: no value for policy with name: " + name,
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
    }

    double Policy::at(const std::string &name) const
    {
        check_name(name);
        return m_values.at(name);
    }

    double &Policy::at(const std::string &name)
    {
        check_name(name);
        return m_values.at(name);
    }

    /// Compare two doubles, return true if equal or both are NaN
    static bool float_equal(double a, double b)
    {
        bool result = false;
        if (a == b || (std::isnan(a) && std::isnan(b))) {
            result = true;
        }
        return result;
    }

    bool Policy::operator==(const Policy &other) const
    {
        if (m_names.size() != other.m_names.size()) {
            return false;
        }

        bool equal = true;
        for (size_t idx = 0; equal && idx < m_names.size(); ++idx) {
            // name at each index must match
            if (m_names[idx] != other.m_names[idx]) {
                equal = false;
            }
            else {
                std::string key = m_names[idx];
                auto tval = m_values.find(key);
                auto oval = other.m_values.find(key);
                if (tval != m_values.end() && oval != other.m_values.end() &&
                    !float_equal(tval->second, oval->second)) {
                    // non-NaN values must match
                    equal = false;
                }
                else if (idx < m_values.size() && idx >= other.m_values.size() &&
                         !std::isnan(tval->second)) {
                    // trailing values in this must be nan
                    equal = false;
                }
                else if (idx < other.m_values.size() && idx >= m_values.size() &&
                         !std::isnan(oval->second)) {
                    // trailing values in other must be nan
                    equal = false;
                }
                else {
#ifdef GEOPM_DEBUG
                    throw Exception("Policy: Unexpected gaps in policy data structures.",
                                    GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
#endif
                }
            }
        }
        return equal;
    }

    bool Policy::operator!=(const Policy &other) const
    {
        return !(*this == other);
    }

    std::string Policy::to_string(const std::string &delimiter) const
    {
        std::ostringstream temp;
        for (size_t ii = 0; ii < m_values.size(); ++ii) {
            if (ii != 0) {
                temp << delimiter;
            }
            if (std::isnan(m_values.at(m_names.at(ii)))) {
                temp << "NAN";
            }
            else {
                temp << geopm::string_format_double(m_values.at(m_names.at(ii)));
            }
        }
        return temp.str();
    }

    std::string Policy::to_json(void) const
    {
        std::ostringstream output_str;
        output_str << "{";
        for (size_t idx = 0; idx < m_names.size() && m_values.find(m_names.at(idx)) != m_values.end(); ++idx) {
            if (idx > 0) {
                output_str << ", ";
            }
            std::string policy_value;
            if (std::isnan(m_values.at(m_names.at(idx)))) {
                policy_value = "\"NAN\"";
            }
            else {
                policy_value = geopm::string_format_double(m_values.at(m_names.at(idx)));
            }
            output_str << "\"" << m_names.at(idx) << "\": " << policy_value;
        }
        output_str << "}";
        return output_str.str();
    }

    std::vector<double> Policy::to_vector(void) const
    {
        std::vector<double> result(m_values.size());
        for (size_t ii = 0; ii < m_values.size(); ++ii) {
#ifdef GEOPM_DEBUG
        if (ii >= m_names.size() || m_values.find(m_names.at(ii)) == m_values.end()) {
            throw Exception("Policy::to_vector(): Unexpected gaps in policy data structures.",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__;
        }
#endif
            result[ii] = m_values.at(m_names.at(ii));
        }
        return result;
    }

    void Policy::pad_nan_to(size_t size)
    {
        if (size < m_values.size()) {
            throw Exception("Policy::pad_to_nan(): size of policy cannot be reduced.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (size > m_names.size()) {
            throw Exception("Policy::pad_to_nan(): cannot pad more than maximum policy size",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        for (size_t ii = m_values.size(); ii < size; ++ii) {
            m_values[m_names.at(ii)] = NAN;
        }
    }
}

std::ostream& operator<<(std::ostream &os, const geopm::Policy &policy)
{
    os << policy.to_string(", ");
    return os;
}
