/*
 * Copyright (c) 2015, 2016, 2017, 2018, Intel Corporation
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

#ifndef IOGROUP_HPP_INCLUDE
#define IOGROUP_HPP_INCLUDE

#include <string>
#include <vector>

namespace geopm
{
    class IOGroup
    {
        public:
            IOGroup() {}
            virtual ~IOGroup() {}
            /// @brief Test if signal_name refers to a signal
            ///        supported by the group.
            /// @param [in] signal_name Name of signal to test.
            /// @return True if signal is supported, false otherwise.
            virtual bool is_valid_signal(const std::string &signal_name) = 0;
            /// @brief Test if control_name refers to a control
            ///        supported by the group.
            /// @param [in] control_name Name of control to test.
            /// @return True if control is supported, false otherwise.
            virtual bool is_valid_control(const std::string &control_name) = 0;
            /// @brief Add a signal to the list of signals that is
            ///        read by read_batch() and sampled by sample().
            /// @param [in] signal_name Name of the signal requested.
            /// @param [in] domain_type One of the values from the
            ///        PlatformTopo::m_domain_e enum described in
            ///        PlatformTopo.hpp.
            /// @param [in] domain_idx The index of the domain within
            ///        the set of domains of the same type on the
            ///        platform.
            /// @return Index of signal when sample() method is called.
            virtual int push_signal(const std::string &signal_name,
                                    int domain_type,
                                    int domain_idx) = 0;
            /// @brief Add a control to the list of controls that is
            ///        written by write_batch() and configured with
            ///        adjust().
            /// @param [in] control_name Name of the control requested.
            /// @param [in] domain_type One of the values from the
            ///        PlatformTopo::m_domain_e enum described in
            ///        PlatformTopo.hpp.
            /// @param [in] domain_idx The index of the domain within
            ///        the set of domains of the same type on the
            ///        platform.
            /// @return Index of control when adjust() method is called.
            virtual int push_control(const std::string &control_name,
                                     int domain_type,
                                     int domain_idx) = 0;
            /// @brief Read all pushed signals from the platform so
            ///        that the next call to sample() will reflect the
            ///        updated data.
            virtual void read_batch(void) = 0;
            /// @brief Write all of the pushed controls so that values
            ///        previously given to adjust() are written to the
            ///        platform.
            virtual void write_batch(void) = 0;
            /// @brief Retrieve signal value from data read by last
            ///        call to read_batch() for a particular signal
            ///        previously pushed with push_signal().
            /// @param [in] batch_idx The index returned by previous
            ///        call to push_signal().
            /// @return Value of signal in SI units.
            virtual double sample(int batch_idx) = 0;
            /// @brief Adjust a setting for a particular control that
            ///        was previously pushed with push_control(). This
            ///        adjustment will be written to the platform on
            ///        the next call to write_batch().
            /// @param [in] batch_idx The index returned by previous
            ///        call to push_control().
            /// @param [in] setting Value of the control in SI units.
            virtual void adjust(int batch_idx,
                                double setting) = 0;
            /// @brief Read from platform and interpret into SI units
            ///        a signal given its name and domain.  Does not
            ///        modify the values stored by calling
            ///        read_batch().
            /// @param [in] signal_name Name of the signal requested.
            /// @param [in] domain_type One of the values from the
            ///        PlatformTopo::m_domain_e enum described in
            ///        PlatformTopo.hpp.
            /// @param [in] domain_idx The index of the domain within
            ///        the set of domains of the same type on the
            ///        platform.
            /// @return The value in SI units of the signal.
            virtual double read_signal(const std::string &signal_name,
                                       int domain_type,
                                       int domain_idx) = 0;
            /// @brief Interpret the setting and write setting to the
            ///        platform.  Does not modify the values stored by
            ///        calling adjust().
            /// @param [in] control_name Name of the control requested.
            /// @param [in] domain_type One of the values from the
            ///        PlatformTopo::m_domain_e enum described in
            ///        PlatformTopo.hpp.
            /// @param [in] domain_idx The index of the domain within
            ///        the set of domains of the same type on the
            ///        platform.
            /// @param [in] setting Value in SI units of the setting
            ///        for the control.
            virtual void write_control(const std::string &control_name,
                                       int domain_type,
                                       int domain_idx,
                                       double setting) = 0;
    };
}

#endif
