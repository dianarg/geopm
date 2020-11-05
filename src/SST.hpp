/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, 2020, Intel Corporation
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

#ifndef SST_HPP_INCLUDE
#define SST_HPP_INCLUDE

#include <stdint>

struct sst_mmio_interface_s
{
    uint32_t is_write;
    uint32_t cpu_index;
    uint32_t register_offset;
    uint32_t value;
};

struct sst_mmio_interfaces_s
{
    uint32_t num_entries;
    sst_mmio_interface_s interfaces[0];
};

struct sst_mbox_interface_s
{
    uint32_t cpu_index;
    uint32_t interface_parameter; // Parameter to the mbox interface itself
    uint32_t write_value; // Mailbox data (write-only)
    uint32_t read_value; // Mailbox data (read-only)
    uint16_t command;
    uint16_t subcommand;
    uint32_t reserved;
};

struct sst_mbox_interfaces_s
{
    uint32_t num_entries;
    sst_mbox_interface_s interfaces[0];
};

// ioctl(fd_to_interface, 2-or=3, struct address) 
// #define GEOPM_IOC_SST_MMIO _IOWR(0xfe, 2, struct geopm::SST::sst_mmio_interfaces_s)      
// #define GEOPM_IOC_SST_MBOX _IOWR(0xfe, 3, struct geopm::SST::sst_mbox_interfaces_s)      
// ioctl(fd, GEOPM_IOC_SST_MMIO, structy)
//

class SSTInterfaceTransaction
{
public:
    // Interact with the mailbox on commands that are expected to return data
    uint32_t mbox_read(uint32_t cpu_index, uint32_t command, uint32_t subcommand,
                       uint32_t subcommand_arg, uint32_t interface_parameter);

    // Interact with the mailbox on commands that are not expected to return data
    void mbox_write(uint32_t cpu_index, uint32_t command, uint32_t subcommand,
                    uint32_t interface_parameter, uint32_t write_value);

    // Read data from the mmio interface
    uint32_t mmio_read(uint32_t cpu_index, uint32_t register_offset);

    // Write data to the mmio interface
    void mmio_write(uint32_t cpu_index, uint32_t register_offset, uint32_t value);

private:
    list 
};

#endif
