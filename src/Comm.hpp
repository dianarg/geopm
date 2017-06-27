/*
 * Copyright (c) 2015, 2016, 2017, Intel Corporation
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

#ifndef COMM_HPP_INCLUDE
#define COMM_HPP_INCLUDE

#include <set>
#include <vector>
#include <string>
#include <list>

namespace geopm
{
    extern const char *MPICOMM_DESCRIPTION;
    // Forward declaration
    class Request;

    /// @brief Abstract base class for interprocess communication in geopm
    class IComm
    {
        public:
            enum m_comm_split_type_e {
                M_COMM_SPLIT_TYPE_CTL,
                M_COMM_SPLIT_TYPE_PPN1,
                M_COMM_SPLIT_TYPE_SHARED,
                M_COMM_SPLIT_TYPE_MAX
            };

            enum m_split_color_e {
                M_SPLIT_COLOR_UNDEFINED,
            };

            /// @brief Constructor for global communicator
            IComm() {}
            /// @brief Constructor for splitting a communictor into a Cartesian grid
            IComm(const IComm *in_comm) {}
            /// @brief Default destructor
            virtual ~IComm() {}

            virtual bool comm_supported(const std::string &description) const = 0;

            // Introspection
            /// @brief Process rank within communicator
            virtual int cart_rank(std::vector<int> coords) const = 0;
            virtual int rank(void) const = 0;
            /// @brief Number of ranks in the communicator
            virtual int num_rank(void) const = 0;
            /// @brief Dimension of Cartesian grid (returns 1 for non-Cartesian communicators
            /// TODO: help, not used?
            virtual int num_dimension(void) const = 0;
            /// @brief Populate vector of optimal dimensions given the number of ranks the communicator
            virtual void dimension_create(int num_nodes, std::vector<int> &dimension) const = 0;
            /// @brief Allocate memory for message passing and RMA
            virtual void free_mem(void *base) = 0;
            virtual void alloc_mem(size_t size, void **base) = 0;
            virtual size_t create_window(size_t size, void *base) = 0;
            virtual void destroy_window(size_t window_id) = 0;
            virtual void lock_window(size_t window_id, bool isExclusive, int rank, int assert) const = 0;
            virtual void unlock_window(size_t window_id, int rank) const = 0;
            /// @brief Coordinate in Cartesian grid for specified rank
            virtual void coordinate(int rank, std::vector<int> &coord) const = 0;

            // Collective communication
            /// @brief Barrier for all ranks
            virtual void barrier(void) const = 0;
            /// @brief Broadcast a message to all ranks
            virtual void broadcast(void *buffer, size_t size, int root) const = 0;
            virtual bool test(bool is_true) const = 0;
            virtual void reduce_sum(double *sendbuf, double *recvbuf, size_t count, int root) const = 0;
            /// @brief Reduce distributed messages across all ranks using specified operation, store result on all ranks
            virtual void gather(const void *send_buf, size_t send_size, void *recv_buf,
                    size_t recv_size, int root) const = 0;
            virtual void gatherv(const void *send_buf, size_t send_size, void *recv_buf,
                    const std::vector<size_t> &recv_sizes, const std::vector<off_t> rank_offset, int root) const = 0;
            virtual void window_put(const void *send_buf, size_t send_size, int rank, int disp, size_t window_id) const = 0;
    };


    /// @brief Abstract base class for handling non-blocking request status for Comm
    class Request
    {
        public:
            /// @brief Default constructor
            Request();
            /// @brief Default destructor
            virtual ~Request();
            /// @brief Cancel a pending non-blocking request
            virtual void cancel(void) = 0;
            /// @brief Check if a non-blocking request has completed
            virtual bool is_complete(void) = 0;
    };

    /// @brief Factory object managing decider objects.
    ///
    /// The DeciderFactory manages all instances of Decider objects. During
    /// construction the factory creates instances of all built in Decider classes
    /// as well as any Decider plugins present on the system. All Deciders then register
    /// themselves with the factory. The factory returns an appropriate Decider object
    /// when queried with a description string. The factory deletes all Decider objects
    /// on destruction.
    class ICommFactory
    {
        public:
            ICommFactory() {}
            ICommFactory(const ICommFactory &other) {}
            virtual ~ICommFactory() {}
            /// @brief Returns an abstract Decider pointer to a concrete decider.
            ///
            /// The concrete Decider is specific to the description string
            /// passed in to select it.
            /// throws a std::invalid_argument if no acceptable
            /// Decider is found.
            ///
            /// @param [in] description The descrition string corresponding
            /// to the desired Decider.
            virtual IComm *comm(const std::string &description) = 0;
            /// @brief Concrete Deciders register with the factory through this API.
            ///
            /// @param [in] decider The unique_ptr to a Decider object
            /// assures that the object cannot be destroyed before it
            /// is copied.
            //virtual void register_comm(IComm *comm, void *dl_ptr) = 0;
    };

    class CommFactory : public ICommFactory
    {
        public:
            /// @brief DeciderFactory default constructor.
            CommFactory();
            /// @brief DeciderFactory Testing constructor.
            ///
            /// This constructor takes in
            /// a specific Decider object  and does not load plugins.
            /// It is intended to be used for testing.
            /// @param [in] decider The unique_ptr to a Decider object
            ///             assures that the object cannot be destroyed before
            ///             it is copied.
            //CommFactory(IComm *comm);
            /// @brief DeciderFactory destructor, virtual.
            virtual ~CommFactory();
            IComm *comm(const std::string &description);
            IComm *comm(const IComm *in_comm);
            IComm *comm(const IComm *in_comm, int color, int key);
            IComm *comm(const IComm *in_comm, std::string tag, int split_type);
            IComm *comm(const IComm *in_comm, std::vector<int> dimension, std::vector<int> periods, bool is_reorder);
            //void register_comm(IComm *comm, void *dl_ptr);
        private:
            // @brief Holds all registered concrete Decider instances
            //std::list<IComm *> m_comm_list;
            //std::list<void *> m_dl_ptr_list;
    };
}

#endif
