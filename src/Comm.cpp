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

#include <inttypes.h>
#include <cpuid.h>
#include <string>
#include <sstream>
#include <dlfcn.h>

#include "geopm_plugin.h"
#include "Exception.hpp"
#include "Comm.hpp"
#include "MPIComm.hpp"
#include "config.h"


/*
void geopm_factory_register(struct geopm_factory_c *factory, geopm::IDecider *decider, void *dl_ptr)
{
    geopm::DeciderFactory *fact_obj = (geopm::DeciderFactory *)(factory);
    if (fact_obj == NULL) {
        throw geopm::Exception(GEOPM_ERROR_FACTORY_NULL, __FILE__, __LINE__);
    }
    fact_obj->register_decider(decider, dl_ptr);
}
*/

namespace geopm
{

    CommFactory::CommFactory()
    {
        // register all the deciders we know about
        //geopm_plugin_load(GEOPM_PLUGIN_TYPE_COMM, (struct geopm_factory_c *)this);
        //register_comm(new Comm(), NULL);
    }

    /*
    CommFactory::CommFactory(IComm *comm)
    {
        register_decider(decider, NULL);
    }
    */

    CommFactory::~CommFactory()
    {
        /*
        for (auto it = m_comm_list.rbegin(); it != m_comm_list.rend(); ++it) {
            delete *it;
        }

        for (auto it = m_dl_ptr_list.rbegin(); it != m_dl_ptr_list.rend(); ++it) {
            dlclose(*it);
        }
        */
    }

    IComm* CommFactory::comm(const std::string &description)
    {
        // TODO: help.
        IComm *result = NULL;
        /*
        for (auto it = m_comm_list.begin(); it != m_comm_list.end(); ++it) {
            if (*it != NULL &&
                (*it)->decider_supported(description)) {
                result = (*it)->clone();
                break;
            }
        }
        */
        //if (description.compare(MPICOMM_DESCRIPTION)) {
            result = new MPIComm();
        //}
        if (!result) {
            // If we get here, no acceptable comm was found
            std::ostringstream ex_str;
            ex_str << "Failure to instaciate Comm type: " << description;
            //throw Exception(ex_str.str(), GEOPM_ERROR_COMM_UNSUPPORTED, __FILE__, __LINE__);
        }

        return result;
    }

    IComm *CommFactory::comm(const IComm *in_comm)
    {
        // TODO, use description string to determine casting target
        return new MPIComm(static_cast<const MPIComm *>(in_comm));
    }

    IComm *CommFactory::comm(const IComm *in_comm, int color, int key)
    {
        return new MPIComm(static_cast<const MPIComm *>(in_comm), color, key);
    }
    
    IComm *CommFactory::comm(const IComm *in_comm, std::string tag, int split_type)
    {
        return new MPIComm(static_cast<const MPIComm *>(in_comm), tag, split_type);
    }

    IComm *CommFactory::comm(const IComm *in_comm, std::vector<int> dimension, std::vector<int> periods, bool is_reorder)
    {
        return new MPIComm(static_cast<const MPIComm *>(in_comm), dimension, periods, is_reorder);
    }

    /*
    void CommFactory::register_comm(IComm *comm, void *dl_ptr)
    {
        m_comm_list.push_back(decider);
        if (dl_ptr) {
            m_dl_ptr_list.push_back(dl_ptr);
        }
    }
    */
}
