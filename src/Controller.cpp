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

#include "Controller.hpp"

#include <algorithm>
#include <cmath>

#include "geopm_signal_handler.h"
#include "ApplicationIO.hpp"
#include "Environment.hpp"
#include "Reporter.hpp"
#include "Tracer.hpp"
#include "Exception.hpp"
#include "Comm.hpp"
#include "PlatformTopo.hpp"
#include "PlatformIO.hpp"
#include "Agent.hpp"
#include "TreeComm.hpp"
#include "Endpoint.hpp"
#include "Helper.hpp"
#include "config.h"

extern "C"
{
    static int geopm_run_imp(struct geopm_ctl_c *ctl);
    /// forward declaration instead of include geopm_ctl.h to avoid mpi.h inclusion
    int geopm_ctl_run(struct geopm_ctl_c *ctl);

    int geopm_ctl_pthread(struct geopm_ctl_c *ctl,
                          const pthread_attr_t *attr,
                          pthread_t *thread)
    {
        long err = 0;
        geopm::Controller *ctl_obj = (geopm::Controller *)ctl;
        try {
            ctl_obj->pthread(attr, thread);
        }
        catch (...) {
            ctl_obj->abort();
            err = geopm::exception_handler(std::current_exception(), true);
        }
        return err;
    }

    static void *geopm_threaded_run(void *args)
    {
        return (void *) (long) geopm_run_imp((struct geopm_ctl_c *)args);
    }

    int geopmctl_main(void)
    {
        int err = 0;
        try {
            geopm::Controller ctl;
            err = geopm_ctl_run((struct geopm_ctl_c *)&ctl);
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception(), true);
        }
        return err;
    }

    int geopm_ctl_destroy(struct geopm_ctl_c *ctl)
    {
        int err = 0;
        geopm::Controller *ctl_obj = (geopm::Controller *)ctl;
        try {
            delete ctl_obj;
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception(), true);
        }
        return err;
    }

    static int geopm_run_imp(struct geopm_ctl_c *ctl)
    {
        int err = 0;
        geopm::Controller *ctl_obj = (geopm::Controller *)ctl;
        try {
            ctl_obj->run();
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception(), true);
        }
        return err;
    }

    int geopm_ctl_run(struct geopm_ctl_c *ctl)
    {
        return geopm_run_imp(ctl);
    }

    int geopm_agent_enforce_policy(void)
    {
        int err = 0;
        try {
            std::string agent_name = geopm::environment().agent();
            std::shared_ptr<geopm::Agent> agent(geopm::agent_factory().make_plugin(agent_name));
            std::vector<double> policy(geopm::Agent::num_policy(geopm::agent_factory().dictionary(agent_name)));
            geopm::EndpointUser::create_endpoint_user(geopm::environment().policy())->read_policy(policy);
            agent->validate_policy(policy);
            agent->enforce_policy(policy);
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception(), false);
        }
        return err;
    }
}

namespace geopm
{
    static std::string get_start_time()
    {
        static bool once = true;
        static std::string ret;

        if (once) {
            const int buf_size = 64;
            char time_buff[buf_size];
            geopm_time_string(buf_size, time_buff);
            std::string tmp(time_buff);
            tmp.erase(std::remove(tmp.begin(), tmp.end(), '\n'), tmp.end());
            ret = tmp;
            once = false;
        }
        return ret;
    }

    Controller::Controller()
        : Controller(comm_factory().make_plugin(environment().comm()))
    {

    }

    Controller::Controller(std::shared_ptr<Comm> ppn1_comm)
        : Controller(ppn1_comm,
                     platform_io(),
                     environment().agent(),
                     Agent::num_policy(agent_factory().dictionary(environment().agent())),
                     Agent::num_sample(agent_factory().dictionary(environment().agent())),
                     std::unique_ptr<TreeComm>(new TreeCommImp(ppn1_comm,
                         Agent::num_policy(agent_factory().dictionary(environment().agent())),
                         Agent::num_sample(agent_factory().dictionary(environment().agent())))),
                     std::shared_ptr<ApplicationIO>(new ApplicationIOImp(environment().shmkey())),
                     std::unique_ptr<Reporter>(new ReporterImp(get_start_time(),
                                                                environment().report(),
                                                                platform_io(),
                                                                platform_topo(),
                                                                ppn1_comm->rank())),
                     nullptr,
                     std::vector<std::unique_ptr<Agent> >{},
                     EndpointUser::create_endpoint_user(environment().policy()))
    {

    }

    Controller::Controller(std::shared_ptr<Comm> comm,
                           PlatformIO &plat_io,
                           const std::string &agent_name,
                           int num_send_down,
                           int num_send_up,
                           std::unique_ptr<TreeComm> tree_comm,
                           std::shared_ptr<ApplicationIO> application_io,
                           std::unique_ptr<Reporter> reporter,
                           std::unique_ptr<Tracer> tracer,
                           std::vector<std::unique_ptr<Agent> > level_agent,
                           std::unique_ptr<EndpointUser> endpoint)
        : m_comm(comm)
        , m_platform_io(plat_io)
        , m_agent_name(agent_name)
        , m_num_send_down(num_send_down)
        , m_num_send_up(num_send_up)
        , m_tree_comm(std::move(tree_comm))
        , m_num_level_ctl(m_tree_comm->num_level_controlled())
        , m_max_level(m_num_level_ctl + 1)
        , m_root_level(m_tree_comm->root_level())
        , m_application_io(std::move(application_io))
        , m_reporter(std::move(reporter))
        , m_tracer(std::move(tracer))
        , m_agent(std::move(level_agent))
        , m_is_root(m_num_level_ctl == m_root_level)
        , m_in_policy(m_num_send_down, NAN)
        , m_out_policy(m_num_level_ctl)
        , m_in_sample(m_num_level_ctl)
        , m_out_sample(m_num_send_up, NAN)
        , m_endpoint(std::move(endpoint))
    {
        // Three dimensional vector over levels, children, and message
        // index.  These are used as temporary storage when passing
        // messages up and down the tree.
        for (int level = 0; level != m_num_level_ctl; ++level) {
            int num_children = m_tree_comm->level_size(level);
            m_out_policy[level] = std::vector<std::vector<double> >(num_children,
                                                                    std::vector<double>(m_num_send_down, NAN));
            m_in_sample[level] = std::vector<std::vector<double> >(num_children,
                                                                   std::vector<double>(m_num_send_up, NAN));
        }
    }

    Controller::~Controller()
    {
        geopm_signal_handler_check();
        geopm_signal_handler_revert();
        m_platform_io.restore_control();
    }

    void Controller::create_agents(void)
    {
        if (m_agent.size() == 0) {
            for (int level = 0; level < m_max_level; ++level) {
                m_agent.push_back(agent_factory().make_plugin(m_agent_name));
            }
        }
#ifdef GEOPM_DEBUG
        if (m_agent.size() == 0) {
            throw Exception("Controller requires at least one Agent",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
        if (m_max_level != (int)m_agent.size()) {
            throw Exception("Controller number of agents is incorrect",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
#endif
    }

    void Controller::init_agents(void)
    {
#ifdef GEOPM_DEBUG
        if (m_agent.size() != (size_t)m_max_level) {
            throw Exception("Controller must call create_agents() before init_agents().",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
#endif
        std::vector<int> fan_in(m_tree_comm->root_level());
        int level = 0;
        for (auto &it : fan_in) {
            it = m_tree_comm->level_size(level);
            ++level;
        }
        for (level = 0; level < m_max_level; ++level) {
            m_agent[level]->init(level, fan_in, (level < m_tree_comm->num_level_controlled()));
        }
    }

    void Controller::run(void)
    {
        m_application_io->connect();
        geopm_signal_handler_check();
        create_agents();
        geopm_signal_handler_check();
        m_platform_io.save_control();
        geopm_signal_handler_check();
        init_agents();
        geopm_signal_handler_check();
        m_reporter->init();
        geopm_signal_handler_check();
        setup_trace();
        geopm_signal_handler_check();
        m_application_io->controller_ready();
        geopm_signal_handler_check();

        m_application_io->update(m_comm);
        geopm_signal_handler_check();
        m_platform_io.read_batch();
        geopm_signal_handler_check();
        m_tracer->update(m_trace_sample, m_application_io->region_info());
        geopm_signal_handler_check();
        m_application_io->clear_region_info();

        while (!m_application_io->do_shutdown()) {
            step();
        }
        m_application_io->update(m_comm);
        geopm_signal_handler_check();
        m_platform_io.read_batch();
        geopm_signal_handler_check();
        m_tracer->update(m_trace_sample, m_application_io->region_info());
        geopm_signal_handler_check();
        m_application_io->clear_region_info();
        generate();
        m_platform_io.restore_control();
    }

    void Controller::generate(void)
    {
        std::vector<std::pair<std::string, std::string> > agent_report_header;
        if (m_is_root) {
            agent_report_header = m_agent[m_root_level]->report_header();
        }

        auto agent_host_report = m_agent[0]->report_host();

        m_reporter->generate(m_agent_name,
                             agent_report_header,
                             agent_host_report,
                             m_agent[0]->report_region(),
                             *m_application_io,
                             m_comm,
                             *m_tree_comm);
        m_tracer->flush();
    }

    void Controller::step(void)
    {
        walk_down();
        geopm_signal_handler_check();

        walk_up();
        geopm_signal_handler_check();
        m_agent[0]->wait();
        geopm_signal_handler_check();
    }

    void Controller::walk_down(void)
    {
        bool do_send = false;
        if (m_is_root) {
            /// @todo Return an is_updated bool.
            m_endpoint->read_policy(m_in_policy);
            do_send = true;
        }
        else {
            do_send = m_tree_comm->receive_down(m_num_level_ctl, m_in_policy);
        }
        for (int level = m_num_level_ctl - 1; level > -1; --level) {
            if (do_send) {
                m_agent[level + 1]->validate_policy(m_in_policy);
                m_agent[level + 1]->split_policy(m_in_policy, m_out_policy[level]);
                do_send = m_agent[level + 1]->do_send_policy();
            }
            if (do_send) {
                m_tree_comm->send_down(level, m_out_policy[level]);
            }
            do_send = m_tree_comm->receive_down(level, m_in_policy);
        }
        m_agent[0]->validate_policy(m_in_policy);
        m_agent[0]->adjust_platform(m_in_policy);
        if (m_agent[0]->do_write_batch()) {
            m_platform_io.write_batch();
        }
    }

    void Controller::walk_up(void)
    {
        m_application_io->update(m_comm);
        m_platform_io.read_batch();
        m_agent[0]->sample_platform(m_out_sample);
        bool do_send = m_agent[0]->do_send_sample();
        m_reporter->update();
        m_agent[0]->trace_values(m_trace_sample);
        m_tracer->update(m_trace_sample, m_application_io->region_info());
        m_application_io->clear_region_info();

        for (int level = 0; level < m_num_level_ctl; ++level) {
            if (do_send) {
                m_tree_comm->send_up(level, m_out_sample);
            }
            do_send = m_tree_comm->receive_up(level, m_in_sample[level]);
            if (do_send) {
                m_agent[level + 1]->aggregate_sample(m_in_sample[level], m_out_sample);
                do_send = m_agent[level + 1]->do_send_sample();
            }
        }
        if (do_send) {
            if (!m_is_root) {
                m_tree_comm->send_up(m_num_level_ctl, m_out_sample);
            }
            else {
                /// @todo At the root of the tree, send signals up to the
                /// resource manager.
            }
        }
    }

    void Controller::pthread(const pthread_attr_t *attr, pthread_t *thread)
    {
        int err = pthread_create(thread, attr, geopm_threaded_run, (void *)this);
        if (err) {
            throw Exception("Controller::pthread(): pthread_create() failed",
                            err, __FILE__, __LINE__);
        }
    }

    void Controller::setup_trace(void)
    {
        if (m_tracer == nullptr) {
            m_tracer = geopm::make_unique<TracerImp>(get_start_time());
        }
        std::vector<std::string> agent_cols = m_agent[0]->trace_names();
        std::vector<std::function<std::string(double)> > agent_formats = m_agent[0]->trace_formats();
        m_tracer->columns(agent_cols, agent_formats);
        m_trace_sample.resize(agent_cols.size());
    }

    void Controller::abort(void)
    {
        m_application_io->abort();
        m_platform_io.restore_control();
    }
}
