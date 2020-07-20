        margin_factor = 0.25
        agent_runtime = dict()
        for agent in self._agent_list:
            run_name = '{}_{}_{}'.format(self._test_name, agent, app_name)
            report_path = '{}.report'.format(run_name)
            trace_path = '{}.trace'.format(run_name)

            runtime_list = self.get_power_data(app_name, agent, report_path, trace_path)
            agent_runtime[agent] = max(runtime_list)

            if agent == 'power_governor':
                mean_runtime = sum(runtime_list) / len(runtime_list)
                max_runtime = max(runtime_list)
                margin = margin_factor * (max_runtime - mean_runtime)

        if self._show_details:
            sys.stdout.write("\nAverage runtime stats:\n")
            sys.stdout.write("governor runtime: {}, balancer runtime: {}, margin: {}\n".format(
                agent_runtime['power_governor'], agent_runtime['power_balancer'], margin))

        self.assertGreater(agent_runtime['power_governor'] - margin,
                           agent_runtime['power_balancer'],
                           "governor runtime: {}, balancer runtime: {}, margin: {}".format(
                               agent_runtime['power_governor'], agent_runtime['power_balancer'], margin))
