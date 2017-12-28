

            // How do we ask how many sockets per board?
            //    num_domain(M_DOMAIN_BOARD, M_DOMAIN_PACKAGE)
            // How do we ask how many CPUs per socket?
            //    num_domain(M_DOMAIN_PACKAGE, M_DOMAIN_CPU)
            // How do we know which Linux logical CPUs are on core 5?
            //    domain_cpus(M_DOMAIN_CORE, 5, cpu_idx_set);
            // How do we ask if there is on package memory?
            //    num_domain(M_DOMAIN_PACKAGE, M_DOMAIN_PACKAGE_MEMORY) > 0
            // How do we ask if the frequency control is per package or per core?
            //    platform_io().control_domain_type("PERF_CTL:FREQ") == M_DOMAIN_PACKAGE
            // How do we ask which socket Linux logical CPU 8 is on?
            //    domain_idx(M_DOMAIN_PACKAGE, 8)
            // How do we find out all of the other Linux logical CPUs that share a socket with CPU 8?
            //    domain_cpus(M_DOMAIN_PACKAGE, domain_idx(M_DOMAIN_PACKAGE, 8), socket_cpu_set)
            // How do we define a group all linux logical CPUs that are divisable by 4?
            //    int num_cpu = num_domain(M_DOMAIN_CPU);
            //    for (int i = 0; i < num_cpu; i +=4) {
            //        domain_idx.push_back(i);
            //    }
            //    uint64_t group_domain = group_ext_define(0, domain_idx);

const char *lscpu_laptop =
"Architecture:          x86_64\n"
"CPU op-mode(s):        32-bit, 64-bit\n"
"Byte Order:            Little Endian\n"
"CPU(s):                2\n"
"On-line CPU(s) list:   0,1\n"
"Thread(s) per core:    1\n"
"Core(s) per socket:    2\n"
"Socket(s):             1\n"
"NUMA node(s):          1\n"
"Vendor ID:             GenuineIntel\n"
"CPU family:            6\n"
"Model:                 61\n"
"Model name:            Intel(R) Core(TM) i7-5650U CPU @ 2.20GHz\n"
"Stepping:              4\n"
"CPU MHz:               2200.000\n"
"BogoMIPS:              4400.00\n"
"Hypervisor vendor:     KVM\n"
"Virtualization type:   full\n"
"L1d cache:             32K\n"
"L1i cache:             32K\n"
"L2 cache:              256K\n"
"L3 cache:              4096K\n"
"NUMA node0 CPU(s):     0,1\n";
