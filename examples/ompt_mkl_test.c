#define _POSIX_C_SOURCE 200809L
#include <stdlib.h>
#include <stdio.h>
#include <mkl.h>
#include <omp.h>
#include <omp-tools.h>

/*
$ # Some notes on running example.
$ # Compiler version:
$ icc --version
icc (ICC) 19.0.4.243 20190416
Copyright (C) 1985-2019 Intel Corporation.  All rights reserved.

$ # Compile command
$ icc -std=c99 -fopenmp -mkl ompt_mkl_test.c
$ # Run without dgemm call, stream callback gives consistant address
$ ./a.out 0
Call stream_fn()
ompt_initialize()
parallel_begin: 0x402505
parallel_end:   0x402505
Call stream_fn()
parallel_begin: 0x402505
parallel_end:   0x402505
Call stream_fn()
parallel_begin: 0x402505
parallel_end:   0x402505
Call stream_fn()
parallel_begin: 0x402505
parallel_end:   0x402505
Call stream_fn()
parallel_begin: 0x402505
parallel_end:   0x402505
Call stream_fn()
parallel_begin: 0x402505
parallel_end:   0x402505
ompt_finalize()
$ # Run with small dgemm call, no change to the stream callback address
$ ./a.out 1024
Call stream_fn()
ompt_initialize()
parallel_begin: 0x402505
parallel_end:   0x402505
Call dgemm_fn()
parallel_begin: 0x78063553
parallel_end:   0x78063553
Call stream_fn()
parallel_begin: 0x402505
parallel_end:   0x402505
Call stream_fn()
parallel_begin: 0x402505
parallel_end:   0x402505
Call stream_fn()
parallel_begin: 0x402505
parallel_end:   0x402505
Call dgemm_fn()
parallel_begin: 0x78063553
parallel_end:   0x78063553
Call stream_fn()
parallel_begin: 0x402505
parallel_end:   0x402505
Call dgemm_fn()
parallel_begin: 0x78063553
parallel_end:   0x78063553
Call stream_fn()
parallel_begin: 0x402505
parallel_end:   0x402505
Call dgemm_fn()
parallel_begin: 0x78063553
parallel_end:   0x78063553
ompt_finalize()
$ # Run with large dgemm call.
$ # The stream callback address comes back different after call to dgemm.
$ # Behavior seems to reset if two stream calls are made in a row.
$ ./a.out 8192
Call stream_fn()
ompt_initialize()
parallel_begin: 0x402505
parallel_end:   0x402505
Call dgemm_fn()
parallel_begin: 0x2af48fac
parallel_end:   0x2af48fac
Call stream_fn()
parallel_begin: 0x2af3fb0f
parallel_end:   0x2af3fb0f
Call stream_fn()
parallel_begin: 0x402505
parallel_end:   0x402505
Call stream_fn()
parallel_begin: 0x402505
parallel_end:   0x402505
Call dgemm_fn()
parallel_begin: 0x2af48fac
parallel_end:   0x2af48fac
Call stream_fn()
parallel_begin: 0x2af3fb0f
parallel_end:   0x2af3fb0f
Call dgemm_fn()
parallel_begin: 0x2af48fac
parallel_end:   0x2af48fac
Call stream_fn()
parallel_begin: 0x2af3fb0f
parallel_end:   0x2af3fb0f
Call dgemm_fn()
parallel_begin: 0x2af48fac
parallel_end:   0x2af48fac
ompt_finalize()

*/

/*
 * STACKS FROM GDB on two different stream calls
 *
 *
 * Note that there are two calls to __kmpc_fork_call() in the stack.
 * The first call is passed the same arguments in both cases, but it
 * passes a different value for "invoker" the next call to
 * __kmp_fork_call()


Call stream_fn()
ompt_initialize()
#0  on_ompt_event_parallel_begin (encountering_task_data=0x7ffd2457fe68, encountering_task_frame=0x7ffd2457fe50, parallel_data=0x7fffffffb230, 
    requested_parallelism=88, flags=-2147483646, codeptr_ra=0x4024d5 <stream_fn+545>) at ompt_mkl_test.c:120
#1  0x00007ffff0772bf7 in __kmp_fork_call (loc=0x7ffd2457fe68, gtid=609746512, call_context=(unknown: 4294947376), argc=88, microtask=0x80000002, 
    invoker=0x4024d5 <stream_fn+545>, ap=0x7fffffffb340) at ../../src/kmp_runtime.cpp:1656
#2  0x00007ffff0735020 in __kmpc_fork_call (loc=0x7ffd2457fe68, argc=609746512, microtask=0x7fffffffb230) at ../../src/kmp_csupport.cpp:361
#3  0x00000000004024d5 in stream_fn () at ompt_mkl_test.c:219
#4  0x0000000000402793 in main (argc=2, argv=0x7fffffffb5a8) at ompt_mkl_test.c:233
parallel_begin: 0x4024d5
parallel_end:   0x4024d5
.
.
.

Call stream_fn()
#0  on_ompt_event_parallel_begin (encountering_task_data=0x7ffd2457fe68, encountering_task_frame=0x7ffd2457fe50, parallel_data=0x7fffffffb230, 
    requested_parallelism=88, flags=-2147483646, codeptr_ra=0x7ffff506bb0f <omp_parallel_acopy_lookahead+2463>) at ompt_mkl_test.c:120
#1  0x00007ffff0772bf7 in __kmp_fork_call (loc=0x7ffd2457fe68, gtid=609746512, call_context=(unknown: 4294947376), argc=88, microtask=0x80000002, 
    invoker=0x7ffff506bb0f <omp_parallel_acopy_lookahead+2463>, ap=0x7fffffffb340) at ../../src/kmp_runtime.cpp:1656
#2  0x00007ffff0735020 in __kmpc_fork_call (loc=0x7ffd2457fe68, argc=609746512, microtask=0x7fffffffb230) at ../../src/kmp_csupport.cpp:361
#3  0x00000000004024d5 in stream_fn () at ompt_mkl_test.c:219
#4  0x00000000004027a2 in main (argc=2, argv=0x7fffffffb5a8) at ompt_mkl_test.c:235
parallel_begin: 0xf506bb0f
parallel_end:   0xf506bb0f


*/


static void on_ompt_event_parallel_begin(ompt_data_t *encountering_task_data,
                                         const ompt_frame_t *encountering_task_frame,
                                         ompt_data_t *parallel_data,
                                         unsigned int requested_parallelism,
                                         int flags,
                                         const void *codeptr_ra)
{
    printf("parallel_begin: 0x%x\n", codeptr_ra);
}

static void on_ompt_event_parallel_end(ompt_data_t *parallel_data,
                                       ompt_data_t *encountering_task_data,
                                       int flags,
                                       const void *codeptr_ra)
{
    printf("parallel_end:   0x%x\n", codeptr_ra);
}


int ompt_initialize(ompt_function_lookup_t lookup,
                    int initial_device_num,
                    ompt_data_t *tool_data)
{
    printf("ompt_initialize()\n");
    ompt_set_callback_t ompt_set_callback = (ompt_set_callback_t) lookup("ompt_set_callback");
    ompt_set_callback(ompt_callback_parallel_begin, (ompt_callback_t) &on_ompt_event_parallel_begin);
    ompt_set_callback(ompt_callback_parallel_end, (ompt_callback_t) &on_ompt_event_parallel_end);
    return 1;
}

void ompt_finalize(ompt_data_t *data)
{
    printf("ompt_finalize()\n");
}

ompt_start_tool_result_t *ompt_start_tool(unsigned int omp_version, const char *runtime_version)
{
    static double time = 0;
    time = omp_get_wtime();
    static ompt_start_tool_result_t ompt_start_tool_result = {&ompt_initialize,
                                                              &ompt_finalize,
                                                              {.ptr=&time}};
    return &ompt_start_tool_result;
}


void dgemm_fn(int matrix_size)
{
    if (matrix_size == 0) {
        return;
    }
    printf("Call dgemm_fn()\n");

    static int is_once = 1;
    static double *matrix_a = NULL;
    static double *matrix_b = NULL;
    static double *matrix_c = NULL;
    static const size_t PAD_SIZE = 128;
    if (is_once) {
        size_t mem_size = sizeof(double) * (matrix_size * (matrix_size + PAD_SIZE));
        posix_memalign((void **)&matrix_a, PAD_SIZE, mem_size);
        posix_memalign((void **)&matrix_b, PAD_SIZE, mem_size);
        posix_memalign((void **)&matrix_c, PAD_SIZE, mem_size);
        for (size_t i = 0; i < mem_size / sizeof(double); ++i) {
            matrix_a[i] = 2.0 * i;
            matrix_b[i] = 3.0 * i;
        }
        is_once = 0;
    }
    int M = matrix_size;
    int N = matrix_size;
    int K = matrix_size;
    int LDA = matrix_size + PAD_SIZE / sizeof(double);
    int LDB = matrix_size + PAD_SIZE / sizeof(double);
    int LDC = matrix_size + PAD_SIZE / sizeof(double);
    double alpha = 2.0;
    double beta = 3.0;
    char transa = 'n';
    char transb = 'n';
    dgemm(&transa, &transb, &M, &N, &K, &alpha,
          matrix_a, &LDA, matrix_b, &LDB, &beta, matrix_c, &LDC);
}

void stream_fn(void)
{
    printf("Call stream_fn()\n");

    static int is_once = 1;
    static double *array_a = NULL;
    static double *array_b = NULL;
    static double *array_c = NULL;
    static const size_t ARRAY_LEN = 500000000ULL;

    if (is_once) {
        posix_memalign((void **)&array_a, 64, ARRAY_LEN * sizeof(double));
        posix_memalign((void **)&array_b, 64, ARRAY_LEN * sizeof(double));
        posix_memalign((void **)&array_c, 64, ARRAY_LEN * sizeof(double));
        for (size_t i = 0; i < ARRAY_LEN; i++) {
            array_a[i] = 0.0;
            array_b[i] = 1.0;
            array_c[i] = 2.0;
        }
        is_once = 0;
    }

    double scalar = 3.0;
#pragma omp parallel for
    for (size_t j = 0; j < ARRAY_LEN; ++j) {
        array_a[j] = array_b[j] + scalar * array_c[j];
    }
}

int main(int argc, char **argv)
{
#pragma noinline
{
    int dgemm_size = 0;
    if (argc != 1) {
        dgemm_size = atoi(argv[1]);
    }
    stream_fn();
    dgemm_fn(dgemm_size);
    stream_fn();
    stream_fn();
    for (int i = 0; i < 3; ++i) {
        stream_fn();
        dgemm_fn(dgemm_size);
    }
}
}
