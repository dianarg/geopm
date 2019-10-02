#include <stdio.h>
#include <mkl.h>
#include <omp.h>
#include <omp-tools.h>

/* Test showing strange behavior from OMPT interface when dgemm from
   MKL is called.  When large dgemm calls are made prior to an OpenMP
   parallel region the OMPT callback for the OpenMP parallel region
   within the executable is passed an instruction address from a
   loaded library.  The incorrect address is from near a function
   called omp_parallel_acopy_lookahead. */

/* Tested with:

    $ icc --version
    icc (ICC) 19.0.4.243 20190416
    Copyright (C) 1985-2019 Intel Corporation.  All rights reserved.


   To compile:

    $ icc -std=c99 -fopenmp -mkl ompt_mkl_test.c

   Output without calls to dgemm, stream codeptr_ra is always 0x402255:

    $ ./a.out
    stream_fn()
    codeptr_ra: 0x402255
    stream_fn()
    codeptr_ra: 0x402255
    stream_fn()
    codeptr_ra: 0x402255
    stream_fn()
    codeptr_ra: 0x402255
    stream_fn()
    codeptr_ra: 0x402255

   Output with calls to small dgemm, stream codeptr_ra still 0x402255:

    $ ./a.out 1024
    stream_fn()
    codeptr_ra: 0x402255
    dgemm_fn()
    codeptr_ra: 0x9cc7fac
    stream_fn()
    codeptr_ra: 0x402255
    stream_fn()
    codeptr_ra: 0x402255
    dgemm_fn()
    codeptr_ra: 0x9cc7fac
    stream_fn()
    codeptr_ra: 0x402255
    stream_fn()
    codeptr_ra: 0x402255

  Output with calls to large dgemm, stream codeptr_ra changes to
  0x19230b0f after each call to dgemm, other calls are still giving
  0x402255.

    $ ./a.out 8192
    stream_fn()
    codeptr_ra: 0x402255
    dgemm_fn()
    codeptr_ra: 0x19239fac
    stream_fn()
    codeptr_ra: 0x19230b0f
    stream_fn()
    codeptr_ra: 0x402255
    dgemm_fn()
    codeptr_ra: 0x19239fac
    stream_fn()
    codeptr_ra: 0x19230b0f
    stream_fn()
    codeptr_ra: 0x402255

*/

static void on_ompt_event_parallel_begin(ompt_data_t *encountering_task_data,
                                         const ompt_frame_t *encountering_task_frame,
                                         ompt_data_t *parallel_data,
                                         unsigned int requested_parallelism,
                                         int flags,
                                         const void *codeptr_ra)
{
    printf("codeptr_ra: 0x%x\n", codeptr_ra);
}

int ompt_initialize(ompt_function_lookup_t lookup,
                    int initial_device_num,
                    ompt_data_t *tool_data)
{
    ompt_set_callback_t ompt_set_callback = (ompt_set_callback_t) lookup("ompt_set_callback");
    ompt_set_callback(ompt_callback_parallel_begin, (ompt_callback_t) &on_ompt_event_parallel_begin);
    return 1;
}

void ompt_finalize(ompt_data_t *data)
{

}

ompt_start_tool_result_t *ompt_start_tool(unsigned int omp_version, const char *runtime_version)
{
    static ompt_start_tool_result_t ompt_start_tool_result = {&ompt_initialize,
                                                              &ompt_finalize,
                                                              {}};
    return &ompt_start_tool_result;
}


void dgemm_fn(int matrix_size)
{
    if (matrix_size == 0) {
        return;
    }
    printf("dgemm_fn()\n");

    static int is_once = 1;
    static int matrix_size_s = 0;
    static double *matrix_a = NULL;
    static double *matrix_b = NULL;
    static double *matrix_c = NULL;
    if (is_once) {
        matrix_size_s = matrix_size;
        size_t num_el = matrix_size_s * matrix_size_s;
        size_t mem_size = sizeof(double) * num_el;
        matrix_a = malloc(mem_size);
        matrix_b = malloc(mem_size);
        matrix_c = malloc(mem_size);
        if (!(matrix_a && matrix_b && matrix_c)) {
            fprintf(stderr, "Error: could not allocate matrices");
            exit(-1);
        }
        for (size_t i = 0; i < num_el; ++i) {
            matrix_a[i] = 2.0 * i;
            matrix_b[i] = 3.0 * i;
        }
        is_once = 0;
    }
    int M = matrix_size_s;
    int N = matrix_size_s;
    int K = matrix_size_s;
    int LDA = matrix_size_s;
    int LDB = matrix_size_s;
    int LDC = matrix_size_s;
    double alpha = 2.0;
    double beta = 3.0;
    char transa = 'n';
    char transb = 'n';
    dgemm(&transa, &transb, &M, &N, &K, &alpha,
          matrix_a, &LDA, matrix_b, &LDB, &beta, matrix_c, &LDC);
}

void stream_fn(void)
{
    printf("stream_fn()\n");

    static int is_once = 1;
    static double *array_a = NULL;
    static double *array_b = NULL;
    static double *array_c = NULL;
    const static size_t ARRAY_LEN = 500000000ULL; // 4 GB for each array

    if (is_once) {
        array_a = malloc(ARRAY_LEN * sizeof(double));
        array_b = malloc(ARRAY_LEN * sizeof(double));
        array_c = malloc(ARRAY_LEN * sizeof(double));
        if (!(array_a && array_b && array_c)) {
            fprintf(stderr, "Error: could not allocate arrays");
            exit(-1);
        }
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
    dgemm_fn(dgemm_size);
    stream_fn();
    stream_fn();
}
}
