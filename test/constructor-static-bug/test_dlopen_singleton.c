#include <stdio.h>
#include <dlfcn.h>

#define CHECK_LOADED

int main(int argc, char **argv)
{
    const char *lib_name = "./libsingleton_name.so";
    int err = 0;

#ifdef CHECK_LOADED
    if (dlopen(lib_name, RTLD_NOLOAD) == NULL)
#endif
    if (dlopen(lib_name, RTLD_LAZY) == NULL) {
        err = -1;
        fprintf(stderr, "Warning: failed to dlopen plugin %s.\n", lib_name);
    }
    return err;
}
