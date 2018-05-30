#include <iostream>

#include "SingletonName.hpp"

static int dlopen_counter = 0;
static void __attribute__((constructor)) dlopen_constructor(void)
{
    std::cerr << "IN:" << SingletonName::singleton_name().name();
    SingletonName::singleton_name().name("dlopen_constructor(" +
                                         std::to_string(dlopen_counter) + ")\n");
    std::cerr << "OUT:" << SingletonName::singleton_name().name();
    ++dlopen_counter;
}

SingletonName::SingletonName()
    : m_name("SingletonName")
{
    std::cerr << "SingletonName::SingletonName(" << m_name << ")\n";
}

SingletonName::~SingletonName()
{
    std::cerr << "SingletonName::~SingletonName(" << m_name << ")\n";
}

std::string SingletonName::name(void)
{
    return m_name;
}

void SingletonName::name(std::string nn)
{
    m_name = nn;
}

SingletonName &SingletonName::singleton_name(void)
{
    static SingletonName instance;
    return instance;
}
