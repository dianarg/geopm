all: test_dlopen_singleton libsingleton_name.so

check: all
	./test_dlopen_singleton

test_dlopen_singleton: test_dlopen_singleton.c
	$(CC) $(CFLAGS) $(LDFLAGS) -rdynamic -ldl $< -o $@

libsingleton_name.so: SingletonName.cpp SingletonName.hpp
	$(CXX) $(CXXFLAGS) -c -shared -fPIC $< -o $@

clean:
	rm -f libsingleton_name.so test_dlopen_singleton

.PHONEY: all clean check
