#ifndef __TIMEOUT_EXCEPTION__
#define __TIMEOUT_EXCEPTION__

#include <stdexcept>

class TimeoutException : public std::runtime_error {
public:
    TimeoutException(const char *msg)
            : std::runtime_error(msg) { }

};


#endif //__TIMEOUT_EXCEPTION__
