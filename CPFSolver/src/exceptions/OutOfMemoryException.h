#ifndef __OUT_OF_MEMORY_EXCEPTION__
#define __OUT_OF_MEMORY_EXCEPTION__

#include <stdexcept>

class OutOfMemoryException : public std::runtime_error {
public:
    OutOfMemoryException(const char *msg)
            : std::runtime_error(msg) { }

};

#endif //__OUT_OF_MEMORY_EXCEPTION__
