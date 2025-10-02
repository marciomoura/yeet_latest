/**
 * @file wchar_stubs.c
 * @brief Stub implementations for wide character functions not provided by newlib-nano
 *
 * These functions are not needed in embedded systems but may be referenced by C++
 * standard library code (especially iostreams). By wrapping them, we avoid linker
 * errors while keeping code size small with nano.specs.
 */

#include <errno.h>
#include <wchar.h>


// Stub implementation for swprintf (wide string printf)
int __wrap_swprintf(wchar_t *wcs, size_t maxlen, const wchar_t *format, ...)
{
    // Not implemented in embedded system - return error
    errno = ENOSYS;
    return -1;
}

// Stub implementation for putwc (put wide character)
wint_t __wrap_putwc(wchar_t wc, FILE *stream)
{
    // Not implemented in embedded system - return error
    errno = ENOSYS;
    return WEOF;
}

// Stub implementation for getwc (get wide character)
wint_t __wrap_getwc(FILE *stream)
{
    // Not implemented in embedded system - return error
    errno = ENOSYS;
    return WEOF;
}

// Stub implementation for ungetwc (unget wide character)
wint_t __wrap_ungetwc(wint_t wc, FILE *stream)
{
    // Not implemented in embedded system - return error
    errno = ENOSYS;
    return WEOF;
}
