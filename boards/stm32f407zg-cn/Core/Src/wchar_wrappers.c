/**
 * @file wchar_wrappers.c
 * @brief Wide character function wrappers for embedded environment
 *
 * These wrapper functions are required by the ARM GCC toolchain when using
 * newlib-nano with C++ standard library. They replace wide character functions
 * with stub implementations since wide character support is not needed in
 * embedded systems and would significantly increase code size.
 *
 * The linker is configured with --wrap flags to redirect calls to these functions.
 */

#include <errno.h>
#include <stddef.h>
#include <wchar.h>

/**
 * @brief Wrapper for putwc - write wide character to stream
 * @return WEOF to indicate unsupported operation
 */
wint_t __wrap_putwc(wchar_t wc, FILE* stream)
{
    (void)wc;
    (void)stream;
    errno = ENOSYS;  // Function not implemented
    return WEOF;
}

/**
 * @brief Wrapper for getwc - get wide character from stream
 * @return WEOF to indicate unsupported operation
 */
wint_t __wrap_getwc(FILE* stream)
{
    (void)stream;
    errno = ENOSYS;  // Function not implemented
    return WEOF;
}

/**
 * @brief Wrapper for ungetwc - push back wide character to stream
 * @return WEOF to indicate unsupported operation
 */
wint_t __wrap_ungetwc(wint_t wc, FILE* stream)
{
    (void)wc;
    (void)stream;
    errno = ENOSYS;  // Function not implemented
    return WEOF;
}

/**
 * @brief Wrapper for swprintf - formatted wide string output
 * @return -1 to indicate unsupported operation
 */
int __wrap_swprintf(wchar_t* wcs, size_t maxlen, const wchar_t* format, ...)
{
    (void)wcs;
    (void)maxlen;
    (void)format;
    errno = ENOSYS;  // Function not implemented
    return -1;
}
