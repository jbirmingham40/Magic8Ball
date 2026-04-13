#include <stdarg.h>
extern int log_printfv(const char *format, va_list arg);
int __wrap_log_printf(const char *format, ...) {
    va_list arg;
    va_start(arg, format);
    int len = log_printfv(format, arg);
    va_end(arg);
    return len;
}
