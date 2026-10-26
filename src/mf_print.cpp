
#include "mf_print.h"
#include <printf/printf.h>  // Eyal Roz's printf library

#include <math.h>
#include <stdio.h>

namespace {
constexpr size_t BUFFER_SIZE = 256;
}

size_t MfPrint::print(float value, int digits) {
    return printFloat(value, digits);
}

size_t MfPrint::print(double value, int digits) {
    return printFloat(value, digits);
}

size_t MfPrint::printFloat(double value, int digits) {
    // Arduino Print::print(float) defaults to two decimal places.
    if (digits < 0) digits = 0;
    if (digits > 9) digits = 9;

    char buffer[BUFFER_SIZE];

    int n = snprintf_(buffer, sizeof(buffer), "%.*f", digits, value);
    if (n < 0) return 0;

    size_t len = (n < (int)sizeof(buffer))
               ? (size_t)n
               : sizeof(buffer) - 1;

    return out.write((const uint8_t*)buffer, len);
}

size_t MfPrint::printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    size_t n = vprintf(format, args);
    va_end(args);
    return n;
}

size_t MfPrint::vprintf(const char* format, va_list args) {
    char buffer[BUFFER_SIZE];

    int n = vsnprintf_(buffer, sizeof(buffer), format, args);
    if (n < 0) return 0;

    size_t len = (n < (int)sizeof(buffer))
               ? (size_t)n
               : sizeof(buffer) - 1;

    return out.write((const uint8_t*)buffer, len);
}
