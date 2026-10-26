#pragma once

#include <Arduino.h>
#include <stdarg.h>

// Thread-safe with respect to formatting, provided the selected
// printf_ implementation is reentrant. Serial output is delegated
// to the underlying Arduino Print object.
class MfPrint {
public:
    explicit MfPrint(Print& output) : out(output) {}

    // Preserve Arduino's existing integer/string/character formatting.
    template<typename T>
    size_t print(const T& value) {
        return out.print(value);
    }

    template<typename T>
    size_t print(const T& value, int base) {
        return out.print(value, base);
    }

    // Floating-point overloads use Eyal Roz's formatter.
    size_t print(float value, int digits = 2);
    size_t print(double value, int digits = 2);

    // Arduino-style println overloads.
    size_t println() {
        return out.write((const uint8_t*)"\r\n", 2);
    }

    template<typename T>
    size_t println(const T& value) {
        size_t n = print(value);
        return n + println();
    }

    size_t println(float value, int digits = 2) {
        size_t n = print(value, digits);
        return n + println();
    }

    size_t println(double value, int digits = 2) {
        size_t n = print(value, digits);
        return n + println();
    }

    // printf-style formatting, including floating-point specifiers.
    size_t printf(const char* format, ...);

private:
    Print& out;

    size_t vprintf(const char* format, va_list args);
    size_t printFloat(double value, int digits);
};
