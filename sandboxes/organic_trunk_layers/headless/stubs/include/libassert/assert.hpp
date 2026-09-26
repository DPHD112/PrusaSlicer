#pragma once
// Local stand-in for libassert (its download is blocked here): the ASSERT family of
// macros, checked in release builds too, and the few names Slic3r/Assert.cpp uses.

// The real header pulls in these, and some PrusaSlicer files rely on it.
#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace libassert {
enum class assert_type { assertion, debug_assertion, assumption, panic, unreachable };
struct assertion_info {
    assert_type type{assert_type::assertion};
    std::string to_string() const { return {}; }
};
inline void set_failure_handler(void (*)(const assertion_info&)) {}
inline void enable_virtual_terminal_processing_if_needed() {}

[[noreturn]] inline void stub_fail(const char* what, const char* file, int line, const char* func)
{
    std::cerr << what << " in " << file << ":" << line << " (function " << func << ")\n";
    std::abort();
}
template<typename Message, typename... Extra>
[[noreturn]] void stub_fail(const char* what, const char* file, int line, const char* func, const Message& message, const Extra&...)
{
    std::cerr << what << ": " << message << "\nin " << file << ":" << line << " (function " << func << ")\n";
    std::abort();
}
// Returns lvalues by reference and rvalues by value, like libassert's ASSERT_VAL.
template<typename T, typename... Message>
T assert_val(T&& value, const char* expr, const char* file, int line, const char* func, const Message&... message)
{
    if (! value)
        stub_fail((std::string("Assertion ") + expr + " failed").c_str(), file, line, func, message...);
    return std::forward<T>(value);
}
} // namespace libassert

#define LIBASSERT_PRIMITIVE_PANIC(msg) ::libassert::stub_fail(msg, __FILE__, __LINE__, __PRETTY_FUNCTION__)

#define ASSERT(expr, ...) do { if (! (expr)) ::libassert::stub_fail("Assertion " #expr " failed", __FILE__, __LINE__, __PRETTY_FUNCTION__ __VA_OPT__(,) __VA_ARGS__); } while (0)
#define ASSERT_VAL(expr, ...) ::libassert::assert_val((expr), #expr, __FILE__, __LINE__, __PRETTY_FUNCTION__ __VA_OPT__(,) __VA_ARGS__)
#define UNREACHABLE(...) ::libassert::stub_fail("Unreachable", __FILE__, __LINE__, __PRETTY_FUNCTION__ __VA_OPT__(,) __VA_ARGS__)
#define PANIC(...) ::libassert::stub_fail("Panic!", __FILE__, __LINE__, __PRETTY_FUNCTION__ __VA_OPT__(,) __VA_ARGS__)

#ifdef NDEBUG
#define DEBUG_ASSERT(...) do {} while (0)
#define DEBUG_ASSERT_VAL(expr, ...) (expr)
#else
#define DEBUG_ASSERT(...) ASSERT(__VA_ARGS__)
#define DEBUG_ASSERT_VAL(...) ASSERT_VAL(__VA_ARGS__)
#endif
