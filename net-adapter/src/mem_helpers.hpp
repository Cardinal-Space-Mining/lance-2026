#pragma once

#include <cstdint>


namespace util
{

template<typename T>
inline constexpr void assignAndIncrement(uint8_t*& ptr, const T& val)
{
    reinterpret_cast<T*>(ptr)[0] = val;
    ptr += sizeof(T);
}
template<typename S, typename T>
inline constexpr void assignAndIncrementAs(uint8_t*& ptr, const T& val)
{
    reinterpret_cast<S*>(ptr)[0] = static_cast<S>(val);
    ptr += sizeof(S);
}

template<typename T>
inline constexpr void extractAndIncrement(uint8_t*& ptr, T& var)
{
    var = reinterpret_cast<T*>(ptr)[0];
    ptr += sizeof(T);
}
template<typename R, typename T>
inline constexpr void extractAndIncrementAs(uint8_t*& ptr, T& var)
{
    var = static_cast<T>(reinterpret_cast<R*>(ptr)[0]);
    ptr += sizeof(R);
}

};  // namespace util
