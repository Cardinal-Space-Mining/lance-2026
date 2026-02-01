/*******************************************************************************
*   Copyright (C) 2024-2026 Cardinal Space Mining Club                         *
*                                                                              *
*                                 ;xxxxxxx:                                    *
*                                ;$$$$$$$$$       ...::..                      *
*                                $$$$$$$$$$x   .:::::::::::..                  *
*                             x$$$$$$$$$$$$$$::::::::::::::::.                 *
*                         :$$$$$&X;      .xX:::::::::::::.::...                *
*                 .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :               *
*                :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.              *
*               :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.              *
*              ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::               *
*               X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.               *
*                .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                *
*                 X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                  *
*                 $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                    *
*                 $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                    *
*                 $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                    *
*                 X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                     *
*                 $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                    *
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                   *
*              +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                  *
*               +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                   *
*                :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                    *
*                ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                     *
*               ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                            *
*               ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                               *
*               :;;;;;;;;;;;;.  :$$$$$$$$$$X                                   *
*                .;;;;;;;;:;;    +$$$$$$$$$                                    *
*                  .;;;;;;.       X$$$$$$$:                                    *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*******************************************************************************/

#pragma once

#include <cstdint>
#include <cstring>
#include <type_traits>

namespace util
{

constexpr bool host_is_little_endian()
{
    return std::endian::native == std::endian::little;
}

// byteswapable = integral OR floating point OR enum
template<typename T>
inline constexpr bool is_byteswapable_v =
    std::is_integral_v<T> || std::is_floating_point_v<T> || std::is_enum_v<T>;

template<typename T>
struct is_byteswapable : std::bool_constant<is_byteswapable_v<T>>
{
};

// You can specialize any struct by opting it in as byteswapable
// and implementing a byteswap overload for the struct

template<typename T>
inline constexpr std::enable_if_t<std::is_integral_v<T>, T> byteswap(T v)
{
#if defined(__clang__) || defined(__GNUC__)
    if constexpr (sizeof(T) == 1)
    {
        return v;
    }
    else if constexpr (sizeof(T) == 2)
    {
        return static_cast<T>(__builtin_bswap16(static_cast<uint16_t>(v)));
    }
    else if constexpr (sizeof(T) == 4)
    {
        return static_cast<T>(__builtin_bswap32(static_cast<uint32_t>(v)));
    }
    else if constexpr (sizeof(T) == 8)
    {
        return static_cast<T>(__builtin_bswap64(static_cast<uint64_t>(v)));
    }
#elif defined(_MSC_VER)
    if constexpr (sizeof(T) == 1)
    {
        return v;
    }
    else if constexpr (sizeof(T) == 2)
    {
        return static_cast<T>(_byteswap_ushort(static_cast<uint16_t>(v)));
    }
    else if constexpr (sizeof(T) == 4)
    {
        return static_cast<T>(_byteswap_ulong(static_cast<uint32_t>(v)));
    }
    else if constexpr (sizeof(T) == 8)
    {
        return static_cast<T>(_byteswap_uint64(static_cast<uint64_t>(v)));
    }
#endif

    // fallback constexpr implementation
    using U = std::make_unsigned_t<T>;
    U u = static_cast<U>(v), r = 0;
    for (size_t i = 0; i < sizeof(U); ++i)
    {
        r = (r << 8) | (u & 0xFF);
        u >>= 8;
    }
    return static_cast<T>(r);
}

template<typename T>
inline constexpr std::enable_if_t<std::is_enum_v<T>, T> byteswap(T v)
{
    using U = std::underlying_type_t<T>;
    return static_cast<T>(byteswap(static_cast<U>(v)));
}

template<typename T>
inline constexpr std::enable_if_t<std::is_floating_point_v<T>, T> byteswap(T v)
{
    if constexpr (sizeof(T) == 4)
    {
        uint32_t tmp;
        std::memcpy(&tmp, &v, sizeof(T));
        tmp = byteswap(tmp);
        std::memcpy(&v, &tmp, sizeof(T));
        return v;
    }
    else if constexpr (sizeof(T) == 8)
    {
        uint64_t tmp;
        std::memcpy(&tmp, &v, sizeof(T));
        tmp = byteswap(tmp);
        std::memcpy(&v, &tmp, sizeof(T));
        return v;
    }
    else
    {
        static_assert(
            sizeof(T) == 4 || sizeof(T) == 8,
            "Unsupported floating point size for byteswap");
    }
}

template<typename T>
inline constexpr std::enable_if_t<is_byteswapable_v<T>, T> little_endian(T v)
{
    if constexpr (sizeof(T) == 1 || host_is_little_endian())
    {
        return v;
    }
    else
    {
        return byteswap(v);
    }
}

template<typename T>
inline void write(uint8_t*& ptr, const T& value)
{
    static_assert(
        is_byteswapable_v<T>,
        "write<T>: T must be a byteswapable type: integral, floating-point, enum, "
        "or a user-defined type specializing util::is_byteswapable<T>");

    if constexpr (sizeof(T) == 1 || host_is_little_endian())
    {
        std::memcpy(ptr, &value, sizeof(T));
    }
    else
    {
        T tmp = byteswap(value);
        std::memcpy(ptr, &tmp, sizeof(T));
    }

    ptr += sizeof(T);
}

template<typename StorageT, typename T>
inline void write_as(uint8_t*& ptr, const T& value)
{
    static_assert(
        is_byteswapable_v<StorageT>,
        "write_as<StorageT>: StorageT must be a byteswapable type: integral, floating-point, enum, "
        "or a user-defined type specializing util::is_byteswapable<StorageT>");

    if constexpr (std::is_same_v<T, StorageT> && host_is_little_endian())
    {
        std::memcpy(ptr, &value, sizeof(StorageT));
    }
    else
    {
        StorageT tmp = little_endian(static_cast<StorageT>(value));
        std::memcpy(ptr, &tmp, sizeof(StorageT));
    }

    ptr += sizeof(StorageT);
}

template<typename T>
inline void read(const uint8_t*& ptr, T& out)
{
    static_assert(
        is_byteswapable_v<T>,
        "read<T>: T must be a byteswapable type: integral, floating-point, enum, "
        "or a user-defined type specializing util::is_byteswapable<T>");

    if constexpr (sizeof(T) == 1 || host_is_little_endian())
    {
        std::memcpy(&out, ptr, sizeof(T));
    }
    else
    {
        T tmp;
        std::memcpy(&tmp, ptr, sizeof(T));
        out = byteswap(tmp);
    }

    ptr += sizeof(T);
}

template<typename StorageT, typename T>
inline void read_as(const uint8_t*& ptr, T& out)
{
    static_assert(
        util::is_byteswapable_v<StorageT>,
        "read_as<StorageT>: T must be a byteswapable type: integral, floating-point, enum, "
        "or a user-defined type specializing util::is_byteswapable<StorageT>");


    if constexpr (std::is_same_v<T, StorageT> && host_is_little_endian())
    {
        std::memcpy(&out, ptr, sizeof(StorageT));
    }
    else
    {
        StorageT tmp{};
        std::memcpy(&tmp, ptr, sizeof(StorageT));
        out = static_cast<T>(little_endian(tmp));
    }

    ptr += sizeof(StorageT);
}

}  // namespace util
