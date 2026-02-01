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

constexpr bool HOST_IS_LITTLE_ENDIAN =
    static_cast<bool>(std::endian::native == std::endian::little);

// byteswapable = integral OR floating point OR enum
template<typename T>
inline constexpr bool is_byteswapable_v =
    std::is_integral_v<T> || std::is_floating_point_v<T> || std::is_enum_v<T>;

template<typename T>
struct is_byteswapable : std::bool_constant<is_byteswapable_v<T>>
{
};

// You can specialize any struct by opting it in as byteswapable
// and implementing a byteSwap overload for the struct

template<typename T>
inline constexpr std::enable_if_t<std::is_integral_v<T>, T> byteSwap(T v)
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
inline constexpr std::enable_if_t<std::is_enum_v<T>, T> byteSwap(T v)
{
    using U = std::underlying_type_t<T>;
    return static_cast<T>(byteSwap(static_cast<U>(v)));
}

template<typename T>
inline constexpr std::enable_if_t<std::is_floating_point_v<T>, T> byteSwap(T v)
{
    // reinterpret float bits as int, swap bytes, reinterpret back to float
    if constexpr (sizeof(T) == 4)
    {
        uint32_t u = std::bit_cast<uint32_t>(v);
        u = byteSwap(u);
        return std::bit_cast<T>(u);
    }
    else if constexpr (sizeof(T) == 8)
    {
        uint64_t u = std::bit_cast<uint64_t>(v);
        u = byteSwap(u);
        return std::bit_cast<T>(u);
    }
    else
    {
        static_assert(
            sizeof(T) == 4 || sizeof(T) == 8,
            "Unsupported floating-point size");
    }
}

template<typename T>
inline constexpr std::enable_if_t<is_byteswapable_v<T>, T> asLittleEndian(T v)
{
    if constexpr (sizeof(T) == 1 || HOST_IS_LITTLE_ENDIAN)
    {
        return v;
    }
    else
    {
        return byteSwap(v);
    }
}

template<typename T>
inline void write(uint8_t* ptr, const T& value)
{
    static_assert(
        HOST_IS_LITTLE_ENDIAN || is_byteswapable_v<T>,
        "write<T>: T must be a byte-swapable type: integral, floating-point, "
        "enum, or a user-defined type specializing util::is_byteswapable<T>");

    if constexpr (HOST_IS_LITTLE_ENDIAN || sizeof(T) == 1)
    {
        std::memcpy(ptr, &value, sizeof(T));
    }
    else
    {
        T tmp = byteSwap(value);
        std::memcpy(ptr, &tmp, sizeof(T));
    }
}
template<typename T>
inline void writeAndIncrement(uint8_t*& ptr, const T& value)
{
    write(ptr, value);

    ptr += sizeof(T);
}

template<typename T>
inline void writeMany(uint8_t* ptr, const std::vector<T>& value)
{
    static_assert(
        HOST_IS_LITTLE_ENDIAN || is_byteswapable_v<T>,
        "write<T>: T must be a byte-swapable type: integral, floating-point, "
        "enum, or a user-defined type specializing util::is_byteswapable<T>");

    if constexpr (HOST_IS_LITTLE_ENDIAN)
    {
        std::memcpy(ptr, value.data(), value.size() * sizeof(T));
    }
    else
    {
        uint8_t* tmp_ptr = ptr;
        for (const T& val : value)
        {
            T tmp = byteSwap(val);
            std::memcpy(tmp_ptr, &tmp, sizeof(T));
            tmp_ptr += sizeof(T);
        }
    }
}
template<typename T>
inline void writeManyAndIncrement(uint8_t*& ptr, const std::vector<T>& value)
{
    writeMany(ptr, value);

    ptr += value.size() * sizeof(T);
}

template<typename StorageT, typename T>
inline void writeAs(uint8_t* ptr, const T& value)
{
    static_assert(
        HOST_IS_LITTLE_ENDIAN || is_byteswapable_v<StorageT>,
        "writeAs<StorageT>: StorageT must be a byteswapable type: integral, "
        "floating-point, enum, or a user-defined type specializing "
        "util::is_byteswapable<StorageT>");

    if constexpr (HOST_IS_LITTLE_ENDIAN && std::is_same_v<T, StorageT>)
    {
        std::memcpy(ptr, &value, sizeof(StorageT));
    }
    else
    {
        StorageT tmp = asLittleEndian(static_cast<StorageT>(value));
        std::memcpy(ptr, &tmp, sizeof(StorageT));
    }
}
template<typename StorageT, typename T>
inline void writeAsAndIncrement(uint8_t*& ptr, const T& value)
{
    writeAs<StorageT>(ptr, value);

    ptr += sizeof(StorageT);
}

template<typename StorageT, typename T>
inline void writeManyAs(uint8_t* ptr, const std::vector<T>& value)
{
    static_assert(
        HOST_IS_LITTLE_ENDIAN || is_byteswapable_v<StorageT>,
        "writeAs<StorageT>: StorageT must be a byteswapable type: integral, "
        "floating-point, enum, or a user-defined type specializing "
        "util::is_byteswapable<StorageT>");

    if constexpr (HOST_IS_LITTLE_ENDIAN && std::is_same_v<T, StorageT>)
    {
        std::memcpy(ptr, value.data(), value.size() * sizeof(T));
    }
    else
    {
        uint8_t* tmp_ptr = ptr;
        for (const T& val : value)
        {
            StorageT tmp = asLittleEndian(static_cast<StorageT>(val));
            std::memcpy(tmp_ptr, &tmp, sizeof(StorageT));
            tmp_ptr += sizeof(StorageT);
        }
    }
}
template<typename StorageT, typename T>
inline void writeManyAsAndIncrement(uint8_t*& ptr, const std::vector<T>& value)
{
    writeManyAs<StorageT>(ptr, value);

    ptr += value.size() * sizeof(StorageT);
}

template<typename T>
inline void read(const uint8_t* ptr, T& out)
{
    static_assert(
        HOST_IS_LITTLE_ENDIAN || is_byteswapable_v<T>,
        "read<T>: T must be a byteswapable type: integral, floating-point, "
        "enum, or a user-defined type specializing util::is_byteswapable<T>");

    if constexpr (HOST_IS_LITTLE_ENDIAN || sizeof(T) == 1)
    {
        std::memcpy(&out, ptr, sizeof(T));
    }
    else
    {
        T tmp;
        std::memcpy(&tmp, ptr, sizeof(T));
        out = byteSwap(tmp);
    }
}
template<typename T>
inline void readAndIncrement(const uint8_t*& ptr, T& out)
{
    read(ptr, out);

    ptr += sizeof(T);
}

template<typename T>
inline void readMany(const uint8_t* ptr, std::vector<T>& out, size_t n)
{
    static_assert(
        HOST_IS_LITTLE_ENDIAN || is_byteswapable_v<T>,
        "read<T>: T must be a byteswapable type: integral, floating-point, "
        "enum, or a user-defined type specializing util::is_byteswapable<T>");

    if constexpr (HOST_IS_LITTLE_ENDIAN)
    {
        out.resize(n);
        std::memcpy(out.data(), ptr, n * sizeof(T));
    }
    else
    {
        out.resize(n);
        const uint8_t* tmp_ptr = ptr;
        for (size_t i = 0; i < n; ++i)
        {
            T tmp;
            std::memcpy(&tmp, tmp_ptr, sizeof(T));
            out[i] = byteSwap(tmp);
            tmp_ptr += sizeof(T);
        }
    }
}
template<typename T>
inline void
    readManyAndIncrement(const uint8_t*& ptr, std::vector<T>& out, size_t n)
{
    readMany(ptr, out, n);

    ptr += n * sizeof(T);
}

template<typename StorageT, typename T>
inline void readAs(const uint8_t* ptr, T& out)
{
    static_assert(
        HOST_IS_LITTLE_ENDIAN || util::is_byteswapable_v<StorageT>,
        "readAs<StorageT>: T must be a byteswapable type: integral, "
        "floating-point, enum, or a user-defined type specializing "
        "util::is_byteswapable<StorageT>");


    if constexpr (HOST_IS_LITTLE_ENDIAN && std::is_same_v<T, StorageT>)
    {
        std::memcpy(&out, ptr, sizeof(StorageT));
    }
    else
    {
        StorageT tmp{};
        std::memcpy(&tmp, ptr, sizeof(StorageT));
        out = static_cast<T>(asLittleEndian(tmp));
    }
}
template<typename StorageT, typename T>
inline void readAsAndIncrement(const uint8_t*& ptr, T& out)
{
    readAs<StorageT>(ptr, out);

    ptr += sizeof(StorageT);
}

template<typename StorageT, typename T>
inline void readManyAs(const uint8_t* ptr, std::vector<T>& out, size_t n)
{
    static_assert(
        HOST_IS_LITTLE_ENDIAN || util::is_byteswapable_v<StorageT>,
        "readAs<StorageT>: T must be a byteswapable type: integral, "
        "floating-point, enum, or a user-defined type specializing "
        "util::is_byteswapable<StorageT>");


    if constexpr (HOST_IS_LITTLE_ENDIAN && std::is_same_v<T, StorageT>)
    {
        out.resize(n);
        std::memcpy(out.data(), ptr, n * sizeof(StorageT));
    }
    else
    {
        out.resize(n);
        const uint8_t* tmp_ptr = ptr;
        for (size_t i = 0; i < n; ++i)
        {
            StorageT tmp{};
            std::memcpy(&tmp, tmp_ptr, sizeof(StorageT));
            out[i] = static_cast<T>(asLittleEndian(tmp));
            tmp_ptr += sizeof(StorageT);
        }
    }
}
template<typename StorageT, typename T>
inline void
    readManyAsAndIncrement(const uint8_t*& ptr, std::vector<T>& out, size_t n)
{
    readManyAs<StorageT>(ptr, out, n);

    ptr += n * sizeof(StorageT);
}

}  // namespace util
