// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file KeyedMap.hpp
/// @brief Similar to KeyedMatrix, but memory is allocated in a map and therefore never reallocated
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-10-08

#pragma once

#include <algorithm>
#include <map>
#include <type_traits>
#include <vector>
#include <span>
#include <fmt/format.h>

#include "util/Assert.h"
#include "util/Container/Unordered_map.hpp"

namespace NAV
{

/// @brief Similar to KeyedMatrix, but memory is allocated in a map and therefore never reallocated
/// @tparam KeyType Type to use as keys
/// @tparam Scalar Type to store in the map
/// @tparam unordered Wether an unordered_map or a std::map should be used
template<typename KeyType, typename Scalar, bool unordered = true>
class KeyedMap
{
  public:
    // ###########################################################################################################
    //                                               Constructors
    // ###########################################################################################################
    //
    // ###########################################################################################################
    //                                               Iterators
    // ###########################################################################################################

    /// @brief Returns an iterator to the first element.
    ///
    /// If the buffer is empty, the returned iterator will be equal to end().
    decltype(auto) begin() { return _lookup.begin(); }
    /// @brief Returns an iterator to the first element.
    ///
    /// If the buffer is empty, the returned iterator will be equal to end().
    [[nodiscard]] decltype(auto) begin() const noexcept { return _lookup.begin(); }
    /// @brief Returns an iterator to the first element.
    ///
    /// If the buffer is empty, the returned iterator will be equal to end().
    [[nodiscard]] decltype(auto) cbegin() const noexcept { return _lookup.cbegin(); }

    /// @brief Returns an iterator to the element following the last element of.
    ///
    /// This element acts as a placeholder; attempting to access it results in undefined behavior.
    decltype(auto) end() { return _lookup.end(); }
    /// @brief Returns an iterator to the element following the last element of.
    ///
    /// This element acts as a placeholder; attempting to access it results in undefined behavior.
    [[nodiscard]] decltype(auto) end() const noexcept { return _lookup.end(); }
    /// @brief Returns an iterator to the element following the last element of.
    ///
    /// This element acts as a placeholder; attempting to access it results in undefined behavior.
    [[nodiscard]] decltype(auto) cend() const noexcept { return _lookup.cend(); }

    // ###########################################################################################################
    //                                                 Capacity
    // ###########################################################################################################

    /// @brief Checks if the container has no elements
    [[nodiscard]] bool empty() const
    {
        return size() == 0;
    }

    /// @brief Returns the number of elements in the container
    [[nodiscard]] size_t size() const
    {
        return _lookup.size();
    }

    // ###########################################################################################################
    //                                                 Modifiers
    // ###########################################################################################################

    /// @brief Erases all elements from the container. After this call, size() returns zero.
    void clear()
    {
        _lookup.clear();
        _data.clear();
    }

    /// @brief Adds a single element for the key to the data storage
    /// @param[in] key Key to add
    void addKey(const KeyType& key) { addKeys(std::vector<KeyType>{ key }); }

    /// @brief Adds a single element for the key to the data storage
    /// @param[in] key Key to add
    /// @param[in] value Value for the key
    void addKey(const KeyType& key, const Scalar& value) { addKeys(std::vector<std::pair<KeyType, Scalar>>{ { key, value } }); }

    /// @brief Adds a continuous vector for the keys to the data storage
    /// @param[in] keys Keys to add
    void addKeys(std::span<const KeyType> keys)
    {
        std::vector<std::pair<KeyType, Scalar>> keyValues;
        keyValues.reserve(keys.size());
        for (const KeyType& key : keys)
        {
            keyValues.emplace_back(key, Scalar{});
        }
        addKeys(keyValues);
    }

    /// @brief Adds a continuous vector for the keys to the data storage
    /// @param[in] keys Keys to add
    /// @param[in] values Values for the keys
    void addKeys(std::span<const KeyType> keys, std::span<const Scalar> values)
    {
        INS_ASSERT_USER_ERROR(keys.size() == values.size(), "Keys and values vector need to be same size");

        std::vector<std::pair<KeyType, Scalar>> keyValues;
        keyValues.reserve(keys.size());
        for (size_t i = 0; i < keys.size(); i++)
        {
            keyValues.emplace_back(keys[i], values[i]);
        }
        addKeys(keyValues);
    }

    /// @brief Adds a continuous vector for the keys to the data storage
    /// @param[in] keyValues Keys and values to add
    void addKeys(std::span<const std::pair<KeyType, Scalar>> keyValues)
    {
        INS_ASSERT_USER_ERROR(!keyValues.empty(), "The vector cannot be empty");
        INS_ASSERT_USER_ERROR(!_data.contains(keyValues.front().first), "Duplicate keys are not allowed");
        for ([[maybe_unused]] const auto& keyValue : keyValues)
        {
            INS_ASSERT_USER_ERROR(!_lookup.contains(keyValue.first), "Duplicate subkeys are not allowed");
        }

        _data.emplace(keyValues.front().first, std::vector<Scalar>(keyValues.size()));
        for (size_t i = 0; i < keyValues.size(); i++)
        {
            _lookup[keyValues[i].first] = &_data.at(keyValues.front().first).at(i);
            _data.at(keyValues.front().first).at(i) = keyValues[i].second;
        }
    }

    // ###########################################################################################################
    //                                                 Lookup
    // ###########################################################################################################

    /// @brief Returns a reference to the mapped value of the element with specified key.
    ///        If no such element exists, an exception of type std::out_of_range is thrown.
    /// @param[in] key the key of the element to find
    /// @return A reference to the mapped value of the requested element.
    Scalar& at(const KeyType& key)
    {
        return *_lookup.at(key);
    }

    /// @brief Returns a reference to the mapped value of the element with specified key.
    ///        If no such element exists, an exception of type std::out_of_range is thrown.
    /// @param[in] key the key of the element to find
    /// @return A reference to the mapped value of the requested element.
    [[nodiscard]] const Scalar& at(const KeyType& key) const
    {
        return *_lookup.at(key);
    }

    /// @brief Returns a reference to the mapped value of the element with specified keys.
    ///        If no such element exists, an exception of type std::out_of_range is thrown.
    /// @param[in] keys the keys of the element to find
    /// @return A reference to the mapped value of the requested element.
    std::vector<Scalar>& at(std::span<const KeyType> keys)
    {
        INS_ASSERT_USER_ERROR(!keys.empty(), "The vector cannot be empty");
        INS_ASSERT_USER_ERROR(_data.at(keys.front()).size() == keys.size(), "The keys vector stored and requested have different length");

        return _data.at(keys.front());
    }

    /// @brief Returns a reference to the mapped value of the element with specified keys.
    ///        If no such element exists, an exception of type std::out_of_range is thrown.
    /// @param[in] keys the key of the element to find
    /// @return A reference to the mapped value of the requested element.
    [[nodiscard]] const std::vector<Scalar>& at(std::span<const KeyType> keys) const
    {
        INS_ASSERT_USER_ERROR(!keys.empty(), "The vector cannot be empty");
        INS_ASSERT_USER_ERROR(_data.at(keys.front()).size() == keys.size(), "The keys vector stored and requested have different length");

        return _data.at(keys.front());
    }

    /// @brief Checks if there is an element with key equivalent to `key` in the container.
    /// @param[in] key key value of the element to search for
    /// @return true if there is such an element, otherwise false.
    [[nodiscard]] bool contains(const KeyType& key) const
    {
        return _lookup.contains(key);
    }

    /// @brief Checks if there are elements with keys equivalent to `keys` in the container.
    /// @param[in] keys keys the elements to search for
    /// @return true if there is such an element, otherwise false.
    [[nodiscard]] bool contains(std::span<const KeyType> keys) const
    {
        INS_ASSERT_USER_ERROR(!keys.empty(), "The vector cannot be empty");

        return std::all_of(keys.begin(), keys.end(), [&](const auto& key) { return _lookup.contains(key); });
    }

    /// @brief Returns the size of parameters represented by the key
    /// @param[in] key key value of the element to search for
    [[nodiscard]] size_t size_of(const KeyType& key) const
    {
        return _data.contains(key) ? _data.at(key).size() : (_lookup.contains(key) ? 1 : 0);
    }

    // ###########################################################################################################
    //                                                  Others
    // ###########################################################################################################

    /// @brief Collect all keys
    /// @return The keys stored
    [[nodiscard]] std::vector<KeyType> keys() const
    {
        std::vector<KeyType> keys;
        keys.reserve(_lookup.size());
        for (const auto& keyVal : _lookup)
        {
            keys.push_back(keyVal.first);
        }
        return keys;
    }

  private:
    /// Lookup for individual keys
    unordered_map<KeyType, Scalar*> _lookup;

    /// Storage container
    std::conditional_t<unordered,
                       unordered_map<KeyType, std::vector<Scalar>>,
                       std::map<KeyType, std::vector<Scalar>>>
        _data;
};

} // namespace NAV

#ifndef DOXYGEN_IGNORE

/// @brief Formatter for KeyedMap
template<typename KeyType, typename Scalar, bool unordered>
struct fmt::formatter<NAV::KeyedMap<KeyType, Scalar, unordered>> : fmt::formatter<std::string>
{
    /// @brief Defines how to format KeyedMap structs
    /// @param[in] map Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    auto format(const NAV::KeyedMap<KeyType, Scalar, unordered>& map, format_context& ctx) const
    {
        std::string result;
        auto length = static_cast<size_t>(map.size());

        if (length > 0)
        {
            std::vector<std::string> rowKeysStr;
            std::vector<size_t> rowKeysLength;
            rowKeysStr.reserve(length);
            rowKeysLength.reserve(length);
            size_t rowKeysColSpace = 0;
            for (const auto& keyVal : map)
            {
                rowKeysStr.push_back(fmt::format("{}", keyVal.first));
                auto rowKeyLength = rowKeysStr.back().length();
                rowKeysColSpace = std::max(rowKeysColSpace, rowKeyLength);
                rowKeysLength.push_back(rowKeyLength);
            }

            size_t colLength = 9UL;

            result.reserve(length * (rowKeysColSpace + 2 + colLength));

            size_t r = 0;
            for (const auto& keyVal : map)
            {
                if (rowKeysColSpace > rowKeysLength.at(r))
                {
                    result += std::string(rowKeysColSpace - rowKeysLength.at(r), ' '); // Spaces in front of row name (if too short)
                }
                result += rowKeysStr.at(r);

                std::string tmp = fmt::format("  {:> {}.{}g}", *keyVal.second, colLength, colLength - 2);
                if (tmp.length() > colLength)
                {
                    tmp = fmt::format("  {:> {}.{}g}", *keyVal.second, colLength, colLength - 6);
                }
                result += tmp;

                if (r != length - 1) { result += '\n'; }
                r++;
            }
        }

        return fmt::formatter<std::string>::format(result, ctx);
    }
};

#endif

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
template<typename KeyType, typename Scalar, bool unordered>
std::ostream& operator<<(std::ostream& os, const NAV::KeyedMap<KeyType, Scalar, unordered>& obj)
{
    return os << fmt::format("{}", obj);
}