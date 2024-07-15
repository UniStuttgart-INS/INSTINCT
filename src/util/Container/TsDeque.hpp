// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TsDeque.hpp
/// @brief Thread-safe deque
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-09-12

#pragma once

#include <deque>
#include <mutex>

namespace NAV
{

/// @brief Thread-safe deque
/// @tparam T The type of the elements.
/// @tparam Alloc Allocator that is used to acquire/release memory and to construct/destroy the elements in that memory.
template<class T, class Alloc = std::allocator<T>>
class TsDeque
{
  public:
    // #######################################################################################################
    //                                           Member functions
    // #######################################################################################################

    /// @brief Default Constructor. Constructs an empty container with a default-constructed allocator.
    TsDeque() = default;

    /// Destructor
    ~TsDeque() = default;

    /// @brief Constructs an empty container with the given allocator alloc.
    /// @param alloc Allocator to use for all memory allocations of this container
    explicit TsDeque(const Alloc& alloc)
        : _queue(alloc) {}

    /// @brief Constructs the container with count copies of elements with value value.
    /// @param count The size of the container
    /// @param value The value to initialize elements of the container with
    /// @param alloc Allocator to use for all memory allocations of this container
    TsDeque(typename std::deque<T, Alloc>::size_type count, const T& value, const Alloc& alloc = Alloc())
        : _queue(count, value, alloc) {}

    /// @brief Constructs the container with count default-inserted instances of T. No copies are made.
    /// @param count The size of the container
    /// @param alloc Allocator to use for all memory allocations of this container
    explicit TsDeque(typename std::deque<T, Alloc>::size_type count, const Alloc& alloc = Alloc())
        : _queue(count, alloc) {}

    /// @brief Constructs the container with the contents of the range [first, last).
    /// @param first First element of the range to copy the elements from
    /// @param last Last element the range to copy the elements from
    /// @param alloc Allocator to use for all memory allocations of this container
    template<class InputIt>
    TsDeque(InputIt first, InputIt last, const Alloc& alloc = Alloc())
        : _queue(first, last, alloc)
    {}

    /// @brief Copy constructor. Constructs the container with the copy of the contents of other.
    /// @param other Another container to be used as source to initialize the elements of the container with
    TsDeque(const TsDeque& other)
    {
        std::scoped_lock lk(other._mutex);
        _queue = std::deque<T, Alloc>(other._queue); // NOLINT(cppcoreguidelines-prefer-member-initializer)
    }
    /// @brief Constructs the container with the copy of the contents of other, using alloc as the allocator.
    /// @param other Another container to be used as source to initialize the elements of the container with
    /// @param alloc Allocator to use for all memory allocations of this container
    TsDeque(const TsDeque& other, const Alloc& alloc)
    {
        std::scoped_lock lk(other._mutex);
        _queue = std::deque<T, Alloc>(other._queue, alloc); // NOLINT(cppcoreguidelines-prefer-member-initializer)
    }
    /// @brief Move constructor. Constructs the container with the contents of other using move semantics.
    ///        Allocator is obtained by move-construction from the allocator belonging to other.
    /// @param other Another container to be used as source to initialize the elements of the container with
    TsDeque(TsDeque&& other) noexcept
    {
        std::scoped_lock lk(other._mutex);
        _queue = std::deque<T, Alloc>(std::move(other._queue)); // NOLINT(cppcoreguidelines-prefer-member-initializer)
    }
    /// @brief Allocator-extended move constructor. Using alloc as the allocator for the new container, moving the contents from other;
    ///        if alloc != other.get_allocator(), this results in an element-wise move.
    /// @param other Another container to be used as source to initialize the elements of the container with
    /// @param alloc Allocator to use for all memory allocations of this container
    TsDeque(TsDeque&& other, const Alloc& alloc) noexcept
    {
        std::scoped_lock lk(other._mutex);
        _queue = std::deque<T, Alloc>(std::move(other._queue), alloc); // NOLINT(cppcoreguidelines-prefer-member-initializer)
    }
    /// @brief Constructs the container with the contents of the initializer list init.
    /// @param init Initializer list to initialize the elements of the container with
    /// @param alloc Allocator to use for all memory allocations of this container
    TsDeque(std::initializer_list<T> init, const Alloc& alloc = Alloc())
        : _queue(init, alloc) {}

    /// @brief Copy assignment operator
    TsDeque& operator=(const TsDeque& other)
    {
        if (this != &other)
        {
            std::scoped_lock lk(_mutex);
            std::scoped_lock lko(other._mutex);
            _queue = other._queue;
        }
        return *this;
    }
    /// @brief Move assignment operator
    TsDeque& operator=(TsDeque&& other) noexcept
    {
        if (this != &other)
        {
            std::scoped_lock lk(_mutex);
            std::scoped_lock lko(other._mutex);
            _queue = std::move(other._queue);
        }
        return *this;
    }
    /// @brief Replaces the contents with those identified by initializer list ilist.
    /// @param ilist Initializer list to use as data source
    TsDeque& operator=(std::initializer_list<T> ilist)
    {
        _queue = ilist;
    }

    /// @brief Replaces the contents with count copies of value value
    ///
    /// All iterators, pointers and references to the elements of the container are invalidated. The past-the-end iterator is also invalidated.
    /// @param count The new size of the container
    /// @param value The value to initialize elements of the container with
    void assign(typename std::deque<T, Alloc>::size_type count, const T& value)
    {
        std::scoped_lock lk(_mutex);
        _queue.assign(count, value);
    }

    /// @brief Replaces the contents with copies of those in the range [first, last). The behavior is undefined if either argument is an iterator into *this.
    ///
    /// All iterators, pointers and references to the elements of the container are invalidated. The past-the-end iterator is also invalidated.
    /// @param[in] first The first element of the range to copy the elements from
    /// @param[in] last The last element of the range to copy the elements from
    template<class InputIt>
    void assign(InputIt first, InputIt last)
    {
        std::scoped_lock lk(_mutex);
        _queue.assign(first, last);
    }
    /// @brief Replaces the contents with the elements from the initializer list ilist.
    ///
    /// All iterators, pointers and references to the elements of the container are invalidated. The past-the-end iterator is also invalidated.
    /// @param ilist Initializer list to copy the values from
    void assign(std::initializer_list<T> ilist)
    {
        std::scoped_lock lk(_mutex);
        _queue.assign(ilist);
    }

    /// @brief Returns the allocator associated with the container.
    /// @return The associated allocator.
    typename std::deque<T, Alloc>::allocator_type get_allocator() const noexcept
    {
        return _queue.get_allocator();
    }

    // #######################################################################################################
    //                                            Element access
    // #######################################################################################################

    /// @brief Returns a reference to the element at specified location pos, with bounds checking.
    ///        If pos is not within the range of the container, an exception of type std::out_of_range is thrown.
    /// @param[in] pos Position of the element to return
    /// @return Reference to the requested element.
    auto& at(typename std::deque<T, Alloc>::size_type pos) { return _queue.at(pos); }
    /// @brief Returns a reference to the element at specified location pos, with bounds checking.
    ///        If pos is not within the range of the container, an exception of type std::out_of_range is thrown.
    /// @param[in] pos Position of the element to return
    /// @return Reference to the requested element.
    const auto& at(typename std::deque<T, Alloc>::size_type pos) const { return _queue.at(pos); }

    /// @brief Returns a reference to the element at specified location pos. No bounds checking is performed.
    /// @param[in] pos Position of the element to return
    /// @return Reference to the requested element.
    auto& operator[](typename std::deque<T, Alloc>::size_type pos) { return _queue[pos]; }
    /// @brief Returns a reference to the element at specified location pos. No bounds checking is performed.
    /// @param[in] pos Position of the element to return
    /// @return Reference to the requested element.
    const auto& operator[](typename std::deque<T, Alloc>::size_type pos) const { return _queue[pos]; }

    /// @brief Returns a reference to the first element in the container.
    /// @return Reference to the first element
    auto& front() { return _queue.front(); }
    /// @brief Returns a reference to the first element in the container.
    /// @return Reference to the first element
    const auto& front() const { return _queue.front(); }

    /// @brief Returns a reference to the last element in the container.
    /// @return Reference to the last element
    auto& back() { return _queue.back(); }
    /// @brief Returns a reference to the last element in the container.
    /// @return Reference to the last element
    const auto& back() const { return _queue.back(); }

    // #######################################################################################################
    //                                               Iterators
    // #######################################################################################################

    /// @brief Returns an iterator to the first element of the deque.
    ///        If the deque is empty, the returned iterator will be equal to end().
    /// @return Iterator to the first element.
    typename std::deque<T>::iterator begin() noexcept { return _queue.begin(); }
    /// @brief Returns an iterator to the first element of the deque.
    ///        If the deque is empty, the returned iterator will be equal to end().
    /// @return Iterator to the first element.
    typename std::deque<T>::const_iterator begin() const noexcept { return _queue.begin(); }
    /// @brief Returns an iterator to the first element of the deque.
    ///        If the deque is empty, the returned iterator will be equal to end().
    /// @return Iterator to the first element.
    typename std::deque<T>::const_iterator cbegin() const noexcept { return _queue.cbegin(); }

    /// @brief Returns an iterator to the element following the last element of the deque.
    ///        This element acts as a placeholder; attempting to access it results in undefined behavior.
    /// @return Iterator to the element following the last element.
    typename std::deque<T>::iterator end() noexcept { return _queue.end(); }
    /// @brief Returns an iterator to the element following the last element of the deque.
    ///        This element acts as a placeholder; attempting to access it results in undefined behavior.
    /// @return Iterator to the element following the last element.
    typename std::deque<T>::const_iterator end() const noexcept { return _queue.end(); }
    /// @brief Returns an iterator to the element following the last element of the deque.
    ///        This element acts as a placeholder; attempting to access it results in undefined behavior.
    /// @return Iterator to the element following the last element.
    typename std::deque<T>::const_iterator cend() const noexcept { return _queue.cend(); }

    /// @brief Returns a reverse iterator to the first element of the reversed deque. It corresponds to the last element of the non-reversed deque.
    ///        If the deque is empty, the returned iterator is equal to rend().
    /// @return Reverse iterator to the first element.
    typename std::deque<T>::reverse_iterator rbegin() noexcept { return _queue.rbegin(); }
    /// @brief Returns a reverse iterator to the first element of the reversed deque. It corresponds to the last element of the non-reversed deque.
    ///        If the deque is empty, the returned iterator is equal to rend().
    /// @return Reverse iterator to the first element.
    typename std::deque<T>::const_reverse_iterator rbegin() const noexcept { return _queue.rbegin(); }
    /// @brief Returns a reverse iterator to the first element of the reversed deque. It corresponds to the last element of the non-reversed deque.
    ///        If the deque is empty, the returned iterator is equal to rend().
    /// @return Reverse iterator to the first element.
    typename std::deque<T>::const_reverse_iterator crbegin() const noexcept { return _queue.crbegin(); }

    /// @brief Returns a reverse iterator to the element following the last element of the reversed deque.
    ///        It corresponds to the element preceding the first element of the non-reversed deque.
    ///        This element acts as a placeholder, attempting to access it results in undefined behavior.
    /// @return Reverse iterator to the element following the last element.
    typename std::deque<T>::reverse_iterator rend() noexcept { return _queue.rend(); }
    /// @brief Returns a reverse iterator to the element following the last element of the reversed deque.
    ///        It corresponds to the element preceding the first element of the non-reversed deque.
    ///        This element acts as a placeholder, attempting to access it results in undefined behavior.
    /// @return Reverse iterator to the element following the last element.
    typename std::deque<T>::const_reverse_iterator rend() const noexcept { return _queue.rend(); }
    /// @brief Returns a reverse iterator to the element following the last element of the reversed deque.
    ///        It corresponds to the element preceding the first element of the non-reversed deque.
    ///        This element acts as a placeholder, attempting to access it results in undefined behavior.
    /// @return Reverse iterator to the element following the last element.
    typename std::deque<T>::const_reverse_iterator crend() const noexcept { return _queue.crend(); }

    // #######################################################################################################
    //                                               Capacity
    // #######################################################################################################

    /// @brief Checks if the container has no elements, i.e. whether 'begin() == end()'.
    /// @return true if the container is empty, false otherwise
    [[nodiscard]] bool empty() const noexcept
    {
        std::scoped_lock lk(_mutex);
        return _queue.empty();
    }

    /// @brief Returns the number of elements in the container, i.e. std::distance(begin(), end()).
    /// @return The number of elements in the container.
    typename std::deque<T, Alloc>::size_type size() const noexcept
    {
        std::scoped_lock lk(_mutex);
        return _queue.size();
    }

    /// @brief Returns the maximum number of elements the container is able to hold due to system or library implementation limitations, i.e. std::distance(begin(), end()) for the largest container.
    /// @return Maximum number of elements.
    typename std::deque<T, Alloc>::size_type max_size() const noexcept
    {
        std::scoped_lock lk(_mutex);
        return _queue.max_size();
    }

    /// @brief Requests the removal of unused capacity.
    void shrink_to_fit()
    {
        std::scoped_lock lk(_mutex);
        _queue.shrink_to_fit();
    }

    // #######################################################################################################
    //                                               Modifiers
    // #######################################################################################################

    /// Erases all elements from the container. After this call, size() returns zero.
    /// Invalidates any references, pointers, or iterators referring to contained elements. Any past-the-end iterators are also invalidated.
    void clear() noexcept
    {
        std::scoped_lock lk(_mutex);
        _queue.clear();
    }

    /// @brief Insert value before pos in the container.
    /// @param[in] pos Iterator before which the content will be inserted. pos may be the end() iterator
    /// @param[in] value Element value to insert
    /// @return Iterator pointing to the inserted value
    typename std::deque<T>::iterator insert(typename std::deque<T>::const_iterator pos, const T& value)
    {
        std::scoped_lock lk(_mutex);
        return _queue.insert(pos, value);
    }

    /// @brief Insert value before pos in the container.
    /// @param[in] pos Iterator before which the content will be inserted. pos may be the end() iterator
    /// @param[in] value Element value to insert
    /// @return Iterator pointing to the inserted value
    typename std::deque<T>::iterator insert(typename std::deque<T>::const_iterator pos, T&& value)
    {
        std::scoped_lock lk(_mutex);
        return _queue.insert(pos, value);
    }

    /// @brief inserts count copies of the value before pos
    /// @param[in] pos Iterator before which the content will be inserted. pos may be the end() iterator
    /// @param[in] count Number of elements to insert
    /// @param[in] value Element value to insert
    /// @return Linear in count plus linear in the lesser of the distances between pos and either of the ends of the container.
    typename std::deque<T>::iterator insert(typename std::deque<T>::const_iterator pos, typename std::deque<T, Alloc>::size_type count, const T& value)
    {
        std::scoped_lock lk(_mutex);
        return _queue.insert(pos, count, value);
    }

    /// @brief inserts elements from range [first, last) before pos.
    /// @tparam InputIt Input iterator type
    /// @param[in] pos Iterator before which the content will be inserted. pos may be the end() iterator
    /// @param[in] first The first element of the range of elements to insert, can't be iterators into container for which insert is called
    /// @param[in] last The last element of the range of elements to insert, can't be iterators into container for which insert is called
    /// @return Iterator pointing to the first element inserted, or pos if first==last.
    template<class InputIt>
    typename std::deque<T>::iterator insert(typename std::deque<T>::const_iterator pos, InputIt first, InputIt last)
    {
        std::scoped_lock lk(_mutex);
        return _queue.insert(pos, first, last);
    }

    /// @brief Inserts elements from initializer list ilist before pos.
    /// @param[in] pos Iterator before which the content will be inserted. pos may be the end() iterator
    /// @param[in] ilist Initializer list to insert the values from
    /// @return Iterator pointing to the first element inserted, or pos if ilist is empty.
    typename std::deque<T>::iterator insert(typename std::deque<T>::const_iterator pos, std::initializer_list<T> ilist)
    {
        std::scoped_lock lk(_mutex);
        return _queue.insert(pos, ilist);
    }

    /// @brief Inserts a new element into the container directly before pos.
    /// @param[in] pos Iterator before which the new element will be constructed
    /// @param[in] args Arguments to forward to the constructor of the element
    /// @return Iterator pointing to the emplaced element.
    template<class... Args>
    typename std::deque<T>::iterator emplace(typename std::deque<T>::const_iterator pos, Args&&... args)
    {
        std::scoped_lock lk(_mutex);
        return _queue.emplace(pos, std::forward<Args>(args)...);
    }

    /// @brief Removes the element at pos.
    /// @param[in] pos Iterator to the element to remove
    /// @return Iterator following the last removed element.
    typename std::deque<T>::iterator erase(typename std::deque<T>::const_iterator pos)
    {
        std::scoped_lock lk(_mutex);
        return _queue.erase(pos);
    }
    /// @brief Removes the elements in the range [first, last).
    ///        All iterators and references are invalidated, unless the erased elements are at the end or the beginning of the container,
    ///        in which case only the iterators and references to the erased elements are invalidated.
    /// @param[in] first First element of the range of elements to remove
    /// @param[in] last Last element of the range of elements to remove
    /// @return Iterator following the last removed element.
    typename std::deque<T>::iterator erase(typename std::deque<T>::const_iterator first, typename std::deque<T>::const_iterator last)
    {
        std::scoped_lock lk(_mutex);
        return _queue.erase(first, last);
    }

    /// @brief Appends the given element value to the end of the container. The new element is initialized as a copy of value.
    /// @param[in] value The value of the element to append
    void push_back(const T& value)
    {
        std::scoped_lock lk(_mutex);
        _queue.push_back(value);
    }
    /// @brief Appends the given element value to the end of the container. Value is moved into the new element.
    /// @param[in] value The value of the element to append
    void push_back(T&& value)
    {
        std::scoped_lock lk(_mutex);
        _queue.push_back(value);
    }

    /// @brief Appends a new element to the end of the container.
    /// @param[in] args Arguments to forward to the constructor of the element
    /// @return A reference to the inserted element.
    template<class... Args>
    auto& emplace_back(Args&&... args)
    {
        std::scoped_lock lk(_mutex);
        return _queue.emplace_back(std::forward<Args>(args)...);
    }

    /// @brief Removes the last element of the container.
    void pop_back()
    {
        std::scoped_lock lk(_mutex);
        _queue.pop_back();
    }

    /// @brief Prepends the given element value to the beginning of the container. The new element is initialized as a copy of value.
    ///        All iterators, including the past-the-end iterator, are invalidated. No references are invalidated.
    /// @param[in] value The value of the element to prepend
    void push_front(const T& value)
    {
        std::scoped_lock lk(_mutex);
        _queue.push_front(value);
    }
    /// @brief Prepends the given element value to the beginning of the container. Value is moved into the new element.
    ///        All iterators, including the past-the-end iterator, are invalidated. No references are invalidated.
    /// @param[in] value The value of the element to prepend
    void push_front(T&& value)
    {
        std::scoped_lock lk(_mutex);
        _queue.push_front(value);
    }

    /// @brief Inserts a new element to the beginning of the container.
    /// @param[in] args Arguments to forward to the constructor of the element
    /// @return A reference to the inserted element.
    template<class... Args>
    auto& emplace_front(Args&&... args)
    {
        std::scoped_lock lk(_mutex);
        return _queue.emplace_front(std::forward<Args>(args)...);
    }

    /// @brief Removes the first element of the container. If there are no elements in the container, the behavior is undefined.
    void pop_front()
    {
        std::scoped_lock lk(_mutex);
        _queue.pop_front();
    }

    /// @brief Resizes the container to contain count elements.
    /// @param[in] count New size of the container
    void resize(typename std::deque<T, Alloc>::size_type count)
    {
        std::scoped_lock lk(_mutex);
        _queue.resize(count);
    }
    /// @brief Resizes the container to contain count elements.
    /// @param[in] count New size of the container
    /// @param[in] value The value to initialize the new elements with
    void resize(typename std::deque<T, Alloc>::size_type count, const T& value)
    {
        std::scoped_lock lk(_mutex);
        _queue.resize(count, value);
    }

    /// @brief Exchanges the contents of the container with those of other. Does not invoke any move, copy, or swap operations on individual elements.
    ///        All iterators and references remain valid. The past-the-end iterator is invalidated.
    /// @param[in, out] other Container to exchange the contents with
    void swap(std::deque<T>& other) noexcept
    {
        std::scoped_lock lk(_mutex);
        _queue.swap(other);
    }

    /// @brief Returns a copy of the first element in the container and removes it from the container.
    /// @return Copy of the first element
    auto extract_front()
    {
        T front;
        {
            std::scoped_lock lk(_mutex);
            front = _queue.front();
        }
        pop_front();
        return front;
    }

  private:
    /// Queue with received data
    std::deque<T, Alloc> _queue;

    /// Mutex to interact with the queue object
    mutable std::mutex _mutex;
};

} // namespace NAV