/// @file ScrollingBuffer.hpp
/// @brief A buffer which is overwriting itself from the start when full
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-09

#pragma once

#include <vector>
#include <string>
#include <iostream>

namespace NAV
{
/// @brief A buffer which is overwriting itself from the start when full
/// @tparam T Type of data stored in the buffer
template<class T>
class ScrollingBuffer
{
  public:
    // ###########################################################################################################
    //                                               Constructors
    // ###########################################################################################################

    /// @brief Reserves space for the buffer but does not fill the buffer with values
    /// @param[in] maxSize The maximum size of the scrolling buffer
    explicit ScrollingBuffer(size_t maxSize = 2000)
        : _maxSize(maxSize)
    {
        _data.reserve(maxSize);

        resize(maxSize); // In case 0 was provided to make the buffer infinite
    }

    /// @brief Constructs a new container with the contents of the initializer list init.
    /// @param[in] init initializer list to initialize the elements of the container with
    ScrollingBuffer(std::initializer_list<T> init)
        : _maxSize(init.size())
    {
        _data.reserve(_maxSize);
        resize(_maxSize); // In case 0 was provided to make the buffer infinite

        for (auto& val : init)
        {
            push_back(val);
        }
    }

    // ###########################################################################################################
    //                                              Element access
    // ###########################################################################################################

    /// @brief Returns a reference to the element at specified location pos, with bounds checking.
    ///        If pos is not within the range of the container, an exception of type std::out_of_range is thrown.
    /// @param[in] pos position of the element to return
    /// @return Reference to the requested element.
    T& at(size_t pos)
    {
        // Cast the const away and reuse the implementation below
        return const_cast<T&>(static_cast<const ScrollingBuffer&>(*this).at(pos)); // NOLINT(cppcoreguidelines-pro-type-const-cast)
    }

    /// @brief Returns a reference to the element at specified location pos, with bounds checking.
    ///        If pos is not within the range of the container, an exception of type std::out_of_range is thrown.
    /// @param[in] pos position of the element to return
    /// @return Reference to the requested element.
    [[nodiscard]] const T& at(size_t pos) const
    {
        if (!(pos < size()))
        {
            throw std::out_of_range("ScrollingBuffer::at: pos (which is "
                                    + std::to_string(pos) + ") >= this->size() (which is " + std::to_string(size()) + ")");
        }

        //     start/end
        //     |
        // 8 9 3 4 5 6 7
        // at(0) = 3, at(4) = 7, at(5) = 8
        if (_dataStart + pos >= _maxSize)
        {
            return _data.at(_dataStart + pos - _maxSize);
        }

        return _data.at(_dataStart + pos);
    }

    /// @brief Returns a reference to the first element in the container.
    ///        Calling front on an empty container is undefined.
    /// @return reference to the first element
    T& front()
    {
        // Cast the const away and reuse the implementation below (don't repeat yourself)
        return const_cast<T&>(static_cast<const ScrollingBuffer&>(*this).front()); // NOLINT(cppcoreguidelines-pro-type-const-cast)
    }

    /// @brief Returns a reference to the first element in the container.
    ///        Calling front on an empty container is undefined.
    /// @return Reference to the first element
    [[nodiscard]] const T& front() const
    {
        if (_data.size() < _maxSize)
        {
            return _data.front();
        }

        return _data.at(_dataStart);
    }

    /// @brief Returns a reference to the last element in the container.
    ///        Calling back on an empty container causes undefined behavior.
    /// @return Reference to the last element.
    T& back()
    {
        return const_cast<T&>(static_cast<const ScrollingBuffer&>(*this).back()); // NOLINT(cppcoreguidelines-pro-type-const-cast)
    }

    /// @brief Returns a reference to the last element in the container.
    ///        Calling back on an empty container causes undefined behavior.Reference to the last element.
    /// @return Reference to the last element.
    [[nodiscard]] const T& back() const
    {
        if (_data.size() < _maxSize || _dataEnd == 0)
        {
            return _data.back();
        }

        return _data.at(_dataEnd - 1);
    }

    // ###########################################################################################################
    //                                                 Capacity
    // ###########################################################################################################

    /// @brief Checks if the container has no elements
    [[nodiscard]] bool empty() const
    {
        return _data.empty();
    }

    /// @brief Returns the number of elements in the container
    [[nodiscard]] size_t size() const
    {
        if (_dataStart == 0 && _dataEnd == 0) // Buffer empty
        {
            return _data.size();
        }
        if (_dataStart < _dataEnd) // unscrolled buffer
        {
            return _dataEnd - _dataStart;
        }

        // scrolled buffer
        return _maxSize - (_dataStart - _dataEnd);
    }

    // ###########################################################################################################
    //                                                 Modifiers
    // ###########################################################################################################

    /// @brief Erases all elements from the container. After this call, size() returns zero.
    void clear()
    {
        _data.clear();
        _dataStart = 0;
        _dataEnd = 0;
    }

    /// @brief Appends the given element value to the end of the container.
    /// @param[in] value the value of the element to append
    void push_back(const T& value)
    {
        if (_infiniteBuffer) // The buffer should grow when adding new values
        {
            _data.push_back(value);
            _maxSize = _data.size();
        }
        else if (_data.size() < _maxSize) // The real buffer is smaller than the allowed buffer size
        {
            _data.push_back(value);
            _dataEnd = (_dataEnd + 1) % _maxSize;
        }
        else // The real buffer as large as or bigger than the allowed buffer size, so we have to scroll the buffer
        {
            _data.at(_dataEnd) = value;
            _dataStart = (_dataStart + 1) % _maxSize;
            _dataEnd = (_dataEnd + 1) % _maxSize;
        }
    }

    /// @brief Resizes the buffer to the specified size
    /// @param[in] targetSize The new buffer size (0 for infinite buffer)
    void resize(size_t targetSize)
    {
        if (targetSize == 0) // Buffer should grow indefinitely when adding new values
        {
            _infiniteBuffer = true;
            // 6, 7, 3, 4, 5,
            // 3, 4, 5, 6, 7,
            if (_dataStart != 0) // Buffer is scrolled and needs to be sorted in order of insertion
            {
                std::vector<T> to_vector;
                std::copy(std::next(_data.begin(), static_cast<int64_t>(_dataStart)), _data.end(),
                          std::back_inserter(to_vector));

                std::copy(_data.begin(), std::next(_data.begin(), static_cast<int64_t>(_dataEnd)),
                          std::back_inserter(to_vector));
                _data.swap(to_vector);

                _maxSize = _data.size();

                _dataStart = 0;
                _dataEnd = 0;
            }
        }
        else // Buffer should have scrolling behaviour when adding new values
        {
            _infiniteBuffer = false;

            if (_maxSize > targetSize) // We make the buffer smaller
            {
                if (_dataStart == 0) // Buffer is not scrolled, so shrinking removes the values from the front of the buffer
                {
                    // 1, 2, 3, _, _,
                    // 1, 2, 3, _,
                    if (_data.size() < targetSize)
                    {
                        _maxSize = targetSize;
                    }
                    // 1, 2, 3, _, _,
                    // 1, 2, 3,
                    else if (_data.size() < _maxSize)
                    {
                        _data.resize(std::max(_data.size(), targetSize));
                        _maxSize = _data.size();
                        resize(targetSize);
                    }
                    // 1, 2, 3,
                    // 2, 3,
                    else
                    {
                        auto diff = static_cast<int64_t>(_maxSize - targetSize);
                        std::copy(std::next(_data.begin(), diff), _data.end(), _data.begin());
                        _data.resize(targetSize);
                        _maxSize = targetSize;
                        _dataEnd = 0;
                    }
                }
                else // (_dataStart != 0) Buffer is scrolled, so the correct values have to be erased from the buffer when shrinking
                {
                    // 5, 6, _, _, 2, 3, 4,
                    // 5, 6, _, 2, 3, 4,

                    // 6, 7, 3, 4, 5,
                    // 6, 7, 4, 5,
                    auto diff = std::min(_maxSize - targetSize, _maxSize - _dataEnd);

                    std::copy(std::next(_data.begin(), static_cast<int64_t>(_dataEnd + diff)), _data.end(),
                              std::next(_data.begin(), static_cast<int64_t>(_dataEnd)));
                    _maxSize -= diff;
                    _data.resize(_maxSize);
                    _dataStart %= _maxSize;
                    _dataEnd %= _maxSize;

                    // 6, 7,
                    // 7,
                    if (_maxSize > targetSize)
                    {
                        resize(targetSize);
                    }
                }
            }
            else if (_maxSize < targetSize) // We make the buffer bigger
            {
                // 1, 2, 3, _, _,
                // 1, 2, 3, _, _, _, _,
                if (_dataStart == 0) // Buffer not scrolled, so we can simply reserve more space
                {
                    _data.reserve(targetSize);
                    _maxSize = targetSize;
                }
                // 6, 7, 3, 4, 5,
                // 6, 7, _, 3, 4, 5,
                else // (_dataStart != 0) // Buffer scrolled, so we need to copy the values to the correct positions
                {
                    _data.resize(targetSize);

                    std::copy_backward(std::next(_data.begin(), static_cast<int64_t>(_dataStart)),
                                       std::next(_data.begin(), static_cast<int64_t>(_maxSize)),
                                       std::next(_data.begin(), static_cast<int64_t>(targetSize)));

                    auto diff = targetSize - _maxSize;
                    _dataStart += diff;

                    _maxSize = targetSize;
                }
            }
        }
    }

    // ###########################################################################################################
    //                                                   Other
    // ###########################################################################################################

    /// @brief Returns the largest value in the buffer
    [[nodiscard]] T max() const
    {
        T currentMax = front();
        for (size_t i = 0; i < _data.size(); i++)
        {
            if (i >= _dataStart || i < _dataEnd)
            {
                currentMax = std::max(currentMax, _data.at(i));
            }
        }
        return currentMax;
    }

    /// @brief Returns the smallest value in the buffer
    [[nodiscard]] T min() const
    {
        T currentMin = front();
        for (size_t i = 0; i < _data.size(); i++)
        {
            if (i >= _dataStart || i < _dataEnd)
            {
                currentMin = std::min(currentMin, _data.at(i));
            }
        }
        return currentMin;
    }

    /// @brief Returns the data index of the first element in the buffer
    [[nodiscard]] int offset() const
    {
        return static_cast<int>(_dataStart);
    }

    /// @brief Returns a pointer to the raw data array (not in scrolled order)
    [[nodiscard]] const T* data()
    {
        return _data.data();
    }

    /// @brief Prints the buffer to the output stream
    /// @param[in, out] os The output stream to print to
    /// @param[in] buffer The buffer to print
    /// @return The output stream given as parameter
    friend std::ostream& operator<<(std::ostream& os, const ScrollingBuffer<T>& buffer)
    {
        for (size_t i = 0; i < buffer._maxSize; i++)
        {
            os << ((i < buffer._data.size() && (i >= buffer._dataStart || i < buffer._dataEnd)) ? std::to_string(buffer._data.at(i)) : "_")
               << ", ";
        }
        return os << '\n';
    }

  private:
    /// A Flag whether the buffer is currently growing every time when values are inserted or if it is overwriting itself from the start when full
    bool _infiniteBuffer{ false };
    /// The maximum amount of objects to store in the buffer before overwriting itself when full
    /// When _infiniteBuffer == true, then this corresponds to m_data.size()
    size_t _maxSize;
    /// The index of the first element in the scrolling buffer (0 if the buffer is empty)
    size_t _dataStart{ 0 };
    /// The index one after the last element (0 if the buffer is empty)
    size_t _dataEnd{ 0 };
    /// The data storage object
    std::vector<T> _data;
};

} // namespace NAV