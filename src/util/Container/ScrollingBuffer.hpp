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
/// @tparam _Padding The padding are empty values at the start of the buffer to prevent overriding the start value in multithreaded applications
template<class T, size_t _Padding = 0>
class ScrollingBuffer
{
  public:
    // ###########################################################################################################
    //                                               Constructors
    // ###########################################################################################################

    /// @brief Reserves space for the buffer but does not fill the buffer with values
    /// @param[in] maxSize The maximum size of the scrolling buffer
    explicit ScrollingBuffer(size_t maxSize = 2000)
        : _infiniteBuffer(maxSize == 0), _maxSize(maxSize + _Padding), _dataStart(_Padding), _dataEnd(_infiniteBuffer ? 0 : _Padding)
    {
        _data.reserve(maxSize + _Padding);
        _data.resize(_Padding);
    }

    /// @brief Constructs a new container with the contents of the initializer list init.
    /// @param[in] init initializer list to initialize the elements of the container with
    ScrollingBuffer(std::initializer_list<T> init)
        : _infiniteBuffer(init.size() == 0), _maxSize(init.size())
    {
        _data.reserve(init.size());

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
        return size() == 0;
    }

    /// @brief Returns the number of elements in the container
    [[nodiscard]] size_t size() const
    {
        if (_dataStart == _Padding && _dataEnd == _Padding) // Buffer empty or full and not scrolled
        {
            return _data.size() - _Padding;
        }
        if (_dataStart < _dataEnd) // not scrolled buffer
        {
            return _dataEnd - _dataStart;
        }

        // scrolled buffer
        return _maxSize - (_dataStart - _dataEnd);
    }

    /// @brief Returns the number of elements that can be held in currently allocated storage
    [[nodiscard]] size_t capacity() const
    {
        return _infiniteBuffer ? 0 : (_maxSize - _Padding);
    }

    // ###########################################################################################################
    //                                                 Modifiers
    // ###########################################################################################################

    /// @brief Erases all elements from the container. After this call, size() returns zero.
    void clear()
    {
        _data.clear();
        _dataStart = _Padding;
        _dataEnd = _infiniteBuffer ? 0 : _Padding;

        for (size_t i = 0; i < _Padding; i++)
        {
            _data.push_back(0);
        }
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

            if (isScrolled()) // Buffer is scrolled and needs to be sorted in order of insertion
            {
                //       se                e     s                 e     s              s               e
                // 5, 6, 2, 3, 4  // 5, 6, _, _, 2, 3, 4  // 5, 6, X, X, 2, 3, 4  // X, 6, 7, 8, 9, 10, X
                // 2, 3, 4, 5, 6  // 2, 3, 4, 5, 6, _, _  // X, X, 2, 3, 4, 5, 6  // X, X, 6, 7, 8, 9, 10
                std::vector<T> to_vector;

                if (_dataEnd > _dataStart)
                {
                    std::copy(std::next(_data.begin(), static_cast<int64_t>(_dataEnd)), _data.end(),
                              std::back_inserter(to_vector));
                    _maxSize = _dataEnd;
                }

                std::copy(std::next(_data.begin(), std::max(static_cast<int>(_dataStart - _Padding), 0)),
                          std::next(_data.begin(), static_cast<int64_t>(std::max(_dataEnd, _maxSize))),
                          std::back_inserter(to_vector));

                if (int64_t elementsFront = std::min(static_cast<int64_t>(_dataEnd), static_cast<int64_t>(_dataStart - _Padding));
                    elementsFront > 0)
                {
                    std::copy(_data.begin(), std::next(_data.begin(), elementsFront),
                              std::back_inserter(to_vector));
                }
                _data.swap(to_vector);

                _maxSize = _data.size();

                _dataStart = _Padding;
                _dataEnd = 0;
            }
        }
        else // Buffer should have scrolling behaviour when adding new values
        {
            _infiniteBuffer = false;

            if (_maxSize - _Padding > targetSize) // We make the buffer smaller
            {
                if (!isScrolled()) // Buffer is not scrolled, so shrinking removes the values from the front of the buffer
                {
                    size_t elementsToDelete = _maxSize - _Padding - targetSize;

                    if (size_t emptyAtTheBack = _maxSize - _dataEnd;
                        size_t emptyAtTheBackToDelete = std::min(emptyAtTheBack, elementsToDelete))
                    {
                        // 1, 2, 3, _, _,  // X, X, 1, 2, 3, _, _,
                        // 1, 2, 3, _,     // X, X, 1, 2, 3, _,
                        _maxSize -= emptyAtTheBackToDelete;
                        _dataEnd %= _maxSize; // NOLINT(clang-analyzer-core.DivideZero) // this lint is wrong, even when wrapped by `if(_maxSize != 0)` appearing
                        elementsToDelete -= emptyAtTheBackToDelete;
                        // 1, 2, 3,        // X, X, 1, 2, 3,
                    }

                    // 1, 2, 3,
                    // 2, 3,
                    if (elementsToDelete)
                    {
                        _data.erase(std::next(_data.begin(), static_cast<int64_t>(_dataStart)),
                                    std::next(_data.begin(), static_cast<int64_t>(_dataStart + elementsToDelete)));
                        _maxSize -= elementsToDelete;
                    }
                }
                else // Buffer is scrolled, so the correct values have to be erased from the buffer when shrinking
                {
                    // 5, 6, _, _, 2, 3, 4, // 5, 6, _, _, X, X, 2, 3, 4,
                    // 6,                   // 6, X, X

                    // 6, 7, 3, 4, 5,       // 5, 6, X, X, 2, 3, 4
                    // 6, 7, 4, 5,          // 5, 6, X, X, 3, 4

                    // X, 6, 7, 8, 9, 10, X
                    // X, 9, 10, X

                    size_t elementsToDelete = _maxSize - _Padding - targetSize;

                    if (size_t emptyInBetween = static_cast<size_t>(std::max(static_cast<int>(_dataStart - _Padding - _dataEnd), 0));
                        size_t emptyInBetweenToDelete = std::min(emptyInBetween, elementsToDelete))
                    {
                        // 5, 6, _, _, 2, 3, 4, // 5, 6, _, _, X, X, 2, 3, 4,
                        _data.erase(std::next(_data.begin(), static_cast<int64_t>(_dataEnd)),
                                    std::next(_data.begin(), static_cast<int64_t>(_dataEnd + emptyInBetweenToDelete)));
                        // 5, 6, _, 2, 3, 4,    // 5, 6, _, X, X, 2, 3, 4,
                        _dataStart -= emptyInBetweenToDelete;
                        _maxSize -= emptyInBetweenToDelete;
                        elementsToDelete -= emptyInBetweenToDelete;
                    }

                    //       s                   e
                    //  X  , 6, 7 ,   8 , 9, 10, X
                    // X(8), 9, 10, X(7)
                    if (size_t paddingAtTheEnd = static_cast<size_t>(std::max(static_cast<int>(_Padding - _dataStart), 0));
                        size_t paddingAtTheEndToDelete = std::min(paddingAtTheEnd, elementsToDelete))
                    {
                        _data.erase(std::next(_data.begin(), static_cast<int64_t>(_dataEnd)),
                                    std::next(_data.begin(), static_cast<int64_t>(_dataEnd + paddingAtTheEndToDelete)));
                        _maxSize -= paddingAtTheEndToDelete;
                        _dataStart += paddingAtTheEndToDelete;
                        _dataEnd %= _maxSize;
                        elementsToDelete -= paddingAtTheEndToDelete;
                    }
                    // e     s
                    // X, X, 6, 7 , 8 , 9, 10
                    // X, X, 7 , 8 , 9, 10

                    if (size_t elementsAtTheBack = _maxSize - _dataStart - (_dataEnd > _dataStart ? _maxSize - _dataEnd : 0);
                        size_t elementsAtTheBackToDelete = std::min(elementsAtTheBack, elementsToDelete))
                    {
                        // 5, 6, 2, 3, 4,       // 5, 6, X, X, 2, 3, 4,
                        _data.erase(std::next(_data.begin(), static_cast<int64_t>(_dataStart - _Padding)),
                                    std::next(_data.begin(), static_cast<int64_t>(_dataStart - _Padding + elementsAtTheBackToDelete)));
                        // 5, 6, 4,             // 5, 6, X, X, 4,
                        // 5, 6,                // 5, 6, X, X,
                        _maxSize -= elementsAtTheBackToDelete;
                        _dataStart %= _maxSize;
                        if (_dataEnd > _maxSize)
                        {
                            _dataEnd -= elementsToDelete;
                        }
                        _dataEnd %= _maxSize;
                        elementsToDelete -= elementsAtTheBackToDelete;
                    }
                    if (elementsToDelete)
                    {
                        // 5, 6,                // 5, 6, X, X,
                        _data.erase(std::next(_data.begin(), static_cast<int64_t>(_dataStart)),
                                    std::next(_data.begin(), static_cast<int64_t>(std::min(_dataStart + elementsToDelete, _maxSize - _Padding))));
                        // 6,                   // 6, X, X
                        _maxSize -= elementsToDelete;
                        if (_dataEnd >= elementsToDelete)
                        {
                            _dataEnd -= elementsToDelete;
                        }
                    }
                }
            }
            else if (_maxSize - _Padding < targetSize) // We make the buffer bigger
            {
                // 1, 2, 3, _, _,        // X, X, 0, 1, 2, 3, _, _
                // 1, 2, 3, _, _, _, _,  // X, X, 0, 1, 2, 3, _, _, _, _
                if (!isScrolled()) // Buffer not scrolled, so we can simply reserve more space
                {
                    _maxSize = targetSize + _Padding;
                    _data.reserve(_maxSize);
                }
                //      se                      e     s                 s               e
                // 6, 7, 3, 4, 5,      // 5, 6, X, X, 2, 3, 4     // X, 6, 7, 8, 9, 10, X
                // 6, 7, _, 3, 4, 5,   // 5, 6, _, X, X, 2, 3, 4  // X, 6, 7, 8, 9, 10, _, _, X
                else // (_dataStart != 0) // Buffer scrolled, so we need to copy the values to the correct positions
                {
                    _data.resize(targetSize + _Padding);

                    std::copy_backward(std::next(_data.begin(), static_cast<int64_t>(_dataEnd)),
                                       std::next(_data.begin(), static_cast<int64_t>(_maxSize)),
                                       std::next(_data.begin(), static_cast<int64_t>(targetSize + _Padding)));

                    auto diff = targetSize + _Padding - _maxSize;
                    if (_dataStart >= _dataEnd)
                    {
                        _dataStart += diff;
                    }

                    _maxSize = targetSize + _Padding;
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
    friend std::ostream& operator<<(std::ostream& os, const ScrollingBuffer<T, _Padding>& buffer)
    {
        // Scrolled
        //       e        s
        // 5, 6, _, _, X, 2, 3, 4

        //      se
        // 5, 6, 2, 3, 4

        // Not scrolled
        //       s           e
        // X, X, 0, 1, 2, 3, _, _

        //    s               e
        // X, 6, 7, 8, 9, 10, _, _, X

        // s  e
        // 6, X, X,

        // s  e
        // 6, _, X, X,

        //       se
        // X, X, _, _, _,

        for (int i = 0; static_cast<size_t>(i) < buffer._maxSize; i++) // X, 6, 7, 8, 9, 10, _, _, X
        {
            if ((i >= static_cast<int>(buffer._dataStart - _Padding) && static_cast<size_t>(i) < buffer._dataStart)
                || (static_cast<size_t>(i) >= buffer._dataStart + buffer._maxSize - _Padding))
            {
                os << "X"; // padding
            }
            else if (bool scrolled = buffer.isScrolled();
                     (scrolled && static_cast<size_t>(i) >= buffer._dataEnd && (static_cast<int>(buffer._dataStart - _Padding) < 0 || i < static_cast<int>(buffer._dataStart - _Padding)))
                     || (!scrolled && static_cast<size_t>(i) >= buffer._dataEnd))
            {
                os << "_"; // empty
            }
            else
            {
                os << std::to_string(buffer._data.at(static_cast<size_t>(i)));
            }
            if (static_cast<size_t>(i) != buffer._maxSize - 1)
            {
                os << ", ";
            }
        }
        return os;
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

    /// @brief Checks if the buffer is scrolled
    [[nodiscard]] bool isScrolled() const
    {
        //       e        s
        // 5, 6, _, _, X, 2, 3, 4

        //                                se
        if (_dataEnd == 0 && !empty()) // 1, 2, 3, 4
        {
            return true;
        }

        return _dataEnd < _dataStart
               || (_dataStart != _Padding);

        //      se        //       se
        // 5, 6, 2, 3, 4  // X, X, _, _, _

        //    s           e
        // X, 0, 1, 2, 3, _, _, X // Scrolled
        //       s           e
        // X, X, 0, 1, 2, 3, _, _ // Not scrolled
    }
};

} // namespace NAV