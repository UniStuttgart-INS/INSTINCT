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
        : m_maxSize(maxSize)
    {
        m_data.reserve(maxSize);

        resize(maxSize); // In case 0 was provided to make the buffer infinite
    }

    /// @brief Constructs a new container with the contents of the initializer list init.
    /// @param[in] init initializer list to initialize the elements of the container with
    ScrollingBuffer(std::initializer_list<T> init)
        : m_maxSize(init.size())
    {
        m_data.reserve(m_maxSize);
        resize(m_maxSize); // In case 0 was provided to make the buffer infinite

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

        // 8 9 3 4 5 6 7
        // at(0) = 3, at(4) = 7, at(5) = 8
        if (m_dataStart + pos >= size())
        {
            return m_data.at(pos - (size() - m_dataStart));
        }

        return m_data.at(m_dataStart + pos);
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
        if (m_data.size() < m_maxSize)
        {
            return m_data.front();
        }

        return m_data.at(m_dataStart);
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
        if (m_data.size() < m_maxSize || m_dataEnd == 0)
        {
            return m_data.back();
        }

        return m_data.at(m_dataEnd - 1);
    }

    // ###########################################################################################################
    //                                                 Capacity
    // ###########################################################################################################

    /// @brief Checks if the container has no elements
    [[nodiscard]] bool empty() const
    {
        return m_data.empty();
    }

    /// @brief Returns the number of elements in the container
    [[nodiscard]] size_t size() const
    {
        return m_data.size() - (m_dataStart - m_dataEnd);
    }

    // ###########################################################################################################
    //                                                 Modifiers
    // ###########################################################################################################

    /// @brief Erases all elements from the container. After this call, size() returns zero.
    void clear()
    {
        m_data.clear();
        m_dataStart = 0;
        m_dataEnd = 0;
    }

    /// @brief Appends the given element value to the end of the container.
    /// @param[in] value the value of the element to append
    void push_back(const T& value)
    {
        if (m_infiniteBuffer) // The buffer should grow when adding new values
        {
            m_data.push_back(value);
            m_maxSize = m_data.size();
        }
        else if (m_data.size() < m_maxSize) // The real buffer is smaller than the allowed buffer size
        {
            m_data.push_back(value);
        }
        else // The real buffer as large as or bigger than the allowed buffer size, so we have to scroll the buffer
        {
            m_data.at(m_dataEnd) = value;
            if (m_dataStart == m_dataEnd)
            {
                m_dataStart = (m_dataStart + 1) % m_maxSize;
            }
            m_dataEnd = (m_dataEnd + 1) % m_maxSize;
        }
    }

    /// @brief Resizes the buffer to the specified size
    /// @param[in] targetSize The new buffer size (0 for infinite buffer)
    void resize(size_t targetSize)
    {
        if (targetSize == 0) // Buffer should grow indefinitely when adding new values
        {
            m_infiniteBuffer = true;
            // 6, 7, 3, 4, 5,
            // 3, 4, 5, 6, 7,
            if (m_dataStart != 0) // Buffer is scrolled and needs to be sorted
            {
                std::vector<T> to_vector;
                std::copy(std::next(m_data.begin(), static_cast<int64_t>(m_dataStart)), m_data.end(),
                          std::back_inserter(to_vector));

                std::copy(m_data.begin(), std::next(m_data.begin(), static_cast<int64_t>(m_dataEnd)),
                          std::back_inserter(to_vector));
                m_data.swap(to_vector);

                m_maxSize = m_data.size();

                m_dataStart = 0;
                m_dataEnd = 0;
            }
        }
        else // Buffer should have scrolling behaviour when adding new values
        {
            m_infiniteBuffer = false;

            if (m_maxSize > targetSize) // We make the buffer smaller
            {
                if (m_dataStart == 0) // Buffer is not scrolled, so shrinking removes the values from the front of the buffer
                {
                    // 1, 2, 3, _, _,
                    // 1, 2, 3, _,
                    if (m_data.size() < targetSize)
                    {
                        m_maxSize = targetSize;
                    }
                    // 1, 2, 3, _, _,
                    // 1, 2, 3,
                    else if (m_data.size() < m_maxSize)
                    {
                        m_data.resize(std::max(m_data.size(), targetSize));
                        m_maxSize = m_data.size();
                        resize(targetSize);
                    }
                    // 1, 2, 3,
                    // 2, 3,
                    else
                    {
                        auto diff = static_cast<int64_t>(m_maxSize - targetSize);
                        std::copy(std::next(m_data.begin(), diff), m_data.end(), m_data.begin());
                        m_data.resize(targetSize);
                        m_maxSize = targetSize;
                    }
                }
                else // (m_dataStart != 0) Buffer is scrolled, so the correct values have to be erased from the buffer when shrinking
                {
                    // 5, 6, _, _, 2, 3, 4,
                    // 5, 6, _, 2, 3, 4,

                    // 6, 7, 3, 4, 5,
                    // 6, 7, 4, 5,
                    auto diff = std::min(m_maxSize - targetSize, m_maxSize - m_dataEnd);

                    std::copy(std::next(m_data.begin(), static_cast<int64_t>(m_dataEnd + diff)), m_data.end(),
                              std::next(m_data.begin(), static_cast<int64_t>(m_dataEnd)));
                    m_maxSize -= diff;
                    m_data.resize(m_maxSize);
                    m_dataStart %= m_maxSize;
                    m_dataEnd %= m_maxSize;

                    // 6, 7,
                    // 7,
                    if (m_maxSize > targetSize)
                    {
                        resize(targetSize);
                    }
                }
            }
            else if (m_maxSize < targetSize) // We make the buffer bigger
            {
                // 1, 2, 3, _, _,
                // 1, 2, 3, _, _, _, _,
                if (m_dataStart == 0) // Buffer not scrolled, so we can simply reserve more space
                {
                    m_data.reserve(targetSize);
                    m_maxSize = targetSize;
                }
                // 6, 7, 3, 4, 5,
                // 6, 7, _, 3, 4, 5,
                else // (m_dataStart != 0) // Buffer scrolled, so we need to copy the values to the correct positions
                {
                    m_data.resize(targetSize);

                    std::copy_backward(std::next(m_data.begin(), static_cast<int64_t>(m_dataStart)),
                                       std::next(m_data.begin(), static_cast<int64_t>(m_maxSize)),
                                       std::next(m_data.begin(), static_cast<int64_t>(targetSize)));

                    auto diff = targetSize - m_maxSize;
                    m_dataStart += diff;

                    m_maxSize = targetSize;
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
        for (size_t i = 0; i < m_data.size(); i++)
        {
            if (i >= m_dataStart || i < m_dataEnd)
            {
                currentMax = std::max(currentMax, m_data.at(i));
            }
        }
        return currentMax;
    }

    /// @brief Returns the smallest value in the buffer
    [[nodiscard]] T min() const
    {
        T currentMin = front();
        for (size_t i = 0; i < m_data.size(); i++)
        {
            if (i >= m_dataStart || i < m_dataEnd)
            {
                currentMin = std::min(currentMin, m_data.at(i));
            }
        }
        return currentMin;
    }

    /// @brief Returns the data index of the first element in the buffer
    [[nodiscard]] int offset() const
    {
        return static_cast<int>(m_dataStart);
    }

    /// @brief Returns a pointer to the raw data array (not in scrolled order)
    [[nodiscard]] const T* data()
    {
        return m_data.data();
    }

    /// @brief Prints the buffer to the output stream
    /// @param[in, out] os The output stream to print to
    /// @param[in] buffer The buffer to print
    /// @return The output stream given as parameter
    friend std::ostream& operator<<(std::ostream& os, const ScrollingBuffer<T>& buffer)
    {
        for (size_t i = 0; i < buffer.m_maxSize; i++)
        {
            os << ((i < buffer.m_data.size() && (i >= buffer.m_dataStart || i < buffer.m_dataEnd)) ? std::to_string(buffer.m_data.at(i)) : "_")
               << ", ";
        }
        return os << '\n';
    }

  private:
    /// A Flag whether the buffer is currently growing every time when values are inserted or if it is overwriting itself from the start when full
    bool m_infiniteBuffer{ false };
    /// The maximum amount of objects to store in the buffer before overwriting itself when full
    /// When m_infiniteBuffer == true, then this corresponds to m_data.size()
    size_t m_maxSize;
    /// The index of the first element in the scrolling buffer (0 if the buffer is empty)
    size_t m_dataStart{ 0 };
    /// The index one after the last element (0 if the buffer is empty)
    size_t m_dataEnd{ 0 };
    /// The data storage object
    std::vector<T> m_data;
};

} // namespace NAV