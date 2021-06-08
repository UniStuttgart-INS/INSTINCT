/// @file ScrollingBuffer.hpp
/// @brief A buffer which is overwriting itself from the start when full
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-09

#pragma once

#include <vector>
#include <iostream>

namespace NAV
{
template<class T>
class ScrollingBuffer
{
  public:
    /// @brief Reserves space for the buffer but does not fill the buffer with values
    /// @param[in] maxSize The maximum size of the scrolling buffer
    explicit ScrollingBuffer(size_t maxSize = 2000)
        : m_maxSize(maxSize)
    {
        m_data.reserve(maxSize);

        resize(maxSize); // In case 0 was provided to make the buffer infinite
    }

    /// @brief Adds a value to the end of the buffer
    /// @param[in] value The value to add to the buffer
    void AddValue(const T& value)
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

    /// @brief Empties the buffer
    void clear()
    {
        m_data.clear();
        m_dataStart = 0;
        m_dataEnd = 0;
    }

    /// @brief Checks if the buffer is currently empty
    [[nodiscard]] bool empty() const
    {
        return m_data.empty();
    }

    /// @brief Returns the amount of elements currently stored in the buffer
    [[nodiscard]] size_t size() const
    {
        return m_data.size() - (m_dataStart - m_dataEnd);
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
                std::copy(std::next(m_data.begin() + static_cast<int64_t>(m_dataStart - 1)), m_data.end(),
                          std::back_inserter(to_vector));

                std::copy(m_data.begin(), std::next(m_data.begin() + static_cast<int64_t>(m_dataEnd)),
                          std::next(m_data.begin() + static_cast<int64_t>(to_vector.size() - 1)));
                std::copy(to_vector.begin(), to_vector.end(), m_data.begin());

                m_data.resize(m_data.size() - (m_dataStart - m_dataEnd));
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

    /// @brief Returns the data at the first element of the buffer
    [[nodiscard]] T front() const
    {
        if (m_data.size() < m_maxSize)
        {
            return m_data.front();
        }

        return m_data.at(m_dataStart);
    }

    /// @brief Returns the data at the last element of the buffer
    [[nodiscard]] T back() const
    {
        if (m_data.size() < m_maxSize || m_dataEnd == 0)
        {
            return m_data.back();
        }

        return m_data.at(m_dataEnd - 1);
    }

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