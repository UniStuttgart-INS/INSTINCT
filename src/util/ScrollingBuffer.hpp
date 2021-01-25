/// @file ScrollingBuffer.hpp
/// @brief
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
    explicit ScrollingBuffer(size_t maxSize = 2000)
        : maxSize(maxSize)
    {
        data_.reserve(maxSize);
    }

    void AddValue(T value)
    {
        if (infiniteBuffer)
        {
            data_.push_back(value);
            maxSize = data_.size();
        }
        else if (data_.size() < maxSize)
        {
            data_.push_back(value);
        }
        else
        {
            data_.at(dataEnd) = value;
            if (dataStart == dataEnd)
            {
                dataStart = (dataStart + 1) % maxSize;
            }
            dataEnd = (dataEnd + 1) % maxSize;
        }
    }

    void clear()
    {
        data_.clear();
        dataStart = 0;
        dataEnd = 0;
    }

    [[nodiscard]] bool empty() const
    {
        return data_.empty();
    }

    [[nodiscard]] size_t size() const
    {
        return data_.size() - (dataStart - dataEnd);
    }

    void resize(size_t targetSize)
    {
        if (targetSize == 0)
        {
            infiniteBuffer = true;
            // 6, 7, 3, 4, 5,
            // 3, 4, 5, 6, 7,
            if (dataStart != 0)
            {
                std::vector<T> to_vector;
                std::copy(std::next(data_.begin() + static_cast<int64_t>(dataStart - 1)), data_.end(),
                          std::back_inserter(to_vector));

                std::copy(data_.begin(), std::next(data_.begin() + static_cast<int64_t>(dataEnd)),
                          std::next(data_.begin() + static_cast<int64_t>(to_vector.size() - 1)));
                std::copy(to_vector.begin(), to_vector.end(), data_.begin());

                data_.resize(data_.size() - (dataStart - dataEnd));
                maxSize = data_.size();

                dataStart = 0;
                dataEnd = 0;
            }
        }
        else
        {
            infiniteBuffer = false;

            if (maxSize > targetSize) // We make the buffer smaller
            {
                if (dataStart == 0)
                {
                    // 1, 2, 3, _, _,
                    // 1, 2, 3, _,
                    if (data_.size() < targetSize)
                    {
                        // data_.reserve(targetSize);
                        maxSize = targetSize;
                    }
                    // 1, 2, 3, _, _,
                    // 1, 2, 3,
                    else if (data_.size() < maxSize)
                    {
                        data_.resize(std::max(data_.size(), targetSize));
                        maxSize = data_.size();
                        resize(targetSize);
                    }
                    // 1, 2, 3,
                    // 2, 3,
                    else
                    {
                        auto diff = static_cast<int64_t>(maxSize - targetSize);
                        std::copy(std::next(data_.begin(), diff), data_.end(), data_.begin());
                        data_.resize(targetSize);
                        maxSize = targetSize;
                    }
                }
                else // (dataStart != 0)
                {
                    // 5, 6, _, _, 2, 3, 4,
                    // 5, 6, _, _, 2, 3, 4,

                    // 6, 7, 3, 4, 5,
                    // 6, 7, 4, 5,
                    auto diff = std::min(maxSize - targetSize, maxSize - dataEnd);

                    std::copy(std::next(data_.begin(), static_cast<int64_t>(dataEnd + diff)), data_.end(),
                              std::next(data_.begin(), static_cast<int64_t>(dataEnd)));
                    maxSize -= diff;
                    data_.resize(maxSize);
                    dataStart %= maxSize;
                    dataEnd %= maxSize;

                    // 6, 7,
                    // 7,
                    if (maxSize > targetSize)
                    {
                        resize(targetSize);
                    }
                }
            }
            else if (maxSize < targetSize) // We make the buffer bigger
            {
                // 1, 2, 3, _, _,
                // 1, 2, 3, _, _, _, _,
                if (dataStart == 0)
                {
                    data_.reserve(targetSize);
                    maxSize = targetSize;
                }
                else // (dataStart != 0)
                {
                    // 6, 7, 3, 4, 5,
                    // 6, 7, _, 3, 4, 5,
                    data_.resize(targetSize);

                    std::copy_backward(std::next(data_.begin(), static_cast<int64_t>(dataStart)),
                                       std::next(data_.begin(), static_cast<int64_t>(maxSize)),
                                       std::next(data_.begin(), static_cast<int64_t>(targetSize)));

                    auto diff = targetSize - maxSize;
                    dataStart += diff;

                    maxSize = targetSize;
                }
            }
        }
    }

    [[nodiscard]] T back() const
    {
        if (data_.size() < maxSize || dataEnd == 0)
        {
            return data_.back();
        }

        return data_.at(dataEnd - 1);
    }

    [[nodiscard]] T front() const
    {
        if (data_.size() < maxSize)
        {
            return data_.front();
        }

        return data_.at(dataStart);
    }

    [[nodiscard]] T max() const
    {
        T currentMax = front();
        for (size_t i = 0; i < data_.size(); i++)
        {
            if (i >= dataStart || i < dataEnd)
            {
                currentMax = std::max(currentMax, data_.at(i));
            }
        }
        return currentMax;
    }

    [[nodiscard]] T min() const
    {
        T currentMin = front();
        for (size_t i = 0; i < data_.size(); i++)
        {
            if (i >= dataStart || i < dataEnd)
            {
                currentMin = std::min(currentMin, data_.at(i));
            }
        }
        return currentMin;
    }

    [[nodiscard]] int offset() const
    {
        return static_cast<int>(dataStart);
    }

    [[nodiscard]] const T* data()
    {
        return data_.data();
    }

    template<class U>
    friend std::ostream& operator<<(std::ostream& os, const NAV::ScrollingBuffer<U>& buffer); // NOLINT(readability-redundant-declaration)

  private:
    bool infiniteBuffer = false;
    size_t maxSize;
    size_t dataStart = 0;
    size_t dataEnd = 0;
    std::vector<T> data_;
};

template<class U>
std::ostream& operator<<(std::ostream& os, const NAV::ScrollingBuffer<U>& buffer)
{
    for (size_t i = 0; i < buffer.maxSize; i++)
    {
        os << ((i < buffer.data_.size() && (i >= buffer.dataStart || i < buffer.dataEnd)) ? std::to_string(buffer.data_.at(i)) : "_")
           << ", ";
    }
    return os << '\n';
}

} // namespace NAV