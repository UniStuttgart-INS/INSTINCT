/// @file ScrollingBuffer.hpp
/// @brief
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-09

#pragma once

#include <vector>

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
        if (data_.size() > 0)
        {
            data_.clear();
            dataStart = 0;
            dataEnd = 0;
        }
    }

    bool empty()
    {
        return data_.empty();
    }

    size_t size()
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
                    // 6, 7, 3, 4, 5,
                    // 6, 7, 4, 5,
                    auto diff = std::min(maxSize - targetSize, maxSize - dataStart);

                    std::copy(std::next(data_.begin(), static_cast<int64_t>(dataStart + diff)), data_.end(),
                              std::next(data_.begin(), static_cast<int64_t>(dataStart)));
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

    T back()
    {
        if (data_.size() < maxSize || dataStart == 0)
        {
            return data_.back();
        }

        return data_.at(dataStart - 1);
    }

    T front()
    {
        if (data_.size() < maxSize)
        {
            return data_.front();
        }

        return data_.at(dataStart);
    }

    [[nodiscard]] int offset() const
    {
        return static_cast<int>(dataStart);
    }

    [[nodiscard]] const T* data()
    {
        return data_.data();
    }

  private:
    bool infiniteBuffer = false;
    size_t maxSize;
    size_t dataStart = 0;
    size_t dataEnd = 0;
    std::vector<T> data_;
};

} // namespace NAV