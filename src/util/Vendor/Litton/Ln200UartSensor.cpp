// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Ln200UartSensor.hpp"

#include "util/Logger.hpp"

NAV::vendor::ln::Ln200UartSensor::Ln200UartSensor(std::string name)
    : _name(std::move(name)), _bitBuffer(uart::sensors::UartSensor::DefaultReadBufferSize), _flagWindow(8)
{}

std::unique_ptr<uart::protocol::Packet> NAV::vendor::ln::Ln200UartSensor::findPacket(uint8_t dataByte)
{
    // If we receive a full idle byte (0xFF), mark the idle period.
    if (dataByte == 0xFF)
    {
        _wasIdle = true;
    }

    // Define the lambda for byte-to-bits conversion.
    auto byteToBits = [](uint8_t byte) -> std::vector<bool> {
        std::vector<bool> bits(8);
        for (uint8_t i = 0; i < 8; ++i)
        {
            bits[7 - i] = (byte >> i) & 0x1;
        }
        return bits;
    };

    // Lambda for checking the flag sequence.
    auto isFlagSequence = [](const ScrollingBuffer<bool>& window) -> bool {
        // The flag is 01111110.
        constexpr std::bitset<8> flagPattern("01111110");

        // Compare the available bits in the window with the corresponding bits in the flag pattern.
        for (size_t i = 0; i < window.size(); ++i)
        {
            if (window.at(i) != flagPattern[i])
            {
                return false;
            }
        }
        return true;
    };

    auto bitBufferToByteVector = [](const ScrollingBuffer<bool>& bitBuffer) -> std::vector<uint8_t> {
        std::vector<uint8_t> byteVector((bitBuffer.size() + 7) / 8, 0); // Allocate enough bytes
        for (size_t i = 0; i < bitBuffer.size(); ++i)
        {
            if (bitBuffer.at(i))
            {
                byteVector[i / 8] |= (1 << (7 - (i % 8))); // Set the corresponding bit in the byte
            }
        }
        return byteVector;
    };

    // Convert the received byte to bits.
    auto bits = byteToBits(dataByte);
    for (auto bit : bits)
    {
        // Update the flag window (we use a sliding window of 8 bits).
        _flagWindow.push_back(bit);
        if (_flagWindow.size() > 8)
        {
            _flagWindow.pop_front();
        }

        // Check for flag sequence.
        if (_flagWindow.size() == 8 && isFlagSequence(_flagWindow))
        {
            if (_state == State::Receiving)
            {
                // End of frame detected.
                if (!_bitBuffer.empty())
                {
                    // Remove the flag bits from the _bitBuffer (assume the last 7 bits are the flag).
                    if (_bitBuffer.size() >= 7)
                    {
                        for (int i = 0; i < 7; ++i)
                        {
                            _bitBuffer.pop_back();
                        }
                    }
                    std::vector<uint8_t> byteVector = bitBufferToByteVector(_bitBuffer);
                    LOG_DATA("{}: Valid binary packet: Length={}", _name, _bitBuffer.size());
                    auto p = std::make_unique<uart::protocol::Packet>(byteVector, &_sensor);
                    _bitBuffer.clear();
                    _state = State::Idle;
                    _consecutiveOnes = 0;
                    _wasIdle = false; // Clear the idle flag.
                    return p;
                }
                _state = State::Idle;
                _wasIdle = false;
            }
            else // _state == Idle
            {
                // Only consider this flag as a start sequence if the bus was idle (0xFF) just before.
                if (_wasIdle)
                {
                    // Start of frame detected.
                    _state = State::Receiving;
                    _consecutiveOnes = 0;
                    _bitBuffer.clear(); // Clear any previous payload.
                    _wasIdle = false;   // Reset the idle flag.
                }
                // If _wasIdle is false, then the flag might be an ending flag from a previous message.
            }
            continue;
        }

        if (_state == State::Receiving)
        {
            // If the current bit is false and it follows five consecutive ones,
            // it's a stuffed zero which should be ignored.
            if (!bit && _consecutiveOnes == 5)
            {
                _consecutiveOnes = 0;
                continue;
            }

            // For a true bit, increment the counter; for a false bit, reset it.
            _consecutiveOnes = bit ? (_consecutiveOnes + 1) : 0;

            // Append the bit to the buffer.
            _bitBuffer.push_back(bit);
        }
    }
    return nullptr;
}

void NAV::vendor::ln::Ln200UartSensor::packetFinderFunction(const std::vector<uint8_t>& data, const uart::xplat::TimeStamp& timestamp, uart::sensors::UartSensor::ValidPacketFoundHandler dispatchPacket, void* dispatchPacketUserData, void* userData)
{
    auto* sensor = static_cast<Ln200UartSensor*>(userData);

    for (size_t i = 0; i < data.size(); ++i, sensor->_runningDataIndex++)
    {
        auto packetPointer = sensor->findPacket(data.at(i));

        if (packetPointer != nullptr)
        {
            uart::protocol::Packet packet = *packetPointer;
            dispatchPacket(dispatchPacketUserData, packet, sensor->_runningDataIndex, timestamp);
        }
    }
}

uart::protocol::Packet::Type NAV::vendor::ln::Ln200UartSensor::packetTypeFunction([[maybe_unused]] const uart::protocol::Packet& packet)
{
    return uart::protocol::Packet::Type::TYPE_BINARY;
}

bool NAV::vendor::ln::Ln200UartSensor::checksumFunction(const uart::protocol::Packet& packet)
{
    // TODO: Use 14th word of message as checksum
    if (packet.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        return true;
    }
    LOG_CRITICAL("Can't calculate checksum of packet with unknown type");
    return false;
}

bool NAV::vendor::ln::Ln200UartSensor::isErrorFunction([[maybe_unused]] const uart::protocol::Packet& packet)
{
    return false;
}

bool NAV::vendor::ln::Ln200UartSensor::isResponseFunction([[maybe_unused]] const uart::protocol::Packet& packet)
{
    return false;
}