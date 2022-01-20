/// @file UlogFileFormat.hpp
/// @brief List of all ulog message types
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2022-01-03
/// @note See PX4 User Guide - ULog File Format (https://docs.px4.io/master/en/dev_log/ulog_file_format.html)

#include <array>
#include <cstdint>
#include <vector>
#include <string>

namespace NAV::Ulog
{
#pragma pack(push, 1) // Syntax for gcc for #pragma pack
// --------------------------------------------------------------- Header ------------------------------------------------------------------
struct ulog_Header_s
{
    std::array<char, 7> fileMagic{};
    char version{ 0 };
    uint64_t timeStamp{};
};

// --------------------------------------------------------- Definitions Section ---------------------------------------------------
struct message_header_s
{
    uint16_t msg_size{ 0 };
    char msg_type{ 0 };
};

// Flag bitset message
struct ulog_message_flag_bits_s
{
    message_header_s header;
    std::array<uint8_t, 8> compat_flags{};
    std::array<uint8_t, 8> incompat_flags{};
    std::array<uint64_t, 3> appended_offsets{};
};

// format definition for a single (composite) type that can be logged or used in another definition as a nested type
struct message_format_s
{
    message_header_s header;
    std::vector<char> format;
};

// Information message
struct message_info_s
{
    message_header_s header;
    uint8_t key_len{ 1 };
    std::string key;
    std::string value;
};

struct ulog_message_info_multiple_header_s
{
    message_header_s header;
    uint8_t is_continued{ 0 }; ///< can be used for arrays
    uint8_t key_len{ 0 };
    std::string key;
    std::string value;
};

struct ulog_message_parameter_default_header_s
{
    message_header_s header;
    uint8_t default_types{ 0 }; //TODO: Validate default value
    uint8_t key_len{ 0 };
    std::string key;
    std::string value;
};

// ------------------------------------------------------------ Data Section --------------------------------------------------------

struct message_add_logged_s
{
    message_header_s header;
    uint8_t multi_id{ 0 };
    uint16_t msg_id{ 0 };
    std::string msg_name;
};

struct message_remove_logged_s
{
    message_header_s header;
    uint16_t msg_id{ 0 };
};

struct message_data_s
{
    message_header_s header;
    uint16_t msg_id{ 0 };
    std::string data;
    // uint8_t data = static_cast<uint8_t>(header.msg_size - 2);
};

struct message_logging_s
{
    message_header_s header;
    uint8_t log_level{ 0 };
    uint64_t timestamp{ 0 };
    std::string message;
};

struct message_logging_tagged_s
{
    message_header_s header;
    uint8_t log_level{ 0 };
    uint16_t tag{ 0 }; //TODO: Validate default value
    uint64_t timestamp{ 0 };
    std::string message;
};

enum class ulog_tag : uint16_t
{
    unassigned,
    mavlink_handler,
    ppk_handler,
    camera_handler,
    ptp_handler,
    serial_handler,
    watchdog,
    io_service,
    cbuf,
    ulg
};

struct message_sync_s
{
    message_header_s header;
    std::array<uint8_t, 8> snyc_magic{};
};

struct message_dropout_s
{
    message_header_s header;
    uint16_t duration{ 0 };
};

#pragma pack(pop)

} // namespace NAV::Ulog
