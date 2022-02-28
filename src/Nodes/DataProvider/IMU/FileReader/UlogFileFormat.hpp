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
/// @brief The header is a fixed-size section and has the following format (16 bytes)
struct ulog_Header_s
{
    std::array<char, 7> fileMagic{}; ///< identifier that contains 'U', 'L', 'o' and 'g' chars
    char version{ 0 };               ///< ULog version (currently only 1, see https://docs.px4.io/master/en/dev_log/ulog_file_format.html)
    uint64_t timeStamp{};            ///< denotes the start of the logging in microseconds
};

// --------------------------------------------------------- Definitions Section ---------------------------------------------------
/// @brief The Definitions and Data sections consist of a stream of messages. Each starts with this header
struct message_header_s
{
    uint16_t msg_size{ 0 }; ///< size of the message in bytes without the header (hdr_size= 3 bytes)
    char msg_type{ 0 };     ///< defines the content and is one of the following
};

/// @brief Flag bitset message. This message must be the first message, right after the header section, so that it has a fixed constant offset.
struct ulog_message_flag_bits_s
{
    message_header_s header;                    ///< msg header
    std::array<uint8_t, 8> compat_flags{};      ///< compatible flag bits
    std::array<uint8_t, 8> incompat_flags{};    ///< incompatible flag bits
    std::array<uint64_t, 3> appended_offsets{}; ///< File offsets (0-based) for appended data
};

/// @brief format definition for a single (composite) type that can be logged or used in another definition as a nested type
struct message_format_s
{
    message_header_s header; ///< msg header
    std::string format;      ///< plain-text string with the following format: message_name:field0;field1;
};

/// @brief Information message
struct message_info_s
{
    message_header_s header; ///< msg header
    uint8_t key_len{ 1 };    ///< length of 'key'
    std::string key;         ///< key, e.g. 'char[value_len] sys_name'
    std::string value;       ///< value, e.g. 'PX4'
};

/// @brief Information message multi. The same as the information message, except that there can be multiple messages with the same key (parsers store them as a list)
struct ulog_message_info_multiple_header_s
{
    message_header_s header;   ///< msg header
    uint8_t is_continued{ 0 }; ///< can be used for arrays
    uint8_t key_len{ 0 };      ///< length of 'key'
    std::string key;           ///< key, e.g. 'char[value_len] sys_name'
    std::string value;         ///< value, e.g. 'PX4'
};

/// @brief parameter default message. If a parameter dynamically changes during runtime, this message can also be used in the Data section. The data type is restricted to: int32_t, float
struct ulog_message_parameter_default_header_s
{
    message_header_s header;    ///< msg header
    uint8_t default_types{ 0 }; //TODO: Validate default value
    uint8_t key_len{ 0 };       ///< length of 'key'
    std::string key;            ///< key
    std::string value;          ///< value
};

// ------------------------------------------------------------ Data Section --------------------------------------------------------

/// @brief Subscribed log message with name and ID. This must come before the first corresponding message_data_s
struct message_add_logged_s
{
    message_header_s header; ///< msg header
    uint8_t multi_id{ 0 };   ///< the same message format can have multiple instances, for example if the system has two sensors of the same type. The default and first instance must be 0
    uint16_t msg_id{ 0 };    ///< unique id to match message_data_s data. The first use must set this to 0, then increase it. The same msg_id must not be used twice for different subscriptions, not even after unsubscribing
    std::string msg_name;    ///< message name to subscribe to. Must match one of the message_format_s definitions
};

/// @brief unsubscribe a message, to mark that it will not be logged anymore
struct message_remove_logged_s
{
    message_header_s header; ///< msg header
    uint16_t msg_id{ 0 };    ///< unique id to match message_data_s data.
};

/// @brief contains logged data
struct message_data_s
{
    message_header_s header; ///< msg header
    uint16_t msg_id{ 0 };    ///< unique id to match message_data_s data.
    std::string data;        ///< contains the logged binary message as defined by message_format_s
};

/// @brief Logged string message, i.e. printf output
struct message_logging_s
{
    message_header_s header; ///< msg header
    uint8_t log_level{ 0 };  ///< same as in the Linux kernel
    uint64_t timestamp{ 0 }; ///< timestamp
    std::string message;     ///< log message
};

/// @brief Tagged Logged string message
struct message_logging_tagged_s
{
    message_header_s header; ///< msg header
    uint8_t log_level{ 0 };  ///< same as in the Linux kernel
    uint16_t tag{ 0 };       ///< id representing source of logged message string. It could represent a process, thread or a class depending upon the system architecture.
    uint64_t timestamp{ 0 }; ///< timestamp
    std::string message;     ///< log message
};

/// @brief tag attributes of message_logging_tagged_s
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

/// @brief synchronization message so that a reader can recover from a corrupt message by searching for the next sync message
struct message_sync_s
{
    message_header_s header;             ///< msg header
    std::array<uint8_t, 8> snyc_magic{}; ///< synchronization message
};

/// @brief dropout (lost logging messages) of a given duration in ms. Dropouts can occur e.g. if the device is not fast enough
struct message_dropout_s
{
    message_header_s header; ///< msg header
    uint16_t duration{ 0 };  ///< duration of dropout
};

#pragma pack(pop)

} // namespace NAV::Ulog
