/// @file InsTime.hpp
/// @brief The class is responsible for all time-related tasks
/// @author T. Lambertus (tomke-jantje.lambertus@nav.uni-stuttgart.de)
/// @date 2020-12-02
///
/// @details This class contains all time-transformations functions. One instance is created for each InsTime object (defined in the structs).
///          Intern, only the MJD-time (Modified Julien Date) is stored (here MJD is a struct which has mjd_day and mjd_frac).

#pragma once

// #include <iostream>
#include <string>
#include <map>
#include <array>
#include <compare>
#include <boost/icl/interval_map.hpp>
//#include <boost/icl/continuous_interval.hpp>

namespace NAV
{
/// The class is responsible for all time-related tasks
class InsTime
{
  public:
    // --------------------------------------- Structs -------------------------------------------
    /// Modified Julien Date [UTC]
    struct InsTime_MJD
    {
        unsigned int mjd_day; //!< Full days of the Modified Julien Date [UTC]
        long double mjd_frac; //!< Decimal fractions of a day of the Modified Julien Date [UTC]

        /// @brief Default constructor
        InsTime_MJD() = default;

        /// @brief Constructor
        /// @param[in] mjd_day Full days of the Modified Julien Date [UTC]
        /// @param[in] mjd_frac Decimal fractions of a day of the Modified Julien Date [UTC]
        constexpr InsTime_MJD(unsigned int mjd_day, long double mjd_frac)
            : mjd_day(mjd_day), mjd_frac(mjd_frac) {}

        /// @brief Spaceship Operator for Consistent comparison
        constexpr auto operator<=>(const InsTime_MJD&) const = default;
    };

    /// Julien Date [UTC]
    struct InsTime_JD
    {
        unsigned int jd_day; //!< Full days of the Julien Date [UTC]
        long double jd_frac; //!< Decimal fractions of a day of the Julien Date [UTC]

        /// @brief Default constructor
        InsTime_JD() = default;

        /// @brief Constructor
        /// @param[in] jd_day Full days of the Julien Date [UTC]
        /// @param[in] jd_frac Decimal fractions of a day of the Julien Date [UTC]
        constexpr InsTime_JD(unsigned int jd_day, long double jd_frac)
            : jd_day(jd_day), jd_frac(jd_frac) {}

        /// @brief Spaceship Operator for Consistent comparison
        constexpr auto operator<=>(const InsTime_JD&) const = default;
    };

    /// GPS week and time of week in GPS standard time [GPST]
    struct InsTime_GPSweekTow
    {
        uint16_t gpsWeek;  //!< Contains GPS week in GPS standard time [GPST]
        long double tow;   //!< Contains GPS time of week in GPS standard time [GPST]
        uint16_t gpsCycle; //!< Contains GPS cycle in GPS standard time [GPST]

        /// @brief Default constructor
        InsTime_GPSweekTow() = default;

        /// @brief Constructor
        /// @param[in] gpsWeek GPS week in GPS standard time [GPST]
        /// @param[in] tow GPS time of week in GPS standard time [GPST]
        /// @param[in] gpsCycle GPS cycle in GPS standard time [GPST]
        constexpr InsTime_GPSweekTow(uint16_t gpsWeek, long double tow, uint16_t gpsCycle)
            : gpsWeek(gpsWeek), tow(tow), gpsCycle(gpsCycle) {}

        /// @brief Spaceship Operator for Consistent comparison
        constexpr auto operator<=>(const InsTime_GPSweekTow&) const = default;
    };

    /// GPS year and day of year in GPS standard time [GPST]
    struct InsTime_YDoySod
    {
        uint16_t year;   //!< Contains year in GPS standard time [GPST]
        uint16_t doy;    //!< Contains day of year in GPS standard time [GPST]
        long double sod; //!< Contains second of day in GPS standard time [GPST]

        /// @brief Default constructor
        InsTime_YDoySod() = default;

        /// @brief Constructor
        /// @param[in] year Year in GPS standard time [GPST]
        /// @param[in] doy Day of year in GPS standard time [GPST]
        /// @param[in] sod Second of day in GPS standard time [GPST]
        constexpr InsTime_YDoySod(uint16_t year, uint16_t doy, long double sod)
            : year(year), doy(doy), sod(sod) {}

        /// @brief Spaceship Operator for Consistent comparison
        constexpr auto operator<=>(const InsTime_YDoySod&) const = default;
    };

    /// Universal Time Coordinated [UTC]
    struct InsTime_YMDHMS
    {
        uint16_t year;   //!< Contains year in Universal Time Coordinated [UTC]
        uint16_t month;  //!< Contains month in Universal Time Coordinated [UTC]
        uint16_t day;    //!< Contains day in Universal Time Coordinated [UTC]
        uint16_t hour;   //!< Contains hour in Universal Time Coordinated [UTC]
        uint16_t min;    //!< Contains minute in Universal Time Coordinated [UTC]
        long double sec; //!< Contains second in Universal Time Coordinated [UTC]

        /// @brief Default constructor
        InsTime_YMDHMS() = default;

        /// @brief Constructor
        /// @param[in] year Year in Universal Time Coordinated [UTC]
        /// @param[in] month Month in Universal Time Coordinated [UTC]
        /// @param[in] day Day in Universal Time Coordinated [UTC]
        /// @param[in] hour Hour in Universal Time Coordinated [UTC]
        /// @param[in] min Minute in Universal Time Coordinated [UTC]
        /// @param[in] sec Second in Universal Time Coordinated [UTC]
        InsTime_YMDHMS(uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t min, long double sec)
            : year(year), month(month), day(day), hour(hour), min(min), sec(sec) {}

        /// @brief Spaceship Operator for Consistent comparison
        constexpr auto operator<=>(const InsTime_YMDHMS&) const = default;
    };

    // --------------------------- Public types and attributes  ----------------------------------

    /// Used as map key for interval maps (time valid from to)
    using INTVAL = boost::icl::continuous_interval<InsTime>;

    /// @brief List of all time systems
    enum TIME
    {
        UTC,
        GPST,
        GLNT,
        GST,
        BDT,
        QZSST,
        IRNSST
    };

    /// Maps GPS leap seconds to a time: map<key time, leap seconds> ({44786,1 }: 1 Jul 1981 with a difference UTC-TAI of 20 etc.)
    static const std::map<unsigned int, uint16_t> GpsLeapSec;

    // --------------------------- Constructors  ----------------------------------

    /// @brief Default Constructor
    constexpr InsTime();

    /// @brief Constructor
    /// @param[in] mjd Time in Modified Julien Date [UTC]
    explicit InsTime(const InsTime_MJD& mjd);

    /// @brief Constructor
    /// @param[in] jd Time in Julien Date [UTC]
    explicit InsTime(const InsTime_JD& jd);
    /// @brief Assignment operator
    /// @param[in] jd Time in Julien Date [UTC]
    InsTime& operator=(const InsTime_JD& jd);

    /// @brief Constructor
    /// @param[in] gpsWeekTow Time in GPS standard time [GPST]
    explicit InsTime(const InsTime_GPSweekTow& gpsWeekTow);
    /// @brief Assignment operator
    /// @param[in] gpsWeekTow Time in GPS standard time [GPST]
    InsTime& operator=(const InsTime_GPSweekTow& gpsWeekTow);

    /// @brief Constructor
    /// @param[in] yearDoySod Time in GPS standard time [GPST]
    explicit InsTime(const InsTime_YDoySod& yearDoySod);
    /// @brief Assignment operator
    /// @param[in] yearDoySod Time in GPS standard time [GPST]
    InsTime& operator=(const InsTime_YDoySod& yearDoySod);

    /// @brief Constructor
    /// @param[in] yearMonthDayHMS Time in Universal Time Coordinated [UTC]
    explicit InsTime(const InsTime_YMDHMS& yearMonthDayHMS);
    /// @brief Assignment operator
    /// @param[in] yearMonthDayHMS Time in Universal Time Coordinated [UTC]
    InsTime& operator=(const InsTime_YMDHMS& yearMonthDayHMS);

    /// @brief Constructor
    /// @param[in] gpsWeek GPS week in GPS standard time [GPST]
    /// @param[in] tow GPS time of week in GPS standard time [GPST]
    /// @param[in] gpsCycle GPS cycle in GPS standard time [GPST]
    InsTime(uint16_t gpsWeek, long double tow, uint16_t gpsCycle);

    /// @brief Constructor
    /// @param[in] year Year in Universal Time Coordinated [UTC]
    /// @param[in] month Month in Universal Time Coordinated [UTC]
    /// @param[in] day Day in Universal Time Coordinated [UTC]
    /// @param[in] hour Hour in Universal Time Coordinated [UTC]
    /// @param[in] min Minute in Universal Time Coordinated [UTC]
    /// @param[in] sec Second in Universal Time Coordinated [UTC]
    InsTime(uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t min, long double sec, TIME timesys);

    /// @brief Destructor
    ~InsTime() = default;
    /// @brief Copy constructor
    InsTime(const InsTime&) = default;
    /// @brief Move constructor
    InsTime(InsTime&&) = default;
    /// @brief Copy assignment operator
    InsTime& operator=(const InsTime&) = default;
    /// @brief Move assignment operator
    InsTime& operator=(InsTime&&) = default;

    // ------------------------------- Update, change time ---------------------------------------
    void SetInsTime(InsTime t_in);                                                                                            /*!< Initializes the private InsTime_MJD struct (mjd) with the provided InsTime object */
    void SetInsTime(InsTime_GPSweekTow gpsWeekTow);                                                                           /*!< Initializes the private InsTime_MJD struct (mjd) with the provided InsTime_GPSweekTow time */
    void SetInsTime(InsTime_MJD mjd);                                                                                         /*!< Initializes the private InsTime_MJD struct (mjd) with the provided InsTime_MJD time */
    void SetInsTime(InsTime_JD jd);                                                                                           /*!< Initializes the private InsTime_MJD struct (mjd) with the provided InsTime_JD time */
    void SetInsTime(InsTime_YDoySod yearDoySod);                                                                              /*!< Initializes the private InsTime_MJD struct (mjd) with the provided InsTime_YDoySod time */
    void SetInsTime(InsTime_YMDHMS yearMonthDayHMS);                                                                          /*!< Initializes the private InsTime_MJD struct (mjd) with the provided InsTime_YMDHMS time */
    void SetInsTime(uint16_t gpsWeek, long double tow, uint16_t gpsCycle);                                                    /*!< Initializes the private InsTime_MJD struct (mjd) with the provided gpsWeek, time of week and gpsCycle parameters */
    void SetInsTime(uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t min, long double sec, TIME timesys); /*!< Initializes the private InsTime_MJD struct (mjd) with the provided year, month, day, hour, minute, second and time system paremeters */

    // ------------------------------- Accessors, get time ---------------------------------------
    [[nodiscard]] InsTime_GPSweekTow GetGPSTime() const; /*!< Returns the current InsTime_MJD struct in InsTime_GPSweekTow format */
    [[nodiscard]] InsTime_MJD GetMJD() const;            /*!< Returns the current InsTime_MJD struct */
    [[nodiscard]] InsTime_JD GetJD() const;              /*!< Returns the current InsTime_MJD struct in InsTime_JD format */
    [[nodiscard]] InsTime_YDoySod GetYDoySod() const;    /*!< Returns the current InsTime_MJD struct in InsTime_YDoySod format */
    [[nodiscard]] InsTime_YMDHMS GetYMDHMS() const;      /*!< Returns the current InsTime_MJD struct in InsTime_YMDHMS format */

    // ---------------------------- Transformation function --------------------------------------
    static InsTime_MJD GPSweekTow2MJD(InsTime_GPSweekTow gpsWeeekTow); /*!< Transforms given InsTime_GPSweekTow to InsTime_MJD (GPST to UTC) */
    static InsTime_MJD YDoySod2MJD(InsTime_YDoySod yearDoySod);        /*!< Transforms given InsTime_YDoySod to InsTime_MJD (GPST to UTC) */
    static InsTime_MJD YMDHMS2MJD(InsTime_YMDHMS yearMonthDayHMS);     /*!< Transforms given InsTime_YMDHMS to InsTime_MJD (UTC to UTC) */
    static InsTime_MJD JD2MJD(InsTime_JD jd);                          /*!< Transforms given InsTime_JD to InsTime_MJD (UTC to UTC) */

    static InsTime_GPSweekTow MJD2GPSweekTow(InsTime_MJD mjd); /*!< Transforms given InsTime_MJD to InsTime_GPSweekTow (UTC to GPST) */
    static InsTime_YDoySod MJD2YDoySod(InsTime_MJD mjd);       /*!< Transforms given InsTime_MJD to InsTime_YDoySod (UTC to GPST) */
    static InsTime_YMDHMS MJD2YMDHMS(InsTime_MJD mjd);         /*!< Transforms given InsTime_MJD to InsTime_YMDHMS (UTC to UTC) */
    static InsTime_JD MJD2JD(InsTime_MJD mjd);                 /*!< Transforms given InsTime_MJD to InsTime_JD (UTC to UTC) */

    static InsTime_YMDHMS YDoySod2YMDHMS(InsTime_YDoySod yearDoySod);      /*!< Transforms given InsTime_YDoySod to InsTime_YMDHMS (Doy to Date) */
    static InsTime_YDoySod YMDHMS2YDoySod(InsTime_YMDHMS yearMonthDayHMS); /*!< Transforms given InsTime_YMDHMS to InsTime_YDoySod (Date to Doy) */

    static long double hms2sec(int hour, int min, long double sec); /*!< Transforms given hour, minute, second to seconds */

    // -------------------------------- Leap functions -------------------------------------------
    [[nodiscard]] uint16_t leapGps2UTC() const;                  /*!< Returns the current number of leap seconds (offset GPST to UTC) */
    static uint16_t leapGps2UTC(InsTime insTime);                /*!< Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime object */
    static uint16_t leapGps2UTC(InsTime_YDoySod yearDoySod);     /*!< Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_YDoySod time */
    static uint16_t leapGps2UTC(InsTime_GPSweekTow gpsWeekTow);  /*!< Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_GPSweekTow time */
    static uint16_t leapGps2UTC(InsTime_YMDHMS yearMonthDayHMS); /*!< Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_YMDHMS time */
    static uint16_t leapGps2UTC(InsTime_MJD mjd_in);             /*!< Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_MJD time */
    static bool isLeapYear(uint16_t year);                       /*!< Returns true if the provided year is a leap year, false otherwise */
    [[nodiscard]] bool isLeapYear() const;                       /*!< Returns true if the current time (InsTime_MJD mjd) is a leap year, false otherwise */

    // ------------------------ Comparison bigger/smaller/equal ----------------------------------
    // (können evt später gelöscht werden, werden nicht gebraucht)
    [[nodiscard]] bool isBigger(InsTime insTime) const;                /*!< Returns true if the provided InsTime object is later than the current time, false otherwise */
    [[nodiscard]] bool isBigger(InsTime_YDoySod yearDoySod) const;     /*!< Returns true if the provided InsTime_YDoySod time is later than the current time, false otherwise */
    [[nodiscard]] bool isBigger(InsTime_GPSweekTow gpsWeekTow) const;  /*!< Returns true if the provided InsTime_GPSweekTow time is later than the current time, false otherwise */
    [[nodiscard]] bool isBigger(InsTime_YMDHMS yearMonthDayHMS) const; /*!< Returns true if the provided InsTime_YMDHMS time is later than the current time, false otherwise */
    [[nodiscard]] bool isBigger(InsTime_MJD mjd_compare) const;        /*!< Returns true if the provided InsTime_MJD time is later than the current time, false otherwise */
    [[nodiscard]] bool isEqual(InsTime insTime) const;                 /*!< Returns true if the provided InsTime object contains the same time as the current time, false otherwise */
    [[nodiscard]] bool isEqual(InsTime_YDoySod yearDoySod) const;      /*!< Returns true if the provided InsTime_YDoySod time is the same as the current time, false otherwise */
    [[nodiscard]] bool isEqual(InsTime_GPSweekTow gpsWeekTow) const;   /*!< Returns true if the provided InsTime_GPSweekTow time is the same as the current time, false otherwise */
    [[nodiscard]] bool isEqual(InsTime_YMDHMS yearMonthDayHMS) const;  /*!< Returns true if the provided InsTime_YMDHMS time is the same as the current time, false otherwise */
    [[nodiscard]] bool isEqual(InsTime_MJD mjd_compare) const;         /*!< Returns true if the provided InsTime_MJD time is the same as the current time, false otherwise */

    // ---------------------------------- Change time --------------------------------------------
    void addDiffSec(long double diffSec);                         /*!< Adds or subtracts the provided time difference [seconds] to/from the current time */
    [[nodiscard]] long double getTimeDiff(InsTime insTime) const; /*!< Returns the time difference [seconds] between the given InsTime object and the current time (current - given) */
    void RoundToFullDay();                                        /*!< Rounds the current time to a full day (changes time to 0:0:0h UTC of current day) */
    [[nodiscard]] std::string outputTime() const;                 /*!< Returns the current time as a string with YMDHMS format for output */
    static std::string outputTime(const InsTime& time);           /*!< Returns the provided InsTime object as a string with YMDHMS format for output */
    [[nodiscard]] std::string outputTowTime() const;              /*!< Returns the current time as a string with time of week (tow) format for output */
    void MakeTimeFromGloOrbit(double UTC_sec);                    /*!< Adds the difference [seconds] between toe (OBRIT-0 last element) and toc (ORBIT-0 first element) to the current time (changes time, so that it corresponds to the time of GLONASS ORBIT last element) */
    [[nodiscard]] std::string GetStringOfDate() const;            /*!< Returns the current time as a string with format YYYYMMDDHHMMSS */

    // ---------------------------- Static void functions ----------------------------------------

    /// @brief Creates a time system (InsTime::TIME) with the provided string. Possibilities are "UTC" and "GPST".
    /// @param[in] sys String which defines the name of the new time system
    /// @param[in] timesys Pointer to where the InsTime::TIME will be saved
    static void MakeTimeSysFromString(const std::string& sys, InsTime::TIME* timesys);
    static std::string MakeStringFromTimeSys(InsTime::TIME sys); /*!< Returns the name of the provided time system as a string (e.g. GPST) */

    bool operator<(const InsTime& mjd_compare) const;  /*!< Returns true if the provided InsTime is bigger/later than the current time */
    bool operator>(const InsTime& mjd_compare) const;  /*!< Returns true if the provided InsTime is smaller/earlier than the current time */
    bool operator==(const InsTime& mjd_compare) const; /*!< Returns true if the provided InsTime is equal to the current time */
    bool operator!=(const InsTime& mjd_compare) const; /*!< Returns true if the provided InsTime is not equal to the current time */
    bool operator<=(const InsTime& mjd_compare) const; /*!< Returns true if the provided InsTime is bigger/later or equal to the current time */
    bool operator>=(const InsTime& mjd_compare) const; /*!< Returns true if the provided InsTime is smaller/earlier or equal to the current time */

  private:
    InsTime_MJD mjd;

    static constexpr long double SEC_IN_WEEK = 604800.0L;
    static constexpr long double SEC_IN_DAY = 86400.0L;
    static constexpr unsigned int endOfTheCentury = 400000U;
};

} // namespace NAV

// was noch fehlt:
// sicherheitsabfragen, ob zeit realistisch
