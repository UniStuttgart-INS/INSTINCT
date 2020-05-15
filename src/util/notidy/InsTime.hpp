/**
 * @file InsTime.hpp
 * @brief The class is responsible for all time-related tasks
 * @author T. Lambertus (tomke-jantje.lambertus@nav.uni-stuttgart.de)
 * @date 2020-12-02
 * 
 * @details This class contains all time-transformations functions. One instance is created for each InsTime object (defined in the structs).
 *          Intern, only the MJD-time (Modified Julien Date) is stored (here MJD is a struct which has mjd_day and mjd_frac).
 */

#pragma once

#include <iostream>
#include <map>
#include <boost/icl/interval_map.hpp>
//#include <boost/icl/continuous_interval.hpp>

namespace NAV
{
#define SEC_IN_WEEK (604800.0)
#define SEC_IN_DAY (86400.0)

/// The class is responsible for all time-related tasks
class InsTime
{
  public:
    // --------------------------------------- Structs -------------------------------------------
    // [UTC]
    using InsTime_MJD = struct
    {
        unsigned int mjd_day; //!< Full days of the Modified Julien Date [UTC]
        long double mjd_frac; //!< Decimal fractions of a day of the Modified Julien Date [UTC]
    };

    // [UTC]
    using InsTime_JD = struct
    {
        unsigned int jd_day; //!< Full days of the Julien Date [UTC]
        long double jd_frac; //!< Decimal fractions of a day of the Julien Date [UTC]
    };

    // [GPST]
    using InsTime_GPSweekTow = struct
    {
        uint16_t gpsWeek;  //!< Contains GPS week in GPS standard time [GPST]
        long double tow;   //!< Contains GPS time of week in GPS standard time [GPST]
        uint16_t gpsCycle; //!< Contains GPS cycle in GPS standard time [GPST]
    };

    // [GPST]
    using InsTime_YDoySod = struct
    {
        uint16_t year;   //!< Contains year in GPS standard time [GPST]
        uint16_t doy;    //!< Contains day of year in GPS standard time [GPST]
        long double sod; //!< Contains second of day in GPS standard time [GPST]
    };

    // [UTC]
    using InsTime_YMDHMS = struct
    {
        uint16_t year;   //!< Contains year in Universal Time Coordinated [UTC]
        uint16_t month;  //!< Contains month in Universal Time Coordinated [UTC]
        uint16_t day;    //!< Contains day in Universal Time Coordinated [UTC]
        uint16_t hour;   //!< Contains hour in Universal Time Coordinated [UTC]
        uint16_t min;    //!< Contains minute in Universal Time Coordinated [UTC]
        long double sec; //!< Contains second in Universal Time Coordinated [UTC]
    };

    // --------------------------- Public types and attributes  ----------------------------------
    using INTVAL = boost::icl::continuous_interval<InsTime>; //!< Used as map key for interval maps (time valid from to)

    enum TIME
    {
        UTC,
        GPST,
        GLNT,
        GST,
        BDT,
        QZSST,
        IRNSST
    }; /*!< List of all time systems (options for InsTime::TIME) */

    static const std::map<unsigned int, uint16_t> GpsLeapSec; //!< Maps GPS leap seconds to a time: map<key time, leap seconds> ({44786,1 }: 1 Jul 1981 with a difference UTC-TAI of 20 etc.)

    // --------------------------- Constructors and destructor -----------------------------------
    InsTime();                                                                                                /*! Constructor for InsTime object */
    explicit InsTime(InsTime_GPSweekTow gpsWeekTow);                                                          /*! Constructor for InsTime_GPSweekTow struct */
    explicit InsTime(InsTime_MJD mjd);                                                                        /*! Constructor for InsTime_MJD struct */
    explicit InsTime(InsTime_JD jd);                                                                          /*! Constructor for InsTime_JDstruct */
    explicit InsTime(InsTime_YDoySod yearDoySod);                                                             /*! Constructor for InsTime_YDoySod struct */
    explicit InsTime(InsTime_YMDHMS yearMonthDayHMS);                                                         /*! Constructor for InsTime_YMDHMS struct */
    InsTime(uint16_t gpsWeek, long double tow, uint16_t gpsCycle);                                            /*! Constructor for InsTime object with GPS */
    InsTime(uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t min, long double sec, TIME); /*! Constructor for InsTime object with year, month, day, hour, minute, second and time system */

    ~InsTime() = default; /*! Default destructor */

    InsTime(const InsTime&) = default;            ///< Copy constructor
    InsTime(InsTime&&) = default;                 ///< Move constructor
    InsTime& operator=(const InsTime&) = default; ///< Copy assignment operator
    InsTime& operator=(InsTime&&) = default;      ///< Move assignment operator

    // ------------------------------- Update, change time ---------------------------------------
    void SetInsTime(InsTime);                                                                                         /*!< Initializes the private InsTime_MJD struct (mjd) with the provided InsTime object */
    void SetInsTime(InsTime_GPSweekTow gpsWeekTow);                                                                   /*!< Initializes the private InsTime_MJD struct (mjd) with the provided InsTime_GPSweekTow time */
    void SetInsTime(InsTime_MJD mjd);                                                                                 /*!< Initializes the private InsTime_MJD struct (mjd) with the provided InsTime_MJD time */
    void SetInsTime(InsTime_JD jd);                                                                                   /*!< Initializes the private InsTime_MJD struct (mjd) with the provided InsTime_JD time */
    void SetInsTime(InsTime_YDoySod yearDoySod);                                                                      /*!< Initializes the private InsTime_MJD struct (mjd) with the provided InsTime_YDoySod time */
    void SetInsTime(InsTime_YMDHMS yearMonthDayHMS);                                                                  /*!< Initializes the private InsTime_MJD struct (mjd) with the provided InsTime_YMDHMS time */
    void SetInsTime(uint16_t gpsWeek, long double tow, uint16_t gpsCycle);                                            /*!< Initializes the private InsTime_MJD struct (mjd) with the provided gpsWeek, time of week and gpsCycle parameters */
    void SetInsTime(uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t min, long double sec, TIME); /*!< Initializes the private InsTime_MJD struct (mjd) with the provided year, month, day, hour, minute, second and time system paremeters */

    // ------------------------------- Accessors, get time ---------------------------------------
    InsTime_GPSweekTow GetGPSTime(); /*!< Returns the current InsTime_MJD struct in InsTime_GPSweekTow format */
    InsTime_MJD GetMJD();            /*!< Returns the current InsTime_MJD struct */
    InsTime_JD GetJD();              /*!< Returns the current InsTime_MJD struct in InsTime_JD format */
    InsTime_YDoySod GetYDoySod();    /*!< Returns the current InsTime_MJD struct in InsTime_YDoySod format */
    InsTime_YMDHMS GetYMDHMS();      /*!< Returns the current InsTime_MJD struct in InsTime_YMDHMS format */

    // ---------------------------- Transformation function --------------------------------------
    InsTime_MJD GPSweekTow2MJD(InsTime_GPSweekTow); /*!< Transforms given InsTime_GPSweekTow to InsTime_MJD (GPST to UTC) */
    InsTime_MJD YDoySod2MJD(InsTime_YDoySod);       /*!< Transforms given InsTime_YDoySod to InsTime_MJD (GPST to UTC) */
    InsTime_MJD YMDHMS2MJD(InsTime_YMDHMS);         /*!< Transforms given InsTime_YMDHMS to InsTime_MJD (UTC to UTC) */
    InsTime_MJD JD2MJD(InsTime_JD);                 /*!< Transforms given InsTime_JD to InsTime_MJD (UTC to UTC) */

    InsTime_GPSweekTow MJD2GPSweekTow(InsTime_MJD); /*!< Transforms given InsTime_MJD to InsTime_GPSweekTow (UTC to GPST) */
    InsTime_YDoySod MJD2YDoySod(InsTime_MJD);       /*!< Transforms given InsTime_MJD to InsTime_YDoySod (UTC to GPST) */
    InsTime_YMDHMS MJD2YMDHMS(InsTime_MJD);         /*!< Transforms given InsTime_MJD to InsTime_YMDHMS (UTC to UTC) */
    InsTime_JD MJD2JD(InsTime_MJD);                 /*!< Transforms given InsTime_MJD to InsTime_JD (UTC to UTC) */

    InsTime_YMDHMS YDoySod2YMDHMS(InsTime_YDoySod); /*!< Transforms given InsTime_YDoySod to InsTime_YMDHMS (Doy to Date) */
    InsTime_YDoySod YMDHMS2YDoySod(InsTime_YMDHMS); /*!< Transforms given InsTime_YMDHMS to InsTime_YDoySod (Date to Doy) */

    long double hms2sec(int hour, int min, long double sec); /*!< Transforms given hour, minute, second to seconds */

    // -------------------------------- Leap functions -------------------------------------------
    uint16_t leapGps2UTC();                   /*!< Returns the current number of leap seconds (offset GPST to UTC) */
    uint16_t leapGps2UTC(InsTime);            /*!< Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime object */
    uint16_t leapGps2UTC(InsTime_YDoySod);    /*!< Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_YDoySod time */
    uint16_t leapGps2UTC(InsTime_GPSweekTow); /*!< Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_GPSweekTow time */
    uint16_t leapGps2UTC(InsTime_YMDHMS);     /*!< Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_YMDHMS time */
    uint16_t leapGps2UTC(InsTime_MJD);        /*!< Returns the number of leap seconds (offset GPST to UTC) for the provided InsTime_MJD time */
    bool isLeapYear(uint16_t);                /*!< Returns true if the provided year is a leap year, false otherwise */
    bool isLeapYear();                        /*!< Returns true if the current time (InsTime_MJD mjd) is a leap year, false otherwise */

    // ------------------------ Comparison bigger/smaller/equal ----------------------------------
    // (können evt später gelöscht werden, werden nicht gebraucht)
    bool isBigger(InsTime);            /*!< Returns true if the provided InsTime object is later than the current time, false otherwise */
    bool isBigger(InsTime_YDoySod);    /*!< Returns true if the provided InsTime_YDoySod time is later than the current time, false otherwise */
    bool isBigger(InsTime_GPSweekTow); /*!< Returns true if the provided InsTime_GPSweekTow time is later than the current time, false otherwise */
    bool isBigger(InsTime_YMDHMS);     /*!< Returns true if the provided InsTime_YMDHMS time is later than the current time, false otherwise */
    bool isBigger(InsTime_MJD);        /*!< Returns true if the provided InsTime_MJD time is later than the current time, false otherwise */
    bool isEqual(InsTime);             /*!< Returns true if the provided InsTime object contains the same time as the current time, false otherwise */
    bool isEqual(InsTime_YDoySod);     /*!< Returns true if the provided InsTime_YDoySod time is the same as the current time, false otherwise */
    bool isEqual(InsTime_GPSweekTow);  /*!< Returns true if the provided InsTime_GPSweekTow time is the same as the current time, false otherwise */
    bool isEqual(InsTime_YMDHMS);      /*!< Returns true if the provided InsTime_YMDHMS time is the same as the current time, false otherwise */
    bool isEqual(InsTime_MJD);         /*!< Returns true if the provided InsTime_MJD time is the same as the current time, false otherwise */

    // ---------------------------------- Change time --------------------------------------------
    void addDiffSec(long double diffSec); /*!< Adds or subtracts the provided time difference [seconds] to/from the current time */
    long double getTimeDiff(InsTime);     /*!< Returns the time difference [seconds] between the given InsTime object and the current time (current - given) */
    void RoundToFullDay();                /*!< Rounds the current time to a full day (changes time to 0:0:0h UTC of current day) */
    std::string outputTime();             /*!< Returns the current time as a string with YMDHMS format for output */
    std::string outputTime(InsTime);      /*!< Returns the provided InsTime object as a string with YMDHMS format for output */
    std::string outputTowTime();          /*!< Returns the current time as a string with time of week (tow) format for output */
    void MakeTimeFromGloOrbit(double);    /*!< Adds the difference [seconds] between toe (OBRIT-0 last element) and toc (ORBIT-0 first element) to the current time (changes time, so that it corresponds to the time of GLONASS ORBIT last element) */
    std::string GetStringOfDate();        /*!< Returns the current time as a string with format YYYYMMDDHHMMSS */

    // ---------------------------- Static void functions ----------------------------------------
    /*! \brief Creates a time system (InsTime::TIME) with the provided string. Possibilities are "UTC" and "GPST".
    \param sys: string which defines the name of the new time system
    \param timesys: pointer to where the InsTime::TIME will be saved
    \return void: No return value
    */
    static void MakeTimeSysFromString(std::string, InsTime::TIME*);
    static std::string MakeStringFromTimeSys(InsTime::TIME); /*!< Returns the name of the provided time system as a string (e.g. GPST) */

    bool operator<(const InsTime& mjd_compare) const;  /*!< Returns true if the provided InsTime is bigger/later than the current time */
    bool operator>(const InsTime& mjd_compare) const;  /*!< Returns true if the provided InsTime is smaller/earlier than the current time */
    bool operator==(const InsTime& mjd_compare) const; /*!< Returns true if the provided InsTime is equal to the current time */
    bool operator!=(const InsTime& mjd_compare) const; /*!< Returns true if the provided InsTime is not equal to the current time */
    bool operator<=(const InsTime& mjd_compare) const; /*!< Returns true if the provided InsTime is bigger/later or equal to the current time */
    bool operator>=(const InsTime& mjd_compare) const; /*!< Returns true if the provided InsTime is smaller/earlier or equal to the current time */

  private:
    InsTime_MJD mjd;
};

} // namespace NAV

// was noch fehlt:
// sicherheitsabfragen, ob zeit realistisch
