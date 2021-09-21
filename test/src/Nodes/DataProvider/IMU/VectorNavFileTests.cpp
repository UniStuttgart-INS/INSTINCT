#include <catch2/catch.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <limits>

#include "Nodes/FlowTester.hpp"

#include "NodeData/IMU/VectorNavBinaryOutput.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "util/Logger.hpp"

namespace NAV::TEST::VectorNavFile
{
auto extractBit(auto& group, auto value)
{
    auto ret = group & value;
    group &= ~value;
    return ret;
}

constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();

enum ImuRef : size_t
{
    ImuRef_GpsCycle,
    ImuRef_GpsWeek,
    ImuRef_GpsTow,
    ImuRef_Time_TimeStartup,
    ImuRef_Time_GpsTow,
    ImuRef_Time_GpsWeek,
    ImuRef_Time_TimeStatus_timeOk,
    ImuRef_Time_TimeStatus_dateOk,
    ImuRef_Time_TimeStatus_utcTimeValid,
    ImuRef_IMU_UncompMag_X,
    ImuRef_IMU_UncompMag_Y,
    ImuRef_IMU_UncompMag_Z,
    ImuRef_IMU_UncompAccel_X,
    ImuRef_IMU_UncompAccel_Y,
    ImuRef_IMU_UncompAccel_Z,
    ImuRef_IMU_UncompGyro_X,
    ImuRef_IMU_UncompGyro_Y,
    ImuRef_IMU_UncompGyro_Z,
    ImuRef_IMU_Temp,
    ImuRef_IMU_Pres,
    ImuRef_IMU_DeltaTime,
    ImuRef_IMU_DeltaTheta_X,
    ImuRef_IMU_DeltaTheta_Y,
    ImuRef_IMU_DeltaTheta_Z,
    ImuRef_IMU_DeltaVel_X,
    ImuRef_IMU_DeltaVel_Y,
    ImuRef_IMU_DeltaVel_Z,
    ImuRef_IMU_Mag_X,
    ImuRef_IMU_Mag_Y,
    ImuRef_IMU_Mag_Z,
    ImuRef_IMU_Accel_X,
    ImuRef_IMU_Accel_Y,
    ImuRef_IMU_Accel_Z,
    ImuRef_IMU_AngularRate_X,
    ImuRef_IMU_AngularRate_Y,
    ImuRef_IMU_AngularRate_Z,
    ImuRef_GNSS1_Tow,
    ImuRef_GNSS1_Week,
    ImuRef_GNSS1_TimeInfo_Status_timeOk,
    ImuRef_GNSS1_TimeInfo_Status_dateOk,
    ImuRef_GNSS1_TimeInfo_Status_utcTimeValid,
    ImuRef_GNSS1_TimeInfo_LeapSeconds,
    ImuRef_Att_YawPitchRoll_Y,
    ImuRef_Att_YawPitchRoll_P,
    ImuRef_Att_YawPitchRoll_R,
    ImuRef_Att_Quaternion_w,
    ImuRef_Att_Quaternion_x,
    ImuRef_Att_Quaternion_y,
    ImuRef_Att_Quaternion_z,
    ImuRef_Att_YprU_Y,
    ImuRef_Att_YprU_P,
    ImuRef_Att_YprU_R,
};

constexpr std::array<std::array<long double, 52>, 18> IMU_REFERENCE_DATA = { {
    { 2, 117, 200579.516417L, 102910005000, 200579516417027, 2165, 1, 1, 1, -1.6342114, 0.41082269, -0.20590797, 9.7877541, 0.17773567, -0.55388463, 0.0013062661, -0.0099795656, 9.1388283e-05, 51.77, 95.317001, 0.0099999998, 0.0009811091, -0.0054370081, 9.4789139e-05, 0.09788689, 0.0017379548, -0.0053900857, -1.6342114, 0.41082269, -0.20590797, 9.7884798, 0.17774901, -0.55392307, 0.0020963906, -0.0096838931, -0.00041343208, 200579399583719, 2165, 1, 1, 1, 18, -145.66151, 86.864487, -19.587507, -0.32297635, -0.6108216, -0.31800929, 0.64919448, 19.99695, 0.024105821, 0.0247557 },
    { 2, 117, 200579.526417L, 102920005000, 200579526417017, 2165, 1, 1, 1, -1.6342114, 0.41082269, -0.20590797, 9.753089, 0.17925365, -0.55523938, -0.0011826784, -0.002124392, -0.0001852288, 51.77, 95.317001, 0.0099999998, -5.4759294e-05, -0.0015981552, -6.803798e-06, 0.097731493, 0.0015991499, -0.0055417381, -1.6342114, 0.41082269, -0.20590797, 9.7538147, 0.17926699, -0.55527782, -0.00039623992, -0.001830076, -0.0006893687, 200579399583719, 2165, 1, 1, 1, 18, -145.63219, 86.861496, -19.561306, -0.32300499, -0.61079603, -0.31801477, 0.64920151, 19.998583, 0.024259934, 0.024831882 },
    { 2, 117, 200579.536417L, 102930005000, 200579536417007, 2165, 1, 1, 1, -1.6473478, 0.40199661, -0.18811163, 9.7726021, 0.18083979, -0.54314888, -0.0010655901, 0.0018558895, 8.1041944e-06, 51.77, 95.317001, 0.0099999998, -0.00018287641, 0.00084728131, 0.00015959112, 0.097662091, 0.0016707397, -0.0055513987, -1.6473478, 0.40199661, -0.18811163, 9.7733278, 0.18085313, -0.54318732, -0.00028429303, 0.0021487402, -0.00049543241, 200579399583719, 2165, 1, 1, 1, 18, -145.62079, 86.861061, -19.553539, -0.32302842, -0.61078316, -0.31803095, 0.64919406, 20.000216, 0.024287194, 0.024773514 },
    { 2, 117, 200579.546418L, 102940006000, 200579546417952, 2165, 1, 1, 1, -1.6473478, 0.40199661, -0.18811163, 9.7913532, 0.17220372, -0.54076344, -0.0021698819, 0.0061732074, -2.7098577e-07, 51.77, 95.317001, 0.010001, -0.00076632842, 0.0024482876, 0.00040221537, 0.097832911, 0.0017555307, -0.0054585109, -1.6473478, 0.40199661, -0.18811163, 9.792079, 0.17221706, -0.54080188, -0.0013948749, 0.0064649959, -0.00050333038, 200579399583719, 2165, 1, 1, 1, 18, -145.62244, 86.862747, -19.560671, -0.32305592, -0.61077774, -0.31806397, 0.64916939, 20.001848, 0.024275372, 0.024735948 },
    { 2, 117, 200579.556418L, 102950006000, 200579556417942, 2165, 1, 1, 1, -1.6986954, 0.39927474, -0.18687437, 9.7869015, 0.18927211, -0.54762006, -0.0027822531, 0.0082405936, 0.00057540589, 51.77, 95.317001, 0.0099999998, -0.00142039, 0.0046483283, 0.000588617, 0.097935461, 0.0018434566, -0.0053492319, -1.6986954, 0.39927474, -0.18687437, 9.7876272, 0.18928544, -0.5476585, -0.0020144617, 0.0085316403, 7.2836177e-05, 200579399583719, 2165, 1, 1, 1, 18, -145.63637, 86.866676, -19.580786, -0.32308117, -0.61078495, -0.31810701, 0.64912885, 20.003481, 0.024024749, 0.024710683 },
    { 2, 117, 200579.566417L, 102960005000, 200579566416979, 2165, 1, 1, 1, -1.6986954, 0.39927474, -0.18687437, 9.7961674, 0.18433754, -0.53296447, -0.0029189235, 0.0098791551, 0.00063324568, 51.77, 95.317001, 0.0099989995, -0.0020187162, 0.0058960365, 0.00047131753, 0.097911626, 0.001867372, -0.0052757249, -1.6986954, 0.39927474, -0.18687437, 9.7968931, 0.18435088, -0.53300291, -0.0021579142, 0.010169338, 0.00013093784, 200579399583719, 2165, 1, 1, 1, 18, -145.66481, 86.871681, -19.615267, -0.32310054, -0.61079794, -0.31815535, 0.64908332, 20.005116, 0.02408547, 0.02469402 },
    { 2, 117, 200579.576417L, 102970005000, 200579576416969, 2165, 1, 1, 1, -1.7081889, 0.40629539, -0.19510326, 9.8201447, 0.18709214, -0.50143731, -0.0032287235, 0.012208453, 2.2484721e-05, 51.77, 95.317001, 0.0099999998, -0.0016303738, 0.0072912858, 0.00020335165, 0.098147362, 0.0018254429, -0.0051234467, -1.7081889, 0.40629539, -0.19510326, 9.8208704, 0.18710548, -0.50147575, -0.0024740631, 0.012498034, -0.00047952574, 200579399583719, 2165, 1, 1, 1, 18, -145.70781, 86.878136, -19.664062, -0.3231121, -0.61082071, -0.3182078, 0.64903039, 20.006748, 0.024287058, 0.024840172 },
    { 2, 117, 200579.586417L, 102980005000, 200579586416960, 2165, 1, 1, 1, -1.7081889, 0.40629539, -0.19510326, 9.8047218, 0.18791598, -0.4894318, -0.0041708238, 0.014185, 0.00025853055, 51.77, 95.317001, 0.0099999998, -0.0022835208, 0.008184379, 8.4275249e-05, 0.097931281, 0.0018852255, -0.0051451907, -1.7081889, 0.40629539, -0.19510326, 9.8054476, 0.18792932, -0.48947027, -0.0034223953, 0.014474172, -0.00024325703, 200579399583719, 2165, 1, 1, 1, 18, -145.75891, 86.885605, -19.7213, -0.32312271, -0.61084735, -0.318266, 0.64897156, 20.008383, 0.024040962, 0.024720406 },
    { 2, 117, 200579.596417L, 102990005000, 200579596416950, 2165, 1, 1, 1, -1.6947013, 0.40476289, -0.18687706, 9.7966623, 0.18044835, -0.47429785, -0.0044359541, 0.013983385, 1.3125027e-06, 51.77, 95.317001, 0.0099999998, -0.0023196433, 0.0078830905, 2.8099452e-05, 0.097955704, 0.0019052824, -0.0048463535, -1.6947013, 0.40476289, -0.18687706, 9.7973881, 0.18046169, -0.47433633, -0.0036939345, 0.014272454, -0.00050042081, 200579399583719, 2165, 1, 1, 1, 18, -145.81599, 86.893471, -19.784986, -0.32313383, -0.61087483, -0.3183293, 0.64890915, 19.889111, 0.024400486, 0.024969805 },
    { 2, 117, 200579.606418L, 103000006000, 200579606417894, 2165, 1, 1, 1, -1.6947013, 0.40476289, -0.18687706, 9.7721386, 0.15425219, -0.49881363, -0.0027319556, 0.014302858, -8.8065099e-06, 51.77, 95.317001, 0.010001, -0.0017067754, 0.0081262104, 0.00012780899, 0.097797513, 0.0016914755, -0.0048401929, -1.6947013, 0.40476289, -0.18687706, 9.7728643, 0.15426552, -0.4988521, -0.0019973344, 0.014592515, -0.00051036617, 200579399583719, 2165, 1, 1, 1, 18, -145.86522, 86.901802, -19.841255, -0.32314894, -0.61090565, -0.31839231, 0.64884168, 19.890734, 0.024425294, 0.024910176 },
    { 2, 117, 200579.616418L, 103010006000, 200579616417885, 2165, 1, 1, 1, -1.6763917, 0.36812639, -0.17352937, 9.7582693, 0.16032176, -0.50280309, -0.0026758087, 0.011109625, 0.0006458204, 51.77, 95.317001, 0.0099999998, -0.0013574018, 0.006842976, 0.00035086769, 0.097707137, 0.0016475263, -0.0049378318, -0.91616923, 0.21074827, -0.10091574, 9.7589951, 0.16033509, -0.50284153, -0.0019452338, 0.011399384, 0.00014491327, 200579399583719, 2165, 1, 1, 1, 18, -145.90265, 86.90863, -19.879625, -0.32313687, -0.61094469, -0.31841588, 0.64879936, 19.892359, 0.024408596, 0.024844816 },
    { 2, 117, 200579.626418L, 103020006000, 200579626417875, 2165, 1, 1, 1, -1.6763917, 0.36812639, -0.17352937, 9.7964935, 0.17084554, -0.45809561, -0.0019035363, 0.0083465511, 0.0011307527, 51.77, 95.317001, 0.0099999998, -0.0011678949, 0.0056579695, 0.00060813775, 0.097838074, 0.0016833149, -0.0047125551, -0.91616923, 0.21074827, -0.10091574, 9.7972193, 0.17085887, -0.45813408, -0.0011729975, 0.0086362753, 0.00063020381, 200579399583719, 2165, 1, 1, 1, 18, -145.92586, 86.914719, -19.90358, -0.32312909, -0.61097944, -0.31843287, 0.64876217, 19.893982, 0.024222987, 0.024788512 },
    { 2, 117, 200579.636418L, 103030006000, 200579636417866, 2165, 1, 1, 1, -1.6866086, 0.3634637, -0.18412501, 9.8005257, 0.18987465, -0.45759663, -0.0008560674, 0.0066498718, 0.0010172947, 51.77, 95.317001, 0.0099999998, -0.00098107837, 0.0044384254, 0.00045620144, 0.097992405, 0.0018082444, -0.0046898755, -0.92185313, 0.20815429, -0.1068104, 9.8012514, 0.18988799, -0.4576351, -0.00012556068, 0.0069404328, 0.00051712152, 200579399583719, 2165, 1, 1, 1, 18, -145.95856, 86.919861, -19.936874, -0.32311788, -0.61100912, -0.31845132, 0.6487307, 19.895609, 0.024376102, 0.024782831 },
    { 2, 117, 200579.646418L, 103040006000, 200579646417856, 2165, 1, 1, 1, -1.6866086, 0.3634637, -0.18412501, 9.806941, 0.20761901, -0.47716901, -0.00068768271, 0.002849963, 0.00065682334, 51.77, 95.364006, 0.0099999998, -0.00013158255, 0.0020408614, 0.00037286407, 0.098054603, 0.0020401469, -0.0047460236, -0.92185313, 0.20815429, -0.1068104, 9.8076668, 0.20763235, -0.47720748, 4.2772735e-05, 0.0031412898, 0.00015627465, 200579399583719, 2165, 1, 1, 1, 18, -145.97964, 86.92234, -19.957712, -0.32310772, -0.61102492, -0.3184588, 0.64871716, 19.896421, 0.024518643, 0.024955751 },
    { 2, 117, 200579.656417L, 103050005000, 200579656416893, 2165, 1, 1, 1, -1.6896552, 0.39631692, -0.19814098, 9.8189106, 0.18692058, -0.47540817, -0.0010218535, -0.0010542489, 2.0561733e-05, 51.77, 95.364006, 0.0099989995, -0.0008775366, -4.9677337e-06, 8.8825116e-05, 0.098191999, 0.0019334499, -0.004671826, -0.9235481, 0.22643146, -0.11460788, 9.8196363, 0.18693392, -0.47544664, -0.00029145129, -0.00076212897, -0.00048074318, 200579599583725, 2165, 1, 1, 1, 18, -146.00545, 86.923103, -19.983997, -0.32310158, -0.61102647, -0.31847075, 0.64871293, 19.898046, 0.024367813, 0.024973176 },
    { 2, 117, 200579.666417L, 103060005000, 200579666416883, 2165, 1, 1, 1, -1.6896552, 0.39631692, -0.19814098, 9.8003969, 0.18023726, -0.46472728, -0.0013154654, -0.0046998095, 0.00020549819, 51.77, 95.364006, 0.0099999998, -0.00030842106, -0.0011498758, 0.00016803552, 0.097982436, 0.0018769782, -0.0047146059, -0.9235481, 0.22643146, -0.11460788, 9.8011265, 0.18025067, -0.46476597, -0.0005851055, -0.0044071726, -0.00029622199, 200579599583725, 2165, 1, 1, 1, 18, -146.01805, 86.923172, -19.996572, -0.32309732, -0.61102635, -0.31847483, 0.64871323, 19.895618, 0.024025559, 0.024689633 },
    { 2, 117, 200579.676417L, 103070005000, 200579676416874, 2165, 1, 1, 1, -1.6832764, 0.39868858, -0.19648349, 9.7789717, 0.17074674, -0.44977117, 0.0023326594, -0.0083173309, 7.0952301e-06, 51.77, 95.364006, 0.0099999998, 0.00073099753, -0.0035910867, 7.7504228e-05, 0.097761124, 0.0016877537, -0.0047033415, -0.91999936, 0.2277509, -0.11368577, 9.7797012, 0.17076015, -0.44980985, 0.003062984, -0.0080238599, -0.00049455359, 200579599583725, 2165, 1, 1, 1, 18, -145.9987, 86.920753, -19.976189, -0.32309958, -0.61101496, -0.31846038, 0.64872992, 19.897243, 0.023983087, 0.024659876 },
    { 2, 117, 200579.686418L, 103080006000, 200579686417818, 2165, 1, 1, 1, -1.6832764, 0.39868858, -0.19648349, 9.7813892, 0.17352287, -0.44804251, 0.00076052221, -0.01025579, 0.00084225507, 51.77, 95.364006, 0.010001, 0.00068908563, -0.0053813723, 0.00031916826, 0.09781041, 0.0016853331, -0.0045879013, -0.91999936, 0.2277509, -0.11368577, 9.7821188, 0.17353629, -0.4480812, 0.0014908176, -0.0099614272, 0.0003409441, 200579599583725, 2165, 1, 1, 1, 18, -145.96375, 86.917007, -19.94018, -0.32310826, -0.61099583, -0.31844005, 0.64875358, 19.898867, 0.024279552, 0.024780776 },
} };

void compareImuObservation(const std::shared_ptr<const NAV::VectorNavBinaryOutput>& obs, size_t messageCounterImuData)
{
    // ------------------------------------------------ InsTime --------------------------------------------------
    REQUIRE(obs->insTime.has_value());

    REQUIRE(obs->insTime->toGPSweekTow().gpsCycle == static_cast<int32_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GpsCycle)));
    REQUIRE(obs->insTime->toGPSweekTow().gpsWeek == static_cast<int32_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GpsWeek)));
    REQUIRE(obs->insTime->toGPSweekTow().tow == Approx(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GpsTow)).margin(EPSILON));

    // ----------------------------------------------- TimeGroup -------------------------------------------------
    REQUIRE(obs->timeOutputs != nullptr);

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP));
    REQUIRE(obs->timeOutputs->timeStartup == static_cast<uint64_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Time_TimeStartup)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW));
    REQUIRE(obs->timeOutputs->gpsTow == static_cast<uint64_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Time_GpsTow)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK));
    REQUIRE(obs->timeOutputs->gpsWeek == static_cast<uint16_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Time_GpsWeek)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS));
    REQUIRE(obs->timeOutputs->timeStatus.timeOk() == static_cast<uint16_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Time_TimeStatus_timeOk)));
    REQUIRE(obs->timeOutputs->timeStatus.dateOk() == static_cast<uint16_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Time_TimeStatus_dateOk)));
    REQUIRE(obs->timeOutputs->timeStatus.utcTimeValid() == static_cast<uint16_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Time_TimeStatus_utcTimeValid)));

    REQUIRE(obs->timeOutputs->timeField == vn::protocol::uart::TimeGroup::TIMEGROUP_NONE);

    // ----------------------------------------------- ImuGroup --------------------------------------------------
    REQUIRE(obs->imuOutputs != nullptr);

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG));
    REQUIRE(obs->imuOutputs->uncompMag(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompMag_X)));
    REQUIRE(obs->imuOutputs->uncompMag(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompMag_Y)));
    REQUIRE(obs->imuOutputs->uncompMag(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompMag_Z)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL));
    REQUIRE(obs->imuOutputs->uncompAccel(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompAccel_X)));
    REQUIRE(obs->imuOutputs->uncompAccel(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompAccel_Y)));
    REQUIRE(obs->imuOutputs->uncompAccel(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompAccel_Z)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO));
    REQUIRE(obs->imuOutputs->uncompGyro(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompGyro_X)));
    REQUIRE(obs->imuOutputs->uncompGyro(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompGyro_Y)));
    REQUIRE(obs->imuOutputs->uncompGyro(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompGyro_Z)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_TEMP));
    REQUIRE(obs->imuOutputs->temp == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Temp)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_PRES));
    REQUIRE(obs->imuOutputs->pres == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Pres)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA));
    REQUIRE(obs->imuOutputs->deltaTime == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_DeltaTime)));
    REQUIRE(obs->imuOutputs->deltaTheta(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_DeltaTheta_X)));
    REQUIRE(obs->imuOutputs->deltaTheta(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_DeltaTheta_Y)));
    REQUIRE(obs->imuOutputs->deltaTheta(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_DeltaTheta_Z)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL));
    REQUIRE(obs->imuOutputs->deltaV(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_DeltaVel_X)));
    REQUIRE(obs->imuOutputs->deltaV(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_DeltaVel_Y)));
    REQUIRE(obs->imuOutputs->deltaV(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_DeltaVel_Z)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_MAG));
    REQUIRE(obs->imuOutputs->mag(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Mag_X)));
    REQUIRE(obs->imuOutputs->mag(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Mag_Y)));
    REQUIRE(obs->imuOutputs->mag(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Mag_Z)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL));
    REQUIRE(obs->imuOutputs->accel(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Accel_X)));
    REQUIRE(obs->imuOutputs->accel(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Accel_Y)));
    REQUIRE(obs->imuOutputs->accel(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Accel_Z)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE));
    REQUIRE(obs->imuOutputs->angularRate(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_AngularRate_X)));
    REQUIRE(obs->imuOutputs->angularRate(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_AngularRate_Y)));
    REQUIRE(obs->imuOutputs->angularRate(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_AngularRate_Z)));

    REQUIRE(obs->imuOutputs->imuField == vn::protocol::uart::ImuGroup::IMUGROUP_NONE);

    // ---------------------------------------------- GpsGroup 1 -------------------------------------------------
    REQUIRE(obs->gnss1Outputs != nullptr);

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TOW));
    REQUIRE(obs->gnss1Outputs->tow == static_cast<uint64_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GNSS1_Tow)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_WEEK));
    REQUIRE(obs->gnss1Outputs->week == static_cast<uint16_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GNSS1_Week)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO));
    REQUIRE(obs->gnss1Outputs->timeInfo.status.timeOk() == static_cast<uint8_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GNSS1_TimeInfo_Status_timeOk)));
    REQUIRE(obs->gnss1Outputs->timeInfo.status.dateOk() == static_cast<uint8_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GNSS1_TimeInfo_Status_dateOk)));
    REQUIRE(obs->gnss1Outputs->timeInfo.status.utcTimeValid() == static_cast<uint8_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GNSS1_TimeInfo_Status_utcTimeValid)));
    REQUIRE(obs->gnss1Outputs->timeInfo.leapSeconds == static_cast<int8_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GNSS1_TimeInfo_LeapSeconds)));

    REQUIRE(obs->gnss1Outputs->gnssField == vn::protocol::uart::GpsGroup::GPSGROUP_NONE);

    // --------------------------------------------- AttitudeGroup -----------------------------------------------
    REQUIRE(obs->attitudeOutputs != nullptr);

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL));
    REQUIRE(obs->attitudeOutputs->ypr(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_YawPitchRoll_Y)));
    REQUIRE(obs->attitudeOutputs->ypr(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_YawPitchRoll_P)));
    REQUIRE(obs->attitudeOutputs->ypr(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_YawPitchRoll_R)));

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION));
    REQUIRE(obs->attitudeOutputs->qtn.w() == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_Quaternion_w)));
    REQUIRE(obs->attitudeOutputs->qtn.x() == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_Quaternion_x)));
    REQUIRE(obs->attitudeOutputs->qtn.y() == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_Quaternion_y)));
    REQUIRE(obs->attitudeOutputs->qtn.z() == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_Quaternion_z)));

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU));
    REQUIRE(obs->attitudeOutputs->yprU(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_YprU_Y)));
    REQUIRE(obs->attitudeOutputs->yprU(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_YprU_P)));
    REQUIRE(obs->attitudeOutputs->yprU(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_YprU_R)));

    REQUIRE(obs->attitudeOutputs->attitudeField == vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE);

    // ----------------------------------------------- InsGroup --------------------------------------------------
    REQUIRE(obs->insOutputs == nullptr);

    // ---------------------------------------------- GpsGroup 2 -------------------------------------------------
    REQUIRE(obs->gnss2Outputs == nullptr);
}

constexpr size_t MESSAGE_COUNT_IMU = 18; ///< Amount of messages expected in the Imu files
size_t messageCounterImuDataCsv = 0;     ///< Message Counter for the Imu data csv file
size_t messageCounterImuDataVnb = 0;     ///< Message Counter for the Imu data vnb file

TEST_CASE("[VectorNavFile] Read 'data/vn310-imu.csv' and compare content with hardcoded values", "[VectorNavFile]")
{
    messageCounterImuDataCsv = 0;

    Logger logger;

    // ###########################################################################################################
    //                                     VectorNavFile-vn310-imu-csv.flow
    // ###########################################################################################################
    //
    // VectorNavFile("data/vn310-imu.csv")
    //
    // ###########################################################################################################

    nm::RegisterWatcherCallbackToOutputPin(1, [](const std::shared_ptr<const NAV::NodeData>& data) {
        LOG_TRACE("messageCounterImuDataCsv = {}", messageCounterImuDataCsv);

        compareImuObservation(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(data), messageCounterImuDataCsv);

        messageCounterImuDataCsv++;
    });

    testFlow("test/flow/Nodes/DataProvider/IMU/VectorNavFile-vn310-imu-csv.flow");

    REQUIRE(messageCounterImuDataCsv == MESSAGE_COUNT_IMU);
}

TEST_CASE("[VectorNavFile] Read 'data/vn310-imu.vnb' and compare content with hardcoded values", "[VectorNavFile]")
{
    messageCounterImuDataVnb = 0;

    Logger logger;

    // ###########################################################################################################
    //                                     VectorNavFile-vn310-imu-vnb.flow
    // ###########################################################################################################
    //
    // VectorNavFile("data/vn310-imu.vnb")
    //
    // ###########################################################################################################

    nm::RegisterWatcherCallbackToOutputPin(1, [](const std::shared_ptr<const NAV::NodeData>& data) {
        LOG_TRACE("messageCounterImuDataVnb = {}", messageCounterImuDataVnb);

        compareImuObservation(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(data), messageCounterImuDataVnb);

        messageCounterImuDataVnb++;
    });

    testFlow("test/flow/Nodes/DataProvider/IMU/VectorNavFile-vn310-imu-vnb.flow");

    REQUIRE(messageCounterImuDataVnb == MESSAGE_COUNT_IMU);
}

enum GnssRef : size_t
{
    GnssRef_GpsCycle,
    GnssRef_GpsWeek,
    GnssRef_GpsTow,
    GnssRef_Time_TimeStartup,
    GnssRef_Time_GpsTow,
    GnssRef_Time_GpsWeek,
    GnssRef_Time_TimeUTC_year,
    GnssRef_Time_TimeUTC_month,
    GnssRef_Time_TimeUTC_day,
    GnssRef_Time_TimeUTC_hour,
    GnssRef_Time_TimeUTC_min,
    GnssRef_Time_TimeUTC_sec,
    GnssRef_Time_TimeUTC_ms,
    GnssRef_Time_TimeStatus_timeOk,
    GnssRef_Time_TimeStatus_dateOk,
    GnssRef_Time_TimeStatus_utcTimeValid,
    GnssRef_GNSS1_UTC_year,
    GnssRef_GNSS1_UTC_month,
    GnssRef_GNSS1_UTC_day,
    GnssRef_GNSS1_UTC_hour,
    GnssRef_GNSS1_UTC_min,
    GnssRef_GNSS1_UTC_sec,
    GnssRef_GNSS1_UTC_ms,
    GnssRef_GNSS1_Tow,
    GnssRef_GNSS1_Week,
    GnssRef_GNSS1_NumSats,
    GnssRef_GNSS1_Fix,
    GnssRef_GNSS1_PosLla_latitude,
    GnssRef_GNSS1_PosLla_longitude,
    GnssRef_GNSS1_PosLla_altitude,
    GnssRef_GNSS1_PosEcef_X,
    GnssRef_GNSS1_PosEcef_Y,
    GnssRef_GNSS1_PosEcef_Z,
    GnssRef_GNSS1_VelNed_N,
    GnssRef_GNSS1_VelNed_E,
    GnssRef_GNSS1_VelNed_D,
    GnssRef_GNSS1_VelEcef_X,
    GnssRef_GNSS1_VelEcef_Y,
    GnssRef_GNSS1_VelEcef_Z,
    GnssRef_GNSS1_PosU_N,
    GnssRef_GNSS1_PosU_E,
    GnssRef_GNSS1_PosU_D,
    GnssRef_GNSS1_VelU,
    GnssRef_GNSS1_TimeU,
    GnssRef_GNSS1_TimeInfo_Status_timeOk,
    GnssRef_GNSS1_TimeInfo_Status_dateOk,
    GnssRef_GNSS1_TimeInfo_Status_utcTimeValid,
    GnssRef_GNSS1_TimeInfo_LeapSeconds,
    GnssRef_GNSS1_DOP_g,
    GnssRef_GNSS1_DOP_p,
    GnssRef_GNSS1_DOP_t,
    GnssRef_GNSS1_DOP_v,
    GnssRef_GNSS1_DOP_h,
    GnssRef_GNSS1_DOP_n,
    GnssRef_GNSS1_DOP_e,
    GnssRef_Att_YawPitchRoll_Y,
    GnssRef_Att_YawPitchRoll_P,
    GnssRef_Att_YawPitchRoll_R,
    GnssRef_Att_Quaternion_w,
    GnssRef_Att_Quaternion_x,
    GnssRef_Att_Quaternion_y,
    GnssRef_Att_Quaternion_z,
    GnssRef_Att_YprU_Y,
    GnssRef_Att_YprU_P,
    GnssRef_Att_YprU_R,
    GnssRef_INS_InsStatus_Mode,
    GnssRef_INS_InsStatus_GpsFix,
    GnssRef_INS_InsStatus_Error_IMU,
    GnssRef_INS_InsStatus_Error_MagPres,
    GnssRef_INS_InsStatus_Error_GNSS,
    GnssRef_INS_InsStatus_GpsHeadingIns,
    GnssRef_INS_InsStatus_GpsCompass,
    GnssRef_INS_PosLla_latitude,
    GnssRef_INS_PosLla_longitude,
    GnssRef_INS_PosLla_altitude,
    GnssRef_INS_PosEcef_X,
    GnssRef_INS_PosEcef_Y,
    GnssRef_INS_PosEcef_Z,
    GnssRef_INS_VelBody_X,
    GnssRef_INS_VelBody_Y,
    GnssRef_INS_VelBody_Z,
    GnssRef_INS_VelNed_N,
    GnssRef_INS_VelNed_E,
    GnssRef_INS_VelNed_D,
    GnssRef_INS_VelEcef_X,
    GnssRef_INS_VelEcef_Y,
    GnssRef_INS_VelEcef_Z,
    GnssRef_INS_MagEcef_X,
    GnssRef_INS_MagEcef_Y,
    GnssRef_INS_MagEcef_Z,
    GnssRef_INS_AccelEcef_X,
    GnssRef_INS_AccelEcef_Y,
    GnssRef_INS_AccelEcef_Z,
    GnssRef_INS_LinearAccelEcef_X,
    GnssRef_INS_LinearAccelEcef_Y,
    GnssRef_INS_LinearAccelEcef_Z,
    GnssRef_INS_PosU,
    GnssRef_INS_VelU,
    GnssRef_GNSS2_UTC_year,
    GnssRef_GNSS2_UTC_month,
    GnssRef_GNSS2_UTC_day,
    GnssRef_GNSS2_UTC_hour,
    GnssRef_GNSS2_UTC_min,
    GnssRef_GNSS2_UTC_sec,
    GnssRef_GNSS2_UTC_ms,
    GnssRef_GNSS2_Tow,
    GnssRef_GNSS2_Week,
    GnssRef_GNSS2_NumSats,
    GnssRef_GNSS2_Fix,
    GnssRef_GNSS2_PosLla_latitude,
    GnssRef_GNSS2_PosLla_longitude,
    GnssRef_GNSS2_PosLla_altitude,
    GnssRef_GNSS2_PosEcef_X,
    GnssRef_GNSS2_PosEcef_Y,
    GnssRef_GNSS2_PosEcef_Z,
    GnssRef_GNSS2_VelNed_N,
    GnssRef_GNSS2_VelNed_E,
    GnssRef_GNSS2_VelNed_D,
    GnssRef_GNSS2_VelEcef_X,
    GnssRef_GNSS2_VelEcef_Y,
    GnssRef_GNSS2_VelEcef_Z,
    GnssRef_GNSS2_PosU_N,
    GnssRef_GNSS2_PosU_E,
    GnssRef_GNSS2_PosU_D,
    GnssRef_GNSS2_VelU,
    GnssRef_GNSS2_TimeU,
    GnssRef_GNSS2_TimeInfo_Status_timeOk,
    GnssRef_GNSS2_TimeInfo_Status_dateOk,
    GnssRef_GNSS2_TimeInfo_Status_utcTimeValid,
    GnssRef_GNSS2_TimeInfo_LeapSeconds,
    GnssRef_GNSS2_DOP_g,
    GnssRef_GNSS2_DOP_p,
    GnssRef_GNSS2_DOP_t,
    GnssRef_GNSS2_DOP_v,
    GnssRef_GNSS2_DOP_h,
    GnssRef_GNSS2_DOP_n,
    GnssRef_GNSS2_DOP_e,
};

constexpr std::array<std::array<long double, 137>, 13> GNSS_REFERENCE_DATA = { {
    { 2, 117, 200632.654227L, 156047505000, 200632654226542, 2165, 21, 7, 6, 7, 43, 34, 654, 1, 1, 1, 21, 7, 6, 7, 43, 34, 599, 200632599585647, 2165, 11, 4, 48.739926652999998, 8.9212790529999992, 576.90600000000006, 4163613.193, 653588.76470000006, 4771968.3865, 1.636, -3.615, -1.285, 0.18284202, -3.6305671, 2.0448713, 3.2110248, 2.3637588, 6.0670466, 0.23040001, 9.9999997e-10, 1, 1, 1, 18, 1.99, 1.73, 0.97999996, 1.38, 1.04, 0.89999998, 0.52999997, -72.441338, 60.264, 5.6273508, -0.68233031, -0.33051941, -0.37940753, 0.53031325, 3.6171904, 0.080585025, 0.029943381, 1, 1, 0, 0, 0, 0, 1, 48.739926429256371, 8.9212791980329609, 576.48458818765357, 4163612.9360300242, 653588.73065747682, 4771968.0532969283, 2.8397217, 0.79355043, 3.0365357, 1.7059624, -3.7606137, -0.92831612, -0.078914225, -3.8190532, 1.8228822, 0.91181087, -1.1165178, 2.0382035, 3.9981194, -2.4473009, 6.8266401, -2.3918548, -3.4504488, -0.54642379, 0.97786552, 0.24471769, 21, 7, 6, 7, 43, 34, 400, 200632400334915, 2165, 13, 4, 48.739921922937519, 8.9212949596775353, 576.44097259175032, 4163613.1000000001, 653589.93000000005, 4771967.6900000004, 1.2, -3.0369999, -1.572, 0.59999996, -2.98, 1.9699999, 2.4870515, 1.855933, 4.7202625, 0.24160002, 1.9999999e-09, 1, 1, 1, 18, 1.8099999, 1.5699999, 0.89999998, 1.3, 0.88, 0.71999997, 0.5 },
    { 2, 117, 200632.854227L, 156247505000, 200632854226589, 2165, 21, 7, 6, 7, 43, 34, 854, 1, 1, 1, 21, 7, 6, 7, 43, 34, 799, 200632799585655, 2165, 11, 4, 48.739929918000001, 8.9212687180000003, 577.05369999999994, 4163613.1384000001, 653587.98100000003, 4771968.7370000007, 2.0139999, -4.125, -0.625, -0.44877204, -4.2459612, 1.7980163, 3.2105498, 2.3692949, 6.0962605, 0.20400001, 9.9999997e-10, 1, 1, 1, 18, 1.99, 1.73, 0.97999996, 1.38, 1.04, 0.89999998, 0.52999997, -72.089249, 58.288464, 0.60221648, -0.70467687, -0.29027125, -0.39106983, 0.51598215, 2.6393163, 0.090866663, 0.029609242, 1, 1, 0, 0, 0, 0, 1, 48.739929589791643, 8.9212677923454873, 576.64558293484151, 4163612.9099927954, 653587.87730932352, 4771968.4061258258, 2.9048371, 0.59321302, 3.841846, 2.0022736, -4.397665, -0.44855493, -0.51273644, -4.5320053, 1.6576447, 0.8744458, -1.1064771, 2.0071206, 3.9114001, -2.3339632, 6.4705634, -2.4785743, -3.3371103, -0.90250087, 0.97536945, 0.20687258, 21, 7, 6, 7, 43, 34, 600, 200632600334998, 2165, 13, 4, 48.739924341344832, 8.9212859482156848, 576.69721707049757, 4163613.1699999999, 653589.27000000002, 4771968.0600000005, 1.562, -3.651, -1.074, 0.11, -3.6799998, 1.8399999, 2.4905741, 1.8530722, 4.7176619, 0.21920002, 1.9999999e-09, 1, 1, 1, 18, 1.8099999, 1.5699999, 0.89999998, 1.3, 0.88, 0.71999997, 0.5 },
    { 2, 117, 200633.059232L, 156452505000, 200633059232473, 2165, 21, 7, 6, 7, 43, 35, 59, 1, 1, 1, 21, 7, 6, 7, 43, 35, 0, 200632999585664, 2165, 11, 4, 48.739933895, 8.9212566950000003, 577.11800000000005, 4163612.9884000001, 653587.06600000011, 4771969.0770000005, 2.4070001, -4.7010002, -0.139, -0.9679296, -4.910511, 1.6918536, 3.2108896, 2.3769212, 6.1271887, 0.18960002, 9.9999997e-10, 1, 1, 1, 18, 1.99, 1.73, 0.97999996, 1.38, 1.04, 0.89999998, 0.52999997, -67.229294, 58.452045, -1.6226697, -0.73052174, -0.25998268, -0.41341016, 0.47732484, 1.989002, 0.10328811, 0.025578482, 1, 1, 0, 0, 0, 0, 1, 48.739933636390518, 8.9212547737700039, 576.69879550533369, 4163612.7589526665, 653586.8842440953, 4771968.7429194394, 2.9073873, 0.12139747, 4.7327442, 2.3835154, -5.0184999, -0.0042469501, -0.989048, -5.2352123, 1.5750684, 0.77528417, -1.0719707, 2.0715151, 4.0705152, -2.1790543, 7.1313729, -2.3194587, -3.1822, -0.24169147, 0.9724443, 0.17651017, 21, 7, 6, 7, 43, 34, 800, 200632800335081, 2165, 13, 4, 48.739927440694274, 8.9212752251921525, 576.88702383637428, 4163613.1600000001, 653588.46999999997, 4771968.4299999997, 1.855, -4.2410002, -0.66100001, -0.28999999, -4.3399997, 1.7199999, 2.4949515, 1.8512576, 4.7151251, 0.20640002, 1.9999999e-09, 1, 1, 1, 18, 1.8099999, 1.5699999, 0.89999998, 1.3, 0.88, 0.71999997, 0.5 },
    { 2, 117, 200633.254231L, 156647504000, 200633254231453, 2165, 21, 7, 6, 7, 43, 35, 254, 1, 1, 1, 21, 7, 6, 7, 43, 35, 199, 200633199585672, 2165, 11, 4, 48.739938602999992, 8.9212432029999995, 577.10550000000001, 4163612.7453000001, 653586.02269999997, 4771969.4128999999, 2.8269999, -5.2290001, 0.226, -1.4357504, -5.5184135, 1.6944551, 3.212122, 2.3844502, 6.156528, 0.17040001, 9.9999997e-10, 1, 1, 1, 18, 1.99, 1.73, 0.97999996, 1.38, 1.04, 0.89999998, 0.52999997, -60.61742, 59.238503, -3.198895, -0.75717747, -0.22837417, -0.43876061, 0.42663413, 1.5677187, 0.11432771, 0.023480428, 1, 1, 0, 0, 0, 0, 1, 48.739938161505101, 8.9212409408563715, 576.66573652205989, 4163612.5214744238, 653585.81697518937, 4771969.0499567371, 2.8140574, -0.64416933, 5.4577627, 2.723726, -5.5274148, 0.38736391, -1.4179133, -5.8176804, 1.5050464, 0.69714254, -0.9671073, 2.2233832, 4.856092, -2.4347184, 7.5630064, -1.5338818, -3.4378629, 0.18994111, 0.96893358, 0.15122667, 21, 7, 6, 7, 43, 35, 0, 200633000335165, 2165, 13, 4, 48.73993104008705, 8.9212629459894437, 576.97483169473708, 4163613.0600000001, 653587.54000000004, 4771968.7599999998, 2.2320001, -4.8080001, -0.20200001, -0.77999997, -4.9899998, 1.62, 2.4988759, 1.8494642, 4.7122574, 0.20320001, 1.9999999e-09, 1, 1, 1, 18, 1.8099999, 1.5699999, 0.89999998, 1.3, 0.88, 0.71999997, 0.5 },
    { 2, 117, 200633.454232L, 156847505000, 200633454232454, 2165, 21, 7, 6, 7, 43, 35, 454, 1, 1, 1, 21, 7, 6, 7, 43, 35, 399, 200633399585681, 2165, 11, 4, 48.739943937, 8.9212285369999993, 577.01670000000001, 4163612.4148000004, 653584.87520000001, 4771969.7374, 3.0999999, -5.5960002, 0.62800002, -1.8434777, -5.9539118, 1.5722998, 3.2147012, 2.3879709, 6.1747017, 0.15600002, 9.9999997e-10, 1, 1, 1, 18, 1.99, 1.73, 0.97999996, 1.38, 1.04, 0.89999998, 0.52999997, -51.248901, 59.557735, -4.2366638, -0.78998882, -0.18571438, -0.46137562, 0.35855332, 1.2745283, 0.12349389, 0.031061068, 1, 1, 0, 0, 0, 0, 1, 48.739943540252966, 8.9212252541509169, 576.55701873963699, 4163612.1853479934, 653584.59618892847, 4771969.3627280509, 2.7140713, -1.8381621, 5.9754, 3.0644791, -6.041491, 0.74817705, -1.8263171, -6.4021606, 1.4585334, 0.68465096, -0.90113771, 2.6847768, 4.3498669, -2.2282727, 6.2225547, -2.0401063, -3.2314157, -1.1505105, 0.96507555, 0.13130882, 21, 7, 6, 7, 43, 35, 200, 200633200335248, 2165, 13, 4, 48.739935324892592, 8.9212491316883913, 576.96916018519551, 4163612.8599999999, 653586.47999999998, 4771969.0700000003, 2.506, -5.369, 0.245, -1.1899999, -5.6199999, 1.4699999, 2.5013661, 1.8472486, 4.7083435, 0.2, 1.9999999e-09, 1, 1, 1, 18, 1.8099999, 1.5699999, 0.89999998, 1.3, 0.88, 0.71999997, 0.5 },
    { 2, 117, 200633.654232L, 157047505000, 200633654232264, 2165, 21, 7, 6, 7, 43, 35, 654, 1, 1, 1, 21, 7, 6, 7, 43, 35, 599, 200633599585690, 2165, 11, 4, 48.739949852999992, 8.9212125530000002, 576.86569999999995, 4163612.0095000002, 653583.62569999998, 4771970.0578000005, 3.473, -6.1269999, 0.82200003, -2.1645231, -6.541811, 1.672451, 3.216866, 2.3878489, 6.1884055, 0.15600002, 9.9999997e-10, 1, 1, 1, 18, 1.99, 1.73, 0.97999996, 1.38, 1.04, 0.89999998, 0.52999997, -39.918285, 60.629932, -5.127768, -0.81830812, -0.13582781, -0.48714262, 0.273157, 1.0750234, 0.1281216, 0.050033435, 1, 1, 0, 0, 0, 0, 0, 48.739949392775962, 8.9212085370351222, 576.39185783546418, 4163611.7850728221, 653583.2886094898, 4771969.6678180508, 2.5409203, -3.4049842, 6.1996331, 3.465683, -6.5989876, 0.9633534, -2.1779976, -7.0216889, 1.5613649, 0.32744509, -0.33921754, 1.5817189, 4.808754, -2.1096444, 7.8242588, -1.5812185, -3.1127856, 0.45119303, 0.96081501, 0.11710527, 21, 7, 6, 7, 43, 35, 400, 200633400335331, 2165, 13, 4, 48.739940091782074, 8.9212338744247468, 576.87654482759535, 4163612.5800000001, 653585.30000000005, 4771969.3500000006, 2.8329999, -5.9070001, 0.57800001, -1.5599999, -6.2199998, 1.4299999, 2.5042062, 1.8446436, 4.7079234, 0.21600001, 1.9999999e-09, 1, 1, 1, 18, 1.8099999, 1.5699999, 0.89999998, 1.3, 0.88, 0.71999997, 0.5 },
    { 2, 117, 200633.854232L, 157247505000, 200633854232311, 2165, 21, 7, 6, 7, 43, 35, 854, 1, 1, 1, 21, 7, 6, 7, 43, 35, 799, 200633799585698, 2165, 11, 4, 48.739956524999997, 8.921195225, 576.67510000000004, 4163611.5326, 653582.25639999995, 4771970.4039000003, 3.9300001, -6.6599998, 1.066, -2.5802143, -7.1465917, 1.7904119, 3.2184916, 2.3839929, 6.1894755, 0.14640002, 9.9999997e-10, 1, 1, 1, 18, 1.99, 1.73, 0.97999996, 1.38, 1.04, 0.89999998, 0.52999997, -27.58736, 60.937729, -5.7989926, -0.84209192, -0.078402713, -0.50221115, 0.1803277, 0.95011288, 0.12671764, 0.074372731, 1, 1, 0, 0, 0, 0, 0, 48.739956152120726, 8.9211900099705552, 576.20254450244829, 4163611.3148125694, 653581.83527687646, 4771970.0212616259, 2.4233956, -5.1719298, 6.043663, 4.0062556, -7.2098336, 1.0563147, -2.5452814, -7.6976686, 1.8479791, 0.23809665, -0.15364313, 1.5365438, 5.4613724, -3.4908905, 9.0501528, -0.92860055, -4.49403, 1.6770864, 0.95698023, 0.10910493, 21, 7, 6, 7, 43, 35, 600, 200633600335415, 2165, 13, 4, 48.739945462355699, 8.9212170188135911, 576.72502971813083, 4163612.23, 653583.98999999999, 4771969.6299999999, 3.24, -6.5159998, 0.889, -1.9699999, -6.9099998, 1.4699999, 2.5065532, 1.8414627, 4.7050419, 0.20640002, 1.9999999e-09, 1, 1, 1, 18, 1.8099999, 1.5699999, 0.89999998, 1.3, 0.88, 0.71999997, 0.5 },
    { 2, 117, 200634.059238L, 157452505000, 200634059238434, 2165, 21, 7, 6, 7, 43, 36, 59, 1, 1, 1, 21, 7, 6, 7, 43, 36, 0, 200633999585707, 2165, 11, 4, 48.739964061000002, 8.9211760609999988, 576.47069999999997, 4163610.9946000003, 653580.75150000001, 4771970.8028999995, 4.447, -7.3740001, 0.91399997, -2.7544003, -7.8966784, 2.2456241, 3.2195816, 2.3777523, 6.1877098, 0.15360001, 9.9999997e-10, 1, 1, 1, 18, 1.99, 1.73, 0.97999996, 1.38, 1.04, 0.89999998, 0.52999997, -16.238077, 67.940178, -6.0560608, -0.8240391, -0.035433803, -0.55857432, 0.087742433, 0.85518247, 0.12083621, 0.096485607, 1, 1, 0, 0, 0, 0, 0, 48.739964177696891, 8.9211695078179165, 575.96767575666308, 4163610.7328226315, 653580.21734226402, 4771970.433329992, 1.4010203, -6.8488393, 5.8846359, 4.6243534, -7.7937922, 1.1707054, -2.9882734, -8.3583136, 2.1696095, 0.30791545, 0.069846213, 1.3595942, 5.354537, -2.8426523, 8.9465208, -1.0354345, -3.845787, 1.5734522, 0.95302755, 0.10181122, 21, 7, 6, 7, 43, 35, 800, 200633800335498, 2165, 13, 4, 48.739951676808822, 8.9211984094698451, 576.55769345816225, 4163611.8200000003, 653582.54000000004, 4771969.96, 3.677, -7.1960001, 1.045, -2.3, -7.6399999, 1.64, 2.5067341, 1.8359523, 4.6998096, 0.2, 1.9999999e-09, 1, 1, 1, 18, 1.8099999, 1.5699999, 0.89999998, 1.3, 0.88, 0.71999997, 0.5 },
    { 2, 117, 200634.254238L, 157647505000, 200634254238367, 2165, 21, 7, 6, 7, 43, 36, 254, 1, 1, 1, 21, 7, 6, 7, 43, 36, 199, 200634199585715, 2165, 11, 4, 48.739972410999997, 8.9211552110000003, 576.27369999999996, 4163610.4158000001, 653579.09939999995, 4771971.2673000004, 4.8590002, -8.0530005, 0.89200002, -2.940733, -8.6132441, 2.5338671, 3.2185855, 2.3700795, 6.1805029, 0.14160001, 9.9999997e-10, 1, 1, 1, 18, 1.99, 1.73, 0.97999996, 1.38, 1.04, 0.89999998, 0.52999997, -7.943922, 76.81321, -5.3943429, -0.78289855, -0.0061980113, -0.62161374, 0.025056537, 0.79782581, 0.11324154, 0.11123195, 1, 1, 0, 0, 0, 0, 0, 48.739972720784131, 8.9211482021217972, 575.76108799874783, 4163610.1356812143, 653578.53719854029, 4771970.9046138665, 0.34811151, -8.0961113, 5.5048323, 5.0548873, -8.3212023, 1.084933, -3.1703315, -8.9207611, 2.5180137, 0.42872164, 0.18525255, 1.0829812, 5.6489358, -1.6775103, 8.5731068, -0.74103558, -2.6806452, 1.2000384, 0.94941837, 0.096209787, 21, 7, 6, 7, 43, 36, 0, 200634000335581, 2165, 13, 4, 48.739958708755218, 8.9211779753359259, 576.32389951497316, 4163611.3200000003, 653580.94000000006, 4771970.2999999998, 4.1789999, -7.7839999, 1.303, -2.75, -8.3099995, 1.78, 2.5066249, 1.8297725, 4.6946793, 0.2016, 1.9999999e-09, 1, 1, 1, 18, 1.8099999, 1.5699999, 0.89999998, 1.3, 0.88, 0.71999997, 0.5 },
    { 2, 117, 200634.454238L, 157847505000, 200634454238415, 2165, 21, 7, 6, 7, 43, 36, 454, 1, 1, 1, 21, 7, 6, 7, 43, 36, 399, 200634399585724, 2165, 11, 4, 48.739981464999993, 8.9211324649999995, 576.1096, 4163609.8202000004, 653577.31520000007, 4771971.8079000004, 5.1760001, -8.6389999, 0.815, -3.0351052, -9.2212343, 2.8008041, 3.2155058, 2.3623924, 6.1724348, 0.14880002, 9.9999997e-10, 1, 1, 1, 18, 1.99, 1.73, 0.97999996, 1.38, 1.04, 0.89999998, 0.52999997, -3.0051265, 83.209785, -5.7283406, -0.74742055, 0.019961415, -0.6639123, -0.013584574, 0.75404465, 0.10841105, 0.11820406, 1, 1, 0, 0, 0, 0, 0, 48.739981986231314, 8.9211246306039325, 575.56743459962308, 4163609.5131513886, 653576.68435865047, 4771971.4386011371, -0.27142119, -9.1858664, 4.9589148, 5.3167143, -8.9360542, 0.96130848, -3.1888866, -9.5460529, 2.7836139, 0.39669332, 0.18792787, 0.76952952, 6.4715681, -1.9635081, 8.1954079, 0.081597596, -2.9666407, 0.82233775, 0.94549859, 0.090294622, 21, 7, 6, 7, 43, 36, 200, 200634200335664, 2165, 13, 4, 48.739966857099162, 8.9211562114514713, 576.10090669617057, 4163610.75, 653579.22999999998, 4771970.7300000004, 4.7810001, -8.2639999, 1.226, -3.0699999, -8.8499994, 2.23, 2.5066233, 1.824353, 4.6918678, 0.21440001, 1.9999999e-09, 1, 1, 1, 18, 1.8099999, 1.5699999, 0.89999998, 1.3, 0.88, 0.71999997, 0.5 },
    { 2, 117, 200634.654237L, 158047504000, 200634654237270, 2165, 21, 7, 6, 7, 43, 36, 654, 1, 1, 1, 21, 7, 6, 7, 43, 36, 599, 200634599585732, 2165, 11, 4, 48.73999105, 8.9211077499999991, 575.9434, 4163609.2017000001, 653575.38080000004, 4771972.3859999999, 5.4860001, -9.4960003, 0.75400001, -3.0926771, -10.097766, 3.0510976, 3.2125504, 2.3568978, 6.1666989, 0.14160001, 9.9999997e-10, 1, 1, 1, 18, 1.99, 1.73, 0.97999996, 1.38, 1.04, 0.89999998, 0.52999997, -24.554335, 86.678452, -28.90358, -0.72460467, 0.036037602, -0.6879968, -0.017602567, 0.73334676, 0.14299557, 0.073686883, 1, 1, 0, 0, 0, 0, 1, 48.739991717240031, 8.9210995528097552, 575.38702384196222, 4163608.8779789708, 653574.71737871028, 4771972.0166890118, -0.46912569, -10.038133, 4.8300877, 5.5388298, -9.6252966, 0.99445438, -3.2685506, -10.256238, 2.9051764, 0.32011884, 0.13269684, 0.52580136, 6.5691142, -2.4957709, 7.7892509, 0.17914446, -3.4989011, 0.41618073, 0.94214028, 0.087126307, 21, 7, 6, 7, 43, 36, 400, 200634400335748, 2165, 13, 4, 48.739975694807775, 8.9211329202674055, 575.87716845329851, 4163610.1400000001, 653577.40000000002, 4771971.21, 5.118, -8.9180002, 1.0319999, -3.0899999, -9.5100002, 2.5999999, 2.5052252, 1.8193276, 4.6877279, 0.20320001, 1.9999999e-09, 1, 1, 1, 18, 1.8099999, 1.5699999, 0.89999998, 1.3, 0.88, 0.71999997, 0.5 },
    { 2, 117, 200634.854238L, 158247505000, 200634854238272, 2165, 21, 7, 6, 7, 43, 36, 854, 1, 1, 1, 21, 7, 6, 7, 43, 36, 799, 200634799585741, 2165, 11, 4, 48.740001043999996, 8.9210809439999998, 575.77120000000002, 4163608.5703000003, 653573.2831, 4771972.9895000001, 5.631, -10.232, 0.866, -3.1591892, -10.85322, 3.0625291, 3.2097898, 2.3516643, 6.1607552, 0.14400001, 9.9999997e-10, 1, 1, 1, 18, 1.99, 1.73, 0.97999996, 1.38, 1.04, 0.89999998, 0.52999997, -85.775627, 86.490768, -89.902695, -0.70713329, 0.047085691, -0.70550054, -0.0038131834, 0.72052753, 0.14062743, 0.078516088, 1, 1, 0, 0, 0, 0, 1, 48.740001936741955, 8.9210718994832146, 575.21402435656637, 4163608.2366676731, 653572.55766372441, 4771972.6361757834, -0.38170612, -10.832948, 4.9052486, 5.6689453, -10.408255, 1.0445988, -3.2764361, -11.050019, 2.953289, 0.34624436, 0.10685552, 0.5074532, 7.2014899, -3.5411835, 6.982769, 0.81152093, -4.5443106, -0.39030248, 0.93854892, 0.084380798, 21, 7, 6, 7, 43, 36, 600, 200634600335831, 2165, 13, 4, 48.739985217408311, 8.9211079253167185, 575.67972671240568, 4163609.5100000002, 653575.44000000006, 4771971.7599999998, 5.494, -9.448, 0.93599999, -3.22, -10.07, 2.9199998, 2.5030346, 1.8158548, 4.6901588, 0.21920002, 1.9999999e-09, 1, 1, 1, 18, 1.8099999, 1.5699999, 0.89999998, 1.3, 0.88, 0.71999997, 0.5 },
} };

void compareGnssObservation(const std::shared_ptr<const NAV::VectorNavBinaryOutput>& obs, size_t messageCounterGnssData)
{
    // ------------------------------------------------ InsTime --------------------------------------------------
    REQUIRE(obs->insTime.has_value());

    REQUIRE(obs->insTime->toGPSweekTow().gpsCycle == static_cast<int32_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GpsCycle)));
    REQUIRE(obs->insTime->toGPSweekTow().gpsWeek == static_cast<int32_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GpsWeek)));
    REQUIRE(obs->insTime->toGPSweekTow().tow == Approx(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GpsTow)).margin(EPSILON));

    // ----------------------------------------------- TimeGroup -------------------------------------------------
    REQUIRE(obs->timeOutputs != nullptr);

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP));
    REQUIRE(obs->timeOutputs->timeStartup == static_cast<uint64_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeStartup)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW));
    REQUIRE(obs->timeOutputs->gpsTow == static_cast<uint64_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_GpsTow)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK));
    REQUIRE(obs->timeOutputs->gpsWeek == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_GpsWeek)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC));
    REQUIRE(obs->timeOutputs->timeUtc.year == static_cast<int8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeUTC_year)));
    REQUIRE(obs->timeOutputs->timeUtc.month == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeUTC_month)));
    REQUIRE(obs->timeOutputs->timeUtc.day == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeUTC_day)));
    REQUIRE(obs->timeOutputs->timeUtc.hour == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeUTC_hour)));
    REQUIRE(obs->timeOutputs->timeUtc.min == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeUTC_min)));
    REQUIRE(obs->timeOutputs->timeUtc.sec == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeUTC_sec)));
    REQUIRE(obs->timeOutputs->timeUtc.ms == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeUTC_ms)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS));
    REQUIRE(obs->timeOutputs->timeStatus.timeOk() == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeStatus_timeOk)));
    REQUIRE(obs->timeOutputs->timeStatus.dateOk() == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeStatus_dateOk)));
    REQUIRE(obs->timeOutputs->timeStatus.utcTimeValid() == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeStatus_utcTimeValid)));

    REQUIRE(obs->timeOutputs->timeField == vn::protocol::uart::TimeGroup::TIMEGROUP_NONE);

    // ----------------------------------------------- ImuGroup --------------------------------------------------
    REQUIRE(obs->imuOutputs == nullptr);

    // ---------------------------------------------- GpsGroup 1 -------------------------------------------------
    REQUIRE(obs->gnss1Outputs != nullptr);

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_UTC));
    REQUIRE(obs->gnss1Outputs->timeUtc.year == static_cast<int8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_UTC_year)));
    REQUIRE(obs->gnss1Outputs->timeUtc.month == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_UTC_month)));
    REQUIRE(obs->gnss1Outputs->timeUtc.day == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_UTC_day)));
    REQUIRE(obs->gnss1Outputs->timeUtc.hour == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_UTC_hour)));
    REQUIRE(obs->gnss1Outputs->timeUtc.min == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_UTC_min)));
    REQUIRE(obs->gnss1Outputs->timeUtc.sec == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_UTC_sec)));
    REQUIRE(obs->gnss1Outputs->timeUtc.ms == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_UTC_ms)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TOW));
    REQUIRE(obs->gnss1Outputs->tow == static_cast<uint64_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_Tow)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_WEEK));
    REQUIRE(obs->gnss1Outputs->week == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_Week)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS));
    REQUIRE(obs->gnss1Outputs->numSats == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_NumSats)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_FIX));
    REQUIRE(obs->gnss1Outputs->fix == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_Fix)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA));
    REQUIRE(obs->gnss1Outputs->posLla(0) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosLla_latitude)));
    REQUIRE(obs->gnss1Outputs->posLla(1) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosLla_longitude)));
    REQUIRE(obs->gnss1Outputs->posLla(2) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosLla_altitude)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF));
    REQUIRE(obs->gnss1Outputs->posEcef(0) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosEcef_X)));
    REQUIRE(obs->gnss1Outputs->posEcef(1) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosEcef_Y)));
    REQUIRE(obs->gnss1Outputs->posEcef(2) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosEcef_Z)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELNED));
    REQUIRE(obs->gnss1Outputs->velNed(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_VelNed_N)));
    REQUIRE(obs->gnss1Outputs->velNed(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_VelNed_E)));
    REQUIRE(obs->gnss1Outputs->velNed(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_VelNed_D)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF));
    REQUIRE(obs->gnss1Outputs->velEcef(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_VelEcef_X)));
    REQUIRE(obs->gnss1Outputs->velEcef(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_VelEcef_Y)));
    REQUIRE(obs->gnss1Outputs->velEcef(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_VelEcef_Z)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSU));
    REQUIRE(obs->gnss1Outputs->posU(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosU_N)));
    REQUIRE(obs->gnss1Outputs->posU(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosU_E)));
    REQUIRE(obs->gnss1Outputs->posU(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosU_D)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELU));
    REQUIRE(obs->gnss1Outputs->velU == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_VelU)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU));
    REQUIRE(obs->gnss1Outputs->timeU == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_TimeU)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO));
    REQUIRE(obs->gnss1Outputs->timeInfo.status.timeOk() == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_TimeInfo_Status_timeOk)));
    REQUIRE(obs->gnss1Outputs->timeInfo.status.dateOk() == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_TimeInfo_Status_dateOk)));
    REQUIRE(obs->gnss1Outputs->timeInfo.status.utcTimeValid() == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_TimeInfo_Status_utcTimeValid)));
    REQUIRE(obs->gnss1Outputs->timeInfo.leapSeconds == static_cast<int8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_TimeInfo_LeapSeconds)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_DOP));
    REQUIRE(obs->gnss1Outputs->dop.gDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_DOP_g)));
    REQUIRE(obs->gnss1Outputs->dop.pDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_DOP_p)));
    REQUIRE(obs->gnss1Outputs->dop.tDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_DOP_t)));
    REQUIRE(obs->gnss1Outputs->dop.vDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_DOP_v)));
    REQUIRE(obs->gnss1Outputs->dop.hDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_DOP_h)));
    REQUIRE(obs->gnss1Outputs->dop.nDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_DOP_n)));
    REQUIRE(obs->gnss1Outputs->dop.eDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_DOP_e)));

    REQUIRE(obs->gnss1Outputs->gnssField == vn::protocol::uart::GpsGroup::GPSGROUP_NONE);

    // --------------------------------------------- AttitudeGroup -----------------------------------------------
    REQUIRE(obs->attitudeOutputs != nullptr);

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL));
    REQUIRE(obs->attitudeOutputs->ypr(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_YawPitchRoll_Y)));
    REQUIRE(obs->attitudeOutputs->ypr(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_YawPitchRoll_P)));
    REQUIRE(obs->attitudeOutputs->ypr(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_YawPitchRoll_R)));

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION));
    REQUIRE(obs->attitudeOutputs->qtn.w() == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_Quaternion_w)));
    REQUIRE(obs->attitudeOutputs->qtn.x() == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_Quaternion_x)));
    REQUIRE(obs->attitudeOutputs->qtn.y() == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_Quaternion_y)));
    REQUIRE(obs->attitudeOutputs->qtn.z() == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_Quaternion_z)));

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU));
    REQUIRE(obs->attitudeOutputs->yprU(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_YprU_Y)));
    REQUIRE(obs->attitudeOutputs->yprU(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_YprU_P)));
    REQUIRE(obs->attitudeOutputs->yprU(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_YprU_R)));

    REQUIRE(obs->attitudeOutputs->attitudeField == vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE);

    // ----------------------------------------------- InsGroup --------------------------------------------------
    REQUIRE(obs->insOutputs != nullptr);

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS));
    REQUIRE(obs->insOutputs->insStatus.mode() == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_InsStatus_Mode)));
    REQUIRE(obs->insOutputs->insStatus.gpsFix() == static_cast<bool>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_InsStatus_GpsFix)));
    REQUIRE(obs->insOutputs->insStatus.errorIMU() == static_cast<bool>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_InsStatus_Error_IMU)));
    REQUIRE(obs->insOutputs->insStatus.errorMagPres() == static_cast<bool>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_InsStatus_Error_MagPres)));
    REQUIRE(obs->insOutputs->insStatus.errorGnss() == static_cast<bool>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_InsStatus_Error_GNSS)));
    REQUIRE(obs->insOutputs->insStatus.gpsHeadingIns() == static_cast<bool>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_InsStatus_GpsHeadingIns)));
    REQUIRE(obs->insOutputs->insStatus.gpsCompass() == static_cast<bool>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_InsStatus_GpsCompass)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_POSLLA));
    REQUIRE(obs->insOutputs->posLla(0) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_PosLla_latitude)));
    REQUIRE(obs->insOutputs->posLla(1) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_PosLla_longitude)));
    REQUIRE(obs->insOutputs->posLla(2) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_PosLla_altitude)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_POSECEF));
    REQUIRE(obs->insOutputs->posEcef(0) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_PosEcef_X)));
    REQUIRE(obs->insOutputs->posEcef(1) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_PosEcef_Y)));
    REQUIRE(obs->insOutputs->posEcef(2) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_PosEcef_Z)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELBODY));
    REQUIRE(obs->insOutputs->velBody(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelBody_X)));
    REQUIRE(obs->insOutputs->velBody(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelBody_Y)));
    REQUIRE(obs->insOutputs->velBody(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelBody_Z)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELNED));
    REQUIRE(obs->insOutputs->velNed(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelNed_N)));
    REQUIRE(obs->insOutputs->velNed(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelNed_E)));
    REQUIRE(obs->insOutputs->velNed(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelNed_D)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELECEF));
    REQUIRE(obs->insOutputs->velEcef(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelEcef_X)));
    REQUIRE(obs->insOutputs->velEcef(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelEcef_Y)));
    REQUIRE(obs->insOutputs->velEcef(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelEcef_Z)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_MAGECEF));
    REQUIRE(obs->insOutputs->magEcef(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_MagEcef_X)));
    REQUIRE(obs->insOutputs->magEcef(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_MagEcef_Y)));
    REQUIRE(obs->insOutputs->magEcef(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_MagEcef_Z)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF));
    REQUIRE(obs->insOutputs->accelEcef(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_AccelEcef_X)));
    REQUIRE(obs->insOutputs->accelEcef(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_AccelEcef_Y)));
    REQUIRE(obs->insOutputs->accelEcef(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_AccelEcef_Z)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF));
    REQUIRE(obs->insOutputs->linearAccelEcef(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_LinearAccelEcef_X)));
    REQUIRE(obs->insOutputs->linearAccelEcef(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_LinearAccelEcef_Y)));
    REQUIRE(obs->insOutputs->linearAccelEcef(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_LinearAccelEcef_Z)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_POSU));
    REQUIRE(obs->insOutputs->posU == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_PosU)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELU));
    REQUIRE(obs->insOutputs->velU == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelU)));

    REQUIRE(obs->insOutputs->insField == vn::protocol::uart::InsGroup::INSGROUP_NONE);

    // ---------------------------------------------- GpsGroup 2 -------------------------------------------------
    REQUIRE(obs->gnss2Outputs != nullptr);

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_UTC));
    REQUIRE(obs->gnss2Outputs->timeUtc.year == static_cast<int8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_UTC_year)));
    REQUIRE(obs->gnss2Outputs->timeUtc.month == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_UTC_month)));
    REQUIRE(obs->gnss2Outputs->timeUtc.day == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_UTC_day)));
    REQUIRE(obs->gnss2Outputs->timeUtc.hour == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_UTC_hour)));
    REQUIRE(obs->gnss2Outputs->timeUtc.min == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_UTC_min)));
    REQUIRE(obs->gnss2Outputs->timeUtc.sec == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_UTC_sec)));
    REQUIRE(obs->gnss2Outputs->timeUtc.ms == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_UTC_ms)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TOW));
    REQUIRE(obs->gnss2Outputs->tow == static_cast<uint64_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_Tow)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_WEEK));
    REQUIRE(obs->gnss2Outputs->week == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_Week)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS));
    REQUIRE(obs->gnss2Outputs->numSats == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_NumSats)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_FIX));
    REQUIRE(obs->gnss2Outputs->fix == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_Fix)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA));
    REQUIRE(obs->gnss2Outputs->posLla(0) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosLla_latitude)));
    REQUIRE(obs->gnss2Outputs->posLla(1) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosLla_longitude)));
    REQUIRE(obs->gnss2Outputs->posLla(2) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosLla_altitude)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF));
    REQUIRE(obs->gnss2Outputs->posEcef(0) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosEcef_X)));
    REQUIRE(obs->gnss2Outputs->posEcef(1) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosEcef_Y)));
    REQUIRE(obs->gnss2Outputs->posEcef(2) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosEcef_Z)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELNED));
    REQUIRE(obs->gnss2Outputs->velNed(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_VelNed_N)));
    REQUIRE(obs->gnss2Outputs->velNed(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_VelNed_E)));
    REQUIRE(obs->gnss2Outputs->velNed(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_VelNed_D)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF));
    REQUIRE(obs->gnss2Outputs->velEcef(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_VelEcef_X)));
    REQUIRE(obs->gnss2Outputs->velEcef(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_VelEcef_Y)));
    REQUIRE(obs->gnss2Outputs->velEcef(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_VelEcef_Z)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSU));
    REQUIRE(obs->gnss2Outputs->posU(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosU_N)));
    REQUIRE(obs->gnss2Outputs->posU(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosU_E)));
    REQUIRE(obs->gnss2Outputs->posU(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosU_D)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELU));
    REQUIRE(obs->gnss2Outputs->velU == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_VelU)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU));
    REQUIRE(obs->gnss2Outputs->timeU == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_TimeU)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO));
    REQUIRE(obs->gnss2Outputs->timeInfo.status.timeOk() == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_TimeInfo_Status_timeOk)));
    REQUIRE(obs->gnss2Outputs->timeInfo.status.dateOk() == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_TimeInfo_Status_dateOk)));
    REQUIRE(obs->gnss2Outputs->timeInfo.status.utcTimeValid() == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_TimeInfo_Status_utcTimeValid)));
    REQUIRE(obs->gnss2Outputs->timeInfo.leapSeconds == static_cast<int8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_TimeInfo_LeapSeconds)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_DOP));
    REQUIRE(obs->gnss2Outputs->dop.gDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_DOP_g)));
    REQUIRE(obs->gnss2Outputs->dop.pDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_DOP_p)));
    REQUIRE(obs->gnss2Outputs->dop.tDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_DOP_t)));
    REQUIRE(obs->gnss2Outputs->dop.vDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_DOP_v)));
    REQUIRE(obs->gnss2Outputs->dop.hDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_DOP_h)));
    REQUIRE(obs->gnss2Outputs->dop.nDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_DOP_n)));
    REQUIRE(obs->gnss2Outputs->dop.eDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_DOP_e)));

    REQUIRE(obs->gnss2Outputs->gnssField == vn::protocol::uart::GpsGroup::GPSGROUP_NONE);
}

constexpr size_t MESSAGE_COUNT_GNSS = 12; ///< Amount of messages expected in the Gnss files
size_t messageCounterGnssDataCsv = 0;     ///< Message Counter for the Gnss data csv file
size_t messageCounterGnssDataVnb = 0;     ///< Message Counter for the Gnss data vnb file

TEST_CASE("[VectorNavFile] Read 'data/vn310-gnss.csv' and compare content with hardcoded values", "[VectorNavFile]")
{
    messageCounterGnssDataCsv = 0;

    Logger logger;

    // ###########################################################################################################
    //                                     VectorNavFile-vn310-gnss-csv.flow
    // ###########################################################################################################
    //
    // VectorNavFile("data/vn310-gnss.csv")
    //
    // ###########################################################################################################

    nm::RegisterWatcherCallbackToOutputPin(7, [](const std::shared_ptr<const NAV::NodeData>& data) {
        LOG_TRACE("messageCounterGnssDataCsv = {}", messageCounterGnssDataCsv);

        compareGnssObservation(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(data), messageCounterGnssDataCsv);

        messageCounterGnssDataCsv++;
    });

    testFlow("test/flow/Nodes/DataProvider/IMU/VectorNavFile-vn310-gnss-csv.flow");

    REQUIRE(messageCounterGnssDataCsv == MESSAGE_COUNT_GNSS);
}

TEST_CASE("[VectorNavFile] Read 'data/vn310-gnss.vnb' and compare content with hardcoded values", "[VectorNavFile]")
{
    messageCounterGnssDataVnb = 0;

    Logger logger;

    // ###########################################################################################################
    //                                     VectorNavFile-vn310-gnss-vnb.flow
    // ###########################################################################################################
    //
    // VectorNavFile("data/vn310-gnss.vnb")
    //
    // ###########################################################################################################

    nm::RegisterWatcherCallbackToOutputPin(7, [](const std::shared_ptr<const NAV::NodeData>& data) {
        LOG_TRACE("messageCounterGnssDataVnb = {}", messageCounterGnssDataVnb);

        compareGnssObservation(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(data), messageCounterGnssDataVnb);

        messageCounterGnssDataVnb++;
    });

    testFlow("test/flow/Nodes/DataProvider/IMU/VectorNavFile-vn310-gnss-vnb.flow");

    REQUIRE(messageCounterGnssDataVnb == MESSAGE_COUNT_GNSS);
}

} // namespace NAV::TEST::VectorNavFile
