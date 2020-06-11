#include "EditDistPeriodicityDetector.hpp"

class MockEditDistPeriodicityDetector : public geopm::EditDistPeriodicityDetector {
    public:
        MOCK_METHOD1(update,
            void(const geopm::record_s &record));
        MOCK_METHOD0(calc_period,
            void());
        MOCK_CONST_METHOD0(get_period,
            int());
        MOCK_CONST_METHOD0(get_score,
            int());
        MOCK_METHOD0(reset,
            void());
};
