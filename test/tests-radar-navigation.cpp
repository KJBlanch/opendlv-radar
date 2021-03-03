/*
 * Copyright (C) 2020  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "opendlv-device-radar-navigation.cpp"

#include <iostream>
#include <string>
#include <vector>

TEST_CASE("Test NCOMDecoder with empty payload.") {
    const std::string DATA;

    NCOMDecoder d;
    auto retVal = d.decode(DATA);

    REQUIRE(!retVal.first);
}

TEST_CASE("Test NCOMDecoder with faulty payload.") {
    const std::string DATA{"Hello World"};

    NCOMDecoder d;
    auto retVal = d.decode(DATA);

    REQUIRE(!retVal.first);
}

TEST_CASE("Test NCOMDecoder with sample payload.") {
    std::vector<uint8_t> sample{
      0xe7, 0x9c, 0x95, 0x95, 0x08, 0x00, 0x7c, 0x0e,
      0x00, 0x06, 0x81, 0xfe, 0x45, 0x00, 0x00, 0xf4,
      0x00, 0x00, 0xaa, 0xff, 0xff, 0x04, 0xc2, 0x92,
      0xf2, 0x9e, 0x60, 0x0a, 0x35, 0xf0, 0x3f, 0x46,
      0x63, 0x83, 0x3b, 0x7c, 0x96, 0xcc, 0x3f, 0x23,
      0x5a, 0xd0, 0x42, 0x32, 0x00, 0x00, 0x05, 0x00,
      0x00, 0x2c, 0x00, 0x00, 0xeb, 0xae, 0xe0, 0x00,
      0x59, 0x00, 0xbe, 0x6b, 0xff, 0xe4, 0x1d, 0x01,
      0x00, 0x00, 0x00, 0xff, 0xff, 0x01, 0xff, 0xe4
    };

    const std::string DATA(reinterpret_cast<char*>(sample.data()), sample.size());

    NCOMDecoder d;
    auto retVal = d.decode(DATA);

    REQUIRE(retVal.first);

    auto msgs = retVal.second;
    opendlv::proxy::AccelerationReading msg1 = msgs.acceleration;
    opendlv::proxy::AngularVelocityReading msg2 = msgs.angularVelocity;
    opendlv::proxy::GeodeticWgs84Reading msg3 = msgs.position;
    opendlv::proxy::GeodeticHeadingReading msg4 = msgs.heading;
    opendlv::proxy::GroundSpeedReading msg5 = msgs.speed;
    opendlv::proxy::AltitudeReading msg6 = msgs.altitude;
    opendlv::logic::sensation::Geolocation msg7 = msgs.geolocation;
    opendlv::logic::sensation::Equilibrioception msg8 = msgs.equilibrioception;
    opendlv::logic::sensation::Orientation msg9 = msgs.orientation;

    REQUIRE(0.2196999937 == Approx(msg1.accelerationX()));
    REQUIRE(0.3707999885 == Approx(msg1.accelerationY()));
    REQUIRE(-9.8042001724 == Approx(msg1.accelerationZ()));

    REQUIRE(0.00069 == Approx(msg2.angularVelocityX()));
    REQUIRE(0.00244 == Approx(msg2.angularVelocityY()));
    REQUIRE(-0.00086 == Approx(msg2.angularVelocityZ()));

    REQUIRE(58.037722605 == Approx(msg3.latitude()));
    REQUIRE(12.796579564 == Approx(msg3.longitude()));

    REQUIRE(2.1584727764 == Approx(msg4.northHeading()));

    REQUIRE(0.0000999998 == Approx(msg5.groundSpeed()));

    REQUIRE(104.176 == Approx(msg6.altitude()));

    REQUIRE(0.022784 == Approx(msgs.pitch));
    REQUIRE(-2.1103 == Approx(msgs.roll));

    REQUIRE(msg7.latitude() == Approx(msg3.latitude()));
    REQUIRE(msg7.longitude() == Approx(msg3.longitude()));
    REQUIRE(msg7.heading() == Approx(msg4.northHeading()));
    REQUIRE(msg7.altitude() == Approx(msg6.altitude()));
}

TEST_CASE("Test NCOMDecoder with sample payload and channel 0 for time stamp.") {
    std::vector<uint8_t> sample{
        0xe7, 0xfd, 0x55, 0x1a, 0x0b, 0x00, 0x9e, 0x04,
        0x00, 0xb5, 0x87, 0xfe, 0x7d, 0xfe, 0xff, 0x91,
        0x00, 0x00, 0xcb, 0xfd, 0xff, 0x02, 0x27, 0xf3,
        0x4d, 0xfb, 0x1f, 0xbf, 0xef, 0xe4, 0x3f, 0xa2,
        0x58, 0x61, 0x3e, 0x9b, 0x10, 0x01, 0xc0, 0x2b,
        0x9d, 0x9f, 0x3f, 0x64, 0xff, 0xff, 0x95, 0x00,
        0x00, 0x9a, 0xfe, 0xff, 0x00, 0x00, 0x80, 0x00,
        0x00, 0x80, 0x00, 0x00, 0x80, 0x32, 0x00, 0x64,
        0x3f, 0x2f, 0x01, 0x0f, 0x03, 0x02, 0xff, 0x4a
    };

    const std::string DATA(reinterpret_cast<char*>(sample.data()), sample.size());

    NCOMDecoder d;
    auto retVal = d.decode(DATA);

    REQUIRE(retVal.first);

    auto msgs = retVal.second;

    REQUIRE(1508382964 == msgs.sampleTime.seconds());
    REQUIRE(13000 == msgs.sampleTime.microseconds());
}
