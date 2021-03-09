# Copyright (C) 2021 Krister Blanch
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Builder
FROM alpine as builder
RUN apk update && \
    apk --no-cache add \
        linux-headers \
        cmake \
        g++ \
        libx11-dev \
        make



ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir docker_build && \
    cd docker_build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp/opendlv-device-radar-navigation-dest .. && \
    make && ctest --verbose && make install 


# Runtime
FROM alpine
RUN apk update && \
    apk --no-cache add \
        libx11

WORKDIR /usr/bin
COPY --from=builder /tmp/opendlv-device-radar-navigation-dest/bin/opendlv-device-radar-navigation .
ENTRYPOINT ["/usr/bin/opendlv-device-radar-navigation"]
