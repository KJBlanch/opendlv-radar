## OpenDLV microservice to provide a PPI for a Navico Radar

## Table of Contents
* [Dependencies](#dependencies)
* [Usage](#usage)
* [Docker](#docker)
* [Build from sources](#build-from-sources-on-example-ubuntu-1804-lts)
* [Demo](#run-x11-demo-on-example-ubuntu-1804-lts)
* [Commit convention](#commit-convention)
* [Branch convention](#branch-convention)
* [License](#license)


## Dependencies
The following dependecies are included and will compile with the software.  

* [libcluon](https://github.com/chrberger/libcluon) - [![License: GPLv3](https://img.shields.io/badge/license-GPL--3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0.txt)

* [Unit Test Framework Catch2](https://github.com/catchorg/Catch2/releases/tag/v2.1.2) - [![License: Boost Software License v1.0](https://img.shields.io/badge/License-Boost%20v1-blue.svg)](http://www.boost.org/LICENSE_1_0.txt)

The following dependecies are NOT included and require installation to build from source. This is only required if NOT using the docker-compose file. 

* [X11] (https://www.x.org/wiki/) - [![License: MIT]](https://opensource.org/licenses/mit-license.php)]

## Usage

It is recommended to use the self-contained docker file to build, test and run. However, demonstration of the current system requires the software built from source. This has been tested on Ubuntu 18.04 LTS. 

### Docker

Requires docker and docker compose. 

From a terminal within the working folder: 

```
docker-compose up

```
Compiling with docker will handle all dependencies as well as run the test-radar-navigation testing file, comprised of 10 unit-tests built using the catch infrastructure. 

*NOTE* Whilst the software is currently compilable with docker, there are communication bugs between docker containers, resulting in a non-functional system, aside from initialising. Therefore, for current build demonstration, it will have to be built from source and deployed locally. All testing can be done either via docker-compose or the local build. 

### Build from sources on example Ubuntu 18.04 LTS

To build this software, you need cmake, C++14 or newer, make, and X11/X11-dev. Having these preconditions, just run cmake and make as follows:

```
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release ..
make && make test && make install
```

## Run X11 Demo on example Ubuntu 18.04 LTS. 

Whilst the program itself relies upon recieving Cluon Envelopes (From OpenDLV Navico software or OpenDLV .rec file playback), it has a demonstrator mode which is self contained. By attaching the --demo flag, a second instance of the program will be able to send a dummy envelope to the first. 

Instance 1. 
```
./opendlv-device-radar-navigation --cid=111 --id=15 --name="/polar1"
```
Instance 2.
```
./opendlv-device-radar-navigation --cid=111 --id=16 --demo
```

## Commit Convention

Commit Number. Author

Fixes

Features

New/Ongoing Bugs

Compilability/Status

## Branch Convention

Main - For release (Build & testing handled by GitLab Actions) 

WIP - For versioning, and bughandling before release.

Other subranches are feature related. Optic = OpticFlow implementation

## License

* This project is released under the terms of the GNU GPLv3 License

