//include "opendlv-device-radar-navigation.cpp"



uint8_t numeral_test_alpha(uint8_t expected, uint8_t input) {
    
    //Basic test
    if (input == expected) {
        return 1;
    } else {
        return 0;
    }

}

uint8_t visual_test_alpha(uint8_t expected, uint8_t input) {
    
    //Output of the visual function
    if (output == expected) {
        return 1;
    } else {
        return 0;
    }

}

uint8_t matrix_test_alpha(uint16_t expected[4], uint16_t input[4]) {
    
    //Output of the matrix sample function. 
    if (output == expected) {
        return 1;
    } else {
        return 0;
    }

}

int32_t main() {

}