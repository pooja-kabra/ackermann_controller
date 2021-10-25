 /**
 * @copyright Copyright (c) 2021
 * 
 **/

#include "supportLib.hpp"

// using namespace std;

unsigned char *DoubleArrayToByteArray(std::vector<double> *data) {
    unsigned char *out;
    size_t i;
    out = new unsigned char[data->size()];
    for (i = 0; i < data->size(); i++) {
        out[i] = data->at(i);
    }
    return out;
}

void WriteToFile(std::vector<double> *data, std::string filename) {
    unsigned char *bytes;
    bytes = DoubleArrayToByteArray(data);
    std::ofstream file(filename.c_str(), std::ios::binary);
    file.write(reinterpret_cast<char *>(bytes), data->size());
    file.close();
    delete bytes;
}

std::vector<double> *ByteArrayToDoubleArray(std::vector<unsigned char> *data) {
    std::vector<double> *out;
    size_t i;
    out = new std::vector<double>(data->size());
    for (i = 0; i < data->size(); i++) {
        out->at(i) = data->at(i);
    }
    return out;
}
