/*
 * dab_aac - DAB+ receiver outputting LATM/AAC to stdout
 *           with optional SLS/DLS metadata extraction
 *
 * Usage:  dab_aac <frequency_hz> <subchannel_id> <bitrate_kbps> [-sls <output_dir>]
 * Example: dab_aac 222064000 17 40
 *          dab_aac 222064000 17 40 -sls /tmp/slideshow
 */

#include "raon_tuner.h"
#include "dabplus_decoder.h"
#include <iostream>
#include <cstring>
#include <cstdlib>

DabPlusServiceComponentDecoder *dabplus_decoder;

class CoutMscObserver: public MscObserver {
    void mscData(const std::vector<uint8_t>& data) {
        dabplus_decoder->componentDataInput(data, false);
    }
};

void usage() {
    std::cout << "Usage: dab_aac frequency subchannel bitrate [-sls <output_dir>]\n"
                 "Examples:\n\n"
                 "  dab_aac 222064000 17 40              # Tune in to Capital XTRA\n"
                 "  dab_aac 222064000 17 40 -sls /tmp/s  # ...with SLS/DLS metadata\n"
                 "\n"
                 "Arguments:\n"
                 "  frequency      DAB frequency in Hz, e.g. 225648000\n"
                 "  subchannel     Subchannel ID on that frequency\n"
                 "  bitrate        DAB+ stream bitrate in kbps\n"
                 "  -sls <dir>     Extract MOT slideshow images and DLS text to <dir>\n"
                 "\n";
}

int main(int argc, char *argv[]) {
    if (argc < 4) {
        usage();
        return EXIT_FAILURE;
    }

    const char* sls_dir = nullptr;

    for (int i = 4; i < argc; i++) {
        if (strcmp(argv[i], "-sls") == 0 && i + 1 < argc) {
            sls_dir = argv[++i];
        } else {
            std::cerr << "Unknown option: " << argv[i] << "\n";
            usage();
            return EXIT_FAILURE;
        }
    }

    RaonTunerInput *tuner = new RaonTunerInput();
    CoutMscObserver *mscObserver = new CoutMscObserver();
    dabplus_decoder = new DabPlusServiceComponentDecoder();
    dabplus_decoder->setSubchannelBitrate(atoi(argv[3]));

    if (sls_dir)
        dabplus_decoder->enableSLS(sls_dir, tuner);

    tuner->initialize();
    tuner->tuneFrequency(atoi(argv[1]));
    tuner->openSubChannel(atoi(argv[2]));
    tuner->setMscObserver(mscObserver);

    while(1) {
        tuner->readData();
    }

    delete tuner;
    delete dabplus_decoder;
    return 0;
}
