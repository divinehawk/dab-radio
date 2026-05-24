/*
 * dab_aac - DAB+ receiver outputting raw s16le PCM or ADTS AAC to stdout
 *
 * Uses dablin's SuperframeFilter + AACDecoderFDKAAC (TT_MP4_RAW).
 *
 * Usage:  dab_aac <frequency_hz> <subchannel_id> <bitrate_kbps> [-adts]
 * Example: dab_aac 218640000 1 72
 *          dab_aac 218640000 1 72 -adts
 *
 * Output: raw signed 16-bit little-endian PCM to stdout (default)
 *         ADTS-framed AAC to stdout (with -adts)
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "raon_tuner.h"
#include "dabplus_decoder.h"
#include "subchannel_sink.h"
#include "tools.h"


/* ── PCM observer ───────────────────────────────────────────────────────── */

class PcmObserver : public SubchannelSinkObserver {
public:
    void StartAudio(int samplerate, int channels) override {
        fprintf(stderr, "[dab_aac] PCM: %d Hz, %d ch\n", samplerate, channels);
    }
    void PutAudio(const uint8_t *data, size_t len) override {
        fwrite(data, 1, len, stdout);
        fflush(stdout);
    }
    void FormatChange(const AUDIO_SERVICE_FORMAT& fmt) override {
        fprintf(stderr, "[dab_aac] Format: %s\n", fmt.GetSummary().c_str());
    }
    void AudioError(const std::string& hint) override {
        fprintf(stderr, "[dab_aac] Audio error: %s\n", hint.c_str());
    }
    void AudioWarning(const std::string& hint) override {
        fprintf(stderr, "[dab_aac] Audio warning: %s\n", hint.c_str());
    }
    void FECInfo(int c, bool uncorr) override {
        if (uncorr)
            fprintf(stderr, "[dab_aac] FEC: %d corrections, uncorrectable errors\n", c);
    }
};


/* ── ADTS observer ──────────────────────────────────────────────────────── */
/*
 * Implements both SubchannelSinkObserver (to receive format/error callbacks
 * from SuperframeFilter) and UntouchedStreamConsumer (to receive raw AU
 * payloads).  For each AU it prepends a 7-byte ADTS header and writes the
 * result to stdout.
 *
 * ADTS header (no-CRC variant, 7 bytes):
 *   syncword          12  0xFFF
 *   ID                 1  0 (MPEG-4)
 *   layer              2  00
 *   protection_absent  1  1 (no CRC)
 *   profile            2  01 (AAC LC — object_type minus 1)
 *   sampling_freq_idx  4  core SR index (see GetCoreSrIndex())
 *   private_bit        1  0
 *   channel_config     3  core channel config
 *   originality/copy   2  00
 *   frame_length      13  7 (header) + AU payload length in bytes
 *   buffer_fullness   11  0x7FF (VBR)
 *   aac_frame_count    2  00 (1 frame per ADTS frame)
 *
 * The core SR index (e.g. 6 → 24 kHz for a 48 kHz HE-AAC stream) is correct.
 * The SBR/PS upsampling is signalled inside the AAC bitstream; the ADTS
 * header only needs to describe the core layer.
 *
 */

class AdtsObserver : public SubchannelSinkObserver, public UntouchedStreamConsumer {
public:
    AdtsObserver() : m_format_received(false) {}

    void FormatChange(const AUDIO_SERVICE_FORMAT& fmt) override {
        fprintf(stderr, "[dab_aac] Format: %s\n", fmt.GetSummary().c_str());
        m_format_received = true;
    }

    void AudioError(const std::string& hint) override {
        fprintf(stderr, "[dab_aac] Audio error: %s\n", hint.c_str());
    }
    void AudioWarning(const std::string& hint) override {
        fprintf(stderr, "[dab_aac] Audio warning: %s\n", hint.c_str());
    }
    void FECInfo(int c, bool uncorr) override {
        if (uncorr)
            fprintf(stderr, "[dab_aac] FEC: %d corrections, uncorrectable errors\n", c);
    }

    void ProcessUntouchedStream(const uint8_t *data, size_t len, size_t /*duration_ms*/) override {
        if (!m_format_received)
            return;
        fwrite(data, 1, len, stdout);
        fflush(stdout);
    }

private:
    bool m_format_received;
};

/* ── Fire code check (from IRT decoder) ─────────────────────────────────── */
/*
 * The fire code CRC covers bytes [2..10] of the first superframe frame,
 * with the result stored in bytes [0..1].
 * We use this to find correct frame phase before feeding SuperframeFilter.
 */
static uint16_t FIRECODE_TABLE[256];

static void init_firecode_table() {
    for (int i = 0; i < 256; i++) {
        uint16_t crc = (uint16_t)(i << 8);
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x782F) : (crc << 1);
        FIRECODE_TABLE[i] = crc;
    }
}

static bool check_firecode(const uint8_t *frame) {
    if (frame[0] == 0 && frame[1] == 0)
        return false;
    uint16_t state = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t sec;
    for (int i = 4; i < 11; i++) {
        sec   = FIRECODE_TABLE[state >> 8];
        state = (uint16_t)(((sec & 0x00FF) ^ frame[i]) | ((sec ^ (state << 8)) & 0xFF00));
    }
    for (int i = 0; i < 2; i++) {
        sec   = FIRECODE_TABLE[state >> 8];
        state = (uint16_t)(((sec & 0x00FF) ^ frame[i]) | ((sec ^ (state << 8)) & 0xFF00));
    }
    return state == 0;
}

/* ── MSC observer with fire-code phase search ────────────────────────────── */

static SuperframeFilter *g_sf_filter = nullptr;

class MscToSuperframe : public MscObserver {
public:
    MscToSuperframe(int bitrate_kbps)
        : m_frame_size(bitrate_kbps * 3)
        , m_phase_locked(false)
    {
        fprintf(stderr, "[dab_aac] Frame size: %d bytes\n", m_frame_size);
    }

    void mscData(const std::vector<uint8_t>& data) override {
        m_buf.insert(m_buf.end(), data.begin(), data.end());
        process();
    }

private:
    void process() {
        if (!m_phase_locked) {
            while ((int)m_buf.size() >= m_frame_size) {
                if (check_firecode(m_buf.data())) {
                    m_phase_locked = true;
                    fprintf(stderr, "[dab_aac] Frame phase locked\n");
                    break;
                }
                m_buf.erase(m_buf.begin());
            }
        }

        if (m_phase_locked) {
            while ((int)m_buf.size() >= m_frame_size) {
                g_sf_filter->Feed(m_buf.data(), m_frame_size);
                m_buf.erase(m_buf.begin(), m_buf.begin() + m_frame_size);
            }
        }
    }

    int                   m_frame_size;
    bool                  m_phase_locked;
    std::vector<uint8_t>  m_buf;
};


/* ── main ────────────────────────────────────────────────────────────────── */

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s <frequency_hz> <subchannel_id> <bitrate_kbps> [-adts]\n"
        "  e.g: %s 218640000 1 72\n"
        "       %s 218640000 1 72 -adts\n"
        "\n"
        "Output modes:\n"
        "  (default)  raw signed 16-bit little-endian PCM to stdout\n"
        "  -adts      ADTS-framed AAC to stdout (no AAC decoding required)\n",
        prog, prog, prog);
}

int main(int argc, char *argv[]) {
    if (argc < 4 || argc > 5) { usage(argv[0]); return EXIT_FAILURE; }

    bool adts_mode = (argc == 5 && strcmp(argv[4], "-adts") == 0);
    if (argc == 5 && !adts_mode) {
        fprintf(stderr, "Unknown option: %s\n", argv[4]);
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    init_firecode_table();
    int bitrate = atoi(argv[3]);

    if (adts_mode) {
        AdtsObserver *adts_obs = new AdtsObserver();
        g_sf_filter = new SuperframeFilter(adts_obs, false /* decode_audio */);
        g_sf_filter->AddUntouchedStreamConsumer(adts_obs);
        fprintf(stderr, "[dab_aac] Output mode: ADTS\n");
    } else {
        PcmObserver *pcm_obs = new PcmObserver();
        g_sf_filter = new SuperframeFilter(pcm_obs, true /* decode_audio */);
        fprintf(stderr, "[dab_aac] Output mode: PCM\n");
    }

    RaonTunerInput  *tuner   = new RaonTunerInput();
    MscToSuperframe *msc_obs = new MscToSuperframe(bitrate);

    tuner->initialize();
    tuner->tuneFrequency(atoi(argv[1]));
    tuner->openSubChannel(atoi(argv[2]));
    tuner->setMscObserver(msc_obs);

    while (true)
        tuner->readData();

    delete tuner;
    return 0;
}
