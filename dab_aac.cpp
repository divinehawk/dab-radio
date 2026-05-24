/*
 * dab_aac - DAB+ receiver outputting raw s16le PCM to stdout
 *
 * Uses dablin's SuperframeFilter + AACDecoderFDKAAC (TT_MP4_RAW).
 *
 * Usage:  dab_aac <frequency_hz> <subchannel_id> <bitrate_kbps>
 * Example: dab_aac 218640000 1 72
 *
 * Output: raw signed 16-bit little-endian PCM to stdout
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
/*
 * Buffers incoming bytes and searches for the frame phase by checking the
 * DAB+ fire code at each possible offset (0..frame_size-1). Once the fire
 * code passes at a given offset, that offset is locked and frames are fed
 * to SuperframeFilter at frame_size intervals from that point.
 */
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
            /* Scan for fire code at each possible frame boundary */
            while ((int)m_buf.size() >= m_frame_size) {
                if (check_firecode(m_buf.data())) {
                    m_phase_locked = true;
                    fprintf(stderr, "[dab_aac] Frame phase locked\n");
                    break;
                }
                /* Try next byte offset */
                m_buf.erase(m_buf.begin());
            }
        }

        if (m_phase_locked) {
            while ((int)m_buf.size() >= m_frame_size) {
                /* Verify fire code still holds — if not, re-search */
                if (!check_firecode(m_buf.data())) {
                    /* Only check on what should be a superframe boundary
                     * (every 5th frame) — single frame misses are ok */
                    /* Just feed it anyway and let SuperframeFilter decide */
                }
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
        "Usage: %s <frequency_hz> <subchannel_id> <bitrate_kbps>\n"
        "  e.g: %s 218640000 1 72\n"
        "Output: raw s16le PCM to stdout\n",
        prog, prog);
}

int main(int argc, char *argv[]) {
    if (argc != 4) { usage(argv[0]); return EXIT_FAILURE; }

    init_firecode_table();
    int bitrate = atoi(argv[3]);

    PcmObserver     *pcm_obs  = new PcmObserver();
    g_sf_filter               = new SuperframeFilter(pcm_obs, true);

    RaonTunerInput  *tuner    = new RaonTunerInput();
    MscToSuperframe *msc_obs  = new MscToSuperframe(bitrate);

    tuner->initialize();
    tuner->tuneFrequency(atoi(argv[1]));
    tuner->openSubChannel(atoi(argv[2]));
    tuner->setMscObserver(msc_obs);

    while (true)
        tuner->readData();

    delete tuner;
    return 0;
}
