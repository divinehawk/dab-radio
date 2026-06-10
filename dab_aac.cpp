/*
 * dab_aac - DAB+ receiver outputting raw s16le PCM or ADTS AAC to stdout
 *
 * Uses dablin's SuperframeFilter + AACDecoderFDKAAC (TT_MP4_RAW).
 *
 * Usage:  dab_aac <frequency_hz> <subchannel_id> <bitrate_kbps> [-adts] [-sls <dir>]
 * Example: dab_aac 218640000 1 72
 * dab_aac 218640000 1 72 -adts -sls /tmp/slideshow
 *
 * Output: raw signed 16-bit little-endian PCM to stdout (default)
 * ADTS-framed AAC to stdout (with -adts)
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <functional>
#include <fstream>
#include <ctime>

#include "raon_tuner.h"
#include "dabplus_decoder.h"
#include "subchannel_sink.h"
#include "tools.h"

// Include the DABlin PAD and MOT decoders
#include "pad_decoder.h"

/* FEC callbacks set by main() after MscToSuperframe is constructed */
static std::function<void()> g_on_fec_failure;
static std::function<void()> g_on_fec_success;

/* Mute flag: stops audio output immediately on phase loss so ffmpeg sees
 * a clean input gap rather than repeated/stale audio. Cleared on re-lock. */
static std::atomic<bool> g_muted{false};

/* ── SLS / PAD Observer ─────────────────────────────────────────────────── */

class SlsExtractor : public PADDecoderObserver {
private:
    std::string m_output_dir;

public:
    SlsExtractor(const std::string& output_dir) : m_output_dir(output_dir) {}

    // Triggered when a new MOT slide is fully reassembled
    void PADChangeSlide(const MOT_FILE& slide) override {
        if (m_output_dir.empty()) return;
        fprintf(stderr, "[dab_aac] Image received: %s\n", slide.content_name.c_str());
        fprintf(stderr, "[dab_aac] Category: %s\n", slide.category_title.c_str());
        fprintf(stderr, "[dab_aac] Click URL: %s\n", slide.click_through_url.c_str());

        // Do not use broadcaster name
        std::string filename = "cover";
        if (slide.content_sub_type == MOT_FILE::CONTENT_SUB_TYPE_JFIF) filename += ".jpg";
            else if (slide.content_sub_type == MOT_FILE::CONTENT_SUB_TYPE_PNG) filename += ".png";

        // Basic filename sanitization to prevent directory traversal/errors
        for (char& c : filename) {
            if (c == '/' || c == '\\') c = '_';
        }

        std::string filepath = m_output_dir + "/" + filename;
        std::ofstream out(filepath, std::ios::binary);

        if (out) {
            out.write(reinterpret_cast<const char*>(slide.data.data()), slide.data.size());
            fprintf(stderr, "[dab_aac] Saved MOT slide: %s (%zu bytes)\n", filepath.c_str(), slide.data.size());
        } else {
            fprintf(stderr, "[dab_aac] Failed to save MOT slide: %s\n", filepath.c_str());
        }
    }

    // Optional: Also catch the DLS (Dynamic Label Segment) radio text
    void PADChangeDynamicLabel(const DL_STATE& dl) override {
        std::string label = CharsetTools::ConvertTextToUTF8(dl.raw.data(), dl.raw.size(), dl.charset, false, nullptr);

        std::ofstream json_file(m_output_dir + "/metadata.json");
        if (json_file.is_open()) {
            json_file << "{\n";
            json_file << "  \"dls\": \"" << label << "\",\n";
            json_file << "  \"dl_plus\": [\n";

            for (size_t i = 0; i < dl.dl_plus_objects.size(); ++i) {
                const auto& obj = dl.dl_plus_objects[i];

                json_file << "    {\n";
                json_file << "      \"tag\": \"" << DynamicLabelDecoder::ConvertDLPlusContentTypeToString(obj.content_type) << "\",\n";
                json_file << "      \"value\": \"" << obj.text << "\"\n";
                json_file << "    }" << (i == dl.dl_plus_objects.size() - 1 ? "" : ",") << "\n";
            }

            json_file << "  ]\n";
            json_file << "}\n";
            json_file.close();
        }

        fprintf(stderr, "[dab_aac] DLS: %s\n", label.c_str());
    }

    void PADFileProgress(const double fraction) override {
        // Optional: print progress for large files
        // fprintf(stderr, "[dab_aac] MOT File Progress: %.0f%%\n", fraction * 100);
    }
};

/* ── PCM observer ───────────────────────────────────────────────────────── */

class PcmObserver : public SubchannelSinkObserver {
private:
    PADDecoder* m_pad_decoder;

public:
    PcmObserver(PADDecoder* pad_decoder = nullptr) : m_pad_decoder(pad_decoder) {}

    void StartAudio(int samplerate, int channels) override {
        fprintf(stderr, "[dab_aac] PCM: %d Hz, %d ch\n", samplerate, channels);
    }
    void PutAudio(const uint8_t *data, size_t len) override {
        if (g_muted) return;
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
        if (uncorr) {
            fprintf(stderr, "[dab_aac] FEC: %d corrections, uncorrectable errors\n", c);
            if (g_on_fec_failure) g_on_fec_failure();
        } else {
            if (g_on_fec_success) g_on_fec_success();
        }
    }

    // Pass extracted PAD data to the DABlin PADDecoder
    void ProcessPAD(const uint8_t *xpad_data, size_t xpad_len, bool short_xpad, const uint8_t *fpad) override {
        if (m_pad_decoder) {
            m_pad_decoder->Process(xpad_data, xpad_len, short_xpad, fpad);
        }
    }
};


/* ── ADTS observer ──────────────────────────────────────────────────────── */
/*
 * Timestamp alignment via ADTS silence injection:
 *
 * ADTS streams have no wall-clock timestamps — ffmpeg generates PTS by
 * counting frames.  When a DAB+ superframe has uncorrectable RS errors,
 * SuperframeFilter calls FECInfo(uncorr=true) and skips ProcessUntouchedStream.
 * This creates a gap of num_aus × AU_duration (typically 3 × 40ms = 120ms)
 * in the output, causing ffmpeg's timestamp counter to fall behind real time.
 *
 * Fix: on each uncorrectable superframe, write num_aus ADTS silence frames so
 * ffmpeg's frame count (and therefore PTS) stays aligned with real time.
 *
 * The silence frames use the same ADTS SR field as the real audio frames
 * (core_sr_idx = 24kHz for HE-AAC with dac_rate=1), so ffmpeg computes all
 * frames — real and silence — as 960/24000 = 40ms each.
 *
 * AU counts mirror SuperframeFilter::CheckSync() in dabplus_decoder.cpp:
 *   dac_rate=1, sbr=1 → 3 AUs   dac_rate=1, sbr=0 → 6 AUs
 *   dac_rate=0, sbr=1 → 2 AUs   dac_rate=0, sbr=0 → 4 AUs
 */

class AdtsObserver : public SubchannelSinkObserver, public UntouchedStreamConsumer {
private:
    bool m_format_received = false;
    PADDecoder* m_pad_decoder;

    // Silence frame state — populated by FormatChange()
    bool                 m_silence_ready = false;
    int                  m_num_aus       = 3;
    std::vector<uint8_t> m_silence_adts;

    /* Build a minimal ADTS silence frame matching the current stream format.
     *
     * The ADTS SR field carries the core (pre-SBR) sample rate index so that
     * ffmpeg computes PTS using the core rate, matching ProcessUntouchedStream.
     * For HE-AAC: core_sr_idx=6 (24kHz) for dac_rate=1, 8 (16kHz) for dac_rate=0.
     *
     * Payload is a bare ID_END element (0x0B 0xC0) — the smallest syntactically
     * valid AAC raw_data_block, accepted as zero-energy audio by all decoders. */
    void buildSilenceAdts(bool sbr, bool dac_rate, bool stereo) {
        // Core SR index — matches GetCoreSrIndex() in dabplus_decoder.cpp
        // HE-AAC (sbr=1): core is half the output rate
        // AAC-LC (sbr=0): core SR equals the output rate
        int core_sr_idx = dac_rate ? (sbr ? 6 : 3) : (sbr ? 8 : 5);
        int ch_cfg      = stereo ? 2 : 1;

        // AU count — mirrors SuperframeFilter::CheckSync() exactly
        m_num_aus = dac_rate ? (sbr ? 3 : 6) : (sbr ? 2 : 4);

        // Minimal valid AAC frame: ID_END element + byte-align padding
        const uint8_t null_au[]  = { 0x0B, 0xC0 };
        const size_t  au_len     = sizeof(null_au);
        const size_t  frame_len  = 7 + au_len;

        m_silence_adts.resize(frame_len);
        uint8_t* h = m_silence_adts.data();

        h[0] = 0xFF;
        h[1] = 0xF1;  // MPEG-4, layer=0, no CRC
        h[2] = (0x01 << 6)               // profile = AAC-LC (profile_ObjectType = AOT-1)
              | (core_sr_idx << 2)        // sampling_frequency_index = core rate
              | ((ch_cfg >> 2) & 0x01);   // channel_configuration high bit
        h[3] = ((ch_cfg & 0x03) << 6)    // channel_configuration low bits
              | ((frame_len >> 11) & 0x03);
        h[4] = (frame_len >> 3) & 0xFF;
        h[5] = ((frame_len & 0x07) << 5) | 0x1F;  // buffer_fullness = 0x7FF (VBR)
        h[6] = 0xFC;                               // number_of_raw_data_blocks_in_frame = 0 (1 block)
        memcpy(h + 7, null_au, au_len);

        m_silence_ready = true;

        fprintf(stderr, "[dab_aac] Silence ADTS: %zu bytes/frame × %d AUs/superframe "
                "(core_sr_idx=%d ch=%d)\n",
                frame_len, m_num_aus, core_sr_idx, ch_cfg);
    }

public:
    AdtsObserver(PADDecoder* pad_decoder = nullptr)
        : m_pad_decoder(pad_decoder) {}

    /* Called on each uncorrectable superframe to keep ffmpeg's PTS aligned. */
    void writeSilenceSuperframe() {
        if (!m_silence_ready) return;
        for (int i = 0; i < m_num_aus; i++)
            fwrite(m_silence_adts.data(), 1, m_silence_adts.size(), stdout);
        fflush(stdout);
    }

    void FormatChange(const AUDIO_SERVICE_FORMAT& fmt) override {
        fprintf(stderr, "[dab_aac] Format: %s\n", fmt.GetSummary().c_str());
        m_format_received = true;
        bool sbr      = (fmt.codec.find("HE-AAC") != std::string::npos);
        bool dac_rate = (fmt.samplerate_khz == 48);
        bool stereo   = (fmt.mode.find("Stereo") != std::string::npos);
        buildSilenceAdts(sbr, dac_rate, stereo);
    }

    void AudioError(const std::string& hint) override {
        fprintf(stderr, "[dab_aac] Audio error: %s\n", hint.c_str());
    }
    void AudioWarning(const std::string& hint) override {
        fprintf(stderr, "[dab_aac] Audio warning: %s\n", hint.c_str());
    }

    void FECInfo(int c, bool uncorr) override {
        if (uncorr) {
            fprintf(stderr, "[dab_aac] FEC: %d corrections, uncorrectable errors\n", c);
            /* Write ADTS silence to fill the gap this failed superframe creates,
             * keeping ffmpeg's PTS counter aligned with real time. */
            writeSilenceSuperframe();
            if (g_on_fec_failure) g_on_fec_failure();
        } else {
            if (g_on_fec_success) g_on_fec_success();
        }
    }

    void ProcessUntouchedStream(const uint8_t *data, size_t len, size_t /*duration_ms*/) override {
        if (!m_format_received) return;
        fwrite(data, 1, len, stdout);
        fflush(stdout);
    }

    void ProcessPAD(const uint8_t *xpad_data, size_t xpad_len, bool short_xpad, const uint8_t *fpad) override {
        if (m_pad_decoder)
            m_pad_decoder->Process(xpad_data, xpad_len, short_xpad, fpad);
    }
};

/* ── Fire code check (from IRT decoder) ─────────────────────────────────── */
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
        , m_consec_failures(0)
    {
        fprintf(stderr, "[dab_aac] Frame size: %d bytes\n", m_frame_size);
    }

    void notifyFecFailure() {
        m_consec_failures++;
        if (m_consec_failures >= 5 && m_phase_locked) {
            fprintf(stderr, "[dab_aac] Phase lost — muting + re-scanning\n");
            m_phase_locked = false;
            m_consec_failures = 0;
            g_muted = true;
            fflush(stdout);
        }
    }

    void notifyFecSuccess() {
        m_consec_failures = 0;
    }

    void mscData(const std::vector<uint8_t>& data) override {
        m_buf.insert(m_buf.end(), data.begin(), data.end());
        const int max_buf = m_frame_size * 50;
        if ((int)m_buf.size() > max_buf)
            m_buf.erase(m_buf.begin(), m_buf.begin() + (m_buf.size() - max_buf));
        process();
    }

private:
    void process() {
        if (!m_phase_locked) {
            while ((int)m_buf.size() >= m_frame_size) {
                if (check_firecode(m_buf.data())) {
                    m_phase_locked = true;
                    m_consec_failures = 0;
                    g_muted = false;
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
    int                   m_consec_failures;
    std::vector<uint8_t>  m_buf;
};


/* ── main ────────────────────────────────────────────────────────────────── */

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s <frequency_hz> <subchannel_id> <bitrate_kbps> [-adts] [-sls <output_dir>]\n"
        "  e.g: %s 218640000 1 72\n"
        "       %s 218640000 1 72 -adts -sls /tmp/slideshow\n"
        "\n"
        "Output modes:\n"
        "  (default)  raw signed 16-bit little-endian PCM to stdout\n"
        "  -adts      ADTS-framed AAC to stdout (no AAC decoding required)\n"
        "  -sls <dir> Extract MOT Slideshow images to the specified directory\n",
        prog, prog, prog);
}

int main(int argc, char *argv[]) {
    if (argc < 4) { usage(argv[0]); return EXIT_FAILURE; }

    bool adts_mode = false;
    std::string sls_dir = "";

    for (int i = 4; i < argc; i++) {
        if (strcmp(argv[i], "-adts") == 0) {
            adts_mode = true;
        } else if (strcmp(argv[i], "-sls") == 0 && i + 1 < argc) {
            sls_dir = argv[++i];
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    init_firecode_table();
    int bitrate = atoi(argv[3]);

    SlsExtractor* sls_extractor = nullptr;
    PADDecoder* pad_decoder = nullptr;

    if (!sls_dir.empty()) {
        fprintf(stderr, "[dab_aac] SLS Extraction enabled. Saving to: %s\n", sls_dir.c_str());

        sls_extractor = new SlsExtractor(sls_dir);
        pad_decoder = new PADDecoder(sls_extractor, false); // 'false' for strict X-PAD length checking

        // CRITICAL: We must explicitly tell the PADDecoder the User Application Type for MOT.
        // In the DAB standard, 12 is the type for MOT Slideshows. Without the FIC decoder running
        // to assign this dynamically, we have to force it.
        pad_decoder->SetMOTAppType(12);
    }

    if (adts_mode) {
        AdtsObserver *adts_obs = new AdtsObserver(pad_decoder);
        g_sf_filter = new SuperframeFilter(adts_obs, false /* decode_audio */);
        g_sf_filter->AddUntouchedStreamConsumer(adts_obs);
        fprintf(stderr, "[dab_aac] Output mode: ADTS\n");
    } else {
        PcmObserver *pcm_obs = new PcmObserver(pad_decoder);
        g_sf_filter = new SuperframeFilter(pcm_obs, true /* decode_audio */);
        fprintf(stderr, "[dab_aac] Output mode: PCM\n");
    }

    RaonTunerInput  *tuner   = new RaonTunerInput();
    MscToSuperframe *msc_obs = new MscToSuperframe(bitrate);
    g_on_fec_failure = [msc_obs]{ msc_obs->notifyFecFailure(); };
    g_on_fec_success = [msc_obs]{ msc_obs->notifyFecSuccess(); };

    tuner->initialize();
    tuner->tuneFrequency(atoi(argv[1]));
    tuner->openSubChannel(atoi(argv[2]));
    tuner->setMscObserver(msc_obs);

    while (true)
        tuner->readData();

    delete tuner;
    delete pad_decoder;
    delete sls_extractor;

    return 0;
}
