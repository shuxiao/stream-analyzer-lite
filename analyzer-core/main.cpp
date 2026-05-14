#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/motion_vector.h>
#include <libswscale/swscale.h>
}

#include <set>
#include <unordered_map>
#include <unordered_map>

static const char b64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static std::string base64_encode(const uint8_t* data, int len) {
    std::string out;
    out.reserve((len + 2) / 3 * 4);
    for (int i = 0; i < len; i += 3) {
        uint32_t v = data[i] << 16;
        if (i + 1 < len) v |= data[i+1] << 8;
        if (i + 2 < len) v |= data[i+2];
        out += b64_table[(v >> 18) & 0x3F];
        out += b64_table[(v >> 12) & 0x3F];
        out += (i + 1 < len) ? b64_table[(v >> 6) & 0x3F] : '=';
        out += (i + 2 < len) ? b64_table[v & 0x3F] : '=';
    }
    return out;
}

// Encode AVFrame to JPEG in memory using the MJPEG encoder
static std::string encode_jpeg(AVFrame* frame, int width, int height, int quality = 5) {
    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
    if (!codec) return {};
    AVCodecContext* enc = avcodec_alloc_context3(codec);
    enc->width = width; enc->height = height;
    enc->pix_fmt = AV_PIX_FMT_YUVJ420P;
    enc->time_base = {1, 25};
    enc->flags |= AV_CODEC_FLAG_QSCALE;
    enc->global_quality = quality * FF_QP2LAMBDA;
    if (avcodec_open2(enc, codec, nullptr) < 0) { avcodec_free_context(&enc); return {}; }

    // Convert to YUVJ420P if needed
    AVFrame* dst = av_frame_alloc();
    dst->format = AV_PIX_FMT_YUVJ420P; dst->width = width; dst->height = height;
    av_frame_get_buffer(dst, 0);
    SwsContext* sws = sws_getContext(frame->width, frame->height, (AVPixelFormat)frame->format,
                                     width, height, AV_PIX_FMT_YUVJ420P, SWS_BILINEAR, nullptr, nullptr, nullptr);
    sws_scale(sws, frame->data, frame->linesize, 0, frame->height, dst->data, dst->linesize);
    sws_freeContext(sws);

    AVPacket* pkt = av_packet_alloc();
    avcodec_send_frame(enc, dst);
    std::string result;
    if (avcodec_receive_packet(enc, pkt) == 0)
        result = base64_encode(pkt->data, pkt->size);
    av_packet_free(&pkt);
    av_frame_free(&dst);
    avcodec_free_context(&enc);
    return result;
}

// Extract reference frame indices from motion vectors + DPB tracking
// ref_i_p: list of I/P frame indices in decode order (the DPB)
static std::set<int> get_ref_frames(AVFrame* frame,
                                     const std::vector<int>& ref_i_p,
                                     int cur_display_idx) {
    std::set<int> refs;
    auto* sd = av_frame_get_side_data(frame, AV_FRAME_DATA_MOTION_VECTORS);
    if (!sd || ref_i_p.empty()) return refs;
    int nmvs = sd->size / sizeof(AVMotionVector);
    auto* mvs = (const AVMotionVector*)sd->data;

    // Collect unique source directions
    bool has_past = false, has_future = false;
    for (int i = 0; i < nmvs; i++) {
        if (mvs[i].source < 0) has_past = true;
        if (mvs[i].source > 0) has_future = true;
    }

    // Find nearest past and future reference frames in display order
    // from the DPB (which contains I/P frames in decode order)
    if (has_past) {
        // Nearest I/P frame before current in display order
        int best = -1;
        for (int r : ref_i_p)
            if (r < cur_display_idx && (best < 0 || r > best)) best = r;
        if (best >= 0) refs.insert(best);
    }
    if (has_future) {
        // Nearest I/P frame after current in display order
        int best = -1;
        for (int r : ref_i_p)
            if (r > cur_display_idx && (best < 0 || r < best)) best = r;
        if (best >= 0) refs.insert(best);
    }
    return refs;
}

static bool g_thumbnails = false;
static int g_thumb_height = 60;
static int g_range_start = -1; // -1 means no range filter
static int g_range_count = -1;

// Simple bitstream reader for exp-golomb parsing
struct BitReader {
    std::vector<uint8_t> rbsp; // owned RBSP data (emulation prevention removed)
    int size;
    int bit_pos;

    // Build BitReader from raw NAL data, stripping emulation prevention bytes
    BitReader(const uint8_t* d, int s) : bit_pos(0) {
        rbsp.reserve(s);
        for (int i = 0; i < s; i++) {
            if (i + 2 < s && d[i] == 0 && d[i+1] == 0 && d[i+2] == 3) {
                rbsp.push_back(0); rbsp.push_back(0);
                i += 2; // skip the 0x03 byte
            } else {
                rbsp.push_back(d[i]);
            }
        }
        size = (int)rbsp.size();
    }

    int available() const { return size * 8 - bit_pos; }

    uint32_t read_bits(int n) {
        uint32_t val = 0;
        for (int i = 0; i < n && bit_pos < size * 8; i++) {
            val = (val << 1) | ((rbsp[bit_pos / 8] >> (7 - bit_pos % 8)) & 1);
            bit_pos++;
        }
        return val;
    }

    uint64_t read_bits64(int n) {
        uint64_t val = 0;
        for (int i = 0; i < n && bit_pos < size * 8; i++) {
            val = (val << 1) | ((rbsp[bit_pos / 8] >> (7 - bit_pos % 8)) & 1);
            bit_pos++;
        }
        return val;
    }

    uint32_t read_ue() {
        int zeros = 0;
        while (bit_pos < size * 8 && read_bits(1) == 0) zeros++;
        return zeros ? ((1 << zeros) - 1 + read_bits(zeros)) : 0;
    }

    int32_t read_se() {
        uint32_t v = read_ue();
        return (v & 1) ? (int32_t)((v + 1) / 2) : -(int32_t)(v / 2);
    }

    uint32_t read_u1() { return read_bits(1); }

    void skip_bits(int n) { read_bits(n); }

    bool bit_at(int pos) const {
        if (pos < 0 || pos >= size * 8) return false;
        return ((rbsp[pos / 8] >> (7 - pos % 8)) & 1) != 0;
    }

    bool has_more_rbsp_data() const {
        if (bit_pos >= size * 8) return false;
        if (!bit_at(bit_pos)) return true;
        for (int pos = bit_pos + 1; pos < size * 8; pos++) {
            if (bit_at(pos)) return true;
        }
        return false;
    }
};

static std::string json_escape(const std::string& input) {
    std::string out;
    out.reserve(input.size() + 8);
    for (unsigned char c : input) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {
                    char buf[7];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += static_cast<char>(c);
                }
        }
    }
    return out;
}

struct JsonObjectBuilder {
    std::string out{"{"};
    bool first = true;

    void key(const std::string& name) {
        if (!first) out += ",";
        first = false;
        out += "\"";
        out += json_escape(name);
        out += "\":";
    }

    void add_uint(const std::string& name, uint64_t value) {
        key(name);
        out += std::to_string(value);
    }

    void add_int(const std::string& name, int64_t value) {
        key(name);
        out += std::to_string(value);
    }

    void add_bool(const std::string& name, bool value) {
        key(name);
        out += value ? "true" : "false";
    }

    void add_string(const std::string& name, const std::string& value) {
        key(name);
        out += "\"";
        out += json_escape(value);
        out += "\"";
    }

    void add_raw(const std::string& name, const std::string& raw_json) {
        key(name);
        out += raw_json.empty() ? "null" : raw_json;
    }

    std::string str() const {
        return out + "}";
    }
};

static std::string json_array_from_uints(const std::vector<uint32_t>& values) {
    std::string out = "[";
    for (size_t i = 0; i < values.size(); i++) {
        if (i) out += ",";
        out += std::to_string(values[i]);
    }
    out += "]";
    return out;
}

static std::string json_array_from_ints(const std::vector<int32_t>& values) {
    std::string out = "[";
    for (size_t i = 0; i < values.size(); i++) {
        if (i) out += ",";
        out += std::to_string(values[i]);
    }
    out += "]";
    return out;
}

static std::string json_array_from_bools(const std::vector<int>& values) {
    std::string out = "[";
    for (size_t i = 0; i < values.size(); i++) {
        if (i) out += ",";
        out += values[i] ? "true" : "false";
    }
    out += "]";
    return out;
}

static std::string json_array_from_raw(const std::vector<std::string>& values) {
    std::string out = "[";
    for (size_t i = 0; i < values.size(); i++) {
        if (i) out += ",";
        out += values[i];
    }
    out += "]";
    return out;
}

struct AVSyncPoint {
    int index = 0;
    int64_t pts = AV_NOPTS_VALUE;
    int64_t dts = AV_NOPTS_VALUE;
    double time_ms = NAN;
    double decode_time_ms = NAN;
};

static double timestamp_to_ms(int64_t ts, AVRational tb) {
    if (ts == AV_NOPTS_VALUE || tb.den == 0) return NAN;
    return (double)ts * (double)tb.num * 1000.0 / (double)tb.den;
}

static double stream_duration_ms(const AVStream* stream) {
    if (!stream) return NAN;
    return timestamp_to_ms(stream->duration, stream->time_base);
}

static int64_t packet_timeline_ts(const AVPacket& pkt) {
    return pkt.pts != AV_NOPTS_VALUE ? pkt.pts : pkt.dts;
}

static int64_t packet_decode_ts(const AVPacket& pkt) {
    return pkt.dts != AV_NOPTS_VALUE ? pkt.dts : pkt.pts;
}

static void print_json_int64_or_null(int64_t value) {
    if (value == AV_NOPTS_VALUE) printf("null");
    else printf("%lld", (long long)value);
}

static void print_json_double_or_null(double value) {
    if (!std::isfinite(value)) printf("null");
    else printf("%.3f", value);
}

static void print_stream_meta_json_or_null(const AVStream* stream) {
    if (!stream) {
        printf("null");
        return;
    }
    std::string codec = json_escape(avcodec_get_name(stream->codecpar->codec_id));
    printf("{\"codec\":\"%s\",\"time_base_num\":%d,\"time_base_den\":%d}",
           codec.c_str(), stream->time_base.num, stream->time_base.den);
}

static void print_avsync_points_json(const std::vector<AVSyncPoint>& points) {
    printf("[");
    for (size_t i = 0; i < points.size(); i++) {
        const auto& point = points[i];
        if (i > 0) printf(",");
        printf("{\"index\":%d,\"pts\":", point.index);
        print_json_int64_or_null(point.pts);
        printf(",\"dts\":");
        print_json_int64_or_null(point.dts);
        printf(",\"time_ms\":");
        print_json_double_or_null(point.time_ms);
        printf(",\"decode_time_ms\":");
        print_json_double_or_null(point.decode_time_ms);
        printf("}");
    }
    printf("]");
}

static std::string hex_string(uint64_t value, int width = 0) {
    std::ostringstream oss;
    oss << "0x" << std::hex << std::uppercase << std::setfill('0');
    if (width > 0) oss << std::setw(width);
    oss << value;
    return oss.str();
}

static int ceil_log2_u32(uint32_t value) {
    if (value <= 1) return 0;
    int bits = 0;
    uint32_t x = 1;
    while (x < value) {
        x <<= 1;
        bits++;
    }
    return bits;
}

static std::vector<int32_t> parse_h264_scaling_list(BitReader& br, int size, bool& use_default) {
    std::vector<int32_t> values;
    values.reserve(size);
    int last_scale = 8;
    int next_scale = 8;
    use_default = false;
    for (int j = 0; j < size; j++) {
        if (next_scale != 0) {
            int delta_scale = br.read_se();
            next_scale = (last_scale + delta_scale + 256) % 256;
            if (j == 0 && next_scale == 0) use_default = true;
        }
        int scale = (next_scale == 0) ? last_scale : next_scale;
        values.push_back(scale);
        last_scale = scale;
    }
    return values;
}

struct HevcHrdCommonInfo {
    bool valid = false;
    bool nal_hrd_parameters_present_flag = false;
    bool vcl_hrd_parameters_present_flag = false;
    bool sub_pic_hrd_params_present_flag = false;
};

static std::string parse_hevc_profile_tier_level_json(BitReader& br, bool profile_present, int max_sub_layers) {
    JsonObjectBuilder obj;

    if (profile_present) {
        uint32_t general_profile_space = br.read_bits(2);
        bool general_tier_flag = br.read_u1() != 0;
        uint32_t general_profile_idc = br.read_bits(5);
        uint32_t general_profile_compatibility_flags = br.read_bits(32);
        uint64_t general_constraint_indicator_flags = br.read_bits64(48);
        obj.add_uint("general_profile_space", general_profile_space);
        obj.add_bool("general_tier_flag", general_tier_flag);
        obj.add_uint("general_profile_idc", general_profile_idc);
        obj.add_string("general_profile_compatibility_flags_hex", hex_string(general_profile_compatibility_flags, 8));
        obj.add_string("general_constraint_indicator_flags_hex", hex_string(general_constraint_indicator_flags, 12));
    }

    uint32_t general_level_idc = br.read_bits(8);
    obj.add_uint("general_level_idc", general_level_idc);

    std::vector<int> sub_layer_profile_present_flags;
    std::vector<int> sub_layer_level_present_flags;
    for (int i = 0; i < max_sub_layers - 1; i++) {
        sub_layer_profile_present_flags.push_back(br.read_u1() ? 1 : 0);
        sub_layer_level_present_flags.push_back(br.read_u1() ? 1 : 0);
    }
    if (max_sub_layers > 1) {
        for (int i = max_sub_layers - 1; i < 8; i++) br.skip_bits(2);
    }

    std::vector<std::string> sub_layers;
    for (int i = 0; i < max_sub_layers - 1; i++) {
        JsonObjectBuilder sub;
        sub.add_bool("profile_present_flag", sub_layer_profile_present_flags[i] != 0);
        sub.add_bool("level_present_flag", sub_layer_level_present_flags[i] != 0);
        if (sub_layer_profile_present_flags[i]) {
            uint32_t profile_space = br.read_bits(2);
            bool tier_flag = br.read_u1() != 0;
            uint32_t profile_idc = br.read_bits(5);
            uint32_t compatibility_flags = br.read_bits(32);
            uint64_t constraint_flags = br.read_bits64(48);
            sub.add_uint("profile_space", profile_space);
            sub.add_bool("tier_flag", tier_flag);
            sub.add_uint("profile_idc", profile_idc);
            sub.add_string("profile_compatibility_flags_hex", hex_string(compatibility_flags, 8));
            sub.add_string("constraint_indicator_flags_hex", hex_string(constraint_flags, 12));
        }
        if (sub_layer_level_present_flags[i]) {
            sub.add_uint("level_idc", br.read_bits(8));
        }
        sub_layers.push_back(sub.str());
    }

    if (!sub_layer_profile_present_flags.empty()) {
        obj.add_raw("sub_layer_profile_present_flags", json_array_from_bools(sub_layer_profile_present_flags));
        obj.add_raw("sub_layer_level_present_flags", json_array_from_bools(sub_layer_level_present_flags));
        obj.add_raw("sub_layers", json_array_from_raw(sub_layers));
    }

    return obj.str();
}

static void skip_hevc_scaling_list_data(BitReader& br) {
    for (int size_id = 0; size_id < 4; size_id++) {
        int matrix_count = (size_id == 3) ? 2 : 6;
        for (int matrix_id = 0; matrix_id < matrix_count; matrix_id++) {
            bool scaling_list_pred_mode_flag = br.read_u1() != 0;
            if (!scaling_list_pred_mode_flag) {
                br.read_ue();
            } else {
                int coef_num = std::min(64, 1 << (4 + (size_id << 1)));
                if (size_id > 1) br.read_se();
                for (int i = 0; i < coef_num; i++) br.read_se();
            }
        }
    }
}

static HevcHrdCommonInfo skip_hevc_hrd_parameters(BitReader& br, bool common_inf_present_flag, int max_num_sub_layers_minus1) {
    HevcHrdCommonInfo info;
    if (common_inf_present_flag) {
        info.valid = true;
        info.nal_hrd_parameters_present_flag = br.read_u1() != 0;
        info.vcl_hrd_parameters_present_flag = br.read_u1() != 0;
        if (info.nal_hrd_parameters_present_flag || info.vcl_hrd_parameters_present_flag) {
            info.sub_pic_hrd_params_present_flag = br.read_u1() != 0;
            if (info.sub_pic_hrd_params_present_flag) {
                br.skip_bits(8);
                br.skip_bits(5);
                br.read_u1();
                br.skip_bits(5);
            }
            br.skip_bits(4);
            br.skip_bits(4);
            if (info.sub_pic_hrd_params_present_flag) br.skip_bits(4);
            br.skip_bits(5);
            br.skip_bits(5);
            br.skip_bits(5);
        }
    }

    for (int i = 0; i <= max_num_sub_layers_minus1; i++) {
        bool fixed_pic_rate_general_flag = br.read_u1() != 0;
        bool fixed_pic_rate_within_cvs_flag = fixed_pic_rate_general_flag ? true : (br.read_u1() != 0);
        bool low_delay_hrd_flag = false;
        uint32_t cpb_cnt_minus1 = 0;
        if (fixed_pic_rate_within_cvs_flag) {
            br.read_ue();
        } else {
            low_delay_hrd_flag = br.read_u1() != 0;
        }
        if (!low_delay_hrd_flag) cpb_cnt_minus1 = br.read_ue();
        auto skip_sub_layer_hrd = [&](bool sub_pic_present) {
            for (uint32_t j = 0; j <= cpb_cnt_minus1; j++) {
                br.read_ue();
                br.read_ue();
                if (sub_pic_present) {
                    br.read_ue();
                    br.read_ue();
                }
                br.read_u1();
            }
        };
        if (info.nal_hrd_parameters_present_flag) skip_sub_layer_hrd(info.sub_pic_hrd_params_present_flag);
        if (info.vcl_hrd_parameters_present_flag) skip_sub_layer_hrd(info.sub_pic_hrd_params_present_flag);
    }

    return info;
}

static bool g_is_hevc = false; // true if H265/HEVC stream

static const char* nal_type_name_h264(int t) {
    switch (t) {
        case 1: return "Slice (non-IDR)";
        case 2: return "Slice Data A";
        case 3: return "Slice Data B";
        case 4: return "Slice Data C";
        case 5: return "Slice (IDR)";
        case 6: return "SEI";
        case 7: return "SPS";
        case 8: return "PPS";
        case 9: return "AUD";
        case 10: return "End of Sequence";
        case 11: return "End of Stream";
        case 12: return "Filler";
        default: return "Other";
    }
}

static const char* nal_type_name_hevc(int t) {
    switch (t) {
        case 0: case 1: return "Slice (TRAIL)";
        case 2: case 3: return "Slice (TSA)";
        case 4: case 5: return "Slice (STSA)";
        case 6: case 7: return "Slice (RADL)";
        case 8: case 9: return "Slice (RASL)";
        case 16: case 17: case 18: return "Slice (BLA)";
        case 19: case 20: return "Slice (IDR)";
        case 21: return "Slice (CRA)";
        case 32: return "VPS";
        case 33: return "SPS";
        case 34: return "PPS";
        case 35: return "AUD";
        case 36: return "EOS";
        case 37: return "EOB";
        case 38: return "Filler";
        case 39: case 40: return "SEI";
        default: return "Other";
    }
}

static const char* nal_type_name(int t) {
    return g_is_hevc ? nal_type_name_hevc(t) : nal_type_name_h264(t);
}

static const char* slice_type_name(int t) {
    switch (t % 5) {
        case 0: return "P";
        case 1: return "B";
        case 2: return "I";
        case 3: return "SP";
        case 4: return "SI";
        default: return "?";
    }
}

static void print_json_string(const char* s) {
    putchar('"');
    for (; *s; s++) {
        if (*s == '"') printf("\\\"");
        else if (*s == '\\') printf("\\\\");
        else putchar(*s);
    }
    putchar('"');
}

struct NALUnit {
    const uint8_t* data;
    int size;
    int type;
};

static int g_nal_length_size = 4; // AVCC NAL length prefix size, default 4

static inline int extract_nal_type(const uint8_t* data) {
    return g_is_hevc ? ((data[0] >> 1) & 0x3F) : (data[0] & 0x1F);
}

static std::vector<NALUnit> parse_nalus_annexb(const uint8_t* data, int size) {
    std::vector<NALUnit> nalus;
    int i = 0;
    while (i < size) {
        int sc_len = 0;
        if (i + 3 <= size && data[i] == 0 && data[i+1] == 0 && data[i+2] == 1)
            sc_len = 3;
        else if (i + 4 <= size && data[i] == 0 && data[i+1] == 0 && data[i+2] == 0 && data[i+3] == 1)
            sc_len = 4;
        if (sc_len == 0) { i++; continue; }
        int start = i + sc_len;
        int j = start;
        while (j + 2 < size) {
            if (data[j] == 0 && data[j+1] == 0 && (data[j+2] == 1 || (j + 3 < size && data[j+2] == 0 && data[j+3] == 1)))
                break;
            j++;
        }
        if (j + 2 >= size) j = size;
        if (j - start > 0) {
            NALUnit n; n.data = data + start; n.size = j - start;
            n.type = extract_nal_type(n.data);
            nalus.push_back(n);
        }
        i = j;
    }
    return nalus;
}

static std::vector<NALUnit> parse_nalus_avcc(const uint8_t* data, int size) {
    std::vector<NALUnit> nalus;
    int i = 0;
    while (i + g_nal_length_size <= size) {
        uint32_t len = 0;
        for (int k = 0; k < g_nal_length_size; k++)
            len = (len << 8) | data[i + k];
        i += g_nal_length_size;
        if (len == 0 || i + (int)len > size) break;
        NALUnit n; n.data = data + i; n.size = len;
        n.type = extract_nal_type(n.data);
        nalus.push_back(n);
        i += len;
    }
    return nalus;
}

static bool g_is_avcc = false;

static std::vector<NALUnit> parse_nalus(const uint8_t* data, int size) {
    return g_is_avcc ? parse_nalus_avcc(data, size) : parse_nalus_annexb(data, size);
}

static void print_sps(const uint8_t* data, int size) {
    if (size < 4) {
        printf("{\"body_length\":%d}", size);
        return;
    }

    BitReader br(data + 1, size - 1); // skip NAL header byte
    JsonObjectBuilder obj;

    uint32_t profile_idc = br.read_bits(8);
    uint32_t constraint_flags = br.read_bits(8);
    uint32_t level_idc = br.read_bits(8);
    uint32_t sps_id = br.read_ue();

    obj.add_uint("profile_idc", profile_idc);
    obj.add_string("constraint_flags_hex", hex_string(constraint_flags, 2));
    obj.add_bool("constraint_set0_flag", (constraint_flags & 0x80) != 0);
    obj.add_bool("constraint_set1_flag", (constraint_flags & 0x40) != 0);
    obj.add_bool("constraint_set2_flag", (constraint_flags & 0x20) != 0);
    obj.add_bool("constraint_set3_flag", (constraint_flags & 0x10) != 0);
    obj.add_bool("constraint_set4_flag", (constraint_flags & 0x08) != 0);
    obj.add_bool("constraint_set5_flag", (constraint_flags & 0x04) != 0);
    obj.add_uint("level_idc", level_idc);
    obj.add_uint("sps_id", sps_id);

    uint32_t chroma_format_idc = 1;
    bool separate_colour_plane_flag = false;
    uint32_t bit_depth_luma_minus8 = 0;
    uint32_t bit_depth_chroma_minus8 = 0;
    bool qpprime_y_zero_transform_bypass_flag = false;
    bool seq_scaling_matrix_present_flag = false;
    std::vector<int> seq_scaling_list_present_flags;
    std::vector<std::string> scaling_list_entries;

    if (profile_idc == 100 || profile_idc == 110 || profile_idc == 122 ||
        profile_idc == 244 || profile_idc == 44 || profile_idc == 83 ||
        profile_idc == 86 || profile_idc == 118 || profile_idc == 128 ||
        profile_idc == 138 || profile_idc == 139 || profile_idc == 134 ||
        profile_idc == 135) {
        chroma_format_idc = br.read_ue();
        obj.add_uint("chroma_format_idc", chroma_format_idc);
        if (chroma_format_idc == 3) {
            separate_colour_plane_flag = br.read_u1() != 0;
            obj.add_bool("separate_colour_plane_flag", separate_colour_plane_flag);
        }
        bit_depth_luma_minus8 = br.read_ue();
        bit_depth_chroma_minus8 = br.read_ue();
        obj.add_uint("bit_depth_luma_minus8", bit_depth_luma_minus8);
        obj.add_uint("bit_depth_chroma_minus8", bit_depth_chroma_minus8);
        obj.add_uint("bit_depth_luma", bit_depth_luma_minus8 + 8);
        obj.add_uint("bit_depth_chroma", bit_depth_chroma_minus8 + 8);
        qpprime_y_zero_transform_bypass_flag = br.read_u1() != 0;
        obj.add_bool("qpprime_y_zero_transform_bypass_flag", qpprime_y_zero_transform_bypass_flag);
        seq_scaling_matrix_present_flag = br.read_u1() != 0;
        obj.add_bool("seq_scaling_matrix_present_flag", seq_scaling_matrix_present_flag);
        if (seq_scaling_matrix_present_flag) {
            int cnt = (chroma_format_idc != 3) ? 8 : 12;
            for (int i = 0; i < cnt; i++) {
                bool present = br.read_u1() != 0;
                seq_scaling_list_present_flags.push_back(present ? 1 : 0);
                if (present) {
                    bool use_default = false;
                    int sz = (i < 6) ? 16 : 64;
                    auto values = parse_h264_scaling_list(br, sz, use_default);
                    JsonObjectBuilder scaling;
                    scaling.add_string("kind", i < 6 ? "4x4" : "8x8");
                    scaling.add_uint("index", i);
                    scaling.add_bool("use_default_scaling_matrix_flag", use_default);
                    scaling.add_raw("values", json_array_from_ints(values));
                    scaling_list_entries.push_back(scaling.str());
                } else {
                    scaling_list_entries.push_back("null");
                }
            }
        }
    } else {
        obj.add_uint("chroma_format_idc", chroma_format_idc);
    }

    uint32_t log2_max_frame_num_minus4 = br.read_ue();
    uint32_t log2_max_frame_num = log2_max_frame_num_minus4 + 4;
    uint32_t pic_order_cnt_type = br.read_ue();
    obj.add_uint("log2_max_frame_num_minus4", log2_max_frame_num_minus4);
    obj.add_uint("log2_max_frame_num", log2_max_frame_num);
    obj.add_uint("pic_order_cnt_type", pic_order_cnt_type);

    if (pic_order_cnt_type == 0) {
        uint32_t log2_max_pic_order_cnt_lsb_minus4 = br.read_ue();
        obj.add_uint("log2_max_pic_order_cnt_lsb_minus4", log2_max_pic_order_cnt_lsb_minus4);
        obj.add_uint("log2_max_pic_order_cnt_lsb", log2_max_pic_order_cnt_lsb_minus4 + 4);
    } else if (pic_order_cnt_type == 1) {
        bool delta_pic_order_always_zero_flag = br.read_u1() != 0;
        int32_t offset_for_non_ref_pic = br.read_se();
        int32_t offset_for_top_to_bottom_field = br.read_se();
        uint32_t num_ref_frames_in_pic_order_cnt_cycle = br.read_ue();
        std::vector<int32_t> offset_for_ref_frame;
        for (uint32_t i = 0; i < num_ref_frames_in_pic_order_cnt_cycle; i++) {
            offset_for_ref_frame.push_back(br.read_se());
        }
        obj.add_bool("delta_pic_order_always_zero_flag", delta_pic_order_always_zero_flag);
        obj.add_int("offset_for_non_ref_pic", offset_for_non_ref_pic);
        obj.add_int("offset_for_top_to_bottom_field", offset_for_top_to_bottom_field);
        obj.add_uint("num_ref_frames_in_pic_order_cnt_cycle", num_ref_frames_in_pic_order_cnt_cycle);
        obj.add_raw("offset_for_ref_frame", json_array_from_ints(offset_for_ref_frame));
    }

    uint32_t max_num_ref_frames = br.read_ue();
    bool gaps_in_frame_num_value_allowed_flag = br.read_u1() != 0;
    uint32_t pic_width_in_mbs_minus1 = br.read_ue();
    uint32_t pic_height_in_map_units_minus1 = br.read_ue();
    bool frame_mbs_only_flag = br.read_u1() != 0;
    bool mb_adaptive_frame_field_flag = false;
    if (!frame_mbs_only_flag) mb_adaptive_frame_field_flag = br.read_u1() != 0;
    bool direct_8x8_inference_flag = br.read_u1() != 0;
    bool frame_cropping_flag = br.read_u1() != 0;
    uint32_t frame_crop_left_offset = 0;
    uint32_t frame_crop_right_offset = 0;
    uint32_t frame_crop_top_offset = 0;
    uint32_t frame_crop_bottom_offset = 0;
    if (frame_cropping_flag) {
        frame_crop_left_offset = br.read_ue();
        frame_crop_right_offset = br.read_ue();
        frame_crop_top_offset = br.read_ue();
        frame_crop_bottom_offset = br.read_ue();
    }

    bool vui_parameters_present_flag = br.read_u1() != 0;

    uint32_t crop_unit_x = 1;
    uint32_t crop_unit_y = 2 - (frame_mbs_only_flag ? 1 : 0);
    if (chroma_format_idc == 1) {
        crop_unit_x = 2;
        crop_unit_y = 2 * (2 - (frame_mbs_only_flag ? 1 : 0));
    } else if (chroma_format_idc == 2) {
        crop_unit_x = 2;
        crop_unit_y = 1 * (2 - (frame_mbs_only_flag ? 1 : 0));
    } else if (chroma_format_idc == 3) {
        crop_unit_x = 1;
        crop_unit_y = 1 * (2 - (frame_mbs_only_flag ? 1 : 0));
    }

    uint32_t coded_width = (pic_width_in_mbs_minus1 + 1) * 16;
    uint32_t coded_height = (pic_height_in_map_units_minus1 + 1) * 16 * (2 - (frame_mbs_only_flag ? 1 : 0));
    uint32_t width = coded_width;
    uint32_t height = coded_height;
    if (frame_cropping_flag) {
        uint32_t crop_w = (frame_crop_left_offset + frame_crop_right_offset) * crop_unit_x;
        uint32_t crop_h = (frame_crop_top_offset + frame_crop_bottom_offset) * crop_unit_y;
        width = (coded_width > crop_w) ? (coded_width - crop_w) : 0;
        height = (coded_height > crop_h) ? (coded_height - crop_h) : 0;
    }

    obj.add_uint("max_num_ref_frames", max_num_ref_frames);
    obj.add_bool("gaps_in_frame_num_value_allowed_flag", gaps_in_frame_num_value_allowed_flag);
    obj.add_uint("pic_width_in_mbs_minus1", pic_width_in_mbs_minus1);
    obj.add_uint("pic_width_in_mbs", pic_width_in_mbs_minus1 + 1);
    obj.add_uint("pic_height_in_map_units_minus1", pic_height_in_map_units_minus1);
    obj.add_uint("pic_height_in_map_units", pic_height_in_map_units_minus1 + 1);
    obj.add_bool("frame_mbs_only_flag", frame_mbs_only_flag);
    if (!frame_mbs_only_flag) obj.add_bool("mb_adaptive_frame_field_flag", mb_adaptive_frame_field_flag);
    obj.add_bool("direct_8x8_inference_flag", direct_8x8_inference_flag);
    obj.add_bool("frame_cropping_flag", frame_cropping_flag);
    if (frame_cropping_flag) {
        obj.add_uint("frame_crop_left_offset", frame_crop_left_offset);
        obj.add_uint("frame_crop_right_offset", frame_crop_right_offset);
        obj.add_uint("frame_crop_top_offset", frame_crop_top_offset);
        obj.add_uint("frame_crop_bottom_offset", frame_crop_bottom_offset);
        obj.add_uint("crop_unit_x", crop_unit_x);
        obj.add_uint("crop_unit_y", crop_unit_y);
    }
    obj.add_bool("vui_parameters_present_flag", vui_parameters_present_flag);
    obj.add_uint("coded_width", coded_width);
    obj.add_uint("coded_height", coded_height);
    obj.add_uint("width", width);
    obj.add_uint("height", height);
    obj.add_uint("body_length", size);

    if (!seq_scaling_list_present_flags.empty()) {
        obj.add_raw("seq_scaling_list_present_flags", json_array_from_bools(seq_scaling_list_present_flags));
        obj.add_raw("scaling_lists", json_array_from_raw(scaling_list_entries));
    }

    printf("%s", obj.str().c_str());
}

static void print_pps(const uint8_t* data, int size) {
    if (size < 2) {
        printf("{\"body_length\":%d}", size);
        return;
    }

    BitReader br(data + 1, size - 1);
    JsonObjectBuilder obj;

    uint32_t pps_id = br.read_ue();
    uint32_t sps_id = br.read_ue();
    bool entropy_coding_mode_flag = br.read_u1() != 0;
    bool bottom_field_pic_order_in_frame_present_flag = br.read_u1() != 0;
    uint32_t num_slice_groups_minus1 = br.read_ue();
    uint32_t num_slice_groups = num_slice_groups_minus1 + 1;

    obj.add_uint("pps_id", pps_id);
    obj.add_uint("sps_id", sps_id);
    obj.add_bool("entropy_coding_mode_flag", entropy_coding_mode_flag);
    obj.add_string("entropy_coding_mode", entropy_coding_mode_flag ? "CABAC" : "CAVLC");
    obj.add_bool("bottom_field_pic_order_in_frame_present_flag", bottom_field_pic_order_in_frame_present_flag);
    obj.add_uint("num_slice_groups_minus1", num_slice_groups_minus1);
    obj.add_uint("num_slice_groups", num_slice_groups);

    if (num_slice_groups_minus1 > 0) {
        uint32_t slice_group_map_type = br.read_ue();
        obj.add_uint("slice_group_map_type", slice_group_map_type);
        if (slice_group_map_type == 0) {
            std::vector<uint32_t> run_length_minus1;
            for (uint32_t i = 0; i <= num_slice_groups_minus1; i++) run_length_minus1.push_back(br.read_ue());
            obj.add_raw("run_length_minus1", json_array_from_uints(run_length_minus1));
        } else if (slice_group_map_type == 2) {
            std::vector<std::string> top_left_bottom_right;
            for (uint32_t i = 0; i < num_slice_groups_minus1; i++) {
                JsonObjectBuilder pair;
                pair.add_uint("top_left", br.read_ue());
                pair.add_uint("bottom_right", br.read_ue());
                top_left_bottom_right.push_back(pair.str());
            }
            obj.add_raw("slice_group_rectangles", json_array_from_raw(top_left_bottom_right));
        } else if (slice_group_map_type == 3 || slice_group_map_type == 4 || slice_group_map_type == 5) {
            obj.add_bool("slice_group_change_direction_flag", br.read_u1() != 0);
            obj.add_uint("slice_group_change_rate_minus1", br.read_ue());
        } else if (slice_group_map_type == 6) {
            uint32_t pic_size_in_map_units_minus1 = br.read_ue();
            int bits = ceil_log2_u32(num_slice_groups);
            std::vector<uint32_t> slice_group_id;
            for (uint32_t i = 0; i <= pic_size_in_map_units_minus1; i++) {
                slice_group_id.push_back(bits > 0 ? br.read_bits(bits) : 0);
            }
            obj.add_uint("pic_size_in_map_units_minus1", pic_size_in_map_units_minus1);
            obj.add_raw("slice_group_id", json_array_from_uints(slice_group_id));
        }
    }

    uint32_t num_ref_idx_l0_default_active_minus1 = br.read_ue();
    uint32_t num_ref_idx_l1_default_active_minus1 = br.read_ue();
    bool weighted_pred_flag = br.read_u1() != 0;
    uint32_t weighted_bipred_idc = br.read_bits(2);
    int32_t pic_init_qp_minus26 = br.read_se();
    int32_t pic_init_qs_minus26 = br.read_se();
    int32_t chroma_qp_index_offset = br.read_se();
    bool deblocking_filter_control_present_flag = br.read_u1() != 0;
    bool constrained_intra_pred_flag = br.read_u1() != 0;
    bool redundant_pic_cnt_present_flag = br.read_u1() != 0;

    obj.add_uint("num_ref_idx_l0_default_active_minus1", num_ref_idx_l0_default_active_minus1);
    obj.add_uint("num_ref_idx_l1_default_active_minus1", num_ref_idx_l1_default_active_minus1);
    obj.add_uint("num_ref_idx_l0", num_ref_idx_l0_default_active_minus1 + 1);
    obj.add_uint("num_ref_idx_l1", num_ref_idx_l1_default_active_minus1 + 1);
    obj.add_bool("weighted_pred_flag", weighted_pred_flag);
    obj.add_uint("weighted_bipred_idc", weighted_bipred_idc);
    obj.add_int("pic_init_qp_minus26", pic_init_qp_minus26);
    obj.add_int("pic_init_qs_minus26", pic_init_qs_minus26);
    obj.add_int("pic_init_qp", pic_init_qp_minus26 + 26);
    obj.add_int("pic_init_qs", pic_init_qs_minus26 + 26);
    obj.add_int("chroma_qp_index_offset", chroma_qp_index_offset);
    obj.add_bool("deblocking_filter_control_present_flag", deblocking_filter_control_present_flag);
    obj.add_bool("constrained_intra_pred_flag", constrained_intra_pred_flag);
    obj.add_bool("redundant_pic_cnt_present_flag", redundant_pic_cnt_present_flag);

    if (br.has_more_rbsp_data()) {
        bool transform_8x8_mode_flag = br.read_u1() != 0;
        bool pic_scaling_matrix_present_flag = br.read_u1() != 0;
        int32_t second_chroma_qp_index_offset = chroma_qp_index_offset;
        obj.add_bool("transform_8x8_mode_flag", transform_8x8_mode_flag);
        obj.add_bool("pic_scaling_matrix_present_flag", pic_scaling_matrix_present_flag);
        if (pic_scaling_matrix_present_flag) {
            int list_count = 6 + (transform_8x8_mode_flag ? 2 : 0);
            std::vector<int> pic_scaling_list_present_flags;
            std::vector<std::string> scaling_list_entries;
            for (int i = 0; i < list_count; i++) {
                bool present = br.read_u1() != 0;
                pic_scaling_list_present_flags.push_back(present ? 1 : 0);
                if (present) {
                    bool use_default = false;
                    int sz = (i < 6) ? 16 : 64;
                    auto values = parse_h264_scaling_list(br, sz, use_default);
                    JsonObjectBuilder scaling;
                    scaling.add_string("kind", i < 6 ? "4x4" : "8x8");
                    scaling.add_uint("index", i);
                    scaling.add_bool("use_default_scaling_matrix_flag", use_default);
                    scaling.add_raw("values", json_array_from_ints(values));
                    scaling_list_entries.push_back(scaling.str());
                } else {
                    scaling_list_entries.push_back("null");
                }
            }
            obj.add_raw("pic_scaling_list_present_flags", json_array_from_bools(pic_scaling_list_present_flags));
            obj.add_raw("pic_scaling_lists", json_array_from_raw(scaling_list_entries));
        }
        second_chroma_qp_index_offset = br.read_se();
        obj.add_int("second_chroma_qp_index_offset", second_chroma_qp_index_offset);
    }

    obj.add_uint("body_length", size);
    printf("%s", obj.str().c_str());
}

static void print_slice_header(const uint8_t* data, int size) {
    if (size < 3) return;
    BitReader br(data + 1, size - 1);
    uint32_t first_mb = br.read_ue();
    uint32_t slice_type = br.read_ue();
    uint32_t pps_id = br.read_ue();
    uint32_t frame_num = br.read_bits(4); // simplified, actual depends on SPS
    printf("{\"first_mb_in_slice\":%u,\"slice_type\":\"%s\",\"slice_type_id\":%u,\"pps_id\":%u,\"frame_num\":%u,\"body_length\":%d}",
           first_mb, slice_type_name(slice_type), slice_type, pps_id, frame_num, size);
}

static void print_sei(const uint8_t* data, int size) {
    if (size < 2) { printf("{\"body_length\":%d}", size); return; }
    int off = g_is_hevc ? 2 : 1; // HEVC NAL header is 2 bytes
    int payload_type = 0, payload_size = 0;
    while (off < size && data[off] == 0xFF) { payload_type += 255; off++; }
    if (off < size) payload_type += data[off++];
    while (off < size && data[off] == 0xFF) { payload_size += 255; off++; }
    if (off < size) payload_size += data[off++];
    printf("{\"payload_type\":%d,\"payload_size\":%d,\"body_length\":%d}", payload_type, payload_size, size);
}

/* ---- HEVC-specific parsers ---- */

static void print_hevc_vps(const uint8_t* data, int size) {
    if (size < 4) { printf("{\"body_length\":%d}", size); return; }
    BitReader br(data + 2, size - 2); // skip 2-byte NAL header
    JsonObjectBuilder obj;

    uint32_t vps_video_parameter_set_id = br.read_bits(4);
    bool vps_base_layer_internal_flag = br.read_u1() != 0;
    bool vps_base_layer_available_flag = br.read_u1() != 0;
    uint32_t vps_max_layers_minus1 = br.read_bits(6);
    uint32_t vps_max_sub_layers_minus1 = br.read_bits(3);
    bool vps_temporal_id_nesting_flag = br.read_u1() != 0;
    uint32_t vps_reserved_0xffff_16bits = br.read_bits(16);

    obj.add_uint("vps_id", vps_video_parameter_set_id);
    obj.add_bool("vps_base_layer_internal_flag", vps_base_layer_internal_flag);
    obj.add_bool("vps_base_layer_available_flag", vps_base_layer_available_flag);
    obj.add_uint("vps_max_layers_minus1", vps_max_layers_minus1);
    obj.add_uint("max_layers", vps_max_layers_minus1 + 1);
    obj.add_uint("vps_max_sub_layers_minus1", vps_max_sub_layers_minus1);
    obj.add_uint("max_sub_layers", vps_max_sub_layers_minus1 + 1);
    obj.add_bool("vps_temporal_id_nesting_flag", vps_temporal_id_nesting_flag);
    obj.add_string("vps_reserved_0xffff_16bits_hex", hex_string(vps_reserved_0xffff_16bits, 4));
    obj.add_raw("profile_tier_level", parse_hevc_profile_tier_level_json(br, true, vps_max_sub_layers_minus1 + 1));

    bool vps_sub_layer_ordering_info_present_flag = br.read_u1() != 0;
    obj.add_bool("vps_sub_layer_ordering_info_present_flag", vps_sub_layer_ordering_info_present_flag);
    std::vector<std::string> sub_layer_ordering;
    int ordering_start = vps_sub_layer_ordering_info_present_flag ? 0 : vps_max_sub_layers_minus1;
    for (int i = ordering_start; i <= (int)vps_max_sub_layers_minus1; i++) {
        JsonObjectBuilder entry;
        entry.add_uint("vps_max_dec_pic_buffering_minus1", br.read_ue());
        entry.add_uint("vps_max_num_reorder_pics", br.read_ue());
        entry.add_uint("vps_max_latency_increase_plus1", br.read_ue());
        sub_layer_ordering.push_back(entry.str());
    }
    obj.add_raw("sub_layer_ordering_info", json_array_from_raw(sub_layer_ordering));

    uint32_t vps_max_layer_id = br.read_bits(6);
    uint32_t vps_num_layer_sets_minus1 = br.read_ue();
    obj.add_uint("vps_max_layer_id", vps_max_layer_id);
    obj.add_uint("vps_num_layer_sets_minus1", vps_num_layer_sets_minus1);
    std::vector<std::string> layer_sets;
    for (uint32_t i = 1; i <= vps_num_layer_sets_minus1; i++) {
        std::vector<int> layer_id_included_flags;
        for (uint32_t j = 0; j <= vps_max_layer_id; j++) layer_id_included_flags.push_back(br.read_u1() ? 1 : 0);
        JsonObjectBuilder layer_set;
        layer_set.add_uint("layer_set_id", i);
        layer_set.add_raw("layer_id_included_flags", json_array_from_bools(layer_id_included_flags));
        layer_sets.push_back(layer_set.str());
    }
    if (!layer_sets.empty()) obj.add_raw("layer_sets", json_array_from_raw(layer_sets));

    bool vps_timing_info_present_flag = br.read_u1() != 0;
    obj.add_bool("vps_timing_info_present_flag", vps_timing_info_present_flag);
    uint32_t vps_num_units_in_tick = 0;
    uint32_t vps_time_scale = 0;
    bool vps_poc_proportional_to_timing_flag = false;
    uint32_t vps_num_ticks_poc_diff_one_minus1 = 0;
    uint32_t vps_num_hrd_parameters = 0;
    if (vps_timing_info_present_flag) {
        vps_num_units_in_tick = br.read_bits(32);
        vps_time_scale = br.read_bits(32);
        vps_poc_proportional_to_timing_flag = br.read_u1() != 0;
        if (vps_poc_proportional_to_timing_flag) vps_num_ticks_poc_diff_one_minus1 = br.read_ue();
        vps_num_hrd_parameters = br.read_ue();
        obj.add_uint("vps_num_units_in_tick", vps_num_units_in_tick);
        obj.add_uint("vps_time_scale", vps_time_scale);
        obj.add_bool("vps_poc_proportional_to_timing_flag", vps_poc_proportional_to_timing_flag);
        if (vps_poc_proportional_to_timing_flag) obj.add_uint("vps_num_ticks_poc_diff_one_minus1", vps_num_ticks_poc_diff_one_minus1);
        obj.add_uint("vps_num_hrd_parameters", vps_num_hrd_parameters);

        std::vector<std::string> hrd_parameters;
        for (uint32_t i = 0; i < vps_num_hrd_parameters; i++) {
            uint32_t hrd_layer_set_idx = br.read_ue();
            bool cprms_present_flag = (i == 0) ? true : (br.read_u1() != 0);
            auto hrd = skip_hevc_hrd_parameters(br, cprms_present_flag, vps_max_sub_layers_minus1);
            JsonObjectBuilder hrd_obj;
            hrd_obj.add_uint("hrd_layer_set_idx", hrd_layer_set_idx);
            hrd_obj.add_bool("cprms_present_flag", cprms_present_flag);
            if (hrd.valid || cprms_present_flag) {
                hrd_obj.add_bool("nal_hrd_parameters_present_flag", hrd.nal_hrd_parameters_present_flag);
                hrd_obj.add_bool("vcl_hrd_parameters_present_flag", hrd.vcl_hrd_parameters_present_flag);
                hrd_obj.add_bool("sub_pic_hrd_params_present_flag", hrd.sub_pic_hrd_params_present_flag);
            }
            hrd_parameters.push_back(hrd_obj.str());
        }
        if (!hrd_parameters.empty()) obj.add_raw("hrd_parameters", json_array_from_raw(hrd_parameters));
    }

    bool vps_extension_flag = br.read_u1() != 0;
    obj.add_bool("vps_extension_flag", vps_extension_flag);
    obj.add_uint("body_length", size);
    printf("%s", obj.str().c_str());
}

static void print_hevc_sps(const uint8_t* data, int size) {
    if (size < 4) { printf("{\"body_length\":%d}", size); return; }
    BitReader br(data + 2, size - 2);
    JsonObjectBuilder obj;

    uint32_t sps_video_parameter_set_id = br.read_bits(4);
    uint32_t sps_max_sub_layers_minus1 = br.read_bits(3);
    bool sps_temporal_id_nesting_flag = br.read_u1() != 0;
    obj.add_uint("sps_video_parameter_set_id", sps_video_parameter_set_id);
    obj.add_uint("sps_max_sub_layers_minus1", sps_max_sub_layers_minus1);
    obj.add_uint("max_sub_layers", sps_max_sub_layers_minus1 + 1);
    obj.add_bool("sps_temporal_id_nesting_flag", sps_temporal_id_nesting_flag);
    obj.add_raw("profile_tier_level", parse_hevc_profile_tier_level_json(br, true, sps_max_sub_layers_minus1 + 1));

    uint32_t sps_seq_parameter_set_id = br.read_ue();
    uint32_t chroma_format_idc = br.read_ue();
    bool separate_colour_plane_flag = false;
    if (chroma_format_idc == 3) separate_colour_plane_flag = br.read_u1() != 0;
    uint32_t pic_width_in_luma_samples = br.read_ue();
    uint32_t pic_height_in_luma_samples = br.read_ue();
    bool conformance_window_flag = br.read_u1() != 0;
    uint32_t conf_win_left_offset = 0;
    uint32_t conf_win_right_offset = 0;
    uint32_t conf_win_top_offset = 0;
    uint32_t conf_win_bottom_offset = 0;
    if (conformance_window_flag) {
        conf_win_left_offset = br.read_ue();
        conf_win_right_offset = br.read_ue();
        conf_win_top_offset = br.read_ue();
        conf_win_bottom_offset = br.read_ue();
    }
    uint32_t bit_depth_luma_minus8 = br.read_ue();
    uint32_t bit_depth_chroma_minus8 = br.read_ue();
    uint32_t log2_max_pic_order_cnt_lsb_minus4 = br.read_ue();
    bool sps_sub_layer_ordering_info_present_flag = br.read_u1() != 0;

    std::vector<std::string> sub_layer_ordering;
    int ordering_start = sps_sub_layer_ordering_info_present_flag ? 0 : sps_max_sub_layers_minus1;
    for (int i = ordering_start; i <= (int)sps_max_sub_layers_minus1; i++) {
        JsonObjectBuilder entry;
        entry.add_uint("max_dec_pic_buffering_minus1", br.read_ue());
        entry.add_uint("max_num_reorder_pics", br.read_ue());
        entry.add_uint("max_latency_increase_plus1", br.read_ue());
        sub_layer_ordering.push_back(entry.str());
    }

    uint32_t log2_min_luma_coding_block_size_minus3 = br.read_ue();
    uint32_t log2_diff_max_min_luma_coding_block_size = br.read_ue();
    uint32_t log2_min_luma_transform_block_size_minus2 = br.read_ue();
    uint32_t log2_diff_max_min_luma_transform_block_size = br.read_ue();
    uint32_t max_transform_hierarchy_depth_inter = br.read_ue();
    uint32_t max_transform_hierarchy_depth_intra = br.read_ue();
    bool scaling_list_enabled_flag = br.read_u1() != 0;
    bool sps_scaling_list_data_present_flag = false;
    if (scaling_list_enabled_flag) {
        sps_scaling_list_data_present_flag = br.read_u1() != 0;
        if (sps_scaling_list_data_present_flag) skip_hevc_scaling_list_data(br);
    }
    bool amp_enabled_flag = br.read_u1() != 0;
    bool sample_adaptive_offset_enabled_flag = br.read_u1() != 0;
    bool pcm_enabled_flag = br.read_u1() != 0;
    uint32_t pcm_sample_bit_depth_luma_minus1 = 0;
    uint32_t pcm_sample_bit_depth_chroma_minus1 = 0;
    uint32_t log2_min_pcm_luma_coding_block_size_minus3 = 0;
    uint32_t log2_diff_max_min_pcm_luma_coding_block_size = 0;
    bool pcm_loop_filter_disabled_flag = false;
    if (pcm_enabled_flag) {
        pcm_sample_bit_depth_luma_minus1 = br.read_bits(4);
        pcm_sample_bit_depth_chroma_minus1 = br.read_bits(4);
        log2_min_pcm_luma_coding_block_size_minus3 = br.read_ue();
        log2_diff_max_min_pcm_luma_coding_block_size = br.read_ue();
        pcm_loop_filter_disabled_flag = br.read_u1() != 0;
    }
    uint32_t num_short_term_ref_pic_sets = br.read_ue();
    // Keep current implementation pragmatic: expose count, skip full RPS expansion for now.
    for (uint32_t i = 0; i < num_short_term_ref_pic_sets; i++) {
        bool inter_ref_pic_set_prediction_flag = false;
        if (i != 0) inter_ref_pic_set_prediction_flag = br.read_u1() != 0;
        if (inter_ref_pic_set_prediction_flag) {
            br.read_u1();
            br.read_ue();
            uint32_t num_delta_pocs = 0;
            for (uint32_t j = 0; j <= num_delta_pocs; j++) {
                br.read_u1();
                br.read_u1();
            }
        } else {
            uint32_t num_negative_pics = br.read_ue();
            uint32_t num_positive_pics = br.read_ue();
            for (uint32_t j = 0; j < num_negative_pics; j++) {
                br.read_ue();
                br.read_u1();
            }
            for (uint32_t j = 0; j < num_positive_pics; j++) {
                br.read_ue();
                br.read_u1();
            }
        }
    }
    bool long_term_ref_pics_present_flag = br.read_u1() != 0;
    uint32_t num_long_term_ref_pics_sps = 0;
    if (long_term_ref_pics_present_flag) {
        num_long_term_ref_pics_sps = br.read_ue();
        int lt_bits = log2_max_pic_order_cnt_lsb_minus4 + 4;
        for (uint32_t i = 0; i < num_long_term_ref_pics_sps; i++) {
            br.read_bits(lt_bits);
            br.read_u1();
        }
    }
    bool sps_temporal_mvp_enabled_flag = br.read_u1() != 0;
    bool strong_intra_smoothing_enabled_flag = br.read_u1() != 0;
    bool vui_parameters_present_flag = br.read_u1() != 0;
    bool aspect_ratio_info_present_flag = false;
    bool overscan_info_present_flag = false;
    bool video_signal_type_present_flag = false;
    bool chroma_loc_info_present_flag = false;
    bool neutral_chroma_indication_flag = false;
    bool field_seq_flag = false;
    bool frame_field_info_present_flag = false;
    bool default_display_window_flag = false;
    bool vui_timing_info_present_flag = false;
    bool vui_poc_proportional_to_timing_flag = false;
    bool vui_hrd_parameters_present_flag = false;
    bool bitstream_restriction_flag = false;
    uint32_t aspect_ratio_idc = 0;
    uint32_t sar_width = 0;
    uint32_t sar_height = 0;
    uint32_t video_format = 0;
    bool video_full_range_flag = false;
    bool colour_description_present_flag = false;
    uint32_t colour_primaries = 0;
    uint32_t transfer_characteristics = 0;
    uint32_t matrix_coeffs = 0;
    uint32_t chroma_sample_loc_type_top_field = 0;
    uint32_t chroma_sample_loc_type_bottom_field = 0;
    uint32_t def_disp_win_left_offset = 0;
    uint32_t def_disp_win_right_offset = 0;
    uint32_t def_disp_win_top_offset = 0;
    uint32_t def_disp_win_bottom_offset = 0;
    uint32_t vui_num_units_in_tick = 0;
    uint32_t vui_time_scale = 0;
    uint32_t vui_num_ticks_poc_diff_one_minus1 = 0;
    if (vui_parameters_present_flag) {
        aspect_ratio_info_present_flag = br.read_u1() != 0;
        if (aspect_ratio_info_present_flag) {
            aspect_ratio_idc = br.read_bits(8);
            if (aspect_ratio_idc == 255) {
                sar_width = br.read_bits(16);
                sar_height = br.read_bits(16);
            }
        }
        overscan_info_present_flag = br.read_u1() != 0;
        if (overscan_info_present_flag) br.read_u1();
        video_signal_type_present_flag = br.read_u1() != 0;
        if (video_signal_type_present_flag) {
            video_format = br.read_bits(3);
            video_full_range_flag = br.read_u1() != 0;
            colour_description_present_flag = br.read_u1() != 0;
            if (colour_description_present_flag) {
                colour_primaries = br.read_bits(8);
                transfer_characteristics = br.read_bits(8);
                matrix_coeffs = br.read_bits(8);
            }
        }
        chroma_loc_info_present_flag = br.read_u1() != 0;
        if (chroma_loc_info_present_flag) {
            chroma_sample_loc_type_top_field = br.read_ue();
            chroma_sample_loc_type_bottom_field = br.read_ue();
        }
        neutral_chroma_indication_flag = br.read_u1() != 0;
        field_seq_flag = br.read_u1() != 0;
        frame_field_info_present_flag = br.read_u1() != 0;
        default_display_window_flag = br.read_u1() != 0;
        if (default_display_window_flag) {
            def_disp_win_left_offset = br.read_ue();
            def_disp_win_right_offset = br.read_ue();
            def_disp_win_top_offset = br.read_ue();
            def_disp_win_bottom_offset = br.read_ue();
        }
        vui_timing_info_present_flag = br.read_u1() != 0;
        if (vui_timing_info_present_flag) {
            vui_num_units_in_tick = br.read_bits(32);
            vui_time_scale = br.read_bits(32);
            vui_poc_proportional_to_timing_flag = br.read_u1() != 0;
            if (vui_poc_proportional_to_timing_flag) vui_num_ticks_poc_diff_one_minus1 = br.read_ue();
            vui_hrd_parameters_present_flag = br.read_u1() != 0;
            if (vui_hrd_parameters_present_flag) skip_hevc_hrd_parameters(br, true, sps_max_sub_layers_minus1);
        }
        bitstream_restriction_flag = br.read_u1() != 0;
        if (bitstream_restriction_flag) {
            br.read_u1();
            br.read_u1();
            br.read_u1();
            br.read_ue();
            br.read_ue();
            br.read_ue();
            br.read_ue();
            br.read_ue();
        }
    }

    uint32_t sub_width_c = (chroma_format_idc == 1 || chroma_format_idc == 2) ? 2 : 1;
    uint32_t sub_height_c = chroma_format_idc == 1 ? 2 : 1;
    uint32_t crop_unit_x = (chroma_format_idc == 0 || separate_colour_plane_flag) ? 1 : sub_width_c;
    uint32_t crop_unit_y = (chroma_format_idc == 0 || separate_colour_plane_flag) ? 1 : sub_height_c;
    uint32_t width = pic_width_in_luma_samples;
    uint32_t height = pic_height_in_luma_samples;
    if (conformance_window_flag) {
        uint32_t crop_w = (conf_win_left_offset + conf_win_right_offset) * crop_unit_x;
        uint32_t crop_h = (conf_win_top_offset + conf_win_bottom_offset) * crop_unit_y;
        width = (width > crop_w) ? (width - crop_w) : 0;
        height = (height > crop_h) ? (height - crop_h) : 0;
    }

    obj.add_uint("sps_id", sps_seq_parameter_set_id);
    obj.add_uint("chroma_format_idc", chroma_format_idc);
    if (chroma_format_idc == 3) obj.add_bool("separate_colour_plane_flag", separate_colour_plane_flag);
    obj.add_uint("pic_width_in_luma_samples", pic_width_in_luma_samples);
    obj.add_uint("pic_height_in_luma_samples", pic_height_in_luma_samples);
    obj.add_bool("conformance_window_flag", conformance_window_flag);
    if (conformance_window_flag) {
        obj.add_uint("conf_win_left_offset", conf_win_left_offset);
        obj.add_uint("conf_win_right_offset", conf_win_right_offset);
        obj.add_uint("conf_win_top_offset", conf_win_top_offset);
        obj.add_uint("conf_win_bottom_offset", conf_win_bottom_offset);
        obj.add_uint("crop_unit_x", crop_unit_x);
        obj.add_uint("crop_unit_y", crop_unit_y);
    }
    obj.add_uint("bit_depth_luma_minus8", bit_depth_luma_minus8);
    obj.add_uint("bit_depth_chroma_minus8", bit_depth_chroma_minus8);
    obj.add_uint("bit_depth_luma", bit_depth_luma_minus8 + 8);
    obj.add_uint("bit_depth_chroma", bit_depth_chroma_minus8 + 8);
    obj.add_uint("log2_max_pic_order_cnt_lsb_minus4", log2_max_pic_order_cnt_lsb_minus4);
    obj.add_bool("sps_sub_layer_ordering_info_present_flag", sps_sub_layer_ordering_info_present_flag);
    obj.add_raw("sub_layer_ordering_info", json_array_from_raw(sub_layer_ordering));
    obj.add_uint("log2_min_luma_coding_block_size_minus3", log2_min_luma_coding_block_size_minus3);
    obj.add_uint("log2_diff_max_min_luma_coding_block_size", log2_diff_max_min_luma_coding_block_size);
    obj.add_uint("log2_min_luma_transform_block_size_minus2", log2_min_luma_transform_block_size_minus2);
    obj.add_uint("log2_diff_max_min_luma_transform_block_size", log2_diff_max_min_luma_transform_block_size);
    obj.add_uint("max_transform_hierarchy_depth_inter", max_transform_hierarchy_depth_inter);
    obj.add_uint("max_transform_hierarchy_depth_intra", max_transform_hierarchy_depth_intra);
    obj.add_bool("scaling_list_enabled_flag", scaling_list_enabled_flag);
    if (scaling_list_enabled_flag) obj.add_bool("sps_scaling_list_data_present_flag", sps_scaling_list_data_present_flag);
    obj.add_bool("amp_enabled_flag", amp_enabled_flag);
    obj.add_bool("sample_adaptive_offset_enabled_flag", sample_adaptive_offset_enabled_flag);
    obj.add_bool("pcm_enabled_flag", pcm_enabled_flag);
    if (pcm_enabled_flag) {
        obj.add_uint("pcm_sample_bit_depth_luma_minus1", pcm_sample_bit_depth_luma_minus1);
        obj.add_uint("pcm_sample_bit_depth_chroma_minus1", pcm_sample_bit_depth_chroma_minus1);
        obj.add_uint("log2_min_pcm_luma_coding_block_size_minus3", log2_min_pcm_luma_coding_block_size_minus3);
        obj.add_uint("log2_diff_max_min_pcm_luma_coding_block_size", log2_diff_max_min_pcm_luma_coding_block_size);
        obj.add_bool("pcm_loop_filter_disabled_flag", pcm_loop_filter_disabled_flag);
    }
    obj.add_uint("num_short_term_ref_pic_sets", num_short_term_ref_pic_sets);
    obj.add_bool("long_term_ref_pics_present_flag", long_term_ref_pics_present_flag);
    if (long_term_ref_pics_present_flag) obj.add_uint("num_long_term_ref_pics_sps", num_long_term_ref_pics_sps);
    obj.add_bool("sps_temporal_mvp_enabled_flag", sps_temporal_mvp_enabled_flag);
    obj.add_bool("strong_intra_smoothing_enabled_flag", strong_intra_smoothing_enabled_flag);
    obj.add_bool("vui_parameters_present_flag", vui_parameters_present_flag);
    if (vui_parameters_present_flag) {
        obj.add_bool("aspect_ratio_info_present_flag", aspect_ratio_info_present_flag);
        if (aspect_ratio_info_present_flag) {
            obj.add_uint("aspect_ratio_idc", aspect_ratio_idc);
            if (aspect_ratio_idc == 255) {
                obj.add_uint("sar_width", sar_width);
                obj.add_uint("sar_height", sar_height);
            }
        }
        obj.add_bool("overscan_info_present_flag", overscan_info_present_flag);
        obj.add_bool("video_signal_type_present_flag", video_signal_type_present_flag);
        if (video_signal_type_present_flag) {
            obj.add_uint("video_format", video_format);
            obj.add_bool("video_full_range_flag", video_full_range_flag);
            obj.add_bool("colour_description_present_flag", colour_description_present_flag);
            if (colour_description_present_flag) {
                obj.add_uint("colour_primaries", colour_primaries);
                obj.add_uint("transfer_characteristics", transfer_characteristics);
                obj.add_uint("matrix_coeffs", matrix_coeffs);
            }
        }
        obj.add_bool("chroma_loc_info_present_flag", chroma_loc_info_present_flag);
        if (chroma_loc_info_present_flag) {
            obj.add_uint("chroma_sample_loc_type_top_field", chroma_sample_loc_type_top_field);
            obj.add_uint("chroma_sample_loc_type_bottom_field", chroma_sample_loc_type_bottom_field);
        }
        obj.add_bool("neutral_chroma_indication_flag", neutral_chroma_indication_flag);
        obj.add_bool("field_seq_flag", field_seq_flag);
        obj.add_bool("frame_field_info_present_flag", frame_field_info_present_flag);
        obj.add_bool("default_display_window_flag", default_display_window_flag);
        if (default_display_window_flag) {
            obj.add_uint("def_disp_win_left_offset", def_disp_win_left_offset);
            obj.add_uint("def_disp_win_right_offset", def_disp_win_right_offset);
            obj.add_uint("def_disp_win_top_offset", def_disp_win_top_offset);
            obj.add_uint("def_disp_win_bottom_offset", def_disp_win_bottom_offset);
        }
        obj.add_bool("vui_timing_info_present_flag", vui_timing_info_present_flag);
        if (vui_timing_info_present_flag) {
            obj.add_uint("vui_num_units_in_tick", vui_num_units_in_tick);
            obj.add_uint("vui_time_scale", vui_time_scale);
            obj.add_bool("vui_poc_proportional_to_timing_flag", vui_poc_proportional_to_timing_flag);
            if (vui_poc_proportional_to_timing_flag) obj.add_uint("vui_num_ticks_poc_diff_one_minus1", vui_num_ticks_poc_diff_one_minus1);
            obj.add_bool("vui_hrd_parameters_present_flag", vui_hrd_parameters_present_flag);
        }
        obj.add_bool("bitstream_restriction_flag", bitstream_restriction_flag);
    }
    obj.add_uint("width", width);
    obj.add_uint("height", height);
    obj.add_uint("body_length", size);
    printf("%s", obj.str().c_str());
}

static void print_hevc_pps(const uint8_t* data, int size) {
    if (size < 4) { printf("{\"body_length\":%d}", size); return; }
    BitReader br(data + 2, size - 2);
    JsonObjectBuilder obj;

    uint32_t pps_pic_parameter_set_id = br.read_ue();
    uint32_t pps_seq_parameter_set_id = br.read_ue();
    bool dependent_slice_segments_enabled_flag = br.read_u1() != 0;
    bool output_flag_present_flag = br.read_u1() != 0;
    uint32_t num_extra_slice_header_bits = br.read_bits(3);
    bool sign_data_hiding_enabled_flag = br.read_u1() != 0;
    bool cabac_init_present_flag = br.read_u1() != 0;
    uint32_t num_ref_idx_l0_default_active_minus1 = br.read_ue();
    uint32_t num_ref_idx_l1_default_active_minus1 = br.read_ue();
    int32_t init_qp_minus26 = br.read_se();
    bool constrained_intra_pred_flag = br.read_u1() != 0;
    bool transform_skip_enabled_flag = br.read_u1() != 0;
    bool cu_qp_delta_enabled_flag = br.read_u1() != 0;
    uint32_t diff_cu_qp_delta_depth = 0;
    if (cu_qp_delta_enabled_flag) diff_cu_qp_delta_depth = br.read_ue();
    int32_t pps_cb_qp_offset = br.read_se();
    int32_t pps_cr_qp_offset = br.read_se();
    bool pps_slice_chroma_qp_offsets_present_flag = br.read_u1() != 0;
    bool weighted_pred_flag = br.read_u1() != 0;
    bool weighted_bipred_flag = br.read_u1() != 0;
    bool transquant_bypass_enabled_flag = br.read_u1() != 0;
    bool tiles_enabled_flag = br.read_u1() != 0;
    bool entropy_coding_sync_enabled_flag = br.read_u1() != 0;
    uint32_t num_tile_columns_minus1 = 0;
    uint32_t num_tile_rows_minus1 = 0;
    bool uniform_spacing_flag = false;
    std::vector<uint32_t> column_width_minus1;
    std::vector<uint32_t> row_height_minus1;
    bool loop_filter_across_tiles_enabled_flag = false;
    if (tiles_enabled_flag) {
        num_tile_columns_minus1 = br.read_ue();
        num_tile_rows_minus1 = br.read_ue();
        uniform_spacing_flag = br.read_u1() != 0;
        if (!uniform_spacing_flag) {
            for (uint32_t i = 0; i < num_tile_columns_minus1; i++) column_width_minus1.push_back(br.read_ue());
            for (uint32_t i = 0; i < num_tile_rows_minus1; i++) row_height_minus1.push_back(br.read_ue());
        }
        loop_filter_across_tiles_enabled_flag = br.read_u1() != 0;
    }
    bool pps_loop_filter_across_slices_enabled_flag = br.read_u1() != 0;
    bool deblocking_filter_control_present_flag = br.read_u1() != 0;
    bool deblocking_filter_override_enabled_flag = false;
    bool pps_deblocking_filter_disabled_flag = false;
    int32_t pps_beta_offset_div2 = 0;
    int32_t pps_tc_offset_div2 = 0;
    if (deblocking_filter_control_present_flag) {
        deblocking_filter_override_enabled_flag = br.read_u1() != 0;
        pps_deblocking_filter_disabled_flag = br.read_u1() != 0;
        if (!pps_deblocking_filter_disabled_flag) {
            pps_beta_offset_div2 = br.read_se();
            pps_tc_offset_div2 = br.read_se();
        }
    }
    bool pps_scaling_list_data_present_flag = br.read_u1() != 0;
    if (pps_scaling_list_data_present_flag) skip_hevc_scaling_list_data(br);
    bool lists_modification_present_flag = br.read_u1() != 0;
    uint32_t log2_parallel_merge_level_minus2 = br.read_ue();
    bool slice_segment_header_extension_present_flag = br.read_u1() != 0;

    obj.add_uint("pps_id", pps_pic_parameter_set_id);
    obj.add_uint("sps_id", pps_seq_parameter_set_id);
    obj.add_bool("dependent_slice_segments_enabled_flag", dependent_slice_segments_enabled_flag);
    obj.add_bool("output_flag_present_flag", output_flag_present_flag);
    obj.add_uint("num_extra_slice_header_bits", num_extra_slice_header_bits);
    obj.add_bool("sign_data_hiding_enabled_flag", sign_data_hiding_enabled_flag);
    obj.add_bool("cabac_init_present_flag", cabac_init_present_flag);
    obj.add_uint("num_ref_idx_l0_default_active_minus1", num_ref_idx_l0_default_active_minus1);
    obj.add_uint("num_ref_idx_l1_default_active_minus1", num_ref_idx_l1_default_active_minus1);
    obj.add_uint("num_ref_idx_l0", num_ref_idx_l0_default_active_minus1 + 1);
    obj.add_uint("num_ref_idx_l1", num_ref_idx_l1_default_active_minus1 + 1);
    obj.add_int("init_qp_minus26", init_qp_minus26);
    obj.add_int("init_qp", init_qp_minus26 + 26);
    obj.add_bool("constrained_intra_pred_flag", constrained_intra_pred_flag);
    obj.add_bool("transform_skip_enabled_flag", transform_skip_enabled_flag);
    obj.add_bool("cu_qp_delta_enabled_flag", cu_qp_delta_enabled_flag);
    if (cu_qp_delta_enabled_flag) obj.add_uint("diff_cu_qp_delta_depth", diff_cu_qp_delta_depth);
    obj.add_int("pps_cb_qp_offset", pps_cb_qp_offset);
    obj.add_int("pps_cr_qp_offset", pps_cr_qp_offset);
    obj.add_bool("pps_slice_chroma_qp_offsets_present_flag", pps_slice_chroma_qp_offsets_present_flag);
    obj.add_bool("weighted_pred_flag", weighted_pred_flag);
    obj.add_bool("weighted_bipred_flag", weighted_bipred_flag);
    obj.add_bool("transquant_bypass_enabled_flag", transquant_bypass_enabled_flag);
    obj.add_bool("tiles_enabled_flag", tiles_enabled_flag);
    obj.add_bool("entropy_coding_sync_enabled_flag", entropy_coding_sync_enabled_flag);
    if (tiles_enabled_flag) {
        obj.add_uint("num_tile_columns_minus1", num_tile_columns_minus1);
        obj.add_uint("num_tile_rows_minus1", num_tile_rows_minus1);
        obj.add_bool("uniform_spacing_flag", uniform_spacing_flag);
        if (!column_width_minus1.empty()) obj.add_raw("column_width_minus1", json_array_from_uints(column_width_minus1));
        if (!row_height_minus1.empty()) obj.add_raw("row_height_minus1", json_array_from_uints(row_height_minus1));
        obj.add_bool("loop_filter_across_tiles_enabled_flag", loop_filter_across_tiles_enabled_flag);
    }
    obj.add_bool("pps_loop_filter_across_slices_enabled_flag", pps_loop_filter_across_slices_enabled_flag);
    obj.add_bool("deblocking_filter_control_present_flag", deblocking_filter_control_present_flag);
    if (deblocking_filter_control_present_flag) {
        obj.add_bool("deblocking_filter_override_enabled_flag", deblocking_filter_override_enabled_flag);
        obj.add_bool("pps_deblocking_filter_disabled_flag", pps_deblocking_filter_disabled_flag);
        if (!pps_deblocking_filter_disabled_flag) {
            obj.add_int("pps_beta_offset_div2", pps_beta_offset_div2);
            obj.add_int("pps_tc_offset_div2", pps_tc_offset_div2);
        }
    }
    obj.add_bool("pps_scaling_list_data_present_flag", pps_scaling_list_data_present_flag);
    obj.add_bool("lists_modification_present_flag", lists_modification_present_flag);
    obj.add_uint("log2_parallel_merge_level_minus2", log2_parallel_merge_level_minus2);
    obj.add_bool("slice_segment_header_extension_present_flag", slice_segment_header_extension_present_flag);
    obj.add_uint("body_length", size);
    printf("%s", obj.str().c_str());
}

static const char* hevc_slice_type_name(int t) {
    switch (t) { case 0: return "B"; case 1: return "P"; case 2: return "I"; default: return "?"; }
}

static void print_hevc_slice_header(const uint8_t* data, int size, int nal_type) {
    if (size < 4) { printf("{\"body_length\":%d}", size); return; }
    BitReader br(data + 2, size - 2);
    uint32_t first_slice = br.read_u1();
    if (nal_type >= 16 && nal_type <= 23) br.read_u1(); // no_output_of_prior_pics
    uint32_t pps_id = br.read_ue();
    // slice_type is only present if not dependent slice; assume not dependent for simplicity
    uint32_t slice_type = br.read_ue();
    printf("{\"first_slice_segment_in_pic\":%u,\"pps_id\":%u,\"slice_type\":\"%s\",\"slice_type_id\":%u,\"body_length\":%d}",
           first_slice, pps_id, hevc_slice_type_name(slice_type), slice_type, size);
}

static bool is_hevc_slice(int nal_type) {
    return nal_type <= 21; // NAL types 0-21 are VCL (slice) NALUs in HEVC
}

static void print_nalu_detail(const NALUnit& n) {
    if (g_is_hevc) {
        if (n.type == 32) print_hevc_vps(n.data, n.size);
        else if (n.type == 33) print_hevc_sps(n.data, n.size);
        else if (n.type == 34) print_hevc_pps(n.data, n.size);
        else if (n.type == 39 || n.type == 40) print_sei(n.data, n.size);
        else if (is_hevc_slice(n.type)) print_hevc_slice_header(n.data, n.size, n.type);
        else printf("{\"body_length\":%d}", n.size);
    } else {
        if (n.type == 7) print_sps(n.data, n.size);
        else if (n.type == 8) print_pps(n.data, n.size);
        else if (n.type == 6) print_sei(n.data, n.size);
        else if (n.type == 1 || n.type == 5) print_slice_header(n.data, n.size);
        else printf("{\"body_length\":%d}", n.size);
    }
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s [--thumbnails] [--range START COUNT] <video_file>\n", argv[0]);
        return 1;
    }

    const char* filename = nullptr;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--thumbnails") == 0) g_thumbnails = true;
        else if (strcmp(argv[i], "--range") == 0 && i + 2 < argc) {
            g_range_start = atoi(argv[++i]);
            g_range_count = atoi(argv[++i]);
        }
        else filename = argv[i];
    }
    if (!filename) { fprintf(stderr, "No input file\n"); return 1; }

    AVFormatContext* fmt_ctx = nullptr;
    if (avformat_open_input(&fmt_ctx, filename, nullptr, nullptr) < 0) {
        fprintf(stderr, "Cannot open %s\n", filename);
        return 1;
    }
    if (avformat_find_stream_info(fmt_ctx, nullptr) < 0) {
        avformat_close_input(&fmt_ctx);
        fprintf(stderr, "Cannot find stream info\n");
        return 1;
    }

    int video_idx = -1;
    int audio_idx = -1;
    for (unsigned i = 0; i < fmt_ctx->nb_streams; i++) {
        auto cid = fmt_ctx->streams[i]->codecpar->codec_id;
        if (cid == AV_CODEC_ID_H264 || cid == AV_CODEC_ID_HEVC) {
            video_idx = i;
            g_is_hevc = (cid == AV_CODEC_ID_HEVC);
            break;
        }
    }
    for (unsigned i = 0; i < fmt_ctx->nb_streams; i++) {
        if ((int)i == video_idx) continue;
        if (fmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
            audio_idx = i;
            break;
        }
    }
    if (video_idx < 0) {
        avformat_close_input(&fmt_ctx);
        fprintf(stderr, "No H264/H265 stream found\n");
        return 1;
    }

    // Detect AVCC/HVCC vs Annex B from extradata
    auto* par = fmt_ctx->streams[video_idx]->codecpar;
    std::vector<NALUnit> extra_nalus;
    if (par->extradata && par->extradata_size > 7 && par->extradata[0] == 1) {
        g_is_avcc = true;
        if (g_is_hevc) {
            // HVCC format
            g_nal_length_size = (par->extradata[21] & 0x03) + 1;
            int num_arrays = par->extradata[22];
            int off = 23;
            const uint8_t* ed = par->extradata;
            for (int a = 0; a < num_arrays && off + 3 <= par->extradata_size; a++) {
                off++; // array_completeness + nal_unit_type
                int num_nalus = (ed[off] << 8) | ed[off+1]; off += 2;
                for (int n = 0; n < num_nalus && off + 2 <= par->extradata_size; n++) {
                    int len = (ed[off] << 8) | ed[off+1]; off += 2;
                    if (off + len <= par->extradata_size) {
                        NALUnit nu; nu.data = ed + off; nu.size = len;
                        nu.type = extract_nal_type(nu.data);
                        extra_nalus.push_back(nu);
                    }
                    off += len;
                }
            }
        } else {
            // AVCC format
            g_nal_length_size = (par->extradata[4] & 0x03) + 1;
            const uint8_t* ed = par->extradata;
            int num_sps = ed[5] & 0x1F;
            int off = 6;
            for (int i = 0; i < num_sps && off + 2 <= par->extradata_size; i++) {
                int len = (ed[off] << 8) | ed[off+1]; off += 2;
                if (off + len <= par->extradata_size) {
                    NALUnit n; n.data = ed + off; n.size = len; n.type = n.data[0] & 0x1F;
                    extra_nalus.push_back(n);
                }
                off += len;
            }
            if (off < par->extradata_size) {
                int num_pps = ed[off++];
                for (int i = 0; i < num_pps && off + 2 <= par->extradata_size; i++) {
                    int len = (ed[off] << 8) | ed[off+1]; off += 2;
                    if (off + len <= par->extradata_size) {
                        NALUnit n; n.data = ed + off; n.size = len; n.type = n.data[0] & 0x1F;
                        extra_nalus.push_back(n);
                    }
                    off += len;
                }
            }
        }
    } else {
        g_is_avcc = false;
        if (par->extradata && par->extradata_size > 0)
            extra_nalus = parse_nalus_annexb(par->extradata, par->extradata_size);
    }

    auto* video_stream = fmt_ctx->streams[video_idx];
    auto* audio_stream = audio_idx >= 0 ? fmt_ctx->streams[audio_idx] : nullptr;
    auto tb = video_stream->time_base;
    double video_duration_ms = stream_duration_ms(video_stream);
    double audio_duration_ms = stream_duration_ms(audio_stream);
    double format_duration_ms = timestamp_to_ms(fmt_ctx->duration, AVRational{1, AV_TIME_BASE});
    double duration_ms = NAN;
    if (std::isfinite(video_duration_ms)) duration_ms = video_duration_ms;
    else if (std::isfinite(audio_duration_ms)) duration_ms = audio_duration_ms;
    else if (std::isfinite(format_duration_ms)) duration_ms = format_duration_ms;

    // Setup decoder if thumbnails requested
    AVCodecContext* dec_ctx = nullptr;
    AVFrame* dec_frame = nullptr;
    if (g_thumbnails) {
        const AVCodec* dec = avcodec_find_decoder(par->codec_id);
        if (dec) {
            dec_ctx = avcodec_alloc_context3(dec);
            avcodec_parameters_to_context(dec_ctx, par);
            dec_ctx->export_side_data |= AV_CODEC_EXPORT_DATA_MVS;
            avcodec_open2(dec_ctx, dec, nullptr);
            dec_frame = av_frame_alloc();
        }
    }
    int thumb_w = 0, thumb_h = g_thumb_height;
    if (par->height > 0) thumb_w = par->width * g_thumb_height / par->height;
    int preview_h = par->height;
    int preview_w = par->width;

    AVPacket pkt;
    int idx = 0;
    int range_end = (g_range_start >= 0) ? g_range_start + g_range_count : -1;
    bool range_mode = g_range_start >= 0 && g_thumbnails;
    int range_out = 0; // count of items output in range mode

    std::vector<AVSyncPoint> video_sync_points;
    std::vector<AVSyncPoint> audio_sync_points;

    if (range_mode)
        printf("{\"thumbnails\":[");
    else {
        printf("{\"video\":{\"width\":%d,\"height\":%d,\"codec\":\"%s\",\"time_base_num\":%d,\"time_base_den\":%d},\"audio\":",
               par->width, par->height, g_is_hevc ? "h265" : "h264", tb.num, tb.den);
        print_stream_meta_json_or_null(audio_stream);
        printf(",\"duration_ms\":");
        print_json_double_or_null(duration_ms);
        printf(",\"frames\":[");
    }

    // For range mode: map packet index -> pts for matching decoded frames
    std::vector<int64_t> frame_pts;
    if (range_mode) frame_pts.resize(range_end, AV_NOPTS_VALUE);

    // DPB: track I/P frame indices in decode order, and pts->index map
    std::vector<int> dpb_ref_frames; // indices of I/P frames seen so far
    std::unordered_map<int64_t, int> pts_to_idx; // pts -> frame index

    while (av_read_frame(fmt_ctx, &pkt) >= 0) {
        if (!range_mode && pkt.stream_index == audio_idx) {
            AVSyncPoint point;
            point.index = (int)audio_sync_points.size();
            point.pts = pkt.pts;
            point.dts = pkt.dts;
            point.time_ms = timestamp_to_ms(packet_timeline_ts(pkt), audio_stream->time_base);
            point.decode_time_ms = timestamp_to_ms(packet_decode_ts(pkt), audio_stream->time_base);
            audio_sync_points.push_back(point);
        }

        if (pkt.stream_index == video_idx) {
            // In range mode, stop early once past the range
            if (range_mode && idx >= range_end) { av_packet_unref(&pkt); break; }

            if (range_mode) {
                if (idx >= g_range_start && idx < range_end)
                    frame_pts[idx] = pkt.pts;
                if (dec_ctx) {
                    avcodec_send_packet(dec_ctx, &pkt);
                    // Drain all available decoded frames
                    while (avcodec_receive_frame(dec_ctx, dec_frame) == 0) {
                        // Match decoded frame back to original frame by pts
                        int fi = -1;
                        for (int k = g_range_start; k < range_end; k++) {
                            if (frame_pts[k] == dec_frame->pts) { fi = k; break; }
                        }
                        if (fi < 0) continue; // decoded frame outside our range
                        if (range_out > 0) printf(",");
                        const char* dec_type = "P";
                        if (dec_frame->pict_type == AV_PICTURE_TYPE_I) dec_type = "I";
                        else if (dec_frame->pict_type == AV_PICTURE_TYPE_B) dec_type = "B";
                        printf("{\"index\":%d,\"dec_type\":\"%s\"", fi, dec_type);
                        auto b64 = encode_jpeg(dec_frame, thumb_w, thumb_h);
                        if (!b64.empty()) {
                            printf(",\"thumbnail\":\"data:image/jpeg;base64,");
                            fwrite(b64.data(), 1, b64.size(), stdout);
                            printf("\"");
                        }
                        auto prev = encode_jpeg(dec_frame, preview_w, preview_h, 2);
                        if (!prev.empty()) {
                            printf(",\"preview\":\"data:image/jpeg;base64,");
                            fwrite(prev.data(), 1, prev.size(), stdout);
                            printf("\"");
                        }
                        printf("}");
                        range_out++;
                    }
                }
            } else {
                // Normal full output mode
                const char* type = (pkt.flags & AV_PKT_FLAG_KEY) ? "I" : "P";
                AVSyncPoint point;
                point.index = idx;
                point.pts = pkt.pts;
                point.dts = pkt.dts;
                point.time_ms = timestamp_to_ms(packet_timeline_ts(pkt), video_stream->time_base);
                point.decode_time_ms = timestamp_to_ms(packet_decode_ts(pkt), video_stream->time_base);
                video_sync_points.push_back(point);
                if (idx > 0) printf(",");
                printf("{\"index\":%d,\"type\":\"%s\",\"pts\":%ld,\"size\":%d", idx, type, (long)pkt.pts, pkt.size);

                if (dec_ctx) {
                    avcodec_send_packet(dec_ctx, &pkt);
                    if (avcodec_receive_frame(dec_ctx, dec_frame) == 0) {
                        const char* dec_type = "P";
                        if (dec_frame->pict_type == AV_PICTURE_TYPE_I) dec_type = "I";
                        else if (dec_frame->pict_type == AV_PICTURE_TYPE_B) dec_type = "B";
                        auto b64 = encode_jpeg(dec_frame, thumb_w, thumb_h);
                        if (!b64.empty()) {
                            printf(",\"dec_type\":\"%s\",\"thumbnail\":\"data:image/jpeg;base64,", dec_type);
                            fwrite(b64.data(), 1, b64.size(), stdout);
                            printf("\"");
                        }
                    }
                }

                printf(",\"nalus\":[");
                auto nalus = parse_nalus(pkt.data, pkt.size);
                bool has_sps = false;
                for (auto& n : nalus)
                    if ((!g_is_hevc && n.type == 7) || (g_is_hevc && n.type == 33)) has_sps = true;
                std::vector<NALUnit> all_nalus;
                if ((pkt.flags & AV_PKT_FLAG_KEY) && !has_sps)
                    all_nalus.insert(all_nalus.end(), extra_nalus.begin(), extra_nalus.end());
                all_nalus.insert(all_nalus.end(), nalus.begin(), nalus.end());
                for (size_t ni = 0; ni < all_nalus.size(); ni++) {
                    auto& n = all_nalus[ni];
                    if (ni > 0) printf(",");
                    printf("{\"type\":%d,\"type_name\":", n.type);
                    print_json_string(nal_type_name(n.type));
                    printf(",\"length\":%d,\"detail\":", n.size);
                    print_nalu_detail(n);
                    printf("}");
                }
                printf("]}");
            }
            idx++;
        }
        av_packet_unref(&pkt);
    }

    // Flush decoder to get remaining buffered frames
    if (range_mode && dec_ctx) {
        avcodec_send_packet(dec_ctx, nullptr); // signal EOF
        while (avcodec_receive_frame(dec_ctx, dec_frame) == 0) {
            int fi = -1;
            for (int k = g_range_start; k < range_end; k++) {
                if (frame_pts[k] == dec_frame->pts) { fi = k; break; }
            }
            if (fi < 0) continue;
            if (range_out > 0) printf(",");
            const char* dec_type = "P";
            if (dec_frame->pict_type == AV_PICTURE_TYPE_I) dec_type = "I";
            else if (dec_frame->pict_type == AV_PICTURE_TYPE_B) dec_type = "B";
            printf("{\"index\":%d,\"dec_type\":\"%s\"", fi, dec_type);
            auto b64 = encode_jpeg(dec_frame, thumb_w, thumb_h);
            if (!b64.empty()) {
                printf(",\"thumbnail\":\"data:image/jpeg;base64,");
                fwrite(b64.data(), 1, b64.size(), stdout);
                printf("\"");
            }
            auto prev = encode_jpeg(dec_frame, preview_w, preview_h, 2);
            if (!prev.empty()) {
                printf(",\"preview\":\"data:image/jpeg;base64,");
                fwrite(prev.data(), 1, prev.size(), stdout);
                printf("\"");
            }
            printf("}");
            range_out++;
        }
    }

    if (range_mode) {
        printf("]}");
    } else {
        printf("],\"avsync\":{\"video\":");
        print_avsync_points_json(video_sync_points);
        printf(",\"audio\":");
        print_avsync_points_json(audio_sync_points);
        printf("}}");
    }
    if (dec_frame) av_frame_free(&dec_frame);
    if (dec_ctx) avcodec_free_context(&dec_ctx);
    avformat_close_input(&fmt_ctx);
    return 0;
}
