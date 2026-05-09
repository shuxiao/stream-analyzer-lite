#include <cstdio>
#include <cstdlib>
#include <cstring>
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
};

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
    if (size < 4) return;
    BitReader br(data + 1, size - 1); // skip NAL header byte
    uint32_t profile_idc = br.read_bits(8);
    uint32_t constraint_flags = br.read_bits(8);
    uint32_t level_idc = br.read_bits(8);
    uint32_t sps_id = br.read_ue();
    printf("{\"profile_idc\":%u,\"constraint_flags\":%u,\"level_idc\":%u,\"sps_id\":%u",
           profile_idc, constraint_flags, level_idc, sps_id);

    uint32_t chroma_format_idc = 1;
    if (profile_idc == 100 || profile_idc == 110 || profile_idc == 122 ||
        profile_idc == 244 || profile_idc == 44 || profile_idc == 83 ||
        profile_idc == 86 || profile_idc == 118 || profile_idc == 128) {
        chroma_format_idc = br.read_ue();
        printf(",\"chroma_format_idc\":%u", chroma_format_idc);
        if (chroma_format_idc == 3) br.read_u1(); // separate_colour_plane_flag
        uint32_t bit_depth_luma = br.read_ue() + 8;
        uint32_t bit_depth_chroma = br.read_ue() + 8;
        printf(",\"bit_depth_luma\":%u,\"bit_depth_chroma\":%u", bit_depth_luma, bit_depth_chroma);
        br.read_u1(); // qpprime_y_zero_transform_bypass
        if (br.read_u1()) { // seq_scaling_matrix_present
            int cnt = (chroma_format_idc != 3) ? 8 : 12;
            for (int i = 0; i < cnt; i++) {
                if (br.read_u1()) {
                    int sz = (i < 6) ? 16 : 64;
                    int last = 8, next = 8;
                    for (int j = 0; j < sz; j++) {
                        if (next != 0) next = (last + br.read_se() + 256) % 256;
                        last = (next == 0) ? last : next;
                    }
                }
            }
        }
    }
    uint32_t log2_max_frame_num = br.read_ue() + 4;
    printf(",\"log2_max_frame_num\":%u", log2_max_frame_num);
    uint32_t poc_type = br.read_ue();
    printf(",\"pic_order_cnt_type\":%u", poc_type);
    if (poc_type == 0) {
        uint32_t log2_max_poc_lsb = br.read_ue() + 4;
        printf(",\"log2_max_pic_order_cnt_lsb\":%u", log2_max_poc_lsb);
    } else if (poc_type == 1) {
        br.read_u1();
        br.read_se();
        br.read_se();
        uint32_t n = br.read_ue();
        for (uint32_t i = 0; i < n; i++) br.read_se();
    }
    uint32_t max_ref = br.read_ue();
    uint32_t gaps = br.read_u1();
    uint32_t mb_w = br.read_ue() + 1;
    uint32_t mb_h = br.read_ue() + 1;
    uint32_t frame_mbs_only = br.read_u1();
    printf(",\"max_num_ref_frames\":%u,\"gaps_in_frame_num_allowed\":%u", max_ref, gaps);
    printf(",\"pic_width_in_mbs\":%u,\"pic_height_in_map_units\":%u", mb_w, mb_h);
    printf(",\"frame_mbs_only_flag\":%u", frame_mbs_only);
    printf(",\"width\":%u,\"height\":%u}", mb_w * 16, mb_h * 16 * (2 - frame_mbs_only));
}

static void print_pps(const uint8_t* data, int size) {
    if (size < 2) return;
    BitReader br(data + 1, size - 1);
    uint32_t pps_id = br.read_ue();
    uint32_t sps_id = br.read_ue();
    uint32_t entropy = br.read_u1();
    uint32_t bottom_field_poc = br.read_u1();
    uint32_t num_slice_groups = br.read_ue() + 1;
    printf("{\"pps_id\":%u,\"sps_id\":%u,\"entropy_coding_mode\":\"%s\"",
           pps_id, sps_id, entropy ? "CABAC" : "CAVLC");
    printf(",\"bottom_field_pic_order_in_frame_present\":%u", bottom_field_poc);
    printf(",\"num_slice_groups\":%u", num_slice_groups);
    // skip slice group details if > 1
    if (num_slice_groups <= 1 && br.available() > 20) {
        uint32_t num_ref_l0 = br.read_ue() + 1;
        uint32_t num_ref_l1 = br.read_ue() + 1;
        uint32_t weighted_pred = br.read_u1();
        uint32_t weighted_bipred = br.read_bits(2);
        int32_t init_qp = br.read_se() + 26;
        int32_t init_qs = br.read_se() + 26;
        int32_t chroma_qp_offset = br.read_se();
        printf(",\"num_ref_idx_l0\":%u,\"num_ref_idx_l1\":%u", num_ref_l0, num_ref_l1);
        printf(",\"weighted_pred_flag\":%u,\"weighted_bipred_idc\":%u", weighted_pred, weighted_bipred);
        printf(",\"pic_init_qp\":%d,\"pic_init_qs\":%d,\"chroma_qp_index_offset\":%d",
               init_qp, init_qs, chroma_qp_offset);
    }
    printf("}");
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
    uint32_t vps_id = br.read_bits(4);
    br.read_bits(2); // vps_base_layer_internal_flag, vps_base_layer_available_flag
    uint32_t max_layers = br.read_bits(6) + 1;
    uint32_t max_sub_layers = br.read_bits(3) + 1;
    uint32_t temporal_id_nesting = br.read_u1();
    printf("{\"vps_id\":%u,\"max_layers\":%u,\"max_sub_layers\":%u,\"temporal_id_nesting_flag\":%u}",
           vps_id, max_layers, max_sub_layers, temporal_id_nesting);
}

static void skip_profile_tier_level(BitReader& br, bool profile_present, int max_sub_layers) {
    if (profile_present) {
        br.read_bits(2); // general_profile_space
        br.read_u1();    // general_tier_flag
        br.read_bits(5); // general_profile_idc
        br.read_bits(32); // general_profile_compatibility_flags
        br.read_bits(32); br.read_bits(16); // general_constraint_indicator_flags (48 bits)
    }
    br.read_bits(8); // general_level_idc
    std::vector<bool> sub_layer_profile(max_sub_layers - 1);
    std::vector<bool> sub_layer_level(max_sub_layers - 1);
    for (int i = 0; i < max_sub_layers - 1; i++) {
        sub_layer_profile[i] = br.read_u1();
        sub_layer_level[i] = br.read_u1();
    }
    if (max_sub_layers > 1)
        for (int i = max_sub_layers - 1; i < 8; i++) br.read_bits(2);
    for (int i = 0; i < max_sub_layers - 1; i++) {
        if (sub_layer_profile[i]) { br.read_bits(2); br.read_u1(); br.read_bits(5); br.read_bits(32); br.read_bits(32); br.read_bits(16); }
        if (sub_layer_level[i]) br.read_bits(8);
    }
}

static void print_hevc_sps(const uint8_t* data, int size) {
    if (size < 4) { printf("{\"body_length\":%d}", size); return; }
    BitReader br(data + 2, size - 2);
    uint32_t vps_id = br.read_bits(4);
    uint32_t max_sub_layers = br.read_bits(3) + 1;
    uint32_t temporal_id_nesting = br.read_u1();
    skip_profile_tier_level(br, true, max_sub_layers);
    uint32_t sps_id = br.read_ue();
    uint32_t chroma_format_idc = br.read_ue();
    printf("{\"vps_id\":%u,\"sps_id\":%u,\"max_sub_layers\":%u,\"chroma_format_idc\":%u",
           vps_id, sps_id, max_sub_layers, chroma_format_idc);
    if (chroma_format_idc == 3) br.read_u1(); // separate_colour_plane_flag
    uint32_t pic_w = br.read_ue();
    uint32_t pic_h = br.read_ue();
    printf(",\"width\":%u,\"height\":%u}", pic_w, pic_h);
}

static void print_hevc_pps(const uint8_t* data, int size) {
    if (size < 4) { printf("{\"body_length\":%d}", size); return; }
    BitReader br(data + 2, size - 2);
    uint32_t pps_id = br.read_ue();
    uint32_t sps_id = br.read_ue();
    uint32_t dependent_slice = br.read_u1();
    uint32_t output_flag_present = br.read_u1();
    uint32_t num_extra_slice_header_bits = br.read_bits(3);
    uint32_t sign_data_hiding = br.read_u1();
    uint32_t cabac_init_present = br.read_u1();
    uint32_t num_ref_l0 = br.read_ue() + 1;
    uint32_t num_ref_l1 = br.read_ue() + 1;
    int32_t init_qp = br.read_se() + 26;
    printf("{\"pps_id\":%u,\"sps_id\":%u,\"dependent_slice_segments_enabled\":%u"
           ",\"num_ref_idx_l0\":%u,\"num_ref_idx_l1\":%u,\"init_qp\":%d}",
           pps_id, sps_id, dependent_slice, num_ref_l0, num_ref_l1, init_qp);
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
    for (unsigned i = 0; i < fmt_ctx->nb_streams; i++) {
        auto cid = fmt_ctx->streams[i]->codecpar->codec_id;
        if (cid == AV_CODEC_ID_H264 || cid == AV_CODEC_ID_HEVC) {
            video_idx = i;
            g_is_hevc = (cid == AV_CODEC_ID_HEVC);
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

    auto tb = fmt_ctx->streams[video_idx]->time_base;

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

    if (range_mode)
        printf("{\"thumbnails\":[");
    else
        printf("{\"video\":{\"width\":%d,\"height\":%d,\"codec\":\"%s\",\"time_base_num\":%d,\"time_base_den\":%d},\"frames\":[", par->width, par->height, g_is_hevc ? "h265" : "h264", tb.num, tb.den);

    // For range mode: map packet index -> pts for matching decoded frames
    std::vector<int64_t> frame_pts;
    if (range_mode) frame_pts.resize(range_end, AV_NOPTS_VALUE);

    // DPB: track I/P frame indices in decode order, and pts->index map
    std::vector<int> dpb_ref_frames; // indices of I/P frames seen so far
    std::unordered_map<int64_t, int> pts_to_idx; // pts -> frame index

    while (av_read_frame(fmt_ctx, &pkt) >= 0) {
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

    if (range_mode) printf("]}");
    else printf("]}");
    if (dec_frame) av_frame_free(&dec_frame);
    if (dec_ctx) avcodec_free_context(&dec_ctx);
    avformat_close_input(&fmt_ctx);
    return 0;
}
