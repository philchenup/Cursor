#include "pcd_io.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

struct PcdHeader {
    std::vector<std::string> fields;
    std::vector<int> sizes;
    std::vector<char> types;
    std::vector<int> counts;
    int width = 0;
    int height = 1;
    int points = 0;
    std::string data_type;
    std::size_t data_offset = 0;
};

bool ParseHeader(const std::string& path, PcdHeader& header, std::vector<char>& file_bytes) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) return false;
    file_bytes.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    if (file_bytes.empty()) return false;

    std::size_t pos = 0;
    auto read_line = [&](std::string& line) -> bool {
        if (pos >= file_bytes.size()) return false;
        std::size_t end = pos;
        while (end < file_bytes.size() && file_bytes[end] != '\n') ++end;
        line.assign(file_bytes.data() + pos, file_bytes.data() + end);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        pos = (end < file_bytes.size()) ? end + 1 : end;
        return true;
    };

    std::string line;
    while (read_line(line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        std::string key;
        iss >> key;
        if (key == "FIELDS") {
            header.fields.clear();
            for (std::string f; iss >> f;) header.fields.push_back(f);
        } else if (key == "SIZE") {
            header.sizes.clear();
            for (int v; iss >> v;) header.sizes.push_back(v);
        } else if (key == "TYPE") {
            header.types.clear();
            for (char t; iss >> t;) header.types.push_back(t);
        } else if (key == "COUNT") {
            header.counts.clear();
            for (int v; iss >> v;) header.counts.push_back(v);
        } else if (key == "WIDTH") {
            iss >> header.width;
        } else if (key == "HEIGHT") {
            iss >> header.height;
        } else if (key == "POINTS") {
            iss >> header.points;
        } else if (key == "DATA") {
            iss >> header.data_type;
            header.data_offset = pos;
            break;
        }
    }
    if (header.points <= 0 && header.width > 0)
        header.points = header.width * std::max(header.height, 1);
    if (header.counts.empty()) header.counts.assign(header.fields.size(), 1);
    return !header.fields.empty() && header.points > 0 && !header.data_type.empty();
}

int FieldIndex(const PcdHeader& header, const std::string& name) {
    for (std::size_t i = 0; i < header.fields.size(); ++i)
        if (header.fields[i] == name) return static_cast<int>(i);
    return -1;
}

int PointStride(const PcdHeader& header) {
    int stride = 0;
    for (std::size_t i = 0; i < header.fields.size(); ++i) {
        const int count = (i < header.counts.size()) ? header.counts[i] : 1;
        const int size = (i < header.sizes.size()) ? header.sizes[i] : 4;
        stride += count * size;
    }
    return stride;
}

bool LzfDecompress(const uint8_t* in, std::size_t in_len, uint8_t* out, std::size_t out_len) {
    std::size_t ip = 0, op = 0;
    while (ip < in_len) {
        const unsigned ctrl = in[ip++];
        if (ctrl < (1u << 5)) {
            const unsigned lit = ctrl + 1;
            if (op + lit > out_len || ip + lit > in_len) return false;
            std::memcpy(out + op, in + ip, lit);
            ip += lit;
            op += lit;
        } else {
            unsigned len = ctrl >> 5;
            std::size_t ref = op - ((ctrl & 0x1fu) << 8) - 1;
            if (len == 7) {
                if (ip >= in_len) return false;
                len += in[ip++];
            }
            if (ip >= in_len) return false;
            ref -= in[ip++];
            len += 2;
            if (op + len > out_len || ref >= op) return false;
            while (len--) out[op++] = out[ref++];
        }
        if (op == out_len) break;
    }
    return op == out_len;
}

bool DecompressBinaryCompressed(const char* data, std::size_t data_size, std::vector<char>& uncompressed) {
    if (data_size < 8) return false;
    uint32_t compressed_size = 0, uncompressed_size = 0;
    std::memcpy(&compressed_size, data, 4);
    std::memcpy(&uncompressed_size, data + 4, 4);
    if (8u + compressed_size > data_size) return false;
    uncompressed.resize(uncompressed_size);
    return LzfDecompress(reinterpret_cast<const uint8_t*>(data + 8), compressed_size,
                         reinterpret_cast<uint8_t*>(uncompressed.data()), uncompressed_size);
}

float ReadFloatField(const char* base, int offset, char type, int size) {
    if (type == 'F' && size == 4) {
        float v;
        std::memcpy(&v, base + offset, 4);
        return v;
    }
    if (type == 'F' && size == 8) {
        double v;
        std::memcpy(&v, base + offset, 8);
        return static_cast<float>(v);
    }
    return 0.f;
}

template <typename FillFn>
bool LoadPcdImpl(const std::string& path, int n_points_hint, FillFn fill) {
    PcdHeader header;
    std::vector<char> file_bytes;
    if (!ParseHeader(path, header, file_bytes)) return false;
    (void)n_points_hint;
    if (FieldIndex(header, "x") < 0 || FieldIndex(header, "y") < 0 || FieldIndex(header, "z") < 0)
        return false;

    std::vector<int> field_offset(header.fields.size(), 0);
    int off = 0;
    for (std::size_t i = 0; i < header.fields.size(); ++i) {
        field_offset[i] = off;
        const int count = (i < header.counts.size()) ? header.counts[i] : 1;
        const int size = (i < header.sizes.size()) ? header.sizes[i] : 4;
        off += count * size;
    }

    auto apply = [&](int p, const std::unordered_map<std::string, float>& values) {
        fill(p, values);
    };

    if (header.data_type == "ascii") {
        std::istringstream iss(std::string(file_bytes.data() + header.data_offset,
                                           file_bytes.size() - header.data_offset));
        for (int p = 0; p < header.points; ++p) {
            std::unordered_map<std::string, float> values;
            for (const auto& field : header.fields) {
                float v = 0.f;
                iss >> v;
                values[field] = v;
            }
            apply(p, values);
        }
        return true;
    }

    std::vector<char> binary_payload;
    const char* payload = nullptr;
    std::size_t payload_size = 0;
    const bool by_field = (header.data_type == "binary_compressed");
    if (header.data_type == "binary") {
        payload = file_bytes.data() + header.data_offset;
        payload_size = file_bytes.size() - header.data_offset;
    } else if (header.data_type == "binary_compressed") {
        if (!DecompressBinaryCompressed(file_bytes.data() + header.data_offset,
                                        file_bytes.size() - header.data_offset, binary_payload))
            return false;
        payload = binary_payload.data();
        payload_size = binary_payload.size();
    } else {
        return false;
    }

    const int stride = PointStride(header);
    if (payload_size < static_cast<std::size_t>(stride) * header.points) return false;

    if (!by_field) {
        for (int p = 0; p < header.points; ++p) {
            const char* base = payload + static_cast<std::size_t>(p) * stride;
            std::unordered_map<std::string, float> values;
            for (std::size_t f = 0; f < header.fields.size(); ++f)
                values[header.fields[f]] =
                    ReadFloatField(base, field_offset[f], header.types[f], header.sizes[f]);
            apply(p, values);
        }
        return true;
    }

    std::vector<std::size_t> field_base(header.fields.size(), 0);
    std::size_t base = 0;
    for (std::size_t f = 0; f < header.fields.size(); ++f) {
        field_base[f] = base;
        const int count = (f < header.counts.size()) ? header.counts[f] : 1;
        const int size = (f < header.sizes.size()) ? header.sizes[f] : 4;
        base += static_cast<std::size_t>(count * size) * header.points;
    }
    for (int p = 0; p < header.points; ++p) {
        std::unordered_map<std::string, float> values;
        for (std::size_t f = 0; f < header.fields.size(); ++f) {
            const int size = header.sizes[f];
            const char* addr = payload + field_base[f] + static_cast<std::size_t>(p) * size;
            values[header.fields[f]] = ReadFloatField(addr, 0, header.types[f], size);
        }
        apply(p, values);
    }
    return true;
}

int CountPoints(const std::string& path) {
    PcdHeader header;
    std::vector<char> file_bytes;
    if (!ParseHeader(path, header, file_bytes)) return 0;
    return header.points;
}

}  // namespace

bool LoadPcdPoints(const std::string& path, Eigen::MatrixX3f& points) {
    const int n = CountPoints(path);
    if (n <= 0) return false;
    points.resize(n, 3);
    return LoadPcdImpl(path, n, [&](int p, const std::unordered_map<std::string, float>& v) {
        points(p, 0) = v.at("x");
        points(p, 1) = v.at("y");
        points(p, 2) = v.at("z");
    });
}

bool LoadPcdPointsAndNormals(const std::string& path,
                             Eigen::MatrixX3f& points,
                             Eigen::MatrixX3f& normals) {
    const int n = CountPoints(path);
    if (n <= 0) return false;
    points.resize(n, 3);
    normals.resize(n, 3);
    normals.setZero();
    return LoadPcdImpl(path, n, [&](int p, const std::unordered_map<std::string, float>& v) {
        points(p, 0) = v.at("x");
        points(p, 1) = v.at("y");
        points(p, 2) = v.at("z");
        if (v.count("normal_x")) {
            normals(p, 0) = v.at("normal_x");
            normals(p, 1) = v.at("normal_y");
            normals(p, 2) = v.at("normal_z");
        }
    });
}

bool SavePlyPoints(const std::string& path, const Eigen::MatrixX3f& points) {
    std::ofstream ofs(path);
    if (!ofs) return false;
    ofs << "ply\nformat ascii 1.0\n"
        << "element vertex " << points.rows() << "\n"
        << "property float x\nproperty float y\nproperty float z\nend_header\n";
    ofs.setf(std::ios::fixed);
    ofs.precision(6);
    for (int i = 0; i < points.rows(); ++i)
        ofs << points(i, 0) << ' ' << points(i, 1) << ' ' << points(i, 2) << '\n';
    return static_cast<bool>(ofs);
}
