#include "io/pcd_io.h"

#include <glog/logging.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace poser {
namespace {

struct PcdHeader {
	std::vector<std::string> fields;
	std::vector<int> sizes;
	std::vector<char> types;
	std::vector<int> counts;
	int width = 0;
	int height = 1;
	int points = 0;
	std::string data_type;  // ascii | binary | binary_compressed
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
			std::string f;
			while (iss >> f) header.fields.push_back(f);
		} else if (key == "SIZE") {
			header.sizes.clear();
			int v;
			while (iss >> v) header.sizes.push_back(v);
		} else if (key == "TYPE") {
			header.types.clear();
			char t;
			while (iss >> t) header.types.push_back(t);
		} else if (key == "COUNT") {
			header.counts.clear();
			int v;
			while (iss >> v) header.counts.push_back(v);
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
	if (header.counts.empty())
		header.counts.assign(header.fields.size(), 1);
	return !header.fields.empty() && header.points > 0 && !header.data_type.empty();
}

int FieldIndex(const PcdHeader& header, const std::string& name) {
	for (std::size_t i = 0; i < header.fields.size(); ++i) {
		if (header.fields[i] == name) return static_cast<int>(i);
	}
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

// PCL LZF decompress (binary_compressed payload).
bool LzfDecompress(const uint8_t* in, std::size_t in_len, uint8_t* out, std::size_t out_len) {
	std::size_t ip = 0;
	std::size_t op = 0;
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
			while (len--) {
				out[op] = out[ref];
				++op;
				++ref;
			}
		}
		if (op == out_len) break;
	}
	return op == out_len;
}

bool DecompressBinaryCompressed(
	const char* data, std::size_t data_size,
	std::vector<char>& uncompressed
) {
	if (data_size < 8) return false;
	uint32_t compressed_size = 0;
	uint32_t uncompressed_size = 0;
	std::memcpy(&compressed_size, data, 4);
	std::memcpy(&uncompressed_size, data + 4, 4);
	if (8u + compressed_size > data_size) return false;
	uncompressed.resize(uncompressed_size);
	return LzfDecompress(
		reinterpret_cast<const uint8_t*>(data + 8),
		compressed_size,
		reinterpret_cast<uint8_t*>(uncompressed.data()),
		uncompressed_size);
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
	LOG(FATAL) << "Unsupported PCD field type/size: " << type << "/" << size;
	return 0.f;
}

template <typename PointT, typename FillFn>
bool LoadPcdImpl(const std::string& path, std::vector<PointT>& points, FillFn fill) {
	PcdHeader header;
	std::vector<char> file_bytes;
	if (!ParseHeader(path, header, file_bytes)) {
		LOG(ERROR) << "Failed to parse PCD header: " << path;
		return false;
	}

	const int ix = FieldIndex(header, "x");
	const int iy = FieldIndex(header, "y");
	const int iz = FieldIndex(header, "z");
	LOG_ASSERT(ix >= 0 && iy >= 0 && iz >= 0) << "PCD must contain x y z fields";

	std::vector<int> field_offset(header.fields.size(), 0);
	{
		int off = 0;
		for (std::size_t i = 0; i < header.fields.size(); ++i) {
			field_offset[i] = off;
			const int count = (i < header.counts.size()) ? header.counts[i] : 1;
			const int size = (i < header.sizes.size()) ? header.sizes[i] : 4;
			off += count * size;
		}
	}

	points.resize(static_cast<std::size_t>(header.points));

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
			fill(points[static_cast<std::size_t>(p)], values);
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
		if (!DecompressBinaryCompressed(
			file_bytes.data() + header.data_offset,
			file_bytes.size() - header.data_offset,
			binary_payload)) {
			LOG(ERROR) << "Failed to decompress binary_compressed PCD: " << path;
			return false;
		}
		payload = binary_payload.data();
		payload_size = binary_payload.size();
	} else {
		LOG(ERROR) << "Unsupported PCD DATA type: " << header.data_type;
		return false;
	}

	const int stride = PointStride(header);
	if (!by_field) {
		LOG_ASSERT(payload_size >= static_cast<std::size_t>(stride) * header.points);
		for (int p = 0; p < header.points; ++p) {
			const char* base = payload + static_cast<std::size_t>(p) * stride;
			std::unordered_map<std::string, float> values;
			for (std::size_t f = 0; f < header.fields.size(); ++f) {
				values[header.fields[f]] = ReadFloatField(
					base, field_offset[f],
					header.types[f], header.sizes[f]);
			}
			fill(points[static_cast<std::size_t>(p)], values);
		}
		return true;
	}

	// binary_compressed layout: all values of field0, then field1, ...
	LOG_ASSERT(payload_size >= static_cast<std::size_t>(stride) * header.points);
	std::vector<std::size_t> field_base(header.fields.size(), 0);
	{
		std::size_t base = 0;
		for (std::size_t f = 0; f < header.fields.size(); ++f) {
			field_base[f] = base;
			const int count = (f < header.counts.size()) ? header.counts[f] : 1;
			const int size = (f < header.sizes.size()) ? header.sizes[f] : 4;
			base += static_cast<std::size_t>(count * size) * header.points;
		}
	}
	for (int p = 0; p < header.points; ++p) {
		std::unordered_map<std::string, float> values;
		for (std::size_t f = 0; f < header.fields.size(); ++f) {
			const int size = header.sizes[f];
			const char* addr = payload + field_base[f] + static_cast<std::size_t>(p) * size;
			values[header.fields[f]] = ReadFloatField(addr, 0, header.types[f], size);
		}
		fill(points[static_cast<std::size_t>(p)], values);
	}
	return true;
}

}  // namespace

bool LoadPcdXYZ(const std::string& path, std::vector<PcdPointXYZ>& points) {
	return LoadPcdImpl(path, points, [](PcdPointXYZ& pt, const std::unordered_map<std::string, float>& v) {
		pt.x = v.at("x");
		pt.y = v.at("y");
		pt.z = v.at("z");
	});
}

bool LoadPcdXYZNormal(const std::string& path, std::vector<PcdPointXYZNormal>& points) {
	return LoadPcdImpl(path, points, [](PcdPointXYZNormal& pt, const std::unordered_map<std::string, float>& v) {
		pt.x = v.at("x");
		pt.y = v.at("y");
		pt.z = v.at("z");
		pt.normal_x = v.count("normal_x") ? v.at("normal_x") : 0.f;
		pt.normal_y = v.count("normal_y") ? v.at("normal_y") : 0.f;
		pt.normal_z = v.count("normal_z") ? v.at("normal_z") : 0.f;
	});
}

void LoadVerticesToFeatureMap(
	FeatureMap& feature_map,
	const FeatureChannelType& channel,
	const std::vector<PcdPointXYZ>& points
) {
	feature_map.AllocateDenseFeature(channel, TensorDim(points.size()));
	auto map_cloud = feature_map.GetTypedFeatureValueReadWrite<float4>(channel, MemoryContext::CpuMemory);
	for (std::size_t i = 0; i < points.size(); ++i) {
		map_cloud[i].x = points[i].x;
		map_cloud[i].y = points[i].y;
		map_cloud[i].z = points[i].z;
		map_cloud[i].w = 1.0f;
	}
}

void LoadVerticesAndNormalsToFeatureMap(
	FeatureMap& feature_map,
	const FeatureChannelType& vertex_channel,
	const FeatureChannelType& normal_channel,
	const std::vector<PcdPointXYZNormal>& points
) {
	feature_map.AllocateDenseFeature(vertex_channel, TensorDim(points.size()));
	feature_map.AllocateDenseFeature(normal_channel, TensorDim(points.size()));
	auto map_vertex = feature_map.GetTypedFeatureValueReadWrite<float4>(vertex_channel, MemoryContext::CpuMemory);
	auto map_normal = feature_map.GetTypedFeatureValueReadWrite<float4>(normal_channel, MemoryContext::CpuMemory);
	for (std::size_t i = 0; i < points.size(); ++i) {
		map_vertex[i].x = points[i].x;
		map_vertex[i].y = points[i].y;
		map_vertex[i].z = points[i].z;
		map_vertex[i].w = 1.0f;
		map_normal[i].x = points[i].normal_x;
		map_normal[i].y = points[i].normal_y;
		map_normal[i].z = points[i].normal_z;
		map_normal[i].w = 0.0f;
	}
}

}  // namespace poser
