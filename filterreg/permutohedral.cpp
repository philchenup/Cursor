#include "permutohedral.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace {

class HashTable {
public:
    HashTable(int key_size, int n_elements)
        : key_size_(static_cast<size_t>(key_size)),
          filled_(0),
          capacity_(static_cast<size_t>(2 * n_elements)),
          keys_((capacity_ / 2 + 10) * key_size_),
          table_(capacity_, -1) {}

    int size() const { return static_cast<int>(filled_); }

    int find(const short* k, bool create = false) {
        if (2 * filled_ >= capacity_) grow();
        size_t h = hash(k) % capacity_;
        while (true) {
            int e = table_[h];
            if (e == -1) {
                if (!create) return -1;
                for (size_t i = 0; i < key_size_; ++i)
                    keys_[filled_ * key_size_ + i] = k[i];
                return table_[h] = static_cast<int>(filled_++);
            }
            bool good = true;
            for (size_t i = 0; i < key_size_ && good; ++i)
                if (keys_[static_cast<size_t>(e) * key_size_ + i] != k[i]) good = false;
            if (good) return e;
            if (++h == capacity_) h = 0;
        }
    }

    const short* getKey(int i) const { return &keys_[static_cast<size_t>(i) * key_size_]; }

private:
    size_t hash(const short* k) const {
        size_t r = 0;
        for (size_t i = 0; i < key_size_; ++i) {
            r += static_cast<size_t>(k[i]);
            r *= 1664525;
        }
        return r;
    }

    void grow() {
        const size_t old_capacity = capacity_;
        capacity_ *= 2;
        std::vector<short> old_keys = keys_;
        std::vector<int> old_table(capacity_, -1);
        table_.swap(old_table);
        keys_.assign((capacity_ / 2 + 10) * key_size_, 0);
        std::copy(old_keys.begin(), old_keys.end(), keys_.begin());
        for (size_t i = 0; i < old_capacity; ++i) {
            if (old_table[i] < 0) continue;
            int e = old_table[i];
            size_t h = hash(getKey(e)) % capacity_;
            for (; table_[h] >= 0; h = (h + 1 == capacity_) ? 0 : h + 1) {
            }
            table_[h] = e;
        }
    }

    size_t key_size_;
    size_t filled_;
    size_t capacity_;
    std::vector<short> keys_;
    std::vector<int> table_;
};

int nearestMultiple(float elevated, float down_factor, float up_factor) {
    const float v = down_factor * elevated;
    const float up = std::ceil(v) * up_factor;
    const float down = std::floor(v) * up_factor;
    return (up - elevated < elevated - down) ? static_cast<int>(up) : static_cast<int>(down);
}

}  // namespace

void Permutohedral::init(const Eigen::MatrixXf& feature, bool with_blur) {
    N_ = static_cast<int>(feature.cols());
    d_ = static_cast<int>(feature.rows());
    with_blur_ = with_blur;

    HashTable hash_table(d_, N_ * (d_ + 1));
    offset_.assign(static_cast<size_t>(d_ + 1) * N_, 0);
    rank_.assign(static_cast<size_t>(d_ + 1) * N_, 0);
    barycentric_.assign(static_cast<size_t>(d_ + 1) * N_, 0.f);

    std::vector<float> scale_factor(d_);
    std::vector<float> elevated(d_ + 1);
    std::vector<float> rem0(d_ + 1);
    std::vector<float> barycentric(d_ + 2);
    std::vector<short> rank(d_ + 1);
    std::vector<short> canonical(static_cast<size_t>(d_ + 1) * (d_ + 1));
    std::vector<short> key(d_ + 1);

    for (int i = 0; i <= d_; ++i) {
        for (int j = 0; j <= d_ - i; ++j) canonical[i * (d_ + 1) + j] = static_cast<short>(i);
        for (int j = d_ - i + 1; j <= d_; ++j)
            canonical[i * (d_ + 1) + j] = static_cast<short>(i - (d_ + 1));
    }

    const float inv_std_dev =
        (with_blur_ ? std::sqrt(2.0 / 3.0) : std::sqrt(1.0 / 6.0)) * static_cast<float>(d_ + 1);
    for (int i = 0; i < d_; ++i)
        scale_factor[i] = inv_std_dev / std::sqrt(static_cast<double>((i + 2) * (i + 1)));

    for (int k = 0; k < N_; ++k) {
        const float* f = &feature(0, k);
        float sm = 0.f;
        for (int j = d_; j > 0; --j) {
            const float cf = f[j - 1] * scale_factor[j - 1];
            elevated[j] = sm - static_cast<float>(j) * cf;
            sm += cf;
        }
        elevated[0] = sm;

        const float down_factor = 1.f / static_cast<float>(d_ + 1);
        const float up_factor = static_cast<float>(d_ + 1);
        int sum = 0;
        for (int i = 0; i <= d_; ++i) {
            rem0[i] = static_cast<float>(nearestMultiple(elevated[i], down_factor, up_factor));
            sum += static_cast<int>(rem0[i] * down_factor);
        }

        std::fill(rank.begin(), rank.end(), 0);
        for (int i = 0; i < d_; ++i) {
            const double di = elevated[i] - rem0[i];
            for (int j = i + 1; j <= d_; ++j) {
                if (di < elevated[j] - rem0[j])
                    ++rank[i];
                else
                    ++rank[j];
            }
        }

        for (int i = 0; i <= d_; ++i) {
            rank[i] = static_cast<short>(rank[i] + sum);
            if (rank[i] < 0) {
                rank[i] = static_cast<short>(rank[i] + d_ + 1);
                rem0[i] += static_cast<float>(d_ + 1);
            } else if (rank[i] > d_) {
                rank[i] = static_cast<short>(rank[i] - d_ - 1);
                rem0[i] -= static_cast<float>(d_ + 1);
            }
        }

        std::fill(barycentric.begin(), barycentric.end(), 0.f);
        for (int i = 0; i <= d_; ++i) {
            const float v = (elevated[i] - rem0[i]) * down_factor;
            barycentric[d_ - rank[i]] += v;
            barycentric[d_ - rank[i] + 1] -= v;
        }
        barycentric[0] += 1.f + barycentric[d_ + 1];

        for (int remainder = 0; remainder <= d_; ++remainder) {
            for (int i = 0; i < d_; ++i)
                key[i] = static_cast<short>(rem0[i] + canonical[remainder * (d_ + 1) + rank[i]]);
            offset_[k * (d_ + 1) + remainder] = hash_table.find(key.data(), true);
            rank_[k * (d_ + 1) + remainder] = rank[remainder];
            barycentric_[k * (d_ + 1) + remainder] = barycentric[remainder];
        }
    }

    M_ = hash_table.size();
    if (!with_blur_) {
        blur_neighbors_.clear();
        return;
    }

    blur_neighbors_.assign(static_cast<size_t>(d_ + 1) * M_, Neighbors{});
    std::vector<short> n1(d_ + 1), n2(d_ + 1);
    for (int j = 0; j <= d_; ++j) {
        for (int i = 0; i < M_; ++i) {
            const short* key_i = hash_table.getKey(i);
            for (int k = 0; k < d_; ++k) {
                n1[k] = static_cast<short>(key_i[k] - 1);
                n2[k] = static_cast<short>(key_i[k] + 1);
            }
            n1[j] = static_cast<short>(key_i[j] + d_);
            n2[j] = static_cast<short>(key_i[j] - d_);
            blur_neighbors_[j * M_ + i].n1 = hash_table.find(n1.data());
            blur_neighbors_[j * M_ + i].n2 = hash_table.find(n2.data());
        }
    }
}

void Permutohedral::splatAndSlice(float* out, const float* in, int value_size, int start) const {
    std::vector<float> values(static_cast<size_t>(M_ + 2) * value_size, 0.f);
    std::vector<float> new_values(static_cast<size_t>(M_ + 2) * value_size, 0.f);

    for (int i = start; i < N_; ++i) {
        for (int j = 0; j <= d_; ++j) {
            const int o = offset_[i * (d_ + 1) + j] + 1;
            const float w = barycentric_[i * (d_ + 1) + j];
            for (int k = 0; k < value_size; ++k)
                values[o * value_size + k] += w * in[i * value_size + k];
        }
    }

    if (with_blur_) {
        for (int j = 0; j <= d_; ++j) {
            for (int i = 0; i < M_; ++i) {
                float* old_val = values.data() + (i + 1) * value_size;
                float* new_val = new_values.data() + (i + 1) * value_size;
                const int n1 = blur_neighbors_[j * M_ + i].n1 + 1;
                const int n2 = blur_neighbors_[j * M_ + i].n2 + 1;
                const float* n1_val = values.data() + n1 * value_size;
                const float* n2_val = values.data() + n2 * value_size;
                for (int k = 0; k < value_size; ++k)
                    new_val[k] = old_val[k] + 0.5f * (n1_val[k] + n2_val[k]);
            }
            values.swap(new_values);
        }
    }

    const float alpha = 1.f / (1.f + std::pow(2.f, static_cast<float>(-d_)));
    for (int i = 0; i < N_; ++i) {
        for (int k = 0; k < value_size; ++k) out[i * value_size + k] = 0.f;
        for (int j = 0; j <= d_; ++j) {
            const int o = offset_[i * (d_ + 1) + j] + 1;
            const float w = barycentric_[i * (d_ + 1) + j];
            for (int k = 0; k < value_size; ++k)
                out[i * value_size + k] += w * values[o * value_size + k] * alpha;
        }
    }
}

Eigen::MatrixXf Permutohedral::compute(const Eigen::MatrixXf& values, int start) const {
    Eigen::MatrixXf out = Eigen::MatrixXf::Zero(values.rows(), values.cols());
    splatAndSlice(out.data(), values.data(), static_cast<int>(values.rows()), start);
    return out;
}
