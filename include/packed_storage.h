#ifndef PACKED_STORAGE_H_
#define PACKED_STORAGE_H_

#include <fstream>

namespace stp::util {

template <std::size_t bits_per_entry = 8>
    requires((bits_per_entry & 7) == 0)
class PackedStorage {
public:
    using value_type = std::conditional_t<
        bits_per_entry == 8, uint8_t,
        std::conditional_t<bits_per_entry == 16, uint16_t,
                           std::conditional_t<bits_per_entry == 32, uint32_t, uint64_t>>>;

    explicit PackedStorage(const std::size_t size = 0) : size_(size) { Resize(size); }

    PackedStorage(const PackedStorage&) = default;

    PackedStorage& operator=(const PackedStorage&) = default;

    PackedStorage(PackedStorage&&) = default;

    PackedStorage& operator=(PackedStorage&&) = default;

    void Resize(const std::size_t size, const uint64_t value = 0) {
        size_ = size;
        constexpr std::size_t num_elements_per_uint64 = 64 / bits_per_entry;
        data_.resize((size + num_elements_per_uint64 - 1) / num_elements_per_uint64, value);
    }

    [[nodiscard]] auto Size() const { return size_; }

    [[nodiscard]] auto& Data() const { return data_; }

    [[nodiscard]] auto& Data() { return data_; }

    friend std::ofstream& operator<<(std::ofstream& ofs, const PackedStorage& storage) {
        ofs.write(reinterpret_cast<const char*>(storage.Data().data()),
                  static_cast<std::streamsize>(storage.Data().size() * sizeof(uint64_t)));
        return ofs;
    }

    friend std::ifstream& operator>>(std::ifstream& ifs, PackedStorage& storage) {
        ifs.read(reinterpret_cast<char*>(storage.Data().data()),
                 static_cast<std::streamsize>(storage.Data().size() * sizeof(uint64_t)));
        return ifs;
    }

    [[nodiscard]] value_type Get(const uint64_t index) const {
        const auto [idx, shift] = Find(index);
        if (idx >= data_.size()) {
            return max_value_;
        }
        return (data_[idx] >> shift) & max_value_;
    }

    [[nodiscard]] value_type GetAtomic(const uint64_t index) {
        const auto [idx, shift] = Find(index);
        if (idx >= data_.size()) {
            return max_value_;
        }
        const std::atomic_ref ref(data_[idx]);
        return (ref.load(std::memory_order_acquire) >> shift) & max_value_;
    }

    void Set(const uint64_t index, const value_type value) {
        const auto [idx, shift] = Find(index);
        if (idx >= data_.size()) {
            return;
        }
        const uint64_t mask = static_cast<uint64_t>(max_value_) << shift;
        const uint64_t desired_bits = static_cast<uint64_t>(value) << shift;
        data_[idx] = (data_[idx] & ~mask) | desired_bits;
    }

    void SetAtomic(const uint64_t index, const value_type value) {
        const auto [idx, shift] = Find(index);
        if (idx >= data_.size()) return;
        const uint64_t mask = static_cast<uint64_t>(max_value_) << shift;
        const uint64_t bits = static_cast<uint64_t>(value) << shift;
        const std::atomic_ref ref(data_[idx]);
        ref.store((ref.load(std::memory_order_acquire) & ~mask) | bits, std::memory_order_release);
    }

    bool CompareAndSet(const uint64_t index, const value_type expected, const value_type desired) {
        const auto [idx, shift] = Find(index);
        if (idx >= data_.size()) {
            return false;
        }
        const uint64_t mask = static_cast<uint64_t>(max_value_) << shift;
        const uint64_t expected_bits = static_cast<uint64_t>(expected) << shift;
        const uint64_t desired_bits = static_cast<uint64_t>(desired) << shift;
        const std::atomic_ref ref(data_[idx]);
        uint64_t stored = ref.load(std::memory_order_acquire);
        while (true) {
            if ((stored & mask) != expected_bits) {
                return false;
            }
            if (const uint64_t new_val = (stored & ~mask) | desired_bits; ref.compare_exchange_weak(
                    stored, new_val, std::memory_order_release, std::memory_order_relaxed)) {
                return true;
            }
        }
    }

private:
    static std::pair<std::size_t, uint32_t> Find(const uint64_t index) {
        constexpr std::size_t num_elements_per_uint64 = 64 / bits_per_entry;
        return {index / num_elements_per_uint64,
                (index % num_elements_per_uint64) * bits_per_entry};
    }

    static constexpr value_type max_value_ = std::numeric_limits<value_type>::max();
    std::size_t size_;
    std::vector<uint64_t> data_;
};
}  // namespace stp::util

#endif  // PACKED_STORAGE_H_
