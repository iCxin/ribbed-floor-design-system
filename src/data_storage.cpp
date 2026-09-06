/**
 * data_storage.cpp —— 模块五：数据存储与管理模块实现
 *
 * 实现说明：记录头含 CRC 校验，配合扇区位图实现原子写事务、
 * 磨损均衡与阈值垃圾回收；MD5 采用 RFC 1321 精简实现，
 * 哈希索引取摘要前 4 字节。
 */
#include "data_storage.h"
#include "hal.h"

#include <cstdio>
#include <algorithm>
#include <array>

namespace rfds {

/* ========================================================================== */
/* MD5（RFC 1321 精简实现）                                                    */
/* ========================================================================== */
namespace {

struct Md5Ctx {
    uint32_t a = 0x67452301, b = 0xefcdab89, c = 0x98badcfe, d = 0x10325476;
    uint64_t total_len = 0;
    uint8_t buffer[64] = {};
    size_t buf_len = 0;
};

inline uint32_t rotl(uint32_t x, int n) { return (x << n) | (x >> (32 - n)); }

inline uint32_t md5_f(uint32_t x, uint32_t y, uint32_t z) { return (x & y) | (~x & z); }
inline uint32_t md5_g(uint32_t x, uint32_t y, uint32_t z) { return (x & z) | (y & ~z); }
inline uint32_t md5_h(uint32_t x, uint32_t y, uint32_t z) { return x ^ y ^ z; }
inline uint32_t md5_i(uint32_t x, uint32_t y, uint32_t z) { return y ^ (x | ~z); }

constexpr uint32_t kMd5K[64] = {
    0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a,
    0xa8304613, 0xfd469501, 0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be,
    0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821, 0xf61e2562, 0xc040b340,
    0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
    0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8,
    0x676f02d9, 0x8d2a4c8a, 0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c,
    0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70, 0x289b7ec6, 0xeaa127fa,
    0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
    0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92,
    0xffeff47d, 0x85845dd1, 0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1,
    0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391,
};
constexpr int kMd5Shift[64] = {
    7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
    5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20,
    4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
    6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21,
};

void md5_block(Md5Ctx& ctx, const uint8_t* block) {
    uint32_t m[16];
    for (int i = 0; i < 16; ++i) {
        m[i] = static_cast<uint32_t>(block[i * 4]) |
               (static_cast<uint32_t>(block[i * 4 + 1]) << 8) |
               (static_cast<uint32_t>(block[i * 4 + 2]) << 16) |
               (static_cast<uint32_t>(block[i * 4 + 3]) << 24);
    }
    uint32_t a = ctx.a, b = ctx.b, c = ctx.c, d = ctx.d;
    for (int i = 0; i < 64; ++i) {
        uint32_t f = 0;
        int g = 0;
        if (i < 16)      { f = md5_f(b, c, d); g = i; }
        else if (i < 32) { f = md5_g(b, c, d); g = (5 * i + 1) % 16; }
        else if (i < 48) { f = md5_h(b, c, d); g = (3 * i + 5) % 16; }
        else             { f = md5_i(b, c, d); g = (7 * i) % 16; }
        f = f + a + kMd5K[i] + m[g];
        a = d;
        d = c;
        c = b;
        b = b + rotl(f, kMd5Shift[i]);
    }
    ctx.a += a; ctx.b += b; ctx.c += c; ctx.d += d;
}

void md5_update(Md5Ctx& ctx, const uint8_t* data, size_t len) {
    ctx.total_len += len;
    while (len > 0) {
        const size_t take = std::min(len, 64 - ctx.buf_len);
        std::memcpy(ctx.buffer + ctx.buf_len, data, take);
        ctx.buf_len += take;
        data += take;
        len -= take;
        if (ctx.buf_len == 64) {
            md5_block(ctx, ctx.buffer);
            ctx.buf_len = 0;
        }
    }
}

void md5_final(Md5Ctx& ctx, Md5Digest& out) {
    const uint64_t bits = ctx.total_len * 8;
    uint8_t pad = 0x80;
    md5_update(ctx, &pad, 1);
    pad = 0x00;
    while (ctx.buf_len != 56) md5_update(ctx, &pad, 1);
    uint8_t len_bytes[8];
    for (int i = 0; i < 8; ++i)
        len_bytes[i] = static_cast<uint8_t>((bits >> (8 * i)) & 0xFF);
    /* md5_update 会继续累加 total_len，但已不再使用 */
    std::memcpy(ctx.buffer + 56, len_bytes, 8);
    md5_block(ctx, ctx.buffer);

    uint32_t regs[4] = {ctx.a, ctx.b, ctx.c, ctx.d};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            out.bytes[i * 4 + j] =
                static_cast<uint8_t>((regs[i] >> (8 * j)) & 0xFF);
}

}  /* anonymous namespace */

void md5_compute(const uint8_t* data, size_t len, Md5Digest& out) {
    Md5Ctx ctx;
    md5_update(ctx, data, len);
    md5_final(ctx, out);
}

void md5_compute(const std::string& text, Md5Digest& out) {
    md5_compute(reinterpret_cast<const uint8_t*>(text.data()), text.size(), out);
}

/* ========================================================================== */
/* FileAllocationTable                                                        */
/* ========================================================================== */
void FileAllocationTable::upsert(const FileMetadata& meta) {
    /* 同编号旧记录置无效（循环覆盖语义） */
    for (auto& m : table_) {
        if (m.record_id == meta.record_id && m.valid) {
            m.valid = false;
            auto it = hash_map_.find(m.hash_index);
            if (it != hash_map_.end() && table_[it->second].record_id == m.record_id)
                hash_map_.erase(it);
        }
    }
    table_.push_back(meta);
    if (meta.valid) hash_map_[meta.hash_index] = table_.size() - 1;
}

bool FileAllocationTable::find(uint32_t record_id, FileMetadata& out) const {
    for (auto it = table_.rbegin(); it != table_.rend(); ++it) {
        if (it->record_id == record_id && it->valid) {
            out = *it;
            return true;
        }
    }
    return false;
}

bool FileAllocationTable::find_by_hash(uint32_t hash_index, FileMetadata& out) const {
    auto it = hash_map_.find(hash_index);
    if (it == hash_map_.end()) return false;
    const FileMetadata& m = table_[it->second];
    if (!m.valid) return false;
    out = m;
    return true;
}

std::vector<FileMetadata> FileAllocationTable::list_valid() const {
    std::vector<FileMetadata> out;
    for (const auto& m : table_)
        if (m.valid) out.push_back(m);
    return out;
}

bool FileAllocationTable::find_oldest(FileMetadata& out) const {
    bool found = false;
    for (const auto& m : table_) {
        if (!m.valid) continue;
        if (!found || m.timestamp_ms < out.timestamp_ms) {
            out = m;
            found = true;
        }
    }
    return found;
}

size_t FileAllocationTable::invalid_count() const {
    size_t n = 0;
    for (const auto& m : table_)
        if (!m.valid) ++n;
    return n;
}

size_t FileAllocationTable::valid_count() const {
    size_t n = 0;
    for (const auto& m : table_)
        if (m.valid) ++n;
    return n;
}

/* ========================================================================== */
/* WearLevelingManager                                                        */
/* ========================================================================== */
int WearLevelingManager::allocate_sector(size_t byte_count) {
    const size_t need = (byte_count + kFlashSectorSize - 1) / kFlashSectorSize;
    if (need > 1) return -1;   /* 简化模型：单记录不超过一个扇区 */

    /* 磨损均衡：在空闲扇区中选择擦写次数最少者 */
    int best = -1;
    uint32_t best_wear = 0xFFFFFFFFu;
    for (size_t s = 0; s < kFlashSectorCount; ++s) {
        if (used_[s]) continue;
        const uint32_t wear = hal_flash_get_erase_count(s);
        if (wear < best_wear) {
            best_wear = wear;
            best = static_cast<int>(s);
        }
    }
    if (best >= 0) used_[best] = true;
    return best;
}

void WearLevelingManager::mark_free(uint16_t sector) {
    if (sector < kFlashSectorCount) used_[sector] = false;
}
void WearLevelingManager::mark_used(uint16_t sector) {
    if (sector < kFlashSectorCount) used_[sector] = true;
}
uint32_t WearLevelingManager::erase_count(uint16_t sector) const {
    return sector < kFlashSectorCount ? hal_flash_get_erase_count(sector) : 0;
}
size_t WearLevelingManager::free_sector_count() const {
    size_t n = 0;
    for (size_t s = 0; s < kFlashSectorCount; ++s)
        if (!used_[s]) ++n;
    return n;
}

/* ========================================================================== */
/* StorageManagementModule                                                    */
/* ========================================================================== */
StorageManagementModule::StorageManagementModule() = default;

void StorageManagementModule::initialize() {
    /* 上电：Flash 出厂态模拟（首次），扫描记录头重建索引 */
    fat_.clear();
    std::vector<uint8_t> header(kRecordHeaderSize);
    for (size_t s = 0; s < kFlashSectorCount; ++s) {
        if (hal_flash_read(s * kFlashSectorSize, header.data(), 8) != HAL_FLASH_OK)
            continue;
        uint32_t magic = 0;
        std::memcpy(&magic, header.data(), 4);
        if (magic != 0x52464453u) {   /* "SDFR" 小端：有效记录起始标记 */
            continue;                 /* 空扇区（0xFF 填充 magic 不会匹配） */
        }
        /* 解析记录头 */
        if (hal_flash_read(s * kFlashSectorSize, header.data(),
                           kRecordHeaderSize) != HAL_FLASH_OK) continue;
        FileMetadata meta;
        std::memcpy(&meta.record_id, header.data() + 4, 4);
        std::memcpy(&meta.timestamp_ms, header.data() + 8, 8);
        StoreType t;
        std::memcpy(&t, header.data() + 16, 1);
        meta.type = t;
        std::memcpy(&meta.length, header.data() + 17, 2);
        std::memcpy(&meta.crc32, header.data() + 19, 4);
        std::memcpy(&meta.hash_index, header.data() + 23, 4);
        meta.sector = static_cast<uint16_t>(s);
        meta.offset = kRecordHeaderSize;
        meta.valid = true;
        fat_.upsert(meta);
        wear_.mark_used(static_cast<uint16_t>(s));
    }
    initialized_ = true;
}

/* 记录头序列化（28 字节：magic+id+时间戳+类型+长度+CRC+哈希索引+保留） */
static void serialize_record_header(uint8_t* out, const FileMetadata& meta) {
    std::memset(out, 0, kRecordHeaderSize);
    const uint32_t magic = 0x52464453u;   /* "SDFR" */
    std::memcpy(out, &magic, 4);
    std::memcpy(out + 4, &meta.record_id, 4);
    std::memcpy(out + 8, &meta.timestamp_ms, 8);
    const uint8_t type = static_cast<uint8_t>(meta.type);
    std::memcpy(out + 16, &type, 1);
    std::memcpy(out + 17, &meta.length, 2);
    std::memcpy(out + 19, &meta.crc32, 4);
    std::memcpy(out + 23, &meta.hash_index, 4);
}

bool StorageManagementModule::atomic_write(const FileMetadata& meta,
                                           const uint8_t* data) {
    const size_t addr = static_cast<size_t>(meta.sector) * kFlashSectorSize;
    /* 1. 擦除目标扇区（NOR 特性：先擦后写） */
    if (hal_flash_erase_sector(meta.sector) != HAL_FLASH_OK) return false;
    /* 2. 写入记录头（临时备份区语义：头先落盘，数据随后，任一步失败
     *    时头中 CRC 不匹配即可判定记录无效，维持一致性） */
    uint8_t header[kRecordHeaderSize];
    serialize_record_header(header, meta);
    if (hal_flash_write_page(addr, header, kRecordHeaderSize) != HAL_FLASH_OK)
        return false;
    /* 3. 分页写入数据体 */
    size_t written = 0;
    while (written < meta.length) {
        const size_t page_off = addr + kRecordHeaderSize + written;
        const size_t take = std::min(kFlashPageSize,
                                     meta.length - written);
        if (hal_flash_write_page(page_off, data + written, take) != HAL_FLASH_OK)
            return false;
        written += take;
    }
    /* 4. 回读校验（写入校验成功后才允许更新 FAT） */
    std::vector<uint8_t> verify(meta.length);
    if (hal_flash_read(addr + kRecordHeaderSize, verify.data(), meta.length)
            != HAL_FLASH_OK) return false;
    return crc32(verify) == meta.crc32;
}

bool StorageManagementModule::Store_Data(StoreType type, const void* data,
                                         uint16_t len, uint32_t record_id) {
    if (!initialized_) initialize();
    if (data == nullptr || len == 0 || len > kMaxRecordSize) return false;

    /* 容量守卫：记录数上限或空闲扇区不足时先尝试 GC，再循环覆盖 */
    if (fat_.valid_count() >= kMaxRecords ||
        wear_.free_sector_count() == 0) {
        run_garbage_collection();
        while (wear_.free_sector_count() == 0) {
            if (!evict_oldest()) return false;   /* 覆盖失败：闪存异常 */
        }
    }

    const uint8_t* bytes = static_cast<const uint8_t*>(data);

    /* 哈希索引：MD5(记录内容) 前 4 字节 */
    Md5Digest digest;
    md5_compute(bytes, len, digest);
    uint32_t hash = 0;
    std::memcpy(&hash, digest.bytes, 4);
    if (hash == 0) hash = 1;   /* 避免与"无索引"冲突 */

    FileMetadata meta;
    meta.record_id = record_id;
    meta.timestamp_ms = ++next_timestamp_;
    meta.type = type;
    meta.length = len;
    meta.crc32 = crc32(bytes, len);
    meta.hash_index = hash;
    meta.valid = true;

    /* 磨损均衡分配物理扇区 */
    const int sector = wear_.allocate_sector(len + kRecordHeaderSize);
    if (sector < 0) {
        if (!evict_oldest()) return false;
        const int retry = wear_.allocate_sector(len + kRecordHeaderSize);
        if (retry < 0) return false;
        meta.sector = static_cast<uint16_t>(retry);
    } else {
        meta.sector = static_cast<uint16_t>(sector);
    }
    meta.offset = kRecordHeaderSize;

    /* 原子写入事务 */
    if (!atomic_write(meta, bytes)) {
        wear_.mark_free(meta.sector);   /* 释放失败扇区 */
        return false;
    }
    fat_.upsert(meta);   /* 校验通过后才更新 FAT（先写后改） */
    return true;
}

bool StorageManagementModule::query_record(uint32_t record_id,
                                           std::vector<uint8_t>& data,
                                           StoreType* type) const {
    FileMetadata meta;
    if (!fat_.find(record_id, meta)) return false;
    data.resize(meta.length);
    const size_t addr = static_cast<size_t>(meta.sector) * kFlashSectorSize +
                        meta.offset;
    if (hal_flash_read(addr, data.data(), meta.length) != HAL_FLASH_OK)
        return false;
    if (crc32(data) != meta.crc32) return false;   /* 数据损坏 */
    if (type) *type = meta.type;
    return true;
}

void StorageManagementModule::run_garbage_collection() {
    /* 自适应触发：空闲扇区低于阈值或无效记录堆积时执行 */
    const double free_r =
        static_cast<double>(wear_.free_sector_count()) / kFlashSectorCount;
    if (free_r > kGcFreeThreshold && fat_.invalid_count() == 0) return;

    /* 释放仅含无效记录的扇区；若该扇区已被某条有效记录占用则跳过 */
    for (const auto& m : fat_.table_) {
        if (m.valid) continue;
        bool claimed = false;
        for (const auto& v : fat_.table_) {
            if (v.valid && v.sector == m.sector) { claimed = true; break; }
        }
        if (!claimed) wear_.mark_free(m.sector);
    }
}

bool StorageManagementModule::evict_oldest() {
    FileMetadata oldest;
    if (!fat_.find_oldest(oldest)) return false;
    /* 同扇区可能存在同 id 的历史无效记录，一并释放 */
    for (auto& m : fat_.table_) {
        if (m.valid && m.sector == oldest.sector &&
            m.timestamp_ms == oldest.timestamp_ms) {
            m.valid = false;
        }
    }
    wear_.mark_free(oldest.sector);
    return true;
}

size_t StorageManagementModule::record_count() const { return fat_.valid_count(); }

double StorageManagementModule::free_ratio() const {
    return static_cast<double>(wear_.free_sector_count()) / kFlashSectorCount;
}

}  /* namespace rfds */
