/**
 * ============================================================================
 *  水工钢筋混凝土肋形楼盖辅助设计系统 V1.0
 *  data_storage.h —— 模块五：数据存储与管理模块
 * ============================================================================
 *  2.1 节数据存储与管理模块、6.5 节工作流程：
 *
 *    1. 哈希索引表构建与内存映射   Md5Index（MD5 -> 4 字节索引）
 *    2. 原子写入事务处理           AtomicWriter（先写备份区后改 FAT）
 *    3. 动态磨损均衡算法执行       WearLevelingManager（最少擦除优先）
 *    4. 自适应垃圾回收与空间清理   StorageManagementModule（阈值触发 GC）
 *
 *  对外接口：
 *    Store_Data(StoreType type, void *data, uint16_t len)
 *  底层基于 FatFS 风格的 NOR Flash 模拟层（hal.h），记录循环覆盖。
 * ============================================================================
 */
#ifndef RIBBED_FLOOR_DATA_STORAGE_H_
#define RIBBED_FLOOR_DATA_STORAGE_H_

#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <vector>

#include "common_types.h"
#include "protocol.h"
#include "rtos_sim.h"

namespace rfds {

/* ------------------------------------------------------------------------ */
/* MD5 摘要（RFC 1321 精简实现，供哈希索引使用；哈希索引算法）       */
/*  输出 16 字节摘要，索引取前 4 字节。                                       */
/* ------------------------------------------------------------------------ */
struct Md5Digest { uint8_t bytes[16]; };
void md5_compute(const uint8_t* data, size_t len, Md5Digest& out);
void md5_compute(const std::string& text, Md5Digest& out);

/* Flash 记录元数据 */
struct FileMetadata {
    uint32_t record_id;        /* 设计编号（唯一标识） */
    uint64_t timestamp_ms;     /* 写入时间戳（毫秒）   */
    StoreType type;            /* 数据类型             */
    uint16_t length;           /* 数据长度 (B)         */
    uint32_t crc32;            /* 数据完整性校验       */
    uint32_t hash_index;       /* MD5 前 4 字节索引值  */
    uint16_t sector;           /* 物理扇区号           */
    uint16_t offset;           /* 扇区内偏移           */
    bool     valid;            /* 有效标志（GC 判据）  */
    FileMetadata() : record_id(0), timestamp_ms(0), type(STORE_DESIGN_PARAM),
        length(0), crc32(0), hash_index(0), sector(0), offset(0), valid(false) {}
};

/* 文件分配表 */
class FileAllocationTable {
public:
    /* 插入/更新记录；同 record_id 旧记录标记无效 */
    void upsert(const FileMetadata& meta);
    /* 按设计编号检索 */
    bool find(uint32_t record_id, FileMetadata& out) const;
    /* 按哈希索引检索（毫秒级内存映射查询） */
    bool find_by_hash(uint32_t hash_index, FileMetadata& out) const;
    /* 全部有效记录 */
    std::vector<FileMetadata> list_valid() const;
    /* 最早的有效记录（循环覆盖判据） */
    bool find_oldest(FileMetadata& out) const;
    /* 无效记录数量（GC 判据） */
    size_t invalid_count() const;
    size_t valid_count() const;
    void clear() { table_.clear(); hash_map_.clear(); }

    /* 存储管理模块需要遍历全表执行 GC/覆盖 */
    friend class StorageManagementModule;

private:
    std::vector<FileMetadata> table_;                  /* 全量元数据     */
    std::map<uint32_t, size_t> hash_map_;              /* 哈希 -> 下标   */
};

/* 磨损均衡管理器 */
class WearLevelingManager {
public:
    /* 为写入 size 字节分配物理扇区（连续占用 count 个扇区时返回起始号）
     * 返回 -1 表示无空闲扇区（需 GC）。 */
    int allocate_sector(size_t byte_count);
    void mark_free(uint16_t sector);
    void mark_used(uint16_t sector);
    uint32_t erase_count(uint16_t sector) const;
    size_t free_sector_count() const;

private:
    bool used_[kFlashSectorCount] = {};
};

/* ------------------------------------------------------------------------ */
/* 存储管理模块                                       */
/* ------------------------------------------------------------------------ */
constexpr size_t kMaxRecordSize = 1024;                 /* 单记录上限 */
/* 记录头 28B：magic(4)+id(4)+时间戳(8)+类型(1)+长度(2)+CRC32(4)+哈希索引(4)+保留(1) */
constexpr size_t kRecordHeaderSize = 28;
constexpr double kGcFreeThreshold = 0.25;               /* 空闲扇区 < 25% 触发 GC */
constexpr size_t kMaxRecords = 128;                     /* 最大记录条数 */

class StorageManagementModule {
public:
    StorageManagementModule();

    /* 接口：Store_Data —— 原子写入一条记录。
     * 返回 false 的情形：数据超长、闪存耗尽且 GC 无法释放。 */
    bool Store_Data(StoreType type, const void* data, uint16_t len,
                    uint32_t record_id);

    /* 按设计编号调取历史数据 */
    bool query_record(uint32_t record_id, std::vector<uint8_t>& data,
                      StoreType* type = nullptr) const;

    /* 上电初始化：扫描 Flash、重建内存哈希索引 */
    void initialize();

    /* 自适应垃圾回收：低优先级任务调用 */
    void run_garbage_collection();

    /* 循环覆盖：闪存满时按时间戳覆盖最早记录 */
    bool evict_oldest();

    /* 状态查询 */
    size_t record_count() const;
    double free_ratio() const;
    const FileAllocationTable& fat() const { return fat_; }

private:
    /* 原子写入：数据 -> 备份扇区 -> 校验 -> FAT 更新 */
    bool atomic_write(const FileMetadata& meta, const uint8_t* data);

    FileAllocationTable fat_;
    WearLevelingManager wear_;
    bool initialized_ = false;
    uint64_t next_timestamp_ = 1;
};

}  /* namespace rfds */

#endif  /* RIBBED_FLOOR_DATA_STORAGE_H_ */
