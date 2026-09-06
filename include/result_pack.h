/**
 * ============================================================================
 *  水工钢筋混凝土肋形楼盖辅助设计系统 V1.0
 *  result_pack.h —— 模块四：结果封装模块
 * ============================================================================
 *  2.1 节结果封装模块、6.4 节工作流程：
 *
 *    1. 数据归一化与定点数转换   （挠度 ×100 保留两位小数、4bit 直径编码）
 *    2. 协议帧头与元数据构建     （设备地址、功能码、长度、时间戳）
 *    3. 循环冗余校验码生成       （CRC16 附加帧尾前）
 *    4. 双缓冲区交替发送机制     （填充与 DMA 发送并行，零拷贝）
 *
 *  对外接口：
 *    Package_Result(OptResult *opt_result, uint8_t *output, uint16_t *out_len)
 *    输出帧总长度 <= 128 字节。
 * ============================================================================
 */
#ifndef RIBBED_FLOOR_RESULT_PACK_H_
#define RIBBED_FLOOR_RESULT_PACK_H_

#include <cstdint>

#include "common_types.h"
#include "protocol.h"

namespace rfds {

/* 钢筋直径 4 比特编码表（4 个比特位表示常用直径规格）
 * 编码 0 保留表示"无钢筋"；1~9 依次对应 D8~D25。 */
int  encode_bar_diameter(int diameter_mm);        /* 无效返回 0 */
int  decode_bar_diameter(int code);               /* 无效返回 0 */

/* 计算结果输出帧载荷布局（FC 0x81，共 40 字节，帧总长 50 字节 <= 128）
 * 偏移  类型   字段
 *  0    u8    状态码：0x00 成功 / 其他为 ParameterErrorCode
 *  1    u32   设计编号（唯一标识）
 *  5    u32   时间戳（秒）
 *  9    u16   跨度 X (mm)
 * 11    u16   宽度 Y (mm)
 * 13    u16   长期挠度 (0.01mm 定点，"乘以 100")
 * 15    u16   允许挠度 (0.01mm)
 * 17    u16   最大裂缝宽度 (0.001mm 定点)
 * 19    u16   钢筋总重 (0.1kg 定点)
 * 21    u8    校验标志位：bit0 挠度通过 bit1 裂缝通过 bit2 承载力通过
 * 22    u8    板底 X 向钢筋：(直径编码 << 4) | (间距/10)
 * 23    u8    板底 Y 向钢筋
 * 24    u8    支座 X 向顶筋
 * 25    u8    支座 Y 向顶筋
 * 26    u16   配筋率 (1e-5 定点)
 * 28    u16   安全储备系数 (0.01 定点)
 * 30    u8    实际迭代次数
 * 31    u8    构造修正标志 + 保留位
 * 32    u8    混凝土强度等级数值
 * 33    u8    板类型（0 单向 1 双向）
 * 34    u8    载荷版本号（0x01）
 * 35    u8    保留
 * 36    u32   CRC32（OptResult 序列化体校验，数据校验）
 * 40    END                                                            */
constexpr size_t kResultPayloadSize = 40;

class ResultPackager {
public:
    ResultPackager() = default;

    /* 验算结论上下文：OptResult 未携带挠度/裂缝数值，由系统控制器在
     * 封装前注入（模块间共享内存传递信息的等价实现）。 */
    void set_check_context(double long_term_defl_mm, double allow_defl_mm,
                           double crack_mm, bool defl_pass, bool crack_pass) {
        defl_mm_ = long_term_defl_mm;
        allow_defl_mm_ = allow_defl_mm;
        crack_mm_ = crack_mm;
        defl_pass_ = defl_pass;
        crack_pass_ = crack_pass;
    }

    /* 接口：Package_Result —— 将优化结果封装为二进制输出帧。
     * output 缓冲区须 >= kMaxOutputFrameSize；out_len 返回实际帧长。 */
    bool Package_Result(const OptResult& opt_result, const DesignParam& param,
                        uint8_t status_code,
                        uint8_t* output, uint16_t* out_len);

    /* 生成载荷（供存储模块与测试直接复用） */
    bool build_result_payload(const OptResult& opt_result, const DesignParam& param,
                              uint8_t status_code, std::vector<uint8_t>& payload);

    /* 双缓冲交替发送：帧写入内部双缓冲后经 UART 输出 */
    bool send_frame_double_buffered(const std::vector<uint8_t>& frame);

private:
    double defl_mm_ = 0.0;
    double allow_defl_mm_ = 0.0;
    double crack_mm_ = 0.0;
    bool   defl_pass_ = false;
    bool   crack_pass_ = false;
};

}  /* namespace rfds */

#endif  /* RIBBED_FLOOR_RESULT_PACK_H_ */
