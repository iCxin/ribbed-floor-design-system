/**
 * ============================================================================
 *  水工钢筋混凝土肋形楼盖辅助设计系统 V1.0
 *  web_server.h —— Web 端服务（本地 HTTP 服务器）
 * ============================================================================
 *  复用同一套五大模块计算内核，通过 HTTP 暴露设计能力：
 *
 *    GET  /              -> 设计界面（web/index.html）
 *    GET  /api/health    -> 健康检查
 *    POST /api/design    -> 结构化参数（JSON）-> 全流水线 -> JSON 报表
 *    POST /api/frame     -> 协议帧（hex 字符串）-> 流水线 -> 结果帧 hex
 *
 *  仅依赖 Winsock2（Windows） / POSIX socket（Linux），无第三方库。
 * ============================================================================
 */
#ifndef RIBBED_FLOOR_WEB_SERVER_H_
#define RIBBED_FLOOR_WEB_SERVER_H_

#include <cstdint>
#include <string>

namespace rfds {

class WebServer {
public:
    explicit WebServer(uint16_t port, std::string static_root = "web");

    /* 阻塞运行直到出错或被终止；socket 初始化失败时返回 false 并填写 err */
    bool run(std::string& err);

    uint16_t port() const { return port_; }

private:
    uint16_t port_;
    std::string static_root_;   /* index.html 等静态资源目录 */
};

}  /* namespace rfds */

#endif  /* RIBBED_FLOOR_WEB_SERVER_H_ */
